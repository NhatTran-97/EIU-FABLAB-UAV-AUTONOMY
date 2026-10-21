"""CommandAdapter: flight commands for QML (core/commands/flight.py).

    commands.*         : buttons under the map -> Jetson (radio 433), plus Send WP
    commands.direct.*  : PX4 box in the top bar -> Take-off / Land directly to PX4 (radio 915)
    commands.gates     : {px4: {takeoff, land}, jetson: {takeoff, land, hold, rtl, upload}}
                         '' = enabled, otherwise the lock reason (cross-route lock, link lost, GPS, DDS)
"""

from PySide6.QtCore import Property, QObject, QTimer, Signal, Slot

from ground_station_ui.core.commands.flight import ROUTE_NAMES, FlightCommander
from ground_station_ui.core.commands.sender import CommandTarget
from ground_station_ui.core.mavlink import drone_dialect as mavlink
from ground_station_ui.qt.adapters import GroupAdapter


class CommandAdapter(GroupAdapter):
    """values = Jetson command state {busy, label, state, attempt, retries, route, message, step, steps};
    direct.values = the same for direct PX4 commands. See core/commands/sender.py"""

    gatesChanged = Signal()

    SETTINGS_DELAY_MS = 600     # live settings are sent once the value has stopped changing for this long

    def __init__(self, sender, direct_sender, settings_sender, uploader, mission, vehicle, environment, cfg,
                 parent=None):
        super().__init__(sender, parent)
        self.uploader = uploader
        self.mission = mission
        self.vehicle = vehicle
        self.settings_sender = settings_sender
        self.live = GroupAdapter(settings_sender, self)    # live mission settings state for QML
        self._settings_timer = QTimer(self)
        self._settings_timer.setSingleShot(True)
        self._settings_timer.setInterval(self.SETTINGS_DELAY_MS)
        self._settings_timer.timeout.connect(self._send_settings)
        mission.on_options_changed = self._settings_timer.start
        self.upload = GroupAdapter(uploader, self)      # waypoint upload state for QML
        self.direct = GroupAdapter(direct_sender, self)  # direct PX4 command state
        self.target = CommandTarget(cfg.get('link', 'jetson'), cfg.get('target_system', 1),
                                    cfg.get('target_component', mavlink.MAV_COMP_ID_ONBOARD_COMPUTER),
                                    cfg.get('ack_timeout_s', 1.5), cfg.get('retries', 3),
                                    name=ROUTE_NAMES['jetson'])
        self.commander = FlightCommander({'px4': direct_sender, 'jetson': sender}, vehicle,
                                         self.target, environment)
        self._gates = {}
        # default take-off altitude per environment (= MIS_TAKEOFF_ALT in config/px4/*.params)
        self._takeoff = {'indoor': float(cfg.get('takeoff_altitude_indoor', 1.5)),
                         'outdoor': float(cfg.get('takeoff_altitude_outdoor', 5.0))}
        self.flush()

    def _do(self, route, action, altitude=float('nan')):
        self.commander.execute(route, action, altitude)
        self.flush()

    # --- buttons under the map: via the Jetson ------------------------------------------------
    @Slot(float)
    def takeoff(self, altitude):
        self._do('jetson', 'takeoff', float(altitude))

    @Slot()
    def land(self):
        self._do('jetson', 'land')

    @Slot()
    def hold(self):
        self._do('jetson', 'hold')

    @Slot()
    def rtl(self):
        self._do('jetson', 'rtl')

    @Slot(bool)
    def startMission(self, avoidance):
        """Fly the uploaded mission (Jetson mission_manager, from HOVER) at the mission speed, optionally
        with obstacle avoidance."""
        v = self.mission.values
        self.commander.execute('jetson', 'mission', options={
            'avoidance': bool(avoidance), 'speed': v['speed'],
            'xy_tolerance': v['xyTolerance'], 'z_tolerance': v['zTolerance']})
        self.flush()

    def mission_running(self):
        p = self.vehicle.groups['missionProgress'].values
        return bool(p.get('valid')) and p.get('state') in ('active', 'paused')

    def _send_settings(self):
        """Send speed and arrival tolerances to the running mission (MAV_CMD_USER_2, 0 = unchanged)."""
        if not self.mission_running() or not self.commander.available()['jetson']:
            return
        v = self.mission.values
        self.settings_sender.send_steps(self.commander._jetson(),
                                        [(mavlink.MAV_CMD_USER_2, [v['speed'], v['xyTolerance'], v['zTolerance']])],
                                        'Mission settings')
        self.live.flush()

    @Slot()
    def continueMission(self):
        """Continue after a manual task at a waypoint, or after a pause (pilot took control)."""
        self._do('jetson', 'continue')

    # --- Test panel: send via the Jetson, skipping GCS-side checks -------------------------------
    @Slot()
    def ping(self):
        self.commander.ping()
        self.flush()

    @Slot(float)
    def testTakeoff(self, altitude):
        self.commander.execute('jetson', 'takeoff', float(altitude), test=True)
        self.flush()

    @Slot()
    def testLand(self):
        self.commander.execute('jetson', 'land', test=True)
        self.flush()

    @Slot()
    def testUploadMission(self):
        """Upload waypoints without the GPS check (the Jetson stores them, it does not fly)."""
        if self.mission.items:
            self.uploader.upload(self.commander._jetson(), [dict(w) for w in self.mission.items])
            self.upload.flush()

    @Slot()
    def uploadMission(self):
        if self.mission.items and not self.commander.blocker('jetson', 'upload'):
            self.uploader.upload(self.commander._jetson(), [dict(w) for w in self.mission.items])
            self.upload.flush()

    @Slot()
    def cancelUpload(self):
        self.uploader.cancel()
        self.upload.flush()

    # --- PX4 box in the top bar: directly to PX4 ----------------------------------------------
    @Slot(float)
    def directTakeoff(self, altitude):
        self._do('px4', 'takeoff', float(altitude))

    @Slot()
    def directLand(self):
        self._do('px4', 'land')

    def tick(self, now):
        self.commander.tick(now)

    def flush(self):
        super().flush()
        self.upload.flush()
        self.live.flush()
        self.direct.flush()
        gates = self.commander.gates()
        gates['owner'] = self.commander.lock.owner or ''
        gates['jetsonLink'] = self.commander.vehicle.companion_link     # 433 / SITL UDP
        if gates != self._gates:
            self._gates = gates
            self.gatesChanged.emit()

    missionUpload = Property(QObject, lambda self: self.upload, constant=True)
    liveSettings = Property(QObject, lambda self: self.live, constant=True)
    directCommand = Property(QObject, lambda self: self.direct, constant=True)
    takeoffAltitudes = Property('QVariantMap', lambda self: self._takeoff, constant=True)
    targetLink = Property(str, lambda self: self.commander.vehicle.companion_link, notify=gatesChanged)
    gates = Property('QVariantMap', lambda self: self._gates, notify=gatesChanged)
