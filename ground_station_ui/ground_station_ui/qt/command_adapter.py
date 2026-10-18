"""CommandAdapter: lenh bay cho QML (core/commands/flight.py).

    commands.*         : 3 nut duoi ban do -> Jetson (radio 433) + Send WP
    commands.direct.*  : o PX4 tren thanh tren cung -> Take-off / Land thang PX4 (radio 915)
    commands.gates     : {px4: {takeoff, land}, jetson: {takeoff, land, hold, rtl, upload}}
                         '' = nut dung duoc, nguoc lai = ly do khoa (khoa cheo, mat link, GPS, DDS)
"""

from PySide6.QtCore import Property, QObject, Signal, Slot

from ground_station_ui.core.commands.flight import ROUTE_NAMES, FlightCommander
from ground_station_ui.core.commands.sender import CommandTarget
from ground_station_ui.core.mavlink import drone_dialect as mavlink
from ground_station_ui.qt.adapters import GroupAdapter


class CommandAdapter(GroupAdapter):
    """values = trang thai lenh Jetson {busy, label, state, attempt, retries, route, message, step, steps};
    direct.values = tuong tu cho lenh thang PX4. Xem core/commands/sender.py"""

    gatesChanged = Signal()

    def __init__(self, sender, direct_sender, uploader, mission, vehicle, environment, cfg, parent=None):
        super().__init__(sender, parent)
        self.uploader = uploader
        self.mission = mission
        self.upload = GroupAdapter(uploader, self)      # trang thai gui waypoint cho QML
        self.direct = GroupAdapter(direct_sender, self)  # trang thai lenh thang PX4
        self.target = CommandTarget(cfg.get('link', 'jetson'), cfg.get('target_system', 1),
                                    cfg.get('target_component', mavlink.MAV_COMP_ID_ONBOARD_COMPUTER),
                                    cfg.get('ack_timeout_s', 1.5), cfg.get('retries', 3),
                                    name=ROUTE_NAMES['jetson'])
        self.commander = FlightCommander({'px4': direct_sender, 'jetson': sender}, vehicle,
                                         self.target, environment)
        self._gates = {}
        # do cao cat canh mac dinh theo moi truong (khop MIS_TAKEOFF_ALT trong config/px4/*.params)
        self._takeoff = {'indoor': float(cfg.get('takeoff_altitude_indoor', 1.5)),
                         'outdoor': float(cfg.get('takeoff_altitude_outdoor', 5.0))}
        self.flush()

    def _do(self, route, action, altitude=float('nan')):
        self.commander.execute(route, action, altitude)
        self.flush()

    # --- 3 nut duoi ban do: qua Jetson ------------------------------------------------------
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

    # --- nut 🧪 Test: kiem tra duong Jetson trong phong thi nghiem ---------------------------
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
        """Gui waypoint bo qua dieu kien GPS (Jetson chi luu, khong bay)."""
        if self.mission.items:
            self.uploader.upload(self.target, [dict(w) for w in self.mission.items])
            self.upload.flush()

    @Slot()
    def uploadMission(self):
        if self.mission.items and not self.commander.blocker('jetson', 'upload'):
            self.uploader.upload(self.target, [dict(w) for w in self.mission.items])
            self.upload.flush()

    @Slot()
    def cancelUpload(self):
        self.uploader.cancel()
        self.upload.flush()

    # --- o PX4 tren thanh tren cung: thang PX4 -----------------------------------------------
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
        self.direct.flush()
        gates = self.commander.gates()
        gates['owner'] = self.commander.lock.owner or ''
        if gates != self._gates:
            self._gates = gates
            self.gatesChanged.emit()

    missionUpload = Property(QObject, lambda self: self.upload, constant=True)
    directCommand = Property(QObject, lambda self: self.direct, constant=True)
    takeoffAltitudes = Property('QVariantMap', lambda self: self._takeoff, constant=True)
    targetLink = Property(str, lambda self: self.target.link, constant=True)
    gates = Property('QVariantMap', lambda self: self._gates, notify=gatesChanged)
