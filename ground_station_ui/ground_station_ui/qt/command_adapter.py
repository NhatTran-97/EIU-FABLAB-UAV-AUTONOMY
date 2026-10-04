"""CommandAdapter: nut Take-off / Land / Gui waypoint (QML) -> Jetson (CommandSender, MissionUploader)."""

from PySide6.QtCore import Property, QObject, Slot

from ground_station_ui.core.commands.sender import CommandTarget
from ground_station_ui.core.mavlink import drone_dialect as mavlink
from ground_station_ui.qt.adapters import GroupAdapter


class CommandAdapter(GroupAdapter):
    """values = {busy, label, state, attempt, retries}; xem core/commands/sender.py"""

    def __init__(self, sender, uploader, mission, cfg, parent=None):
        super().__init__(sender, parent)
        self.sender = sender
        self.uploader = uploader
        self.mission = mission
        self.upload = GroupAdapter(uploader, self)      # trang thai gui waypoint cho QML
        self.target = CommandTarget(cfg.get('link', 'jetson'), cfg.get('target_system', 1),
                                    cfg.get('target_component', mavlink.MAV_COMP_ID_ONBOARD_COMPUTER),
                                    cfg.get('ack_timeout_s', 1.5), cfg.get('retries', 3))
        # do cao cat canh mac dinh theo moi truong (khop MIS_TAKEOFF_ALT trong config/px4/*.params)
        self._takeoff = {'indoor': float(cfg.get('takeoff_altitude_indoor', 1.5)),
                         'outdoor': float(cfg.get('takeoff_altitude_outdoor', 5.0))}

    @Slot(float)
    def takeoff(self, altitude):
        # NAV_TAKEOFF: param7 = do cao (m); con lai de 0 / NaN cho dich tu chon
        self.sender.send(self.target, mavlink.MAV_CMD_NAV_TAKEOFF,
                         (0, 0, 0, float('nan'), float('nan'), float('nan'), float(altitude)),
                         label=f'Take-off {altitude:g} m')
        self.flush()

    @Slot()
    def land(self):
        self.sender.send(self.target, mavlink.MAV_CMD_NAV_LAND,
                         (0, 0, 0, float('nan'), float('nan'), float('nan'), 0), label='Land')
        self.flush()

    @Slot()
    def hold(self):
        # Jetson doi thanh DO_SET_MODE AUTO.LOITER cho PX4
        self.sender.send(self.target, mavlink.MAV_CMD_NAV_LOITER_UNLIM, (), label='Hold')
        self.flush()

    @Slot()
    def rtl(self):
        self.sender.send(self.target, mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH, (), label='RTL')
        self.flush()

    @Slot()
    def uploadMission(self):
        if self.mission.items:
            self.uploader.upload(self.target, [dict(w) for w in self.mission.items])
            self.upload.flush()

    @Slot()
    def cancelUpload(self):
        self.uploader.cancel()
        self.upload.flush()

    def flush(self):
        super().flush()
        self.upload.flush()

    missionUpload = Property(QObject, lambda self: self.upload, constant=True)
    takeoffAltitudes = Property('QVariantMap', lambda self: self._takeoff, constant=True)
    targetLink = Property(str, lambda self: self.target.link, constant=True)
