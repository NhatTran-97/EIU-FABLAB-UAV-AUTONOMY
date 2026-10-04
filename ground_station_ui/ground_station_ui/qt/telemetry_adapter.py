"""TelemetryAdapter: tan so / tuoi message + lich su do thi cho trang Telemetry."""

import time

from PySide6.QtCore import Property, QObject, Signal, Slot

from ground_station_ui.core.telemetry.stats import History, MessageStats

# Tin hieu co the ve: ten hien thi -> (group, key)
SIGNALS = {
    'Altitude (m)': ('flight', 'altRel', ('LOCAL_POSITION_NED', 'GLOBAL_POSITION_INT')),
    'Ground speed (m/s)': ('flight', 'groundSpeed', ('VFR_HUD',)),
    'Vertical speed (m/s)': ('flight', 'climbRate', ('VFR_HUD',)),
    'Battery (V)': ('battery', 'voltage', ('SYS_STATUS',)),
    'Roll (°)': ('attitude', 'roll', ('ATTITUDE',)),
    'Pitch (°)': ('attitude', 'pitch', ('ATTITUDE',)),
    'Yaw (°)': ('attitude', 'heading', ('ATTITUDE',)),
    'RSSI 915 (dBm)': ('@links', 'px4', ('RADIO_STATUS',)),
    'Gimbal pitch (°)': ('gimbal', 'pitch', ('DRONE_GIMBAL_STATE',)),
}
FRESH_S = 2.5     # message nguon cu hon -> KHONG lay mau (do thi ngat, khong ve duong ngang gia)


class TelemetryAdapter(QObject):
    changed = Signal()

    def __init__(self, vehicle, router, links=None, parent=None):
        super().__init__(parent)
        self.stats = MessageStats()
        self.stats.attach(router)
        g = vehicle.groups
        self._now = time.monotonic()

        def getter(grp, key, sources):
            def get():
                if not any(self.stats.age(t, self._now) <= FRESH_S for t in sources):
                    return None
                if grp == '@links':
                    return ((links.values.get(key) or {}).get('rssiDbm')) if links else None
                return g[grp].values.get(key)
            return get

        self.history = History({name: getter(*spec) for name, spec in SIGNALS.items()})
        self._snap = {}

    def flush(self):
        now = self._now = time.monotonic()
        self.history.sample(now)
        self._snap = self.stats.snapshot(now)
        self.changed.emit()

    @Slot(str, float, result='QVariantList')
    def series(self, name, window_s):
        """[[t (s, am = qua khu), gia tri], ...] trong window_s giay gan nhat."""
        return self.history.series(name, time.monotonic(), window_s)

    msgs = Property('QVariantMap', lambda self: self._snap, notify=changed)
    signalNames = Property('QStringList', lambda self: list(SIGNALS), constant=True)
