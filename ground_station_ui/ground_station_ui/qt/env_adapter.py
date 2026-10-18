"""EnvironmentAdapter: moi truong bay (Tu dong / Trong nha / Ngoai troi) cho ca app.

Tu dong: theo bo tham so PX4 dang nap (EKF2_GPS_CTRL / EKF2_HGT_REF); chua doc duoc thi tam
theo GPS fix. Quyet dinh: nguon do cao hien thi, do cao cat canh mac dinh, mau canh bao GPS,
va canh bao khi PX4 dang nap bo tham so khong khop (vd ra ngoai troi ma quen nap outdoor.params).
"""

from PySide6.QtCore import Property, QObject, QSettings, Signal, Slot

from ground_station_ui.core.vehicle.groups import ALT_MODES
from ground_station_ui.core.vehicle.params import resolve_environment

LABELS = {'indoor': 'Indoor', 'outdoor': 'Outdoor', 'unknown': 'unknown'}


class EnvironmentAdapter(QObject):
    changed = Signal()

    def __init__(self, vehicle, params, parent=None):
        super().__init__(parent)
        self.flight = vehicle.groups['flight']
        self.gps = vehicle.groups['gps']
        self.params = params
        self._settings = QSettings()
        mode = self._settings.value('env/mode', 'auto', type=str)
        self._mode = mode if mode in ALT_MODES else 'auto'
        self._state = {}
        self.flush(force=True)

    # --- tinh trang thai -------------------------------------------------------------------
    def _compute(self):
        fix = self.gps.values.get('fixType', 0) or 0
        profile = self.params.values['profile']
        effective, source = resolve_environment(self._mode, profile, fix)
        mismatch = ''
        # Chi canh bao khi NGUY HIEM: PX4 tat GPS / do cao theo range (cau hinh trong nha MTF-02) ma
        # dang bay ngoai troi. PX4 cau hinh GPS + baro (outdoor / no_mtf02) dung trong nha van an toan.
        if profile == 'indoor' and effective == 'outdoor':
            mismatch = ("PX4 is using the indoor (optical-flow, GPS off) config but the app is in outdoor "
                        "mode — load config/px4/outdoor.params in QGC or change Flight environment in Settings")
        return {'mode': self._mode, 'effective': effective, 'effectiveLabel': LABELS[effective],
                'source': source, 'px4Profile': profile, 'px4ProfileLabel': LABELS[profile], 'mismatch': mismatch,
                'gpsCtrl': self.params.values['EKF2_GPS_CTRL'], 'hgtRef': self.params.values['EKF2_HGT_REF'],
                'gpsFix': fix}

    def flush(self, force=False):
        state = self._compute()
        # do cao hien thi: da biet moi truong (tay / PX4) -> ep theo do; chua biet -> FlightGroup tu theo GPS
        self.flight.set_mode(state['effective'] if state['source'] != 'gps' else 'auto')
        if force or state != self._state:
            self._state = state
            self.changed.emit()

    # --- QML -------------------------------------------------------------------------------
    @Slot(str)
    def setMode(self, mode):
        if mode in ALT_MODES and mode != self._mode:
            self._mode = mode
            self._settings.setValue('env/mode', mode)
            self.flush(force=True)

    state = Property('QVariantMap', lambda self: self._state, notify=changed)
