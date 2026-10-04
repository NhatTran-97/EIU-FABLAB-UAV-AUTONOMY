"""EnvironmentAdapter: moi truong bay (Tu dong / Trong nha / Ngoai troi) cho ca app.

Quyet dinh: nguon do cao hien thi, do cao cat canh mac dinh, mau canh bao GPS, va canh bao
khi PX4 dang nap bo tham so khong khop (vd ra ngoai troi ma quen nap outdoor.params).
"""

from PySide6.QtCore import Property, QObject, QSettings, Signal, Slot

from ground_station_ui.core.vehicle.groups import ALT_MODES

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
        self.flight.set_mode(self._mode)
        self._state = {}
        self.flush(force=True)

    # --- tinh trang thai -------------------------------------------------------------------
    def _compute(self):
        fix = self.gps.values.get('fixType', 0) or 0
        effective = self._mode if self._mode != 'auto' else ('outdoor' if fix >= 3 else 'indoor')
        profile = self.params.values['profile']
        mismatch = ''
        if profile != 'unknown' and profile != effective:
            mismatch = (f"PX4 is using the {LABELS[profile].lower()} config but the app is in "
                        f"{LABELS[effective].lower()} mode — load config/px4/{effective}.params in QGC "
                        f"or change Flight environment in Settings")
        return {'mode': self._mode, 'effective': effective, 'effectiveLabel': LABELS[effective],
                'px4Profile': profile, 'px4ProfileLabel': LABELS[profile], 'mismatch': mismatch,
                'gpsCtrl': self.params.values['EKF2_GPS_CTRL'], 'hgtRef': self.params.values['EKF2_HGT_REF'],
                'gpsFix': fix}

    def flush(self, force=False):
        state = self._compute()
        if force or state != self._state:
            self._state = state
            self.changed.emit()

    # --- QML -------------------------------------------------------------------------------
    @Slot(str)
    def setMode(self, mode):
        if mode in ALT_MODES and mode != self._mode:
            self._mode = mode
            self._settings.setValue('env/mode', mode)
            self.flight.set_mode(mode)
            self.flush(force=True)

    state = Property('QVariantMap', lambda self: self._state, notify=changed)
