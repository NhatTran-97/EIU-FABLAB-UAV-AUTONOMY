"""EnvironmentAdapter: flight environment (Auto / Indoor / Outdoor) for the whole app.

Auto follows the loaded PX4 parameter set (EKF2_GPS_CTRL / EKF2_HGT_REF), or the GPS fix until the
parameters are read. It drives the displayed altitude source, the default take-off altitude, the GPS
warning colour, and a warning when the loaded PX4 parameters do not match (e.g. outdoors with
outdoor.params not loaded).
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

    # --- state computation ----------------------------------------------------------------
    def _compute(self):
        fix = self.gps.values.get('fixType', 0) or 0
        profile = self.params.values['profile']
        effective, source = resolve_environment(self._mode, profile, fix)
        mismatch = ''
        # Warn only when PX4 is configured for indoors (GPS off / MTF-02 range height) but flying outdoors.
        # A GPS + baro configuration used indoors is not dangerous, so it is not flagged.
        if profile == 'indoor' and effective == 'outdoor':
            mismatch = ("PX4 is using the indoor (optical-flow, GPS off) config but the app is in outdoor "
                        "mode — load config/px4/outdoor.params in QGC or change Flight environment in Settings")
        return {'mode': self._mode, 'effective': effective, 'effectiveLabel': LABELS[effective],
                'source': source, 'px4Profile': profile, 'px4ProfileLabel': LABELS[profile], 'mismatch': mismatch,
                'gpsCtrl': self.params.values['EKF2_GPS_CTRL'], 'hgtRef': self.params.values['EKF2_HGT_REF'],
                'gpsFix': fix}

    def flush(self, force=False):
        state = self._compute()
        # altitude source: from the environment when known; otherwise FlightGroup decides by GPS fix
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
