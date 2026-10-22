"""Read a few PX4 parameters (PARAM_REQUEST_READ -> PARAM_VALUE): the EKF2 ones tell whether the indoor or
outdoor parameter set is loaded (config/px4/indoor.params / outdoor.params); the safety ones (RTL, geofence,
landing, speed) are checked by core/vehicle/param_checks.py and shown on the Telemetry page.

Re-read periodically, since another file may be loaded with QGC while the app is running.
"""

import struct
import time

import os

from ground_station_ui.core.facts import FactGroup
from ground_station_ui.core.mavlink import drone_dialect as mavlink
from ground_station_ui.core.vehicle import param_checks

PROFILE_PARAMS = ('EKF2_GPS_CTRL', 'EKF2_HGT_REF')
WATCHED = PROFILE_PARAMS + param_checks.NAMES
RETRY_S = 3.0          # no value yet -> ask again after this
REFRESH_S = 30.0       # value known -> re-read to catch changes


def decode_param(m):
    """PX4 sends INT32 parameters 'bytewise' in the float field; convert back to an integer."""
    if m.param_type in (mavlink.MAV_PARAM_TYPE_INT32, mavlink.MAV_PARAM_TYPE_UINT32,
                        mavlink.MAV_PARAM_TYPE_INT16, mavlink.MAV_PARAM_TYPE_UINT16,
                        mavlink.MAV_PARAM_TYPE_INT8, mavlink.MAV_PARAM_TYPE_UINT8):
        return struct.unpack('<i', struct.pack('<f', m.param_value))[0]
    return m.param_value


def px4_profile(gps_ctrl, hgt_ref):
    """'indoor' | 'outdoor' | 'unknown' from the two EKF2 parameters."""
    if gps_ctrl is None and hgt_ref is None:
        return 'unknown'
    if gps_ctrl == 0 or hgt_ref == 2:          # GPS off / height from the range sensor
        return 'indoor'
    if gps_ctrl and hgt_ref in (0, 1):          # GPS on, height from baro / GPS
        return 'outdoor'
    return 'unknown'


def resolve_environment(mode, profile, gps_fix):
    """Effective environment -> (effective, source).

    mode 'indoor' / 'outdoor' -> chosen by the user (source 'manual').
    mode 'auto' -> follows the loaded PX4 parameter set (source 'px4'); while the parameters are unknown
    (PX4 not connected / unexpected values) it follows the GPS fix (source 'gps').
    """
    if mode in ('indoor', 'outdoor'):
        return mode, 'manual'
    if profile in ('indoor', 'outdoor'):
        return profile, 'px4'
    return ('outdoor' if (gps_fix or 0) >= 3 else 'indoor'), 'gps'


class Px4ParamWatcher(FactGroup):
    """values: every WATCHED parameter (None = not read yet), profile, problems (safety check texts);
    items: one row per safety parameter (param_checks.check).

    context: callable returning (mission waypoints, home (lat, lon) or None, mission speed), set by the app so
    the checks can compare the geofence with the mission."""

    def __init__(self, vehicle, links, link_name='px4', params_dir=None):
        super().__init__(profile='unknown', problems=[], **{n: None for n in WATCHED})
        self.params_dir = params_dir
        self.context = lambda: ([], None, None)
        self._expected = {}             # profile -> {name: value} from the parameter file
        self.vehicle = vehicle
        self.links = links
        self.link_name = link_name
        self._asked = {n: 0.0 for n in WATCHED}
        self._got = {n: 0.0 for n in WATCHED}

    def attach(self, router):
        router.subscribe('PARAM_VALUE', self.on_param)

    def on_param(self, m, _link):
        if not self.vehicle.is_autopilot(m, _link):
            return
        name = m.param_id.rstrip('\x00') if isinstance(m.param_id, str) else m.param_id
        if name not in WATCHED:
            return
        self._got[name] = time.monotonic()
        self.set(**{name: decode_param(m)})
        self.set(profile=px4_profile(self.values['EKF2_GPS_CTRL'], self.values['EKF2_HGT_REF']))

    def _check(self):
        profile = self.values['profile']
        if profile not in self._expected:
            path = os.path.join(self.params_dir, f'{profile}.params') if self.params_dir else ''
            self._expected[profile] = param_checks.load_params_file(path) if profile != 'unknown' else {}
        mission, home, speed = self.context()
        rows, problems = param_checks.check(self.values, self._expected[profile], mission, home, speed)
        self.set_items(rows)
        self.set(problems=problems)

    def tick(self, now):
        self._check()
        if self.vehicle.sysid is None:
            return
        link = self.links.get(self.link_name)
        if link is None:
            return
        for name in WATCHED:
            wait = REFRESH_S if self._got[name] else RETRY_S
            if now - max(self._asked[name], self._got[name]) < wait:
                continue
            self._asked[name] = now
            link.send(mavlink.MAVLink_param_request_read_message(
                self.vehicle.sysid, self.vehicle.compid, name.encode(), -1))
