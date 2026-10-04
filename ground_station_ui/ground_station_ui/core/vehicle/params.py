"""Doc mot so tham so PX4 (PARAM_REQUEST_READ -> PARAM_VALUE) de app biet PX4 dang nap
bo tham so trong nha hay ngoai troi (config/px4/indoor.params / outdoor.params).

Hoi lai dinh ky vi nguoi dung co the nap file khac bang QGC roi mo lai app.
"""

import struct
import time

from ground_station_ui.core.facts import FactGroup
from ground_station_ui.core.mavlink import drone_dialect as mavlink

WATCHED = ('EKF2_GPS_CTRL', 'EKF2_HGT_REF')
RETRY_S = 3.0          # chua co gia tri -> hoi lai sau
REFRESH_S = 30.0       # da co -> hoi lai de bat thay doi


def decode_param(m):
    """PX4 gui tham so INT32 kieu 'bytewise' trong truong float -> doi lai so nguyen."""
    if m.param_type in (mavlink.MAV_PARAM_TYPE_INT32, mavlink.MAV_PARAM_TYPE_UINT32,
                        mavlink.MAV_PARAM_TYPE_INT16, mavlink.MAV_PARAM_TYPE_UINT16,
                        mavlink.MAV_PARAM_TYPE_INT8, mavlink.MAV_PARAM_TYPE_UINT8):
        return struct.unpack('<i', struct.pack('<f', m.param_value))[0]
    return m.param_value


def px4_profile(gps_ctrl, hgt_ref):
    """'indoor' | 'outdoor' | 'unknown' tu 2 tham so EKF2."""
    if gps_ctrl is None and hgt_ref is None:
        return 'unknown'
    if gps_ctrl == 0 or hgt_ref == 2:          # GPS tat / do cao tu range sensor
        return 'indoor'
    if gps_ctrl and hgt_ref in (0, 1):          # GPS bat, do cao baro / GPS
        return 'outdoor'
    return 'unknown'


class Px4ParamWatcher(FactGroup):
    """values: EKF2_GPS_CTRL, EKF2_HGT_REF (None = chua biet), profile."""

    def __init__(self, vehicle, links, link_name='px4'):
        super().__init__(profile='unknown', **{n: None for n in WATCHED})
        self.vehicle = vehicle
        self.links = links
        self.link_name = link_name
        self._asked = {n: 0.0 for n in WATCHED}
        self._got = {n: 0.0 for n in WATCHED}

    def attach(self, router):
        router.subscribe('PARAM_VALUE', self.on_param)

    def on_param(self, m, _link):
        if not self.vehicle.is_autopilot(m):
            return
        name = m.param_id.rstrip('\x00') if isinstance(m.param_id, str) else m.param_id
        if name not in WATCHED:
            return
        self._got[name] = time.monotonic()
        self.set(**{name: decode_param(m)})
        self.set(profile=px4_profile(self.values['EKF2_GPS_CTRL'], self.values['EKF2_HGT_REF']))

    def tick(self, now):
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
