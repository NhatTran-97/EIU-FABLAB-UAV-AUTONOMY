"""Test tang core (Python thuan, khong Qt, khong phan cung)."""

import socket
import threading
import time

from ground_station_ui.core.links.udp_link import UdpLink
from ground_station_ui.core.mavlink import drone_dialect as d
from ground_station_ui.core.router import MessageRouter
from ground_station_ui.core.vehicle import px4_modes
from ground_station_ui.core.vehicle.vehicle import Vehicle

PX4 = d.MAVLink(None, srcSystem=1, srcComponent=1)
JETSON = d.MAVLink(None, srcSystem=1, srcComponent=191)
OTHER = d.MAVLink(None, srcSystem=7, srcComponent=1)


def roundtrip(mav, msg):
    """Encode -> bytes -> decode: message giong het khi nhan tu link that."""
    return d.MAVLink(None).parse_buffer(msg.pack(mav))[0]


def px4_heartbeat(armed=False, custom_mode=3 << 16, mav=PX4):
    base = d.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED | (d.MAV_MODE_FLAG_SAFETY_ARMED if armed else 0)
    return roundtrip(mav, mav.heartbeat_encode(d.MAV_TYPE_QUADROTOR, d.MAV_AUTOPILOT_PX4,
                                               base, custom_mode, d.MAV_STATE_STANDBY))


def make_vehicle():
    router = MessageRouter()
    return router, Vehicle(router)


def test_px4_modes():
    assert px4_modes.decode(3 << 16) == 'Position'
    assert px4_modes.decode((4 << 16) | (3 << 24)) == 'Hold'
    assert px4_modes.decode((4 << 16) | (4 << 24)) == 'Mission'
    assert px4_modes.decode(6 << 16) == 'Offboard'


def test_router_isolates_failing_handler():
    router, seen = MessageRouter(), []
    router.subscribe('HEARTBEAT', lambda m, l: 1 / 0)
    router.subscribe('HEARTBEAT', lambda m, l: seen.append(l))
    router.subscribe('*', lambda m, l: seen.append('any'))
    router.dispatch('px4', [px4_heartbeat()])
    assert seen == ['px4', 'any']


def test_vehicle_status_from_heartbeat():
    router, v = make_vehicle()
    router.dispatch('px4', [px4_heartbeat(armed=True, custom_mode=(4 << 16) | (3 << 24))])
    v.tick(time.monotonic())
    s = v.groups['status'].values
    assert (s['mode'], s['armed'], s['linkAlive']) == ('Hold', True, True)
    assert (v.sysid, v.compid) == (1, 1)


def test_data_from_other_system_is_ignored():
    router, v = make_vehicle()
    router.dispatch('px4', [px4_heartbeat()])
    gps = OTHER.gps_raw_int_encode(0, 3, 0, 0, 0, 0, 0, 0, 0, 12)
    router.dispatch('px4', [roundtrip(OTHER, gps)])
    assert v.groups['gps'].values['fixType'] == 0


def test_battery_unknown_percent_and_current():
    router, v = make_vehicle()
    router.dispatch('px4', [px4_heartbeat()])
    # PX4 that gui battery_remaining = -100 khi khong uoc luong duoc
    msg = PX4.sys_status_encode(0, 0, 0, 0, 15940, 0, -100, 0, 0, 0, 0, 0, 0)
    router.dispatch('px4', [roundtrip(PX4, msg)])
    b = v.groups['battery'].values
    assert (b['percent'], b['voltage'], b['current']) == (-1, 15.94, 0.0)


def test_sensor_health_bits():
    router, v = make_vehicle()
    router.dispatch('px4', [px4_heartbeat()])
    present = d.MAV_SYS_STATUS_SENSOR_3D_GYRO | d.MAV_SYS_STATUS_SENSOR_3D_MAG
    health = d.MAV_SYS_STATUS_SENSOR_3D_GYRO
    msg = PX4.sys_status_encode(present, present, health, 0, 0, -1, -1, 0, 0, 0, 0, 0, 0)
    router.dispatch('px4', [roundtrip(PX4, msg)])
    status = {s['name']: s['status'] for s in v.groups['sensors'].items}
    assert status['Gyro'] == 'ok' and status['Compass'] == 'error' and status['RC'] == 'none'


def test_altitude_falls_back_to_local_position_without_gps():
    router, v = make_vehicle()
    router.dispatch('px4', [px4_heartbeat()])
    router.dispatch('px4', [roundtrip(PX4, PX4.local_position_ned_encode(0, 0, 0, -14.4, 0, 0, 0))])
    assert v.groups['flight'].values['altRel'] == 14.4


def test_attitude_in_degrees_and_heading_wraps():
    import math
    router, v = make_vehicle()
    router.dispatch('px4', [px4_heartbeat()])
    att = PX4.attitude_encode(0, math.radians(-12.5), math.radians(4.2), math.radians(-25.0), 0, 0, 0)
    router.dispatch('px4', [roundtrip(PX4, att)])
    a = v.groups['attitude'].values
    assert a['valid'] and a['roll'] == -12.5 and a['pitch'] == 4.2
    assert a['heading'] == 335.0                      # yaw -25 deg -> 335 (nhu la ban QGC)
    router.dispatch('px4', [roundtrip(OTHER, OTHER.attitude_encode(0, 1, 1, 1, 0, 0, 0))])
    assert v.groups['attitude'].values['roll'] == -12.5   # drone khac: bo qua


def test_altitude_source_follows_environment():
    router, v = make_vehicle()
    fl = v.groups['flight']
    router.dispatch('px4', [px4_heartbeat()])
    # trong nha: Home chot luc bat nguon lech -> relative_alt 2.3 m, EKF local 0.2 m (dung)
    glob = PX4.global_position_int_encode(0, 0, 0, 48000, 2300, 0, 0, 0, 0)
    loc = PX4.local_position_ned_encode(0, 0, 0, -0.2, 0, 0, 0)
    no_fix = PX4.gps_raw_int_encode(0, 0, 0, 0, 0, 0, 0, 0, 0, 0)
    router.dispatch('px4', [roundtrip(PX4, m) for m in (glob, loc, no_fix)])
    assert (fl.values['altRel'], fl.values['altSource']) == (0.2, 'local')     # auto, khong fix
    fix3 = PX4.gps_raw_int_encode(0, 3, 0, 0, 0, 0, 0, 0, 0, 9)
    router.dispatch('px4', [roundtrip(PX4, fix3)])
    assert (fl.values['altRel'], fl.values['altSource']) == (2.3, 'home')      # auto, co fix
    fl.set_mode('indoor')
    assert fl.values['altRel'] == 0.2
    fl.set_mode('outdoor')
    router.dispatch('px4', [roundtrip(PX4, no_fix)])
    assert fl.values['altRel'] == 2.3                                         # ep ngoai troi


def test_px4_param_profile_and_bytewise_int():
    import struct
    from ground_station_ui.core.vehicle.params import Px4ParamWatcher, px4_profile

    class Link:
        def __init__(self):
            self.sent = []

        def send(self, m, force_v1=False):
            self.sent.append(m)
            return True

    class Links:
        def __init__(self, link):
            self.link = link

        def get(self, name):
            return self.link if name == 'px4' else None

    router, v = make_vehicle()
    link = Link()
    w = Px4ParamWatcher(v, Links(link))
    w.attach(router)
    w.tick(100.0)
    assert link.sent == []                                  # chua thay PX4 -> chua hoi
    router.dispatch('px4', [px4_heartbeat()])
    w.tick(100.0)
    assert sorted(m.param_id for m in link.sent) == ['EKF2_GPS_CTRL', 'EKF2_HGT_REF']
    w.tick(101.0)
    assert len(link.sent) == 2                              # chua het RETRY_S

    def pv(name, value):                                    # PX4 gui INT32 kieu bytewise
        f = struct.unpack('<f', struct.pack('<i', value))[0]
        return roundtrip(PX4, PX4.param_value_encode(name.encode(), f, d.MAV_PARAM_TYPE_INT32, 1000, 0))

    router.dispatch('px4', [pv('EKF2_GPS_CTRL', 0), pv('EKF2_HGT_REF', 2)])
    assert (w.values['EKF2_GPS_CTRL'], w.values['EKF2_HGT_REF'], w.values['profile']) == (0, 2, 'indoor')
    router.dispatch('px4', [pv('EKF2_GPS_CTRL', 7), pv('EKF2_HGT_REF', 0)])
    assert w.values['profile'] == 'outdoor'
    assert px4_profile(None, None) == 'unknown'


def test_gps_unknown_sentinels_hidden():
    router, v = make_vehicle()
    router.dispatch('px4', [px4_heartbeat()])
    # khong fix: PX4 gui eph 9999, h_acc UINT32_MAX -> khong hien so rac
    nofix = PX4.gps_raw_int_encode(0, 0, 0, 0, 0, 9999, 9999, 0, 0, 0, 0, 0xFFFFFFFF, 3778957570, 0, 0, 0)
    router.dispatch('px4', [roundtrip(PX4, nofix)])
    g = v.groups['gps'].values
    assert (g['hdop'], g['hAcc'], g['vAcc']) == (None, None, None)
    fix = PX4.gps_raw_int_encode(0, 3, 0, 0, 0, 72, 110, 0, 0, 14, 0, 340, 680, 0, 0, 0)
    router.dispatch('px4', [roundtrip(PX4, fix)])
    g = v.groups['gps'].values
    assert (g['hdop'], g['hAcc'], g['vAcc']) == (0.72, 0.34, 0.68)


def test_statustext_before_first_heartbeat_is_kept():
    router, v = make_vehicle()
    router.dispatch('px4', [roundtrip(PX4, PX4.statustext_encode(4, b'Preflight Fail: Compass'))])
    items = v.groups['messages'].items
    assert items[0]['severity'] == 'WARNING' and items[0]['text'] == 'Preflight Fail: Compass'


def test_companion_components_and_stale():
    router, v = make_vehicle()
    hb = JETSON.heartbeat_encode(d.MAV_TYPE_ONBOARD_CONTROLLER, d.MAV_AUTOPILOT_INVALID, 0, 0, 4)
    st = JETSON.drone_component_status_encode(d.DRONE_COMPONENT_LIDAR, 0, 7.0, b'STREAMING')
    router.dispatch('jetson', [roundtrip(JETSON, hb), roundtrip(JETSON, st)])
    comp = v.groups['companion']
    now = time.monotonic()
    comp.tick(now)
    assert comp.values['linkAlive'] and comp.items[0]['state'] == 'STREAMING'
    comp.tick(now + 5.0)
    assert comp.items[0]['level'] == 'stale' and not comp.values['linkAlive']


def test_dirty_only_on_change():
    router, v = make_vehicle()
    router.dispatch('px4', [px4_heartbeat()])
    g = v.groups['status']
    g.dirty = False
    router.dispatch('px4', [px4_heartbeat()])     # cung trang thai
    assert not g.dirty


def test_udp_link_learns_peer_and_replies():
    """Gia lap PX4 SITL: gui heartbeat toi GCS, GCS tra heartbeat ve dung dia chi."""
    got = []
    done = threading.Event()

    def on_messages(link, msgs):
        got.extend(msgs)
        done.set()

    link = UdpLink('sitl', {'listen': '127.0.0.1:0'}, on_messages, lambda l: None)
    sock_holder = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock_holder.bind(('127.0.0.1', 0))
    port = sock_holder.getsockname()[1]
    sock_holder.close()
    link.listen = ('127.0.0.1', port)
    link.start()
    try:
        sitl = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sitl.settimeout(3.0)
        deadline = time.monotonic() + 3.0
        while not done.is_set() and time.monotonic() < deadline:
            sitl.sendto(px4_heartbeat().pack(PX4), ('127.0.0.1', port))
            done.wait(0.2)
        assert got and got[0].get_type() == 'HEARTBEAT'
        data, _ = sitl.recvfrom(1024)              # heartbeat GCS gui lai (1 Hz)
        reply = d.MAVLink(None).parse_buffer(data)[0]
        assert reply.get_type() == 'HEARTBEAT' and reply.type == d.MAV_TYPE_GCS
        sitl.close()
    finally:
        link.stop()
