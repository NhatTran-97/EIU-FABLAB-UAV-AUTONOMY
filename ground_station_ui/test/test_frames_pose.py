import math

from ground_station_ui.core.geometry.frames import (ned_to_enu, q_ned_frd_to_enu_flu, q_rotate,
                                                    yaw_enu_deg)
from ground_station_ui.core.mavlink import drone_dialect as d
from ground_station_ui.core.router import MessageRouter
from ground_station_ui.core.vehicle.vehicle import Vehicle

JETSON = d.MAVLink(None, srcSystem=1, srcComponent=191)


def close(a, b, tol=1e-6):
    return all(abs(x - y) < tol for x, y in zip(a, b))


def q_yaw_ned(deg):
    h = math.radians(deg) / 2
    return (math.cos(h), 0.0, 0.0, math.sin(h))


def test_position_ned_to_enu():
    assert ned_to_enu(1.0, 2.0, -3.0) == (2.0, 1.0, 3.0)      # Bac 1, Dong 2, cao 3


def test_attitude_level_facing_north():
    q = q_ned_frd_to_enu_flu(q_yaw_ned(0))
    assert close(q_rotate(q, (1, 0, 0)), (0, 1, 0))            # mui (FLU x) -> North (+Y ENU)
    assert close(q_rotate(q, (0, 0, 1)), (0, 0, 1))            # truc Up than drone -> Up
    assert close(q_rotate(q, (0, 1, 0)), (-1, 0, 0))           # trai -> West
    assert abs(yaw_enu_deg(q) - 90) < 1e-6


def test_attitude_facing_east_and_pitch_up():
    q = q_ned_frd_to_enu_flu(q_yaw_ned(90))                    # PX4 yaw 90 = huong Dong
    assert close(q_rotate(q, (1, 0, 0)), (1, 0, 0))
    h = math.radians(10) / 2                                   # chui mui len 10 do (pitch +)
    q = q_ned_frd_to_enu_flu((math.cos(h), 0.0, math.sin(h), 0.0))
    fwd = q_rotate(q, (1, 0, 0))
    assert fwd[2] > 0.17 and fwd[1] > 0.98                     # mui di len, van ve Bac


def pose(x, y, z, reset=0, yaw=0.0):
    m = JETSON.drone_local_pose_encode(0, x, y, z, list(q_yaw_ned(yaw)), 0.3, 0.4, 0.0, reset)
    return d.MAVLink(None).parse_buffer(m.pack(JETSON))[0]


def test_local_pose_group_trail_and_reset():
    router = MessageRouter()
    v = Vehicle(router)
    g = v.groups['localPose']
    router.dispatch('jetson', [pose(0, 0, -1.0)])
    assert g.values['valid'] and (g.values['e'], g.values['n'], g.values['u']) == (0.0, 0.0, 1.0)
    assert abs(g.values['speed'] - 0.5) < 1e-6
    router.dispatch('jetson', [pose(0.01, 0, -1.0)])           # < 5 cm: khong them diem
    router.dispatch('jetson', [pose(1.0, 0, -1.0)])            # di 1 m ve Bac
    assert g.trail.segments == [[(0.0, 0.0, 1.0), (0.0, 1.0, 1.0)]]
    router.dispatch('jetson', [pose(5.0, 5.0, -1.0, reset=1)]) # EKF dat lai -> xoa vet cu
    assert g.trail.segments == [[(5.0, 5.0, 1.0)]] and g.values['trailPoints'] == 1
    g.tick(g._seen['jetson'] + 3.0)
    assert not g.values['valid']                               # qua 2 s khong co pose


def test_pose_prefers_jetson_falls_back_to_px4_direct(monkeypatch):
    """3D: Jetson gui pose -> dung Jetson; Jetson im -> PX4 truc tiep (915 / SITL); Jetson lai -> Jetson."""
    import ground_station_ui.core.vehicle.groups as groups
    now = [100.0]
    monkeypatch.setattr(groups.time, 'monotonic', lambda: now[0])
    router = MessageRouter()
    v = Vehicle(router, {'autopilot_links': ['px4'], 'companion_link': 'jetson'})
    g = v.groups['localPose']
    PX4 = d.MAVLink(None, srcSystem=1, srcComponent=1)
    rt = lambda mav, m: d.MAVLink(None).parse_buffer(m.pack(mav))[0]
    hb = PX4.heartbeat_encode(d.MAV_TYPE_QUADROTOR, d.MAV_AUTOPILOT_PX4, 0, 0, d.MAV_STATE_STANDBY)
    router.dispatch('px4', [rt(PX4, hb)])
    lp = lambda n: rt(PX4, PX4.local_position_ned_encode(0, n, 0.0, -2.0, 0, 0, 0))
    att = rt(PX4, PX4.attitude_quaternion_encode(0, *q_yaw_ned(90), 0, 0, 0))
    router.dispatch('px4', [att, lp(1.0)])                    # chi co PX4 truc tiep
    assert g.values['source'] == 'px4' and g.values['valid'] and g.values['n'] == 1.0 and g.values['u'] == 2.0
    router.dispatch('jetson', [pose(5.0, 0, -1.0)])           # Jetson co pose -> uu tien
    router.dispatch('px4', [lp(9.0)])                         # PX4 van gui nhung bi bo qua
    assert g.values['source'] == 'jetson' and g.values['n'] == 5.0
    now[0] = 103.0                                            # Jetson im > 2 s
    router.dispatch('px4', [lp(9.0)])
    assert g.values['source'] == 'px4' and g.values['n'] == 9.0
    g.tick(103.1)
    assert g.values['valid']
    router.dispatch('jetson', [pose(6.0, 0, -1.0)])           # Jetson co lai
    assert g.values['source'] == 'jetson' and g.values['n'] == 6.0
