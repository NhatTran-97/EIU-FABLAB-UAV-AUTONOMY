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
    assert ned_to_enu(1.0, 2.0, -3.0) == (2.0, 1.0, 3.0)      # North 1, East 2, Up 3


def test_attitude_level_facing_north():
    q = q_ned_frd_to_enu_flu(q_yaw_ned(0))
    assert close(q_rotate(q, (1, 0, 0)), (0, 1, 0))            # nose (FLU x) -> North (+Y ENU)
    assert close(q_rotate(q, (0, 0, 1)), (0, 0, 1))            # body Up axis -> Up
    assert close(q_rotate(q, (0, 1, 0)), (-1, 0, 0))           # left -> West
    assert abs(yaw_enu_deg(q) - 90) < 1e-6


def test_attitude_facing_east_and_pitch_up():
    q = q_ned_frd_to_enu_flu(q_yaw_ned(90))                    # PX4 yaw 90 = East
    assert close(q_rotate(q, (1, 0, 0)), (1, 0, 0))
    h = math.radians(10) / 2                                   # nose up 10 deg (pitch +)
    q = q_ned_frd_to_enu_flu((math.cos(h), 0.0, math.sin(h), 0.0))
    fwd = q_rotate(q, (1, 0, 0))
    assert fwd[2] > 0.17 and fwd[1] > 0.98                     # nose points up, still towards North


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
    router.dispatch('jetson', [pose(0.01, 0, -1.0)])           # < 5 cm: no new point
    router.dispatch('jetson', [pose(1.0, 0, -1.0)])            # 1 m north
    assert g.trail.segments == [[(0.0, 0.0, 1.0), (0.0, 1.0, 1.0)]]
    router.dispatch('jetson', [pose(5.0, 5.0, -1.0, reset=1)]) # EKF reset -> old trail cleared
    assert g.trail.segments == [[(5.0, 5.0, 1.0)]] and g.values['trailPoints'] == 1
    g.tick(g._seen['jetson'] + 3.0)
    assert not g.values['valid']                               # no pose for over 2 s


def test_pose_prefers_jetson_falls_back_to_px4_direct(monkeypatch):
    """3D: Jetson pose used when available; Jetson silent -> direct PX4 (915 / SITL); Jetson back -> Jetson."""
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
    router.dispatch('px4', [att, lp(1.0)])                    # direct PX4 only
    assert g.values['source'] == 'px4' and g.values['valid'] and g.values['n'] == 1.0 and g.values['u'] == 2.0
    router.dispatch('jetson', [pose(5.0, 0, -1.0)])           # Jetson pose -> preferred
    router.dispatch('px4', [lp(9.0)])                         # PX4 still sending but ignored
    assert g.values['source'] == 'jetson' and g.values['n'] == 5.0
    now[0] = 103.0                                            # Jetson silent > 2 s
    router.dispatch('px4', [lp(9.0)])
    assert g.values['source'] == 'px4' and g.values['n'] == 9.0
    g.tick(103.1)
    assert g.values['valid']
    router.dispatch('jetson', [pose(6.0, 0, -1.0)])           # Jetson back
    assert g.values['source'] == 'jetson' and g.values['n'] == 6.0


def test_extent_grows_with_the_flight_area_and_resets_with_the_trail():
    import ground_station_ui.core.vehicle.groups as groups
    from ground_station_ui.core.vehicle.vehicle import Vehicle
    from ground_station_ui.core.router import MessageRouter
    g = groups.LocalPoseGroup(Vehicle(MessageRouter()))
    for x, y in [(3.2, 0.0), (-12.4, 5.0), (2.0, 30.6), (0.0, 0.0)]:
        g._apply('px4', x, y, -2.0, (1.0, 0.0, 0.0, 0.0), 0.0, 0.0, 0.0)
    assert g.values['extentM'] == 31.0                 # largest |north| / |east| reached, rounded up
    g.clear_trail()
    assert g.values['extentM'] == 0.0
