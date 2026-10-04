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
    assert g.items == [[0.0, 0.0, 1.0], [0.0, 1.0, 1.0]]
    router.dispatch('jetson', [pose(5.0, 5.0, -1.0, reset=1)]) # EKF dat lai -> xoa vet cu
    assert g.items == [[5.0, 5.0, 1.0]]
    g.tick(g._last + 3.0)
    assert not g.values['valid']                               # qua 2 s khong co pose
