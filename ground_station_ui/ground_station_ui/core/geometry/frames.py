"""Doi frame PX4 (NED, than drone FRD) -> ROS / ve 3D (ENU, than drone FLU).

Giong px4_ros_com/frame_transforms: chi doi O MOT CHO khi nhan du lieu PX4; lidar, ArUco,
TF tu Jetson da theo quy uoc ROS (ENU / FLU) nen dung thang.

Quaternion theo thu tu (w, x, y, z).
"""

import math

_S = math.sqrt(0.5)
NED_ENU_Q = (0.0, _S, _S, 0.0)            # quaternion_from_euler(pi, 0, pi/2)
AIRCRAFT_BASELINK_Q = (0.0, 1.0, 0.0, 0.0)  # quaternion_from_euler(pi, 0, 0): FRD <-> FLU


def q_mul(a, b):
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return (aw * bw - ax * bx - ay * by - az * bz,
            aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw)


def q_rotate(q, v):
    """Xoay vector v bang quaternion q."""
    w, x, y, z = q
    p = q_mul(q_mul(q, (0.0, *v)), (w, -x, -y, -z))
    return p[1], p[2], p[3]


def ned_to_enu(x, y, z):
    return y, x, -z


def q_ned_frd_to_enu_flu(q):
    """Tu the PX4 (FRD -> NED) -> tu the ROS (FLU -> ENU)."""
    return q_mul(q_mul(NED_ENU_Q, q), AIRCRAFT_BASELINK_Q)


def yaw_enu_deg(q_enu):
    """Goc quay quanh truc Up (0 = huong East, 90 = North) -- tien kiem tra."""
    w, x, y, z = q_enu
    return math.degrees(math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z)))
