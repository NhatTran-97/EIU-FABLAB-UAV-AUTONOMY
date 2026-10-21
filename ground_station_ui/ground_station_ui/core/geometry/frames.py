"""PX4 frame conversions (NED, FRD body) to ROS / 3D view frames (ENU, FLU body).

Same as px4_ros_com/frame_transforms: convert once, when PX4 data arrives; lidar, ArUco and TF data
from the Jetson already follow ROS conventions (ENU / FLU) and are used as is.

Quaternions are ordered (w, x, y, z).
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
    """Rotate vector v by quaternion q."""
    w, x, y, z = q
    p = q_mul(q_mul(q, (0.0, *v)), (w, -x, -y, -z))
    return p[1], p[2], p[3]


def ned_to_enu(x, y, z):
    return y, x, -z


def q_ned_frd_to_enu_flu(q):
    """PX4 attitude (FRD -> NED) to ROS attitude (FLU -> ENU)."""
    return q_mul(q_mul(NED_ENU_Q, q), AIRCRAFT_BASELINK_Q)


def yaw_enu_deg(q_enu):
    """Rotation about the Up axis (0 = East, 90 = North), for checks."""
    w, x, y, z = q_enu
    return math.degrees(math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z)))
