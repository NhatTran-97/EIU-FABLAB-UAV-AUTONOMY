"""Coordinate frame conversions and rotation representations.

Camera optical frame (OpenCV, same as REP-145): x right, y down, z forward.
PX4 body FRD frame: x forward, y right, z down.
"""
import math

import numpy as np

# Camera looking straight down, top edge of the image facing the drone nose.
# Therefore: X_cam = right, Y_cam = backward, Z_cam = down.
# A different mount MUST be fixed here, never by patching signs across the code.
R_BODY_FROM_CAM_DOWN = np.array([[0.0, -1.0, 0.0],
                                 [1.0,  0.0, 0.0],
                                 [0.0,  0.0, 1.0]])


def rotmat_to_quat(R):
    """Return (x, y, z, w), the order used by geometry_msgs/Quaternion."""
    tr = R[0, 0] + R[1, 1] + R[2, 2]
    if tr > 0:
        s = math.sqrt(tr + 1.0) * 2
        w, x = 0.25 * s, (R[2, 1] - R[1, 2]) / s
        y, z = (R[0, 2] - R[2, 0]) / s, (R[1, 0] - R[0, 1]) / s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2
        w, x = (R[2, 1] - R[1, 2]) / s, 0.25 * s
        y, z = (R[0, 1] + R[1, 0]) / s, (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2
        w, x = (R[0, 2] - R[2, 0]) / s, (R[0, 1] + R[1, 0]) / s
        y, z = 0.25 * s, (R[1, 2] + R[2, 1]) / s
    else:
        s = math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2
        w, x = (R[1, 0] - R[0, 1]) / s, (R[0, 2] + R[2, 0]) / s
        y, z = (R[1, 2] + R[2, 1]) / s, 0.25 * s
    return x, y, z, w


def quat_to_rotmat(q):
    """q = [w, x, y, z] (Hamilton), the order px4_msgs/VehicleAttitude uses.

    Careful: ROS uses [x, y, z, w] but PX4 uses [w, x, y, z]. Easy to get wrong.
    """
    w, x, y, z = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
        [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
        [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
    ])


def rotation_to_euler(rmat):
    """ZYX -> (roll, pitch, yaw) in degrees, about the CAMERA axes.

    'yaw' here is rotation about the optical axis, NOT the drone heading. And
    when the marker faces the camera head-on, roll sits near +-180 and will flip
    sign between frames -- never filter or run a PID directly on it.
    """
    sy = math.sqrt(rmat[0, 0] ** 2 + rmat[1, 0] ** 2)
    if sy > 1e-6:
        roll = math.atan2(rmat[2, 1], rmat[2, 2])
        pitch = math.atan2(-rmat[2, 0], sy)
        yaw = math.atan2(rmat[1, 0], rmat[0, 0])
    else:
        roll = math.atan2(-rmat[1, 2], rmat[1, 1])
        pitch = math.atan2(-rmat[2, 0], sy)
        yaw = 0.0
    return math.degrees(roll), math.degrees(pitch), math.degrees(yaw)


def camera_in_marker(rmat, tvec):
    """Camera position expressed in the marker frame: -R^T * t.

    Tied to the GROUND rather than to the camera, so the signs match intuition
    and the third component (height) stays constant when the camera only tilts.

    For inspection and visual checks ONLY. Do not feed it to a controller: it
    depends on rvec, which is exactly what the ambiguity warning is about.
    """
    return (-rmat.T @ np.asarray(tvec).reshape(3, 1)).ravel()


def cam_to_body(p_cam, R_body_from_cam=R_BODY_FROM_CAM_DOWN):
    """Marker position in the body FRD frame: (forward, right, down)."""
    return R_body_from_cam @ np.asarray(p_cam).reshape(3)


def marker_yaw_in_ned(rmat_cam_from_marker, q_attitude,
                      R_body_from_cam=R_BODY_FROM_CAM_DOWN):
    """Marker heading about the vertical axis, in the NED frame [rad].

    This is the ONLY trustworthy component of rvec: the ambiguity flips the tilt
    but leaves in-plane rotation essentially untouched.
    """
    R_nm = quat_to_rotmat(q_attitude) @ R_body_from_cam @ rmat_cam_from_marker
    return math.atan2(R_nm[1, 0], R_nm[0, 0])
