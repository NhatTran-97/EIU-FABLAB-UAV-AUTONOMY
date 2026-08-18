"""Doi he toa do va bieu dien goc quay.

He camera optical (OpenCV, cung la REP-145): x phai, y xuong, z toi truoc.
He body FRD cua PX4: x truoc, y phai, z xuong.
"""
import math

import numpy as np

# Camera nhin thang xuong, canh TREN cua khung hinh quay ve mui drone.
# Suy ra: X_cam = phai, Y_cam = ra sau, Z_cam = xuong.
# Doi mount camera thi PHAI sua ma tran nay, dung va dau rai rac trong code.
R_BODY_FROM_CAM_DOWN = np.array([[0.0, -1.0, 0.0],
                                 [1.0,  0.0, 0.0],
                                 [0.0,  0.0, 1.0]])


def rotmat_to_quat(R):
    """Tra ve (x, y, z, w) theo thu tu cua geometry_msgs/Quaternion."""
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
    """q = [w, x, y, z] (Hamilton) - dung thu tu cua px4_msgs/VehicleAttitude.

    Chu y: ROS dung [x, y, z, w], PX4 dung [w, x, y, z]. Rat de sai o day.
    """
    w, x, y, z = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
        [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
        [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
    ])


def rotation_to_euler(rmat):
    """ZYX -> (roll, pitch, yaw) do, quanh cac truc CUA CAMERA.

    'yaw' o day la xoay quanh truc quang, KHONG phai heading cua drone. Va khi
    marker quay mat thang vao camera thi roll luon quanh +-180, se nhay dau
    giua cac frame - dung loc hay PID truc tiep tren no.
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
    """Vi tri camera trong he marker: -R^T * t.

    Gan voi MAT DAT chu khong gan voi camera, nen dau thuan truc giac va thanh
    phan thu 3 (do cao) khong doi khi camera chi nghieng.

    CHI dung de xem va kiem tra bang mat. Khong dua vao dieu khien: no phu thuoc
    rvec, ma rvec chinh la thu bi canh bao ambiguity.
    """
    return (-rmat.T @ np.asarray(tvec).reshape(3, 1)).ravel()


def cam_to_body(p_cam, R_body_from_cam=R_BODY_FROM_CAM_DOWN):
    """Vi tri marker trong he body FRD: (forward, right, down)."""
    return R_body_from_cam @ np.asarray(p_cam).reshape(3)


def marker_yaw_in_ned(rmat_cam_from_marker, q_attitude,
                      R_body_from_cam=R_BODY_FROM_CAM_DOWN):
    """Huong marker quanh truc thang dung, trong he NED [rad].

    Day la thanh phan DUY NHAT cua rvec dang tin: ambiguity lat phan nghieng,
    con xoay trong mat phang marker gan nhu khong bi anh huong.
    """
    R_nm = quat_to_rotmat(q_attitude) @ R_body_from_cam @ rmat_cam_from_marker
    return math.atan2(R_nm[1, 0], R_nm[0, 0])
