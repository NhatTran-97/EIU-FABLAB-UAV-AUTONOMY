"""Rotations and coordinate frames for the landing pipeline.

Doc theo ba tang, tang duoi khong biet gi ve tang tren:

    1. Bieu dien phep quay   -- khong dinh he toa do nao
    2. He quang hoc camera   -- OpenCV: x phai, y xuong, z theo truc quang
    3. He bay MAVROS         -- ENU/FLU theo REP-103

Tang 3 la cho duy nhat cham vao quy uoc cua drone. Neu doi sang px4_msgs
(NED/FRD, quaternion [w,x,y,z]) thi chi viet lai tang 3 -- va viet ra file
KHAC, dung sua o day, vi hai quy uoc lech nhau 180 do quanh truc x cong voi
thu tu phan tu quaternion khac nhau. Tron lan thi ket qua trong van hop ly
nhung sai dau truc y va z.
"""
import math

import numpy as np

# ══════════════════════════════════════════════════════════════════════
# 1. Bieu dien phep quay
# ══════════════════════════════════════════════════════════════════════


def rotmat_to_quat(R):
    """3x3 -> (x, y, z, w), dung thu tu cua geometry_msgs/Quaternion."""
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


def quat_to_rotmat(x, y, z, w):
    """(x, y, z, w) -> 3x3. Thu tu geometry_msgs, KHONG phai px4_msgs."""
    n = math.sqrt(x * x + y * y + z * z + w * w)
    if n < 1e-9:
        return np.eye(3)
    x, y, z, w = x / n, y / n, z / n, w / n
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
        [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
        [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
    ])


def euler_to_rotmat(roll, pitch, yaw):
    """Z-Y-X noi tai, don vi DO."""
    r, p, y_ = math.radians(roll), math.radians(pitch), math.radians(yaw)
    cr, sr, cp, sp, cy, sy = (math.cos(r), math.sin(r), math.cos(p),
                              math.sin(p), math.cos(y_), math.sin(y_))
    return np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp,     cp * sr,                cp * cr],
    ])


def rotation_to_euler(rmat):
    """Z-Y-X -> (roll, pitch, yaw) DO, quanh truc CAMERA.

    'yaw' o day la quay quanh truc quang hoc, KHONG phai huong mui drone. Va
    khi marker doi dien camera, roll nam quanh +-180 va se doi dau giua cac
    frame -- dung bao gio loc hay dua thang vao PID.
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


# ══════════════════════════════════════════════════════════════════════
# 2. He quang hoc camera (OpenCV / REP-145: x phai, y xuong, z ra truoc)
# ══════════════════════════════════════════════════════════════════════


def camera_in_marker(rmat, tvec):
    """Vi tri camera trong he marker: -R^T * t.

    Gan voi MAT DAT thay vi voi camera, nen dau khop truc giac va thanh phan
    thu ba (do cao) giu nguyen khi camera chi nghieng.

    CHI de xem va kiem tra bang mat. Dung dua vao bo dieu khien: no phu thuoc
    rvec, dung cai ma canh bao ambiguity dang noi toi.
    """
    return (-rmat.T @ np.asarray(tvec).reshape(3, 1)).ravel()


# ══════════════════════════════════════════════════════════════════════
# 3. He bay MAVROS -- ENU/FLU (REP-103)
#
#       the gioi ENU : x Dong,  y Bac,  z Len
#       than    FLU  : x Truoc, y Trai, z Len
#       quaternion   : [x, y, z, w]
# ══════════════════════════════════════════════════════════════════════

# Camera chuc thang xuong, canh tren cua anh huong ve mui drone.
# Moi cot la mot truc camera bieu dien trong he than FLU:
#   X_cam (phai trong anh) -> -Y_flu (ben phai)
#   Y_cam (xuong trong anh) -> -X_flu (ve phia duoi)
#   Z_cam (truc quang hoc)  -> -Z_flu (huong xuong)
R_FLU_FROM_CAM_DOWN = np.array([[0.0, -1.0,  0.0],
                                [-1.0, 0.0,  0.0],
                                [0.0,  0.0, -1.0]])


class CameraExtrinsic:
    """Camera nam o dau tren khung may bay, trong he than FLU."""

    def __init__(self, mount_roll=0.0, mount_pitch=0.0, mount_yaw=0.0,
                 offset=(0.0, 0.0, 0.0)):
        # Sai so goc lap ap o truc THAN, chong len phep quay chuc-xuong danh
        # dinh. Lech 2 do o 5 m la 17 cm sai lech ngang, va vi la sai so he
        # thong nen khong bo loc nao phia sau khu duoc.
        self.R_flu_from_cam = (euler_to_rotmat(mount_roll, mount_pitch, mount_yaw)
                               @ R_FLU_FROM_CAM_DOWN)
        self.offset = np.asarray(offset, dtype=float).reshape(3)

    def to_body(self, p_cam):
        """Vi tri marker trong he than FLU, da cong tay don camera."""
        return self.R_flu_from_cam @ np.asarray(p_cam, dtype=float).reshape(3) + self.offset


def marker_yaw_enu(R_cam_from_marker, q_xyzw, extrinsic):
    """Huong cua MARKER quanh truc dung, trong ENU [rad].

    Day la thanh phan DUY NHAT cua rvec dang tin: ambiguity lat phan nghieng
    nhung gan nhu khong dong den goc quay trong mat phang marker.

    Chu y phan biet voi huong cua DRONE -- drone yaw lay tu mot minh
    q_xyzw, con ham nay phai di qua ca chuoi camera -> than -> ENU.
    """
    R = quat_to_rotmat(*q_xyzw) @ extrinsic.R_flu_from_cam @ R_cam_from_marker
    return math.atan2(R[1, 0], R[0, 0])


def marker_relative_enu(p_cam, q_xyzw, extrinsic):
    """Vi tri marker so voi drone, trong ENU, da khu nghieng.

    Day moi la gia tri bo dieu khien can dung. Dua thang tvec vao thay vi gia
    tri nay se ghep attitude vao vi tri: o 3 m, nghieng 10 do lam marker dich
    ~0.52 m trong anh du no khong he di chuyen, va bo dieu khien se nghieng
    them de duoi theo -- cang duoi cang lech.

    Tra ve (p_enu, R_enu_from_flu). p_enu[2] AM khi marker o duoi drone.
    """
    R_enu_from_flu = quat_to_rotmat(*q_xyzw)
    return R_enu_from_flu @ extrinsic.to_body(p_cam), R_enu_from_flu
