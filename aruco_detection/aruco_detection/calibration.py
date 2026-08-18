"""Doc va scale intrinsic camera."""
import json

import numpy as np


def parse_size(text):
    """'640x480' -> (640, 480). Chuoi rong -> None."""
    if not text:
        return None
    try:
        w, h = str(text).lower().split("x")
        return int(w), int(h)
    except ValueError:
        raise RuntimeError(f"Kich thuoc phai co dang WxH, vd 640x480 (nhan: {text})")


def load_calibration(path, side, override_size=None):
    """Tra ve (camera_matrix, dist_coeffs, calib_size).

    Do phan giai luc calib la BAT BUOC. Thieu no thi khong biet ma tran co dung
    thang do cho do phan giai dang chay hay khong, va moi khoang cach se sai
    theo dung ti le do ma van trong hoan toan hop ly.
    """
    with open(path) as f:
        data = json.load(f)
    if side not in data:
        raise RuntimeError(f"File calib {path} khong co entry '{side}'")

    camera_matrix = np.array(data[side]["matrix"], dtype=np.float64)
    dist_coeffs = np.array(data[side]["distortion"], dtype=np.float64).reshape(-1, 1)

    if override_size:
        return camera_matrix, dist_coeffs, override_size

    w = data.get("image_width") or data[side].get("image_width")
    h = data.get("image_height") or data[side].get("image_height")
    if not (w and h):
        cx, cy = camera_matrix[0, 2], camera_matrix[1, 2]
        raise RuntimeError(
            f"File calib {path} [{side}] khong ghi image_width/image_height. "
            f"Suy doan tu cx={cx:.1f} cy={cy:.1f}: khoang {cx * 2:.0f}x{cy * 2:.0f}. "
            f"Ghi vao JSON hoac dat tham so calib_size.")
    return camera_matrix, dist_coeffs, (int(w), int(h))


def fit_camera_matrix(camera_matrix, calib_size, frame_size):
    """Scale intrinsic tu do phan giai luc calib sang do phan giai luc chay."""
    cw, ch = calib_size
    fw, fh = frame_size
    if (cw, ch) == (fw, fh):
        return camera_matrix

    sx, sy = fw / cw, fh / ch
    if abs(sx - sy) > 0.01:
        raise RuntimeError(
            f"Ti le khung hinh khac nhau (sx={sx:.3f} sy={sy:.3f}): camera dang "
            f"crop chu khong thu nho nen khong the scale dung. Calib lai o {fw}x{fh}.")

    scaled = camera_matrix.copy()
    scaled[0, 0] *= sx
    scaled[0, 2] *= sx
    scaled[1, 1] *= sy
    scaled[1, 2] *= sy
    return scaled
