"""Camera intrinsic loading and resolution fitting."""
import json

import numpy as np


def parse_size(text):
    """'640x480' -> (640, 480). Empty string -> None."""
    if not text:
        return None
    try:
        w, h = str(text).lower().split("x")
        return int(w), int(h)
    except ValueError:
        raise RuntimeError(f"Size must be WxH, e.g. 640x480 (got: {text})")


def load_calibration(path, side, override_size=None):
    """Return (camera_matrix, dist_coeffs, calib_size).

    The calibration resolution is MANDATORY. Without it we cannot tell whether
    the matrix matches the resolution we are running at, and every distance
    would be off by exactly that ratio while still looking perfectly plausible.
    """
    with open(path) as f:
        data = json.load(f)
    if side not in data:
        raise RuntimeError(f"Calibration file {path} has no '{side}' entry")

    camera_matrix = np.array(data[side]["matrix"], dtype=np.float64)
    dist_coeffs = np.array(data[side]["distortion"], dtype=np.float64).reshape(-1, 1)

    if override_size:
        return camera_matrix, dist_coeffs, override_size

    w = data.get("image_width") or data[side].get("image_width")
    h = data.get("image_height") or data[side].get("image_height")
    if not (w and h):
        cx, cy = camera_matrix[0, 2], camera_matrix[1, 2]
        raise RuntimeError(
            f"Calibration file {path} [{side}] has no image_width/image_height. "
            f"Guessing from cx={cx:.1f} cy={cy:.1f}: around {cx * 2:.0f}x{cy * 2:.0f}. "
            f"Write it into the JSON or set the calib_size parameter.")
    return camera_matrix, dist_coeffs, (int(w), int(h))


def fit_camera_matrix(camera_matrix, calib_size, frame_size):
    """Scale intrinsics from the calibration resolution to the running one."""
    cw, ch = calib_size
    fw, fh = frame_size
    if (cw, ch) == (fw, fh):
        return camera_matrix

    sx, sy = fw / cw, fh / ch
    if abs(sx - sy) > 0.01:
        raise RuntimeError(
            f"Aspect ratio mismatch (sx={sx:.3f} sy={sy:.3f}): the camera is "
            f"cropping rather than scaling, so the intrinsics cannot be scaled "
            f"correctly. Recalibrate at {fw}x{fh}.")

    scaled = camera_matrix.copy()
    scaled[0, 0] *= sx
    scaled[0, 2] *= sx
    scaled[1, 1] *= sy
    scaled[1, 2] *= sy
    return scaled
