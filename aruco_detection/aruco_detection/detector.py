"""ArUco detection and marker pose estimation."""
from dataclasses import dataclass

import cv2
import cv2.aruco as aruco
import numpy as np


@dataclass
class Detection:
    marker_id: int
    corners: np.ndarray      # (4, 2) pixels, ordered TL, TR, BR, BL
    rvec: np.ndarray         # (3, 1)
    tvec: np.ndarray         # (3, 1), same unit as marker_size
    ambiguity: float         # closer to 1.0 means the rotation is less trustworthy


def ssr(gray, sigma=100.0):
    """Single-Scale Retinex for shadow suppression.

    Order matters: take the LOG first, blur in the log domain, then subtract.
    """
    img = gray.astype(np.float32) + 1.0
    cv2.log(img, img)
    ksize = int(round((sigma - 0.8) / 0.15 + 2.0)) | 1
    blur = cv2.GaussianBlur(img, (ksize, ksize), sigma, sigmaY=sigma,
                            borderType=cv2.BORDER_REPLICATE)
    out = cv2.normalize(img - blur, None, 0, 255, cv2.NORM_MINMAX)
    return out.astype(np.uint8)


def make_detector(dict_id):
    """Wrap the old and new aruco APIs (OpenCV 4.5 vs 4.7+)."""
    if hasattr(aruco, "ArucoDetector"):
        dictionary = aruco.getPredefinedDictionary(dict_id)
        return aruco.ArucoDetector(dictionary, aruco.DetectorParameters()).detectMarkers
    dictionary = aruco.Dictionary_get(dict_id)
    params = aruco.DetectorParameters_create()
    return lambda gray: aruco.detectMarkers(gray, dictionary, parameters=params)


def estimate_pose(corners, marker_size, camera_matrix, dist_coeffs):
    """solvePnP + IPPE_SQUARE, replacing estimatePoseSingleMarkers (gone in 4.7).

    obj_points must match the corner order detectMarkers returns: TL, TR, BR, BL.
    Corners 0 and 1 lie on the +Y edge, so marker +Y points towards the top of
    the printed pattern.

    Pass corners that are still distorted together with dist_coeffs and let
    solvePnP undistort them. If the image was undistorted beforehand, pass
    dist_coeffs = 0 instead.

    'ratio' is err(best solution) / err(second solution). IPPE_SQUARE always
    returns two solutions (the true one and its mirror through the marker
    plane); a ratio near 1.0 means they are indistinguishable, so the rotation
    cannot be trusted.
    """
    half = marker_size / 2.0
    obj_points = np.array([
        [-half,  half, 0],
        [ half,  half, 0],
        [ half, -half, 0],
        [-half, -half, 0],
    ], dtype=np.float32)

    rvecs, tvecs, ratios = [], [], []
    for c in corners:
        img_points = c.reshape(4, 2).astype(np.float32)
        n, rv, tv, errs = cv2.solvePnPGeneric(
            obj_points, img_points, camera_matrix, dist_coeffs,
            flags=cv2.SOLVEPNP_IPPE_SQUARE)
        if n == 0:
            rvecs.append(None)
            tvecs.append(None)
            ratios.append(None)
            continue
        e = [float(np.ravel(x)[0]) for x in errs]
        best = int(np.argmin(e))
        rvecs.append(rv[best])
        tvecs.append(tv[best])
        ratios.append(min(e) / max(e) if n > 1 and max(e) > 0 else 0.0)
    return rvecs, tvecs, ratios


class MarkerDetector:
    """Detection, pose estimation and optional smoothing in one place."""

    def __init__(self, dict_id, marker_size, camera_matrix, dist_coeffs,
                 use_ssr=False, lpf_alpha=1.0):
        self.detect = make_detector(dict_id)
        self.marker_size = marker_size
        self.camera_matrix = camera_matrix
        self.dist_coeffs = dist_coeffs
        self.use_ssr = use_ssr
        self.lpf_alpha = lpf_alpha      # 1.0 disables the filter entirely
        self._tvec_prev = {}

    def process(self, gray, target_id=None):
        """Return list[Detection], filtered by target_id when given."""
        corners, ids, _ = self.detect(ssr(gray) if self.use_ssr else gray)
        if ids is None or len(ids) == 0:
            return []

        rvecs, tvecs, ratios = estimate_pose(
            corners, self.marker_size, self.camera_matrix, self.dist_coeffs)

        out = []
        for i, marker_id in enumerate(ids.flatten()):
            marker_id = int(marker_id)
            if target_id is not None and marker_id != target_id:
                continue
            if rvecs[i] is None:
                continue

            tvec = tvecs[i]
            if self.lpf_alpha < 1.0 and marker_id in self._tvec_prev:
                a = self.lpf_alpha
                tvec = a * tvec + (1 - a) * self._tvec_prev[marker_id]
            self._tvec_prev[marker_id] = tvec

            out.append(Detection(marker_id=marker_id,
                                 corners=corners[i].reshape(4, 2),
                                 rvec=rvecs[i],
                                 tvec=tvec,
                                 ambiguity=ratios[i]))
        return out
