"""Phat hien ArUco va uoc luong pose marker."""
from dataclasses import dataclass

import cv2
import cv2.aruco as aruco
import numpy as np


@dataclass
class Detection:
    marker_id: int
    corners: np.ndarray      # (4, 2) pixel, thu tu TL, TR, BR, BL
    rvec: np.ndarray         # (3, 1)
    tvec: np.ndarray         # (3, 1), cung don vi voi marker_size
    ambiguity: float         # cang gan 1.0 thi goc xoay cang kho tin


def ssr(gray, sigma=100.0):
    """Single-Scale Retinex chong bong do.

    Thu tu bat buoc: LOG truoc, blur trong log-domain, roi moi tru.
    """
    img = gray.astype(np.float32) + 1.0
    cv2.log(img, img)
    ksize = int(round((sigma - 0.8) / 0.15 + 2.0)) | 1
    blur = cv2.GaussianBlur(img, (ksize, ksize), sigma, sigmaY=sigma,
                            borderType=cv2.BORDER_REPLICATE)
    out = cv2.normalize(img - blur, None, 0, 255, cv2.NORM_MINMAX)
    return out.astype(np.uint8)


def make_detector(dict_id):
    """Bao API aruco cu/moi (OpenCV 4.5 vs 4.7+)."""
    if hasattr(aruco, "ArucoDetector"):
        dictionary = aruco.getPredefinedDictionary(dict_id)
        return aruco.ArucoDetector(dictionary, aruco.DetectorParameters()).detectMarkers
    dictionary = aruco.Dictionary_get(dict_id)
    params = aruco.DetectorParameters_create()
    return lambda gray: aruco.detectMarkers(gray, dictionary, parameters=params)


def estimate_pose(corners, marker_size, camera_matrix, dist_coeffs):
    """solvePnP + IPPE_SQUARE, thay estimatePoseSingleMarkers (deprecated tu 4.7).

    Thu tu obj_points phai khop thu tu goc ma detectMarkers tra ve: TL, TR, BR, BL.
    Goc 0 va 1 nam o canh +Y, nen +Y cua marker chi len phia tren cua hinh in.

    Truyen corner CHUA undistort kem dist_coeffs de solvePnP tu khu meo. Neu da
    undistort anh truoc thi phai truyen dist_coeffs = 0.

    'ratio' = err nghiem tot / err nghiem thu hai. IPPE_SQUARE luon tra 2 nghiem
    (that va lat guong qua mat phang marker); ratio gan 1.0 nghia la hai nghiem
    khong phan biet duoc -> goc xoay khong dang tin.
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
    """Gom detect + pose + loc lam muot vao mot cho."""

    def __init__(self, dict_id, marker_size, camera_matrix, dist_coeffs,
                 use_ssr=False, lpf_alpha=1.0):
        self.detect = make_detector(dict_id)
        self.marker_size = marker_size
        self.camera_matrix = camera_matrix
        self.dist_coeffs = dist_coeffs
        self.use_ssr = use_ssr
        self.lpf_alpha = lpf_alpha
        self._tvec_prev = {}

    def process(self, gray, target_id=None):
        """Tra ve list[Detection], loc theo target_id neu co."""
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
