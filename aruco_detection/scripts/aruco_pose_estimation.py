#!/usr/bin/env python3
"""Standalone ArUco detection + pose estimation, with SSR, timing and logging.

A learning/debugging tool for the landing perception pipeline:
  - SSR (Single-Scale Retinex): shadow suppression, as in the reference paper
  - Perception delay and FPS: shows the cost of each stage
  - Ambiguity detection: shows when the pose rotation cannot be trusted
  - Optional CSV logging: replay the data after a run

Basic run:
    python3 aruco_pose_estimation.py --marker-size 26.7

Enable SSR to see the shadow suppression:
    python3 aruco_pose_estimation.py --marker-size 26.7 --ssr

Log to CSV for later review:
    python3 aruco_pose_estimation.py --marker-size 26.7 --log run1.csv

Hotkeys while running:
    q = quit
    s = toggle SSR (direct A/B comparison)
    d = toggle the SSR preview window
"""
import argparse
import csv
import json
import math
import os
import time

import cv2
import cv2.aruco as aruco
import numpy as np


DEFAULT_CALIB = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),"aruco_detection","calib_data_mono.json",)

# Switch to 6x6 1000 to match a generated marker.
# For a quick test with a printed 4x4 marker keep "DICT_4X4_50".
DEFAULT_DICT = "DICT_4X4_50"

DEFAULT_MARKER_SIZE = 26.7

# Error ratio between the two PnP solutions. Closer to 1.0 means the two are
# harder to tell apart, so the rotation flips easily. Above this we warn.
AMBIGUITY_WARN_RATIO = 0.7

# Pose low-pass coefficient. 1.0 = no filtering; smaller is smoother but lags.
# Only light smoothing here; heavy filtering belongs in the estimator (LKF).
LPF_ALPHA = 0.6


# ============================================================
#  CALIBRATION
# ============================================================
def load_calibration(path, side, override_size=None):
    """Read intrinsics plus the calibration resolution.

    The calibration resolution is MANDATORY: fx, fy, cx, cy all scale with
    image size, so without it we cannot check whether the matrix matches the
    resolution we are running at. Missing -> stop, rather than run on with an
    undefined scale.
    """
    with open(path) as f:
        data = json.load(f)
    if side not in data:
        raise SystemExit(f"Calibration file {path} has no '{side}' entry")
    camera_matrix = np.array(data[side]["matrix"], dtype=np.float64)
    dist_coeffs = np.array(data[side]["distortion"], dtype=np.float64).reshape(-1, 1)

    if override_size:
        return camera_matrix, dist_coeffs, override_size

    w = data.get("image_width") or data[side].get("image_width")
    h = data.get("image_height") or data[side].get("image_height")
    if not (w and h):
        cx, cy = camera_matrix[0, 2], camera_matrix[1, 2]
        raise SystemExit(
            f"\n!! Calibration file {path} has no image_width/image_height.\n"
            f"!! Its calibration resolution is unknown, so every distance would be\n"
            f"!! off by exactly that ratio while still looking plausible. Stopping.\n"
            f"!!\n"
            f"!! Goi y tu cx={cx:.1f} cy={cy:.1f} (diem chinh luon gan tam anh):\n"
            f"!!   calibration resolution is roughly {cx * 2:.0f}x{cy * 2:.0f}\n"
            f"!! This is ONLY a guess. Settle it one of two ways:\n"
            f"!!   1) Recalibrate and record the resolution (most reliable), or\n"
            f"!!   2) One ruler measurement: W_calib = (z_shown / z_measured) * W_running\n"
            f"!!\n"
            f"!! Once known, write it into the JSON:  \"image_width\": W, \"image_height\": H\n"
            f"!! or run temporarily with:  --calib-size WxH\n")
    return camera_matrix, dist_coeffs, (int(w), int(h))


def fit_camera_matrix(camera_matrix, calib_size, frame_size):
    """Scale intrinsics from the calibration resolution to the running one.

    fx, fy, cx, cy all scale with image size. Skipping this makes every
    distance wrong by exactly the resolution ratio.
    """
    cw, ch = calib_size
    fw, fh = frame_size
    if (cw, ch) == (fw, fh):
        return camera_matrix

    sx, sy = fw / cw, fh / ch
    print(f"!! Resolution mismatch: calibrated {cw}x{ch} vs running {fw}x{fh}")
    if abs(sx - sy) > 0.01:
        raise SystemExit(
            f"!! TI LE KHUNG HINH KHAC NHAU (sx={sx:.3f}, sy={sy:.3f}).\n"
            f"!! The camera is cropping rather than scaling -> intrinsics CANNOT be\n"
            f"!! scaled correctly. Recalibrate at the resolution you will run at.")
    print(f"!! Intrinsics scaled by {sx:.3f}")

    scaled = camera_matrix.copy()
    scaled[0, 0] *= sx
    scaled[0, 2] *= sx
    scaled[1, 1] *= sy
    scaled[1, 2] *= sy
    return scaled


# ============================================================
#  SSR - Single Scale Retinex (shadow suppression)
# ============================================================
def ssr(gray, sigma=100.0):
    """Single-Scale Retinex.

    Principle: image = reflectance x illumination. Illumination is the slowly
    varying part (shadows, brightness gradients). SSR separates reflectance
    (the marker information) from it.

    IMPORTANT, in this order:
      1) LOG FIRST:  img = log(I)
      2) BLUR in the log domain:  blur = G * log(I)
      3) SUBTRACT:  retinex = log(I) - G*log(I)
    sigma = 10 is the value used in the reference paper (Arducam 800x600). A
    very different resolution may need a small adjustment, but nothing like 100.
    """
    img = gray.astype(np.float32) + 1.0            # +1 avoids log(0)
    cv2.log(img, img)                              # 1) LOG FIRST (in-place)
    # ksize derived from sigma using the paper's formula (forced odd with |1)
    ksize = int(round((sigma - 0.8) / 0.15 + 2.0)) | 1
    blur = cv2.GaussianBlur(img, (ksize, ksize), sigma, sigmaY=sigma, borderType=cv2.BORDER_REPLICATE)  # 2) blur log-domain
    retinex = img - blur                           # 3) subtract in the log domain
    out = cv2.normalize(retinex, None, 0, 255, cv2.NORM_MINMAX)
    return out.astype(np.uint8)

# ============================================================
#  DETECTOR (old/new OpenCV compatible)
# ============================================================
def make_detector(dict_id):
    """Return a detect(gray) -> (corners, ids, rejected) callable."""
    if hasattr(aruco, "ArucoDetector"):
        dictionary = aruco.getPredefinedDictionary(dict_id)
        detector = aruco.ArucoDetector(dictionary, aruco.DetectorParameters())
        return detector.detectMarkers
    dictionary = aruco.Dictionary_get(dict_id)
    parameters = aruco.DetectorParameters_create()
    return lambda gray: aruco.detectMarkers(gray, dictionary, parameters=parameters)


# ============================================================
#  POSE ESTIMATION (solvePnP + IPPE_SQUARE)
# ============================================================
def estimate_pose(corners, marker_size, camera_matrix, dist_coeffs):
    """Pose estimation via solvePnP + SOLVEPNP_IPPE_SQUARE.

    Replaces estimatePoseSingleMarkers (deprecated since OpenCV 4.7).
    Returns (rvecs, tvecs, ratios); entries are None when the solve fails.
    'ratio' = reprojection error of the best solution over the second one. The
    closer to 1.0, the harder the two are to tell apart -> unreliable rotation.
    """
    half = marker_size / 2.0
    obj_points = np.array([
        [-half,  half, 0],   # corner 0: top-left
        [ half,  half, 0],   # corner 1: top-right
        [ half, -half, 0],   # corner 2: bottom-right
        [-half, -half, 0],   # corner 3: bottom-left
    ], dtype=np.float32)

    rvecs, tvecs, ratios = [], [], []
    for c in corners:
        img_points = c.reshape(4, 2).astype(np.float32)
        n, rv, tv, errs = cv2.solvePnPGeneric(obj_points, img_points, camera_matrix, dist_coeffs, flags=cv2.SOLVEPNP_IPPE_SQUARE)
        if n == 0:
            rvecs.append(None); tvecs.append(None); ratios.append(None)
            continue
        e = [float(np.ravel(x)[0]) for x in errs]
        best = int(np.argmin(e))
        rvecs.append(rv[best])
        tvecs.append(tv[best])
        ratios.append(min(e) / max(e) if n > 1 and max(e) > 0 else 0.0)
    return rvecs, tvecs, ratios


def rotation_to_euler(rmat):
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


# ============================================================
#  MAIN
# ============================================================
def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--marker-size", type=float, default=DEFAULT_MARKER_SIZE,
                        help=f"Outer black edge length of the marker [cm] "
                             f"(default {DEFAULT_MARKER_SIZE})")
    parser.add_argument("--cam", type=int, default=2, help="Camera index")
    parser.add_argument("--id", type=int, default=None,
                        help="Only show the marker with this id (default: all)")
    parser.add_argument("--calib", default=DEFAULT_CALIB, help="Path to the calibration JSON")
    parser.add_argument("--side", default="left", choices=["left", "right"],
                        help="Use the left or right camera intrinsics")
    parser.add_argument("--calib-size", default=None, metavar="WxH",
                        help="Calibration resolution (e.g. 640x400) when the JSON "
                             "does not record it. Temporary; prefer writing it into the JSON.")
    parser.add_argument("--dict", default=DEFAULT_DICT,
                        help=f"ArUco dictionary name (default {DEFAULT_DICT})")
    parser.add_argument("--ssr", action="store_true",
                        help="Enable SSR preprocessing (shadow suppression)")
    parser.add_argument("--lpf", action="store_true",
                        help="Enable the light pose low-pass filter")
    parser.add_argument("--log", default=None,
                        help="Write a CSV log (e.g. run1.csv)")
    args = parser.parse_args()

    override_size = None
    if args.calib_size:
        try:
            ow, oh = args.calib_size.lower().split("x")
            override_size = (int(ow), int(oh))
        except ValueError:
            raise SystemExit(f"--calib-size must be WxH, e.g. 640x400 "
                             f"(got: {args.calib_size})")

    camera_matrix, dist_coeffs, calib_size = load_calibration(
        args.calib, args.side, override_size)
    print(f"Calib: {args.calib} [{args.side}] @ {calib_size[0]}x{calib_size[1]}"
          f"{'  (from --calib-size)' if override_size else ''}")

    print(f"OpenCV {cv2.__version__}  |  dictionary {args.dict}  |  camera {args.side}")
    print(f"Marker size: {args.marker_size} cm", end="")
    if args.marker_size == DEFAULT_MARKER_SIZE:
        print("  (default - re-measure if distances look wrong)")
    else:
        print()
    print(f"SSR: {'on' if args.ssr else 'off'}  |  LPF: {'on' if args.lpf else 'off'}")
    print("Hotkeys: q=quit  s=toggle SSR  d=SSR preview")

    detect = make_detector(getattr(aruco, args.dict))

    cap = cv2.VideoCapture(args.cam)
    if not cap.isOpened():
        raise SystemExit(f"Cannot open camera index {args.cam}")

    # Force the camera to the calibration resolution. cap.set() fails silently
    # on many USB cameras, so read a real frame back to verify.
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, calib_size[0])
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, calib_size[1])

    ok, frame = cap.read()
    if not ok:
        raise SystemExit("Cannot read the first frame")
    frame_size = (frame.shape[1], frame.shape[0])
    print(f"Running resolution: {frame_size[0]}x{frame_size[1]}")
    if frame_size != calib_size:
        print(f"!! Camera did NOT accept {calib_size[0]}x{calib_size[1]}, "
              f"it returns {frame_size[0]}x{frame_size[1]}")
    camera_matrix = fit_camera_matrix(camera_matrix, calib_size, frame_size)

    # Setup logging
    log_file = None
    log_writer = None
    if args.log:
        log_file = open(args.log, "w", newline="")
        log_writer = csv.writer(log_file)
        log_writer.writerow(["t", "detected", "id", "dist_cm",
                             "x", "y", "z",                       # camera frame
                             "cam_x", "cam_y", "cam_alt",         # marker frame
                             "roll", "pitch", "yaw",
                             "ambiguity", "detect_ms", "total_ms", "fps"])

    # Runtime state
    use_ssr = args.ssr
    show_ssr = False
    tvec_prev = {}          # previous pose per id, for the LPF

    # Run statistics
    frame_total = 0
    frame_detected = 0
    t_start = time.time()
    fps_smooth = 0.0

    while True:
        t0 = time.perf_counter()
        ok, frame = cap.read()
        if not ok:
            continue
        frame_total += 1

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # ---- SSR preprocessing (timed separately) ----
        t_ssr0 = time.perf_counter()

        gray_proc = ssr(gray) if use_ssr else gray

        t_ssr = (time.perf_counter() - t_ssr0) * 1000
        # ---- Detection (timed separately) ----
        t_det0 = time.perf_counter()
        corners, ids, _ = detect(gray_proc)
        t_det = (time.perf_counter() - t_det0) * 1000

        detected_this_frame = ids is not None and len(ids) > 0
        if detected_this_frame:
            frame_detected += 1

        if detected_this_frame:
            rvecs, tvecs, ratios = estimate_pose(
                corners, args.marker_size, camera_matrix, dist_coeffs)
            aruco.drawDetectedMarkers(frame, corners, ids)
            y = 30
            for i, marker_id in enumerate(ids.flatten()):
                if args.id is not None and marker_id != args.id:
                    continue
                rvec, tvec = rvecs[i], tvecs[i]
                if rvec is None:
                    continue

                # Light pose smoothing
                if args.lpf and marker_id in tvec_prev:
                    tvec = LPF_ALPHA * tvec + (1 - LPF_ALPHA) * tvec_prev[marker_id]
                tvec_prev[marker_id] = tvec

                cv2.drawFrameAxes(frame, camera_matrix, dist_coeffs,rvec, tvec, args.marker_size * 0.5)

                x, yy, z = tvec.ravel()

                distance = float(np.linalg.norm(tvec))
                rmat, _ = cv2.Rodrigues(rvec)
                roll, pitch, yaw = rotation_to_euler(rmat)

                # CAMERA position in the MARKER frame (= -R^T * t).
                # Unlike x,y,z above, this is tied to the GROUND rather than the
                # camera, so the signs match intuition - moving the camera along
                # marker +X/+Y/+Z increases the value - and cam_alt stays put
                # when the camera merely tilts.
                # For inspection ONLY. Never feed it to a controller: it depends
                # on rvec, which is exactly what the ambiguity warning is about.
                cam_x, cam_y, cam_alt = (-rmat.T @ tvec.reshape(3, 1)).ravel()

                lines = [
                    f"id={marker_id}  dist={distance:.1f}cm",
                    f"x={x:.1f} y={yy:.1f} z={z:.1f} cm",
                    f"roll={roll:.1f} pitch={pitch:.1f} yaw={yaw:.1f} deg",
                ]
                for line in lines:
                    cv2.putText(frame, line, (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                    y += 25

                # Marker frame in yellow, to distinguish it from the camera frame (green)
                cv2.putText(frame,
                            f"[marker] camx={cam_x:.1f} camy={cam_y:.1f} alt={cam_alt:.1f} cm",
                            (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
                y += 25

                ratio = ratios[i]
                if ratio is not None and ratio > AMBIGUITY_WARN_RATIO:
                    cv2.putText(frame, f"! rotation unreliable ({ratio:.2f})",(10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 165, 255), 2)
                    y += 25
                y += 10

                if log_writer:
                    log_writer.writerow([
                        f"{time.time():.3f}", 1, int(marker_id), f"{distance:.2f}",
                        f"{x:.2f}", f"{yy:.2f}", f"{z:.2f}",
                        f"{cam_x:.2f}", f"{cam_y:.2f}", f"{cam_alt:.2f}",
                        f"{roll:.2f}", f"{pitch:.2f}", f"{yaw:.2f}",
                        f"{ratio:.3f}" if ratio is not None else "",
                        f"{t_det:.2f}", "", ""])
        else:
            cv2.putText(frame, "No marker", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
            if log_writer:
                log_writer.writerow([f"{time.time():.3f}", 0, "", "", "", "", "",
                                     "", "", "", "", "", "", "",
                                     f"{t_det:.2f}", "", ""])

        # ---- Timing statistics ----
        t_total = (time.perf_counter() - t0) * 1000
        fps_inst = 1000.0 / t_total if t_total > 0 else 0
        fps_smooth = 0.9 * fps_smooth + 0.1 * fps_inst if fps_smooth > 0 else fps_inst
        det_rate = 100.0 * frame_detected / frame_total

        # System info overlay (bottom-left)
        h = frame.shape[0]
        info = [f"FPS: {fps_smooth:.1f}  |  frame: {t_total:.1f}ms",
                f"SSR: {t_ssr:.1f}ms {'[ON]' if use_ssr else '[off]'}  detect: {t_det:.1f}ms",
                f"Detection rate: {det_rate:.1f}% ({frame_detected}/{frame_total})",]
        yy2 = h - 70
        for line in info:
            cv2.putText(frame, line, (10, yy2), cv2.FONT_HERSHEY_SIMPLEX,
                        0.55, (255, 255, 0), 2)
            yy2 += 22

        cv2.imshow("aruco_pose_estimation", frame)
        if show_ssr and use_ssr:
            cv2.imshow("SSR output", gray_proc)

        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break
        elif key == ord('s'):
            use_ssr = not use_ssr
            print(f"SSR: {'on' if use_ssr else 'off'}")
        elif key == ord('d'):
            show_ssr = not show_ssr
            if not show_ssr:
                cv2.destroyWindow("SSR output")

    cap.release()
    if log_file:
        log_file.close()
        print(f"Log written: {args.log}")

    # Print the summary
    elapsed = time.time() - t_start
    print("\n=== SUMMARY ===")
    print(f"Total frames: {frame_total}  |  detected: {frame_detected} "
          f"({100.0*frame_detected/max(frame_total,1):.1f}%)")
    print(f"Elapsed: {elapsed:.1f}s  |  Average FPS: {frame_total/max(elapsed,1):.1f}")

    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()