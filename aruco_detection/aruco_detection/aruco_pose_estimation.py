#!/usr/bin/env python3
"""Detect ArUco marker + uoc luong pose, kem SSR, do delay, logging.

Ban nay danh cho muc dich HOC + HIEU pipeline computer vision cho landing:
  - SSR (Single-Scale Retinex): chong bong do, giong tien xu ly cua bai bao
  - Do perception delay va FPS: hieu chi phi tinh toan tung khoi
  - Ambiguity detection: hieu khi nao pose xoay khong tin duoc
  - Logging CSV (tuy chon): xem lai du lieu sau khi chay

Chay co ban:
    python3 aruco_pose_estimation_full.py --marker-size 26.7

Bat SSR de thay tac dung chong bong:
    python3 aruco_pose_estimation_full.py --marker-size 26.7 --ssr

Ghi log ra CSV de xem lai:
    python3 aruco_pose_estimation_full.py --marker-size 26.7 --log run1.csv

Phim tat khi dang chay:
    q = thoat
    s = bat/tat SSR (so sanh truc tiep co/khong SSR)
    d = bat/tat hien thi anh SSR o cua so phu
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

# Doi sang 6x6 1000 de khop voi marker ban da generate.
# De test nhanh voi marker 4x4 in san thi doi lai "DICT_4X4_50".
DEFAULT_DICT = "DICT_4X4_50"

DEFAULT_MARKER_SIZE = 26.7

# Ti le sai so giua 2 nghiem PnP. Cang gan 1.0 = 2 nghiem cang giong nhau
# -> goc xoay de bi lat. Tren nguong nay thi canh bao tren man hinh.
AMBIGUITY_WARN_RATIO = 0.7

# He so lam muot pose (low-pass). 1.0 = khong loc, cang nho cang muot nhung tre.
# Chi loc rat nhe o day; loc manh de cho estimation (LKF) lo sau nay.
LPF_ALPHA = 0.6


# ============================================================
#  CALIBRATION
# ============================================================
def load_calibration(path, side, override_size=None):
    """Doc intrinsic + do phan giai luc calib.

    Do phan giai luc calib la thong tin BAT BUOC: fx, fy, cx, cy deu ti le
    thuan voi kich thuoc anh, nen neu khong biet no thi khong the kiem tra
    ma tran co dung cho do phan giai dang chay hay khong. Thieu -> dung han,
    khong chay tiep voi thang do khong xac dinh.
    """
    with open(path) as f:
        data = json.load(f)
    if side not in data:
        raise SystemExit(f"File calib {path} khong co entry '{side}'")
    camera_matrix = np.array(data[side]["matrix"], dtype=np.float64)
    dist_coeffs = np.array(data[side]["distortion"], dtype=np.float64).reshape(-1, 1)

    if override_size:
        return camera_matrix, dist_coeffs, override_size

    w = data.get("image_width") or data[side].get("image_width")
    h = data.get("image_height") or data[side].get("image_height")
    if not (w and h):
        cx, cy = camera_matrix[0, 2], camera_matrix[1, 2]
        raise SystemExit(
            f"\n!! File calib {path} khong ghi image_width/image_height.\n"
            f"!! Khong biet no duoc calib o do phan giai nao -> moi khoang cach\n"
            f"!! se sai theo dung ti le do, ma van trong hop ly. Dung tai day.\n"
            f"!!\n"
            f"!! Goi y tu cx={cx:.1f} cy={cy:.1f} (diem chinh luon gan tam anh):\n"
            f"!!   do phan giai luc calib vao khoang {cx * 2:.0f}x{cy * 2:.0f}\n"
            f"!! Day CHI la suy doan. Xac dinh dut diem bang 1 trong 2 cach:\n"
            f"!!   1) Calib lai va ghi kem do phan giai (chac chan nhat), hoac\n"
            f"!!   2) Do thuoc 1 lan:  W_calib = (z_hien_thi / z_do_thuoc) * W_dang_chay\n"
            f"!!\n"
            f"!! Co so roi thi ghi vao JSON:  \"image_width\": W, \"image_height\": H\n"
            f"!! hoac chay tam voi:  --calib-size WxH\n")
    return camera_matrix, dist_coeffs, (int(w), int(h))


def fit_camera_matrix(camera_matrix, calib_size, frame_size):
    """Scale intrinsic tu do phan giai luc calib sang do phan giai luc chay.

    fx, fy, cx, cy deu ti le thuan voi kich thuoc anh. Neu bo qua buoc nay,
    moi khoang cach se sai theo dung ti le do phan giai.
    """
    cw, ch = calib_size
    fw, fh = frame_size
    if (cw, ch) == (fw, fh):
        return camera_matrix

    sx, sy = fw / cw, fh / ch
    print(f"!! Do phan giai lech: calib {cw}x{ch} vs dang chay {fw}x{fh}")
    if abs(sx - sy) > 0.01:
        raise SystemExit(
            f"!! TI LE KHUNG HINH KHAC NHAU (sx={sx:.3f}, sy={sy:.3f}).\n"
            f"!! Camera dang crop chu khong phai thu nho -> KHONG the scale dung.\n"
            f"!! Hay calib lai o dung do phan giai se dung khi chay.")
    print(f"!! Da scale intrinsic theo he so {sx:.3f}")

    scaled = camera_matrix.copy()
    scaled[0, 0] *= sx
    scaled[0, 2] *= sx
    scaled[1, 1] *= sy
    scaled[1, 2] *= sy
    return scaled


# ============================================================
#  SSR - Single Scale Retinex (chong bong do)
# ============================================================
def ssr(gray, sigma=100.0):
    """Single-Scale Retinex 

    Nguyen ly: anh = reflectance x illumination. Illumination la thanh phan
    thay doi cham (bong do, gradient sang). SSR tach reflectance (thong tin
    marker) ra khoi illumination.

    QUAN TRONG
      1) LOG TRUOC:  img = log(I)
      2) BLUR trong log-domain:  blur = G * log(I)
      3) TRU:  retinex = log(I) - G*log(I)
    sigma = 10 la gia tri tac gia dung cung (camera Arducam 800x600). Neu
    camera cua ban do phan giai khac nhieu thi co the chinh nhe, nhung khong
    lon nhu 100.
    """
    img = gray.astype(np.float32) + 1.0            # +1 tranh log(0)
    cv2.log(img, img)                              # 1) LOG TRUOC (in-place)
    # ksize tinh tu sigma dung cong thuc cua tac gia (bat le bang |1)
    ksize = int(round((sigma - 0.8) / 0.15 + 2.0)) | 1
    blur = cv2.GaussianBlur(img, (ksize, ksize), sigma, sigmaY=sigma, borderType=cv2.BORDER_REPLICATE)  # 2) blur log-domain
    retinex = img - blur                           # 3) tru trong log-domain
    out = cv2.normalize(retinex, None, 0, 255, cv2.NORM_MINMAX)
    return out.astype(np.uint8)

# ============================================================
#  DETECTOR (tuong thich OpenCV cu/moi)
# ============================================================
def make_detector(dict_id):
    """Tra ve ham detect(gray) -> (corners, ids, rejected)."""
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
    """Uoc luong pose bang solvePnP + SOLVEPNP_IPPE_SQUARE.

    Thay cho estimatePoseSingleMarkers (deprecated tu OpenCV 4.7).
    Tra ve (rvecs, tvecs, ratios); phan tu None neu giai that bai.
    'ratio' = sai so tai chieu cua nghiem tot / nghiem thu hai. Cang gan 1.0
    thi hai nghiem cang kho phan biet -> pose xoay khong dang tin (ambiguity).
    """
    half = marker_size / 2.0
    obj_points = np.array([
        [-half,  half, 0],   # goc 0: tren-trai
        [ half,  half, 0],   # goc 1: tren-phai
        [ half, -half, 0],   # goc 2: duoi-phai
        [-half, -half, 0],   # goc 3: duoi-trai
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
                        help=f"Chieu dai canh den ngoai cung cua marker [cm] "
                             f"(mac dinh {DEFAULT_MARKER_SIZE})")
    parser.add_argument("--cam", type=int, default=2, help="Index camera")
    parser.add_argument("--id", type=int, default=None,
                        help="Chi hien thi marker co id nay (mac dinh: tat ca)")
    parser.add_argument("--calib", default=DEFAULT_CALIB, help="Duong dan file calib JSON")
    parser.add_argument("--side", default="left", choices=["left", "right"],
                        help="Dung intrinsic cua camera trai hay phai")
    parser.add_argument("--calib-size", default=None, metavar="WxH",
                        help="Do phan giai luc calib (vd 640x400) neu file JSON "
                             "chua ghi. Chi dung tam; nen ghi thang vao JSON.")
    parser.add_argument("--dict", default=DEFAULT_DICT,
                        help=f"Ten dictionary ArUco (mac dinh {DEFAULT_DICT})")
    parser.add_argument("--ssr", action="store_true",
                        help="Bat SSR preprocessing (chong bong do)")
    parser.add_argument("--lpf", action="store_true",
                        help="Bat low-pass filter lam muot pose (nhe)")
    parser.add_argument("--log", default=None,
                        help="Ghi log ra file CSV (vd: run1.csv)")
    args = parser.parse_args()

    override_size = None
    if args.calib_size:
        try:
            ow, oh = args.calib_size.lower().split("x")
            override_size = (int(ow), int(oh))
        except ValueError:
            raise SystemExit(f"--calib-size phai co dang WxH, vd 640x400 "
                             f"(nhan duoc: {args.calib_size})")

    camera_matrix, dist_coeffs, calib_size = load_calibration(
        args.calib, args.side, override_size)
    print(f"Calib: {args.calib} [{args.side}] @ {calib_size[0]}x{calib_size[1]}"
          f"{'  (tu --calib-size)' if override_size else ''}")

    print(f"OpenCV {cv2.__version__}  |  dictionary {args.dict}  |  camera {args.side}")
    print(f"Marker size: {args.marker_size} cm", end="")
    if args.marker_size == DEFAULT_MARKER_SIZE:
        print("  (mac dinh - do lai neu khoang cach sai)")
    else:
        print()
    print(f"SSR: {'BAT' if args.ssr else 'tat'}  |  LPF: {'BAT' if args.lpf else 'tat'}")
    print("Phim tat: q=thoat  s=bat/tat SSR  d=xem anh SSR")

    detect = make_detector(getattr(aruco, args.dict))

    cap = cv2.VideoCapture(args.cam)
    if not cap.isOpened():
        raise SystemExit(f"Khong mo duoc camera index {args.cam}")

    # Ep camera ve dung do phan giai luc calib. cap.set() rat hay that bai
    # im lang tren USB cam -> phai doc lai frame thuc te de kiem chung.
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, calib_size[0])
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, calib_size[1])

    ok, frame = cap.read()
    if not ok:
        raise SystemExit("Khong doc duoc frame dau tien")
    frame_size = (frame.shape[1], frame.shape[0])
    print(f"Do phan giai dang chay: {frame_size[0]}x{frame_size[1]}")
    if frame_size != calib_size:
        print(f"!! Camera KHONG nhan do phan giai {calib_size[0]}x{calib_size[1]}, "
              f"dang tra ve {frame_size[0]}x{frame_size[1]}")
    camera_matrix = fit_camera_matrix(camera_matrix, calib_size, frame_size)

    # Setup logging
    log_file = None
    log_writer = None
    if args.log:
        log_file = open(args.log, "w", newline="")
        log_writer = csv.writer(log_file)
        log_writer.writerow(["t", "detected", "id", "dist_cm",
                             "x", "y", "z",                       # he camera
                             "cam_x", "cam_y", "cam_alt",         # he marker
                             "roll", "pitch", "yaw",
                             "ambiguity", "detect_ms", "total_ms", "fps"])

    # Trang thai runtime
    use_ssr = args.ssr
    show_ssr = False
    tvec_prev = {}          # luu pose truoc de LPF (theo id)

    # Thong ke chay
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

        # ---- SSR preprocessing (do rieng thoi gian) ----
        t_ssr0 = time.perf_counter()

        gray_proc = ssr(gray) if use_ssr else gray

        t_ssr = (time.perf_counter() - t_ssr0) * 1000
        # ---- Detection (do rieng thoi gian) ----
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

                # LPF lam muot pose (nhe)
                if args.lpf and marker_id in tvec_prev:
                    tvec = LPF_ALPHA * tvec + (1 - LPF_ALPHA) * tvec_prev[marker_id]
                tvec_prev[marker_id] = tvec

                cv2.drawFrameAxes(frame, camera_matrix, dist_coeffs,rvec, tvec, args.marker_size * 0.5)

                x, yy, z = tvec.ravel()

                distance = float(np.linalg.norm(tvec))
                rmat, _ = cv2.Rodrigues(rvec)
                roll, pitch, yaw = rotation_to_euler(rmat)

                # Vi tri CAMERA trong he MARKER (= -R^T * t).
                # Khac han x,y,z o tren: bo nay gan voi MAT DAT chu khong gan voi
                # camera, nen dau thuan truc giac - camera dich theo +X/+Y/+Z cua
                # marker thi so tang, va cam_alt khong doi khi camera chi nghieng.
                # CHI de xem/kiem tra bang mat. KHONG dua vao dieu khien: no phu
                # thuoc rvec, ma rvec chinh la thu bi canh bao ambiguity ben duoi.
                cam_x, cam_y, cam_alt = (-rmat.T @ tvec.reshape(3, 1)).ravel()

                lines = [
                    f"id={marker_id}  dist={distance:.1f}cm",
                    f"x={x:.1f} y={yy:.1f} z={z:.1f} cm",
                    f"roll={roll:.1f} pitch={pitch:.1f} yaw={yaw:.1f} deg",
                ]
                for line in lines:
                    cv2.putText(frame, line, (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                    y += 25

                # He marker - mau vang de phan biet voi he camera (mau xanh la)
                cv2.putText(frame,
                            f"[marker] camx={cam_x:.1f} camy={cam_y:.1f} alt={cam_alt:.1f} cm",
                            (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
                y += 25

                ratio = ratios[i]
                if ratio is not None and ratio > AMBIGUITY_WARN_RATIO:
                    cv2.putText(frame, f"! goc xoay khong tin cay ({ratio:.2f})",(10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 165, 255), 2)
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
            cv2.putText(frame, "Khong thay marker", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
            if log_writer:
                log_writer.writerow([f"{time.time():.3f}", 0, "", "", "", "", "",
                                     "", "", "", "", "", "", "",
                                     f"{t_det:.2f}", "", ""])

        # ---- Thong ke thoi gian ----
        t_total = (time.perf_counter() - t0) * 1000
        fps_inst = 1000.0 / t_total if t_total > 0 else 0
        fps_smooth = 0.9 * fps_smooth + 0.1 * fps_inst if fps_smooth > 0 else fps_inst
        det_rate = 100.0 * frame_detected / frame_total

        # Overlay thong tin he thong (goc duoi)
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
            print(f"SSR: {'BAT' if use_ssr else 'tat'}")
        elif key == ord('d'):
            show_ssr = not show_ssr
            if not show_ssr:
                cv2.destroyWindow("SSR output")

    cap.release()
    if log_file:
        log_file.close()
        print(f"Da ghi log: {args.log}")

    # In tong ket
    elapsed = time.time() - t_start
    print(f"\n=== TONG KET ===")
    print(f"Tong frame: {frame_total}  |  detect: {frame_detected} "
          f"({100.0*frame_detected/max(frame_total,1):.1f}%)")
    print(f"Thoi gian: {elapsed:.1f}s  |  FPS trung binh: {frame_total/max(elapsed,1):.1f}")

    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()