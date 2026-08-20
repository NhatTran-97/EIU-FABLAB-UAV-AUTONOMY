import argparse
import os
import time

import cv2
import nanofractal as nf
import numpy as np

# Chay bang: python3 scripts/fractal_test.py  (sau khi source install/setup.bash).
# Fallback "from calibration import" cu da bo -- file khong con nam canh
# calibration.py nua nen no chi che di loi that.
from aruco_detection.calibration import fit_camera_matrix, load_calibration


MARKER_SIZE_CM = 28.2  # chiều dài cạnh ngoài của marker
DEFAULT_CALIB_FILE = os.path.join(
    os.path.dirname(os.path.abspath(__file__)),
    "calib_data_mono.json",
)

detector = nf.FractalDetector(
    "FRACTAL_5L_6",
    marker_size=MARKER_SIZE_CM / 100.0  # nanofractal nhận đơn vị mét
)


def detect_fractal_marker(frame, camera_matrix=None, dist_coeffs=None):
    """Phát hiện Fractal Marker; tvec của pose được trả về theo cm."""
    result = detector.detect(frame, with_inner_points=True)

    if result.ids.size == 0:
        return result, None

    pose = None
    if camera_matrix is not None and dist_coeffs is not None:
        pose_m = detector.estimate_pose(
            result,
            camera_matrix,
            dist_coeffs.reshape(-1),
        )
        if pose_m is not None:
            rvec, tvec_m, reprojection_error = pose_m
            pose = (rvec, tvec_m * 100.0, reprojection_error)

    return result, pose


def parse_args():
    parser = argparse.ArgumentParser(description="Fractal Marker camera test")
    parser.add_argument(
        "--camera",
        default="2",
        help="Số camera, đường dẫn video hoặc RTSP URL (mặc định: 2)",
    )
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--height", type=int, default=480)
    parser.add_argument("--calib-file", default=DEFAULT_CALIB_FILE)
    parser.add_argument("--calib-side", default="left")
    return parser.parse_args()


def open_camera(source, width, height):
    if source.lstrip("-").isdigit():
        source = int(source)

    camera = cv2.VideoCapture(source)
    camera.set(cv2.CAP_PROP_FRAME_WIDTH, width)
    camera.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
    camera.set(cv2.CAP_PROP_BUFFERSIZE, 1)

    if not camera.isOpened():
        camera.release()
        raise RuntimeError(f"Không mở được camera: {source}")

    return camera


def draw_text(frame, text, row, color):
    cv2.putText(
        frame,
        text,
        (10, 30 + row * 30),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.7,
        color,
        2,
        cv2.LINE_AA,
    )


def main():
    args = parse_args()

    camera_matrix, dist_coeffs, calib_size = load_calibration(
        args.calib_file,
        args.calib_side,
    )
    camera = open_camera(args.camera, args.width, args.height)

    fitted_camera_matrix = None
    dist_coeffs = np.ascontiguousarray(
        dist_coeffs,
        dtype=np.float64,
    ).reshape(-1)

    fps = 0.0
    previous_time = None

    print(f"Fractal Marker: FRACTAL_5L_6, cạnh ngoài {MARKER_SIZE_CM:.1f} cm")
    print("Nhấn q hoặc Esc để thoát")

    try:
        while True:
            success, frame = camera.read()
            if not success or frame is None:
                print("Không đọc được hình ảnh từ camera")
                break

            frame = np.ascontiguousarray(frame, dtype=np.uint8)
            frame_height, frame_width = frame.shape[:2]

            if fitted_camera_matrix is None:
                fitted_camera_matrix = fit_camera_matrix(
                    camera_matrix,
                    calib_size,
                    (frame_width, frame_height),
                )
                fitted_camera_matrix = np.ascontiguousarray(
                    fitted_camera_matrix,
                    dtype=np.float64,
                )
                print(f"Độ phân giải camera: {frame_width}x{frame_height}")

            start_time = time.perf_counter()
            result, pose = detect_fractal_marker(
                frame,
                fitted_camera_matrix,
                dist_coeffs,
            )
            detection_ms = (time.perf_counter() - start_time) * 1000.0

            detector.draw(frame, result)

            if pose is not None:
                rvec, tvec_cm, reprojection_error = pose

                # tvec và axis_length đều dùng cm nên phép chiếu vẫn đúng tỉ lệ.
                cv2.drawFrameAxes(
                    frame,
                    fitted_camera_matrix,
                    dist_coeffs,
                    rvec,
                    tvec_cm,
                    MARKER_SIZE_CM * 0.5,
                    2,
                )

                draw_text(frame, "DETECTED", 0, (0, 255, 0))
                draw_text(
                    frame,
                    (
                        f"x={tvec_cm[0]:+.1f}  y={tvec_cm[1]:+.1f}  "
                        f"z={tvec_cm[2]:+.1f} cm"
                    ),
                    1,
                    (0, 255, 0),
                )
                draw_text(
                    frame,
                    f"Reprojection error: {reprojection_error:.2f} px",
                    2,
                    (0, 255, 0),
                )
            else:
                draw_text(frame, "NO FRACTAL MARKER", 0, (0, 0, 255))

            current_time = time.perf_counter()
            if previous_time is not None:
                current_fps = 1.0 / max(current_time - previous_time, 1e-9)
                fps = current_fps if fps == 0.0 else 0.9 * fps + 0.1 * current_fps
            previous_time = current_time

            draw_text(
                frame,
                f"FPS: {fps:.1f} | Detect: {detection_ms:.1f} ms",
                3,
                (255, 255, 0),
            )

            cv2.imshow("Fractal Marker Detection", frame)
            key = cv2.waitKey(1) & 0xFF
            if key in (ord("q"), 27):
                break
    finally:
        camera.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    try:
        main()
    except (RuntimeError, OSError, ValueError) as error:
        print(f"[ERROR] {error}")
