"""Debug overlay drawing.

Colour convention (BGR):
    green  - CAMERA frame (raw tvec, the values a controller should use)
    yellow - MARKER frame (inspection only, depends on rvec)
    orange - warnings
    cyan   - timing statistics
"""
import cv2
import cv2.aruco as aruco
import numpy as np

from .geometry import camera_in_marker, rotation_to_euler

GREEN = (0, 255, 0)
YELLOW = (0, 255, 255)
ORANGE = (0, 165, 255)
CYAN = (255, 255, 0)
RED = (0, 0, 255)

_FONT = cv2.FONT_HERSHEY_SIMPLEX
_SCALE = 0.6
_THICK = 2
_LINE_H = 25


def _put(frame, text, y, color):
    cv2.putText(frame, text, (10, y), _FONT, _SCALE, color, _THICK)
    return y + _LINE_H


def draw_markers(frame, detections):
    """Outline each marker and mark corner 0 (the -X, +Y corner) in red."""
    if not detections:
        return
    corners = [d.corners.reshape(1, 4, 2).astype(np.float32) for d in detections]
    ids = np.array([[d.marker_id] for d in detections])
    aruco.drawDetectedMarkers(frame, corners, ids)


def draw_detection(frame, det, camera_matrix, dist_coeffs, marker_size,
                   y=30, ambiguity_warn=0.7, to_cm=100.0, show_marker_frame=True):
    """Draw axes and readouts for one marker. Returns the next free y."""
    cv2.drawFrameAxes(frame, camera_matrix, dist_coeffs,
                      det.rvec, det.tvec, marker_size * 0.5)

    x, yy, z = det.tvec.ravel() * to_cm
    distance = float(np.linalg.norm(det.tvec)) * to_cm
    rmat, _ = cv2.Rodrigues(det.rvec)
    roll, pitch, yaw = rotation_to_euler(rmat)

    y = _put(frame, f"id={det.marker_id}  dist={distance:.1f}cm", y, GREEN)
    y = _put(frame, f"x={x:.1f} y={yy:.1f} z={z:.1f} cm", y, GREEN)
    y = _put(frame, f"roll={roll:.1f} pitch={pitch:.1f} yaw={yaw:.1f} deg", y, GREEN)

    if show_marker_frame:
        cam_x, cam_y, cam_alt = camera_in_marker(rmat, det.tvec) * to_cm
        y = _put(frame, f"[marker] camx={cam_x:.1f} camy={cam_y:.1f} "
                        f"alt={cam_alt:.1f} cm", y, YELLOW)

    if det.ambiguity is not None and det.ambiguity > ambiguity_warn:
        y = _put(frame, f"! rotation unreliable ({det.ambiguity:.2f})", y, ORANGE)

    if det.reprojection_error is not None:
        color = GREEN if det.reprojection_error <= 5.0 else ORANGE
        y = _put(frame, f"reprojection RMS={det.reprojection_error:.2f}px", y, color)

    return y + 10


def draw_no_marker(frame, y=30):
    return _put(frame, "No marker", y, RED)


def draw_stats(frame, fps, frame_ms, detect_ms, ssr_ms=None, use_ssr=False,
               detected=None, total=None):
    """Draw the timing block at the bottom of the frame."""
    h = frame.shape[0]
    lines = [f"FPS: {fps:.1f}  |  frame: {frame_ms:.1f}ms"]
    if ssr_ms is not None:
        lines.append(f"SSR: {ssr_ms:.1f}ms [{'on' if use_ssr else 'off'}]  "
                     f"detect: {detect_ms:.1f}ms")
    else:
        lines.append(f"detect: {detect_ms:.1f}ms")
    if total:
        lines.append(f"Detection rate: {100.0 * detected / total:.1f}% "
                     f"({detected}/{total})")

    y = h - _LINE_H * len(lines) - 5
    for line in lines:
        y = _put(frame, line, y, CYAN)
