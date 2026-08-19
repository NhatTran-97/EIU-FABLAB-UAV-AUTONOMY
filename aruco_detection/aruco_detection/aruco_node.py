#!/usr/bin/env python3
"""ROS2 node: detect ArUco markers and publish the marker pose.

Publishes:
    ~/pose       geometry_msgs/PoseStamped  - METERS
    ~/detected   std_msgs/Bool
    ~/ambiguity  std_msgs/Float32
    ~/reprojection_error std_msgs/Float32   - Fractal pose RMS in pixels
    ~/image      sensor_msgs/Image          - debug overlay (publish_debug_image)

frame_id defaults to the camera optical frame per REP-145: x right, y down,
z forward. That matches the OpenCV convention exactly, so tvec is published
as-is with no frame conversion.

Usage:
    ros2 run tf2_ros static_transform_publisher \
      --frame-id map --child-frame-id camera_optical_frame

    ros2 run aruco_detection aruco_node.py --ros-args \
      -p target_id:=1 -p use_reliable_qos:=true -p publish_debug_image:=true

      
    pip install nanofractal

    ros2 launch aruco_detection aruco_pose_launch.py \
        camera_source:=2 \
        detector_type:=fractal \
        fractal_config:=FRACTAL_5L_6 \
        marker_size:=0.282 \
        target_id:=-1

    ros2 run aruco_detection aruco_node.py --ros-args \
    -p camera_source:='"2"' \
    -p detector_type:=fractal \
    -p fractal_config:=FRACTAL_5L_6 \
    -p marker_size:=0.282 \
    -p target_id:=-1 \
    -p publish_debug_image:=true \
    -p use_reliable_qos:=true \
    --log-level debug
"""
import os
import time

import cv2
import cv2.aruco as aruco
import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, Float32

from aruco_detection.calibration import (fit_camera_matrix, load_calibration,
                                         parse_size)
from aruco_detection.detector import MarkerDetector
from aruco_detection.load_camera import open_camera
from aruco_detection.transforms import rotmat_to_quat
from aruco_detection import visualization as viz

try:
    from cv_bridge import CvBridge
except ImportError:
    CvBridge = None


class ArucoPoseNode(Node):

    def __init__(self):
        super().__init__("aruco_pose_node")

        # Accepts a USB index ("2"), an RTSP URL, or a video file path.
        self.declare_parameter("camera_source", "2")
        self.declare_parameter("threaded_capture", True)
        self.declare_parameter("calib_file", self._default_calib_path())
        self.declare_parameter("calib_side", "left")
        self.declare_parameter("calib_size", "")
        self.declare_parameter("marker_size", 0.267)
        self.declare_parameter("detector_type", "aruco")
        self.declare_parameter("dictionary", "DICT_4X4_50")
        self.declare_parameter("fractal_config", "FRACTAL_5L_6")
        self.declare_parameter("target_id", 1)   # -1 accepts any marker (real IDs are >= 0)
        self.declare_parameter("frame_id", "camera_optical_frame")
        self.declare_parameter("publish_rate", 30.0)
        self.declare_parameter("use_ssr", False)
        self.declare_parameter("lpf_alpha", 1.0)  # 1.0 = no smoothing
        self.declare_parameter("ambiguity_warn", 0.7)
        self.declare_parameter("publish_debug_image", False)
        # BEST_EFFORT suits a 30Hz sensor stream and matches the QoS px4_msgs
        # topics use, but "ros2 topic hz" on Humble cannot request it and so
        # receives nothing. Turn this on to debug with the standard tools.
        self.declare_parameter("use_reliable_qos", False)

        p = self.get_parameter
        self.marker_size = p("marker_size").value
        self.detector_type = str(p("detector_type").value).lower()
        if self.detector_type not in ("aruco", "fractal"):
            raise RuntimeError(
                f"detector_type must be 'aruco' or 'fractal' "
                f"(got: {self.detector_type})"
            )
        target = p("target_id").value
        self.target_id = None if target < 0 else target
        self.frame_id = p("frame_id").value
        self.ambiguity_warn = p("ambiguity_warn").value
        self.publish_debug_image = p("publish_debug_image").value

        self.camera_matrix, self.dist_coeffs, calib_size = load_calibration(
            p("calib_file").value, p("calib_side").value,parse_size(p("calib_size").value))
        
        self.get_logger().info(f"Calibration {p('calib_file').value} [{p('calib_side').value}] "
            f"@ {calib_size[0]}x{calib_size[1]}")

        dict_name = p("dictionary").value
        if self.detector_type == "aruco" and not hasattr(aruco, dict_name):
            raise RuntimeError(f"Unknown dictionary: {dict_name}")
        dict_id = getattr(aruco, dict_name) if self.detector_type == "aruco" else None

        frame_size = self._open_camera(p("camera_source").value, calib_size, p("threaded_capture").value)
        self.camera_matrix = fit_camera_matrix(self.camera_matrix, calib_size, frame_size)

        self.detector = MarkerDetector(
            dict_id=dict_id,
            marker_size=self.marker_size,
            camera_matrix=self.camera_matrix,
            dist_coeffs=self.dist_coeffs,
            use_ssr=p("use_ssr").value,
            lpf_alpha=p("lpf_alpha").value,
            detector_type=self.detector_type,
            fractal_config=p("fractal_config").value)

        reliable = p("use_reliable_qos").value
        qos = QoSProfile(depth=1, reliability=(ReliabilityPolicy.RELIABLE if reliable
                                               else ReliabilityPolicy.BEST_EFFORT))
        self.get_logger().info(f"QoS reliability: {'RELIABLE' if reliable else 'BEST_EFFORT'}")
        self.pub_pose = self.create_publisher(PoseStamped, "~/pose", qos)
        self.pub_detected = self.create_publisher(Bool, "~/detected", qos)
        self.pub_ambiguity = self.create_publisher(Float32, "~/ambiguity", qos)
        self.pub_reprojection_error = self.create_publisher(
            Float32, "~/reprojection_error", qos)

        self.bridge = None
        if self.publish_debug_image:
            if CvBridge is None:
                self.get_logger().warn("cv_bridge missing -> publish_debug_image disabled")
                self.publish_debug_image = False
            else:
                from sensor_msgs.msg import Image
                self.bridge = CvBridge()
                self.pub_image = self.create_publisher(Image, "~/image", qos)

        self.n_frame = 0
        self.n_detected = 0
        self._last_cb = None
        self._fps = 0.0
        self.create_timer(1.0 / p("publish_rate").value, self.on_timer)
        marker_family = (dict_name if self.detector_type == "aruco"
                         else p("fractal_config").value)
        self.get_logger().info(f"Ready: {self.detector_type}, marker "
                               f"{self.marker_size * 100:.1f}cm, {marker_family}, "
                               f"target_id={self.target_id}, frame_id '{self.frame_id}'")

    @staticmethod
    def _default_calib_path():
        try:
            from ament_index_python.packages import get_package_share_directory
            return os.path.join(get_package_share_directory("aruco_detection"),"calib_data_mono.json")
        except Exception:
            return os.path.join(os.path.dirname(os.path.abspath(__file__)),"calib_data_mono.json")

    def _open_camera(self, source, calib_size, threaded):
        self.cam = open_camera(source, width=calib_size[0], height=calib_size[1],
                               threaded=threaded)
        self.get_logger().info(f"Camera: {self.cam}")

        # The requested resolution is only a request: cap.set() fails silently
        # on many USB cameras. Compare against what actually came back, because
        # a mismatch makes every distance wrong by the resolution ratio.
        frame_size = self.cam.resolution
        if frame_size != calib_size:
            self.get_logger().warn(f"Camera returns {frame_size[0]}x{frame_size[1]} instead of the "
                f"calibrated {calib_size[0]}x{calib_size[1]}")
        return frame_size

    def on_timer(self):
        t0 = time.perf_counter()
        ok, frame = self.cam.read()
        if not ok:
            return
        self.n_frame += 1

        if self._last_cb is not None:
            dt = t0 - self._last_cb
            if dt > 0:
                self._fps = 0.9 * self._fps + 0.1 * (1.0 / dt) if self._fps else 1.0 / dt
        self._last_cb = t0

        stamp = self.get_clock().now().to_msg()
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        t_det0 = time.perf_counter()
        detections = self.detector.process(gray, self.target_id)
        detect_ms = (time.perf_counter() - t_det0) * 1000

        if detections:
            self.n_detected += 1
            det = detections[0]
            self._publish_pose(stamp, det)

            if det.ambiguity is not None:
                self.pub_ambiguity.publish(Float32(data=float(det.ambiguity)))
                if det.ambiguity > self.ambiguity_warn:
                    self.get_logger().warn(f"Rotation unreliable ({det.ambiguity:.2f}): use the "f"position, ignore the orientation",
                        throttle_duration_sec=2.0)
            if det.reprojection_error is not None:
                self.pub_reprojection_error.publish(Float32(data=float(det.reprojection_error)))
        self.pub_detected.publish(Bool(data=bool(detections)))
        if self.publish_debug_image:
            self._publish_debug(stamp, frame, detections, t0, detect_ms)

    def _publish_pose(self, stamp, det):
        msg = PoseStamped()
        msg.header.stamp = stamp
        msg.header.frame_id = self.frame_id
        t = det.tvec.ravel()
        msg.pose.position.x = float(t[0])
        msg.pose.position.y = float(t[1])
        msg.pose.position.z = float(t[2])
        qx, qy, qz, qw = rotmat_to_quat(cv2.Rodrigues(det.rvec)[0])
        msg.pose.orientation.x = qx
        msg.pose.orientation.y = qy
        msg.pose.orientation.z = qz
        msg.pose.orientation.w = qw
        self.pub_pose.publish(msg)

    def _publish_debug(self, stamp, frame, detections, t0, detect_ms):
        viz.draw_markers(frame, detections)
        y = 30
        if detections:
            for det in detections:
                y = viz.draw_detection(frame, det, self.camera_matrix,
                                       self.dist_coeffs, self.marker_size, y=y, ambiguity_warn=self.ambiguity_warn)
        else:
            viz.draw_no_marker(frame, y)
        frame_ms = (time.perf_counter() - t0) * 1000
        viz.draw_stats(frame, fps=self._fps, frame_ms=frame_ms,
                       detect_ms=detect_ms, detected=self.n_detected, total=self.n_frame)

        msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        msg.header.stamp = stamp
        msg.header.frame_id = self.frame_id
        self.pub_image.publish(msg)

    def destroy_node(self):
        if getattr(self, "cam", None) is not None:
            self.cam.release()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = ArucoPoseNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except RuntimeError as e:
        print(f"[aruco_pose_node] {e}")
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
