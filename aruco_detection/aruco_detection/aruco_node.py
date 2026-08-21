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

ros2 run aruco_detection aruco_node.py --ros-args \
    -p camera_source:='"2"' -p publish_debug_image:=true
"""
import os
import time

import cv2
import cv2.aruco as aruco
import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
import numpy as np
from std_msgs.msg import Bool, Float32, Int32

from aruco_detection.calibration import (fit_camera_matrix, load_calibration, parse_size)
from aruco_detection.detector import MarkerDetector
from aruco_detection.load_camera import open_camera
from aruco_detection.geometry import rotmat_to_quat
from aruco_detection import visualization as viz
from rclpy.duration import Duration

try:
    from cv_bridge import CvBridge
except ImportError:
    CvBridge = None

class ArucoPoseNode(Node):
    def __init__(self):
        super().__init__("aruco_pose_node")
        self._declare_parameters()
        self._load_calibration()
        self._setup_camera()
        self._setup_detector()
        self._setup_publishers()

        self.n_frame = 0
        self.n_detected = 0
        self._last_cb = None
        self._fps = 0.0
        self._last_frame_wall = time.monotonic()

        rate = self.get_parameter("publish_rate").value
        self.create_timer(1.0 / rate, self.on_timer)
        self.get_logger().info(f"Ready: {self.detector_type}, marker {self.marker_size * 100:.1f}cm, "
            f"{self.marker_family}, target_id={self.target_id}, "
            f"frame_id '{self.frame_id}', poll {rate:.0f}Hz")

    # ─────────────────────────────────────────────────────────── setup
    def _declare_parameters(self):
        self.declare_parameter("camera_source", "2")
        self.declare_parameter("threaded_capture", True)
        # Chi co tac dung khi camera_source la URL RTSP. Jitter buffer nay la
        # mot phan DA BIET cua pipeline_latency_s -- giam no thi phai giam
        # pipeline_latency_s tuong ung.

        self.declare_parameter("rtsp_latency_ms", 100)
        self.declare_parameter("rtsp_protocol", "tcp")
        self.declare_parameter("calib_file", self._default_calib_path())
        self.declare_parameter("calib_side", "left")
        self.declare_parameter("calib_size", "")
        self.declare_parameter("marker_size", 0.281) # 0.281 0.267
        self.declare_parameter("detector_type", "fractal") # aruco  fractal
        self.declare_parameter("dictionary", "DICT_4X4_50")
        self.declare_parameter("fractal_config", "FRACTAL_5L_6")
        self.declare_parameter("target_id", -1)   # -1 = nhan moi marker
        self.declare_parameter("frame_id", "camera_optical_frame")
        self.declare_parameter("use_ssr", False)
        self.declare_parameter("ambiguity_warn", 0.7)

        # Poll NHANH HON camera: hai dong ho 30Hz khong dong bo, khi hai frame
        # ve giua hai lan poll thi frame dau bi ghi de va mat luon.
        self.declare_parameter("publish_rate", 30.0)
        # Do tre TRUOC khi frame toi read(): jitter buffer RTSP + giai ma.
        # Do bang cach chia camera vao dong ho bam giay.
        self.declare_parameter("pipeline_latency_s", 0.0)
        # Nguong tach "chua co frame moi" (binh thuong) khoi "camera chet".
        self.declare_parameter("camera_timeout_s", 1.0)

        self.declare_parameter("publish_debug_image", False)
        # Publish moi frame mac dinh de luong debug bam sat camera (~30 FPS).
        # Voi anh lon/RTSP co the tang gia tri nay de giam bang thong DDS.
        self.declare_parameter("debug_image_every", 1)
        self.declare_parameter("use_reliable_qos", False)

        p = self.get_parameter
        self.marker_size = p("marker_size").value
        self.detector_type = str(p("detector_type").value).lower()
        if self.detector_type not in ("aruco", "fractal"):
            raise RuntimeError(f"detector_type phai la 'aruco' hoac 'fractal' " f"(nhan duoc: {self.detector_type})")

        target = p("target_id").value
        self.target_id = None if target < 0 else target
        self.frame_id = p("frame_id").value
        self.ambiguity_warn = p("ambiguity_warn").value
        self.pipeline_latency = p("pipeline_latency_s").value
        self.camera_timeout = p("camera_timeout_s").value
        self.debug_every = max(1, int(p("debug_image_every").value))
        self.publish_debug_image = p("publish_debug_image").value

    def _load_calibration(self):
        p = self.get_parameter
        self.camera_matrix, self.dist_coeffs, self.calib_size = load_calibration(
            p("calib_file").value, p("calib_side").value,parse_size(p("calib_size").value))
        self.get_logger().info(f"Calibration {p('calib_file').value} [{p('calib_side').value}] "
            f"@ {self.calib_size[0]}x{self.calib_size[1]}")

    def _setup_camera(self):
        p = self.get_parameter
        self.cam = open_camera(p("camera_source").value,
                               width=self.calib_size[0], height=self.calib_size[1],
                               threaded=p("threaded_capture").value,
                               latency=p("rtsp_latency_ms").value,
                               protocol=p("rtsp_protocol").value)
        self.get_logger().info(f"Camera: {self.cam}")

        # Do phan giai yeu cau chi la yeu cau: cap.set() im lang that bai tren
        # nhieu USB camera. Lech thi moi khoang cach sai theo dung ti le do
        # phan giai, ma van trong hoan toan hop ly.
        frame_size = self.cam.resolution
        if frame_size != self.calib_size:
            self.get_logger().warn(f"Camera tra {frame_size[0]}x{frame_size[1]} thay vi "
                f"{self.calib_size[0]}x{self.calib_size[1]} da hieu chuan")
        self.camera_matrix = fit_camera_matrix(self.camera_matrix,self.calib_size, frame_size)

    def _setup_detector(self):
        p = self.get_parameter
        dict_name = p("dictionary").value
        if self.detector_type == "aruco" and not hasattr(aruco, dict_name):
            raise RuntimeError(f"Dictionary khong ton tai: {dict_name}")
        dict_id = getattr(aruco, dict_name) if self.detector_type == "aruco" else None
        self.marker_family = (dict_name if self.detector_type == "aruco"
                              else p("fractal_config").value)

        self.detector = MarkerDetector(
            dict_id=dict_id,
            marker_size=self.marker_size,
            camera_matrix=self.camera_matrix,
            dist_coeffs=self.dist_coeffs,
            use_ssr=p("use_ssr").value,
            detector_type=self.detector_type,
            fractal_config=p("fractal_config").value)

    def _setup_publishers(self):
        reliable = self.get_parameter("use_reliable_qos").value
        
        qos = QoSProfile(depth=1, reliability=(ReliabilityPolicy.BEST_EFFORT))


        self.pub_pose = self.create_publisher(PoseStamped, "~/pose", qos)

        self.pub_detected = self.create_publisher(Bool, "~/detected", qos)
        self.pub_marker_id = self.create_publisher(Int32, "~/marker_id", qos)
        self.pub_ambiguity = self.create_publisher(Float32, "~/ambiguity", qos)
        self.pub_reprojection_error = self.create_publisher(
            Float32, "~/reprojection_error", qos)

        self.bridge = None
        if self.publish_debug_image:
            if CvBridge is None:
                self.get_logger().warn("thieu cv_bridge -> tat publish_debug_image")
                self.publish_debug_image = False
            else:
                from sensor_msgs.msg import Image
                self.bridge = CvBridge()
                self.pub_image = self.create_publisher(Image, "~/image", qos)

    @staticmethod
    def _default_calib_path():
        try:
            from ament_index_python.packages import get_package_share_directory
            return os.path.join(get_package_share_directory("aruco_detection"),"calib_data_mono.json")
        except Exception:
            return os.path.join(os.path.dirname(os.path.abspath(__file__)),"calib_data_mono.json")

    def on_timer(self):

        t0 = time.perf_counter()
        ok, frame, t_capture = self.cam.read()

        if not ok:
            self._check_camera_alive()
            return
        
        self._last_frame_wall = time.monotonic()
        self.n_frame += 1

        if self._last_cb is not None:
            dt = t0 - self._last_cb
            if dt > 0:
                inst = 1.0 / dt
                self._fps = (0.9 * self._fps + 0.1 * inst) if self._fps else inst
        self._last_cb = t0

        age = time.monotonic() - t_capture + self.pipeline_latency

        stamp = (self.get_clock().now() - Duration(seconds=age)).to_msg()

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        t_det0 = time.perf_counter()
        detections = self.detector.process(gray, self.target_id)
        detect_ms = (time.perf_counter() - t_det0) * 1000

        if detections:
            self.n_detected += 1
            det = self._pick(detections)
            self._publish_pose(stamp, det)
            self.pub_marker_id.publish(Int32(data=int(det.marker_id)))

            if det.ambiguity is not None:
                self.pub_ambiguity.publish(Float32(data=float(det.ambiguity)))
                if det.ambiguity > self.ambiguity_warn:
                    self.get_logger().warn(f"Rotation unreliable ({det.ambiguity:.2f}): use the "f"position, ignore the orientation",
                        throttle_duration_sec=2.0)
            if det.reprojection_error is not None:
                self.pub_reprojection_error.publish(Float32(data=float(det.reprojection_error)))
        self.pub_detected.publish(Bool(data=bool(detections)))
        if self.publish_debug_image and self.n_frame % self.debug_every == 0:
            self._publish_debug(stamp, frame, detections, t0, detect_ms)

    @staticmethod
    def _pick(detections):
        """Marker gan nhat = 4 goc chiem dien tich lon nhat trong anh.

        Voi target_id=-1, detections[0] chi la thu tu detector tra ve. Hai
        marker trong khung hinh lam pose nhay qua lai, va phia duoi khong the
        phan biet cu nhay do voi marker that su di chuyen -- do la ly do
        ~/marker_id duoc publish kem.
        """
        if len(detections) == 1:
            return detections[0]
        return max(detections,
                   key=lambda d: cv2.contourArea(d.corners.astype(np.float32)))

    def _check_camera_alive(self):
        """read() tra False cho CA HAI: chua co frame moi, va camera chet.

        Chi co dong ho tach duoc chung. Khong tach thi ~/detected dung nguyen
        gia tri cuoi, va bo dieu khien ha canh se tuong marker van con do.
        """
        dt = time.monotonic() - self._last_frame_wall
        if dt < self.camera_timeout:
            return
        self.pub_detected.publish(Bool(data=False))
        err = getattr(self.cam, "last_error", None)
        self.get_logger().error(f"Khong co frame trong {dt:.1f}s" + (f": {err}" if err else ""),
                                throttle_duration_sec=2.0)

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
