#!/usr/bin/env python3
"""ROS2 node phat hien ArUco va publish pose marker trong he camera optical.

Publish:
    ~/pose       geometry_msgs/PoseStamped  - DON VI MET
    ~/detected   std_msgs/Bool
    ~/ambiguity  std_msgs/Float32
    ~/image      sensor_msgs/Image          - anh debug (neu publish_debug_image)

frame_id mac dinh la camera optical frame theo REP-145: x phai, y xuong, z toi
truoc. Trung khop he cua OpenCV nen tvec publish thang, khong phai doi he.


ros2 run tf2_ros static_transform_publisher \
  --frame-id map --child-frame-id camera_optical_frame

ros2 run aruco_detection aruco_node.py --ros-args \
  -p target_id:=1 -p use_reliable_qos:=true -p publish_debug_image:=true

"""
import os
import time

import cv2

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node

from std_msgs.msg import Bool, Float32

# Import TUYET DOI, khong dung relative: file nay duoc cai vao
# lib/aruco_detection/ va chay nhu script (__main__) nen khong co package cha.
import cv2.aruco as aruco
from rclpy.qos import QoSProfile, ReliabilityPolicy
from aruco_detection.calibration import (fit_camera_matrix, load_calibration,
                                         parse_size)
from aruco_detection.detector import MarkerDetector
from aruco_detection.transforms import rotmat_to_quat
from aruco_detection import visualization as viz

try:
    from cv_bridge import CvBridge
except ImportError:
    CvBridge = None
from sensor_msgs.msg import Image

class ArucoPoseNode(Node):

    def __init__(self):
        super().__init__("aruco_pose_node")


   

        self.declare_parameter("camera_index", 0)
        self.declare_parameter("calib_file", self._default_calib_path())
        self.declare_parameter("calib_side", "left")
        self.declare_parameter("calib_size", "")
        self.declare_parameter("marker_size", 0.267)
        self.declare_parameter("dictionary", "DICT_4X4_50")
        self.declare_parameter("target_id", 1)   # -1 = nhan moi marker (ID that luon >= 0)
        self.declare_parameter("frame_id", "camera_optical_frame")
        self.declare_parameter("publish_rate", 30.0)
        self.declare_parameter("use_ssr", False)
        self.declare_parameter("lpf_alpha", 1.0)
        self.declare_parameter("ambiguity_warn", 0.7)
        self.declare_parameter("publish_debug_image", False)
       
        self.declare_parameter("use_reliable_qos", False)

        p = self.get_parameter
        self.marker_size = p("marker_size").value
        target = p("target_id").value
        self.target_id = None if target < 0 else target
        self.frame_id = p("frame_id").value
        self.ambiguity_warn = p("ambiguity_warn").value
        self.publish_debug_image = p("publish_debug_image").value

        self.camera_matrix, self.dist_coeffs, calib_size = load_calibration(
            p("calib_file").value, p("calib_side").value,
            parse_size(p("calib_size").value))
        self.get_logger().info(
            f"Calib {p('calib_file').value} [{p('calib_side').value}] "
            f"@ {calib_size[0]}x{calib_size[1]}")

        dict_name = p("dictionary").value
        if not hasattr(aruco, dict_name):
            raise RuntimeError(f"Dictionary khong ton tai: {dict_name}")

        frame_size = self._open_camera(p("camera_index").value, calib_size)
        self.camera_matrix = fit_camera_matrix(
            self.camera_matrix, calib_size, frame_size)

        self.detector = MarkerDetector(
            dict_id=getattr(aruco, dict_name),
            marker_size=self.marker_size,
            camera_matrix=self.camera_matrix,
            dist_coeffs=self.dist_coeffs,
            use_ssr=p("use_ssr").value,
            lpf_alpha=p("lpf_alpha").value)

        reliable = p("use_reliable_qos").value
        qos = QoSProfile(depth=1, reliability=(ReliabilityPolicy.RELIABLE if reliable
                                               else ReliabilityPolicy.BEST_EFFORT))
        self.get_logger().info(f"QoS reliability: {'RELIABLE' if reliable else 'BEST_EFFORT'}")
        self.pub_pose = self.create_publisher(PoseStamped, "~/pose", qos)
        self.pub_detected = self.create_publisher(Bool, "~/detected", qos)
        self.pub_ambiguity = self.create_publisher(Float32, "~/ambiguity", qos)

        self.bridge = None
        if self.publish_debug_image:
            if CvBridge is None:
                self.get_logger().warn("Khong co cv_bridge -> tat publish_debug_image")
                self.publish_debug_image = False
            else:
                from sensor_msgs.msg import Image
                self.bridge = CvBridge()
                self.pub_image = self.create_publisher(Image, "~/image", qos)

        self.n_frame = 0
        self.n_detected = 0
        self.create_timer(1.0 / p("publish_rate").value, self.on_timer)
        self.get_logger().info(
            f"San sang: marker {self.marker_size * 100:.1f}cm, {dict_name}, "
            f"target_id={self.target_id}, frame_id '{self.frame_id}'")

    @staticmethod
    def _default_calib_path():
        try:
            from ament_index_python.packages import get_package_share_directory
            return os.path.join(get_package_share_directory("aruco_detection"),
                                "calib_data_mono.json")
        except Exception:
            return os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                "calib_data_mono.json")

    def _open_camera(self, index, calib_size):
        self.cap = cv2.VideoCapture(index)
        if not self.cap.isOpened():
            raise RuntimeError(f"Khong mo duoc camera index {index}")

        # cap.set() rat hay that bai im lang tren USB cam nen phai doc frame that
        # de kiem chung, khong thi moi khoang cach sai theo ti le do phan giai.
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, calib_size[0])
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, calib_size[1])
        ok, frame = self.cap.read()
        if not ok:
            raise RuntimeError("Khong doc duoc frame dau tien")

        frame_size = (frame.shape[1], frame.shape[0])
        if frame_size != calib_size:
            self.get_logger().warn(
                f"Camera tra ve {frame_size[0]}x{frame_size[1]} chu khong phai "
                f"{calib_size[0]}x{calib_size[1]} nhu luc calib")
        return frame_size

    def on_timer(self):
        t0 = time.perf_counter()
        ok, frame = self.cap.read()
        if not ok:
            return
        self.n_frame += 1


        stamp = self.get_clock().now().to_msg()
        
        msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        msg.header.stamp = stamp
        msg.header.frame_id = self.frame_id
        self.pub_image.publish(msg)

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
                    self.get_logger().warn(
                        f"Goc xoay khong tin cay ({det.ambiguity:.2f}): dung "
                        f"position, bo qua orientation",
                        throttle_duration_sec=2.0)
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
                y = viz.draw_detection(frame, det, self.camera_matrix,self.dist_coeffs, self.marker_size,
                                       y=y, ambiguity_warn=self.ambiguity_warn)
        else:
            viz.draw_no_marker(frame, y)
        frame_ms = (time.perf_counter() - t0) * 1000
        viz.draw_stats(frame, fps=1000.0 / max(frame_ms, 1e-6), frame_ms=frame_ms,
                       detect_ms=detect_ms, detected=self.n_detected,
                       total=self.n_frame)

        msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        msg.header.stamp = stamp
        msg.header.frame_id = self.frame_id
        self.pub_image.publish(msg)

    def destroy_node(self):
        if getattr(self, "cap", None) is not None:
            self.cap.release()
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
