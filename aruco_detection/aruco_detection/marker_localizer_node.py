#!/usr/bin/env python3
"""Turn the camera-frame marker pose into a tilt-free position for MAVROS.

Giai doan 1 cua chuoi ha canh chinh xac. Node nay CHI lam bien doi he toa do
va dong bo thoi gian -- khong co bo dieu khien, khong co state machine, khong
publish setpoint. Nho vay no chay lai duoc tu rosbag ma khong can drone.

Subscribes
    <marker_pose_topic>  geometry_msgs/PoseStamped  tvec trong he quang hoc
    attitude, mot trong hai (tham so attitude_source):
      local_position     geometry_msgs/PoseStamped  /mavros/local_position/pose
      imu                sensor_msgs/Imu            /mavros/imu/data

Publishes
    ~/marker_rel    geometry_msgs/PointStamped  marker so voi drone, ENU, da khu nghieng
    ~/marker_pose   geometry_msgs/PoseStamped   marker trong he local ENU cua MAVROS
    ~/height        std_msgs/Float32            khoang cach THANG DUNG drone-marker [m]
    ~/valid         std_msgs/Bool               co pose dung khong

Chay:
    # Ngoai troi / co uoc luong vi tri
    ros2 run aruco_detection marker_localizer_node.py

    # Trong nha, khong GPS -- du de hieu chuan goc lap camera
    ros2 run aruco_detection marker_localizer_node.py --ros-args \
        -p attitude_source:=imu
"""
import math
import time
from collections import deque

import rclpy
from geometry_msgs.msg import PointStamped, PoseStamped
from sensor_msgs.msg import Imu
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, Float32

from aruco_detection.geometry import (CameraExtrinsic, marker_relative_enu,
                                      marker_yaw_enu, quat_to_rotmat)


def stamp_to_sec(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


class MarkerLocalizer(Node):
    def __init__(self):
        super().__init__("marker_localizer")

        self.declare_parameter("marker_pose_topic", "/aruco_pose_node/pose")
        self.declare_parameter("mavros_pose_topic", "/mavros/local_position/pose")
        self.declare_parameter("imu_topic", "/mavros/imu/data")
        # "local_position" can EKF co uoc luong vi tri -- trong nha khong GPS
        # thi topic do co the im lang hoan toan.
        # "imu" chi lay attitude, coi vi tri drone la goc. Du cho ~/marker_rel
        # va ~/height (hai thu quan trong nhat), nhung ~/marker_pose khi do la
        # TUONG DOI so voi drone chu khong con la toa do tuyet doi.
        self.declare_parameter("attitude_source", "local_position")
        self.declare_parameter("map_frame_id", "map")
        self.declare_parameter("level_frame_id", "base_link_level")

        # Sai so lap camera, do trong he BODY FLU. Lech 2 deg o 5 m = 17 cm.
        self.declare_parameter("mount_roll_deg", 0.0)
        self.declare_parameter("mount_pitch_deg", 0.0)
        self.declare_parameter("mount_yaw_deg", 0.0)
        # Vi tri camera so voi CG, body FLU [m]: x truoc, y trai, z len
        self.declare_parameter("cam_offset", [0.0, 0.0, 0.0])

        # Do tre tu luc chup den luc pose duoc publish. Chua do duoc thi de 0;
        # o 2 m/s moi 100 ms bo qua la 20 cm sai vi tri.
        self.declare_parameter("camera_latency_s", 0.0)
        # Attitude cu hon nguong nay thi bo pose -- khu nghieng bang attitude sai
        # con te hon la khong khu.
        self.declare_parameter("max_attitude_age_s", 0.15)
        self.declare_parameter("attitude_buffer_s", 2.0)
        # Mat marker thi aruco_node ngung publish, va node nay se im lang theo
        # neu khong co dong ho rieng -- ~/valid ket o True mai mai.
        self.declare_parameter("marker_timeout_s", 0.5)
        self.declare_parameter("use_reliable_qos", False)

        p = self.get_parameter
        self.map_frame_id = p("map_frame_id").value
        self.level_frame_id = p("level_frame_id").value
        self.camera_latency = float(p("camera_latency_s").value)
        self.max_attitude_age = float(p("max_attitude_age_s").value)
        self.buffer_s = float(p("attitude_buffer_s").value)
        self.marker_timeout = float(p("marker_timeout_s").value)

        offset = list(p("cam_offset").value)
        self.extrinsic = CameraExtrinsic(mount_roll=float(p("mount_roll_deg").value),
                                         mount_pitch=float(p("mount_pitch_deg").value),
                                         mount_yaw=float(p("mount_yaw_deg").value), offset=offset)

        reliable = p("use_reliable_qos").value
        qos = QoSProfile(depth=10, reliability=(ReliabilityPolicy.RELIABLE if reliable
                                                else ReliabilityPolicy.BEST_EFFORT))

        # (t, (qx, qy, qz, qw), (x, y, z)) theo thu tu thoi gian tang dan
        self._attitude = deque()
        self._last_marker = None      # time.monotonic() cua pose hop le cuoi

        self.attitude_source = str(p("attitude_source").value).lower()

        if self.attitude_source == "imu":
            self.create_subscription(Imu, p("imu_topic").value, self.on_imu, qos)
            self.get_logger().warn( f"attitude_source=imu ({p('imu_topic').value}): ~/marker_rel va "
                "~/height dung, nhung ~/marker_pose la TUONG DOI so voi drone")
            
        elif self.attitude_source == "local_position":
            self.create_subscription(PoseStamped, p("mavros_pose_topic").value,self.on_mavros_pose, qos)
        else:
            raise RuntimeError("attitude_source phai la 'local_position' hoac " f"'imu' (nhan duoc: {self.attitude_source})")
        
        self.create_subscription(PoseStamped, p("marker_pose_topic").value,self.on_marker_pose, qos)
        self.pub_rel = self.create_publisher(PointStamped, "~/marker_rel", qos)
        self.pub_pose = self.create_publisher(PoseStamped, "~/marker_pose", qos)
        self.pub_height = self.create_publisher(Float32, "~/height", qos)
        self.pub_valid = self.create_publisher(Bool, "~/valid", qos)

        self.create_timer(self.marker_timeout / 2.0, self._check_marker_fresh)

        self.get_logger().info(
            f"MarkerLocalizer ENU/FLU | mount rpy=("
            f"{p('mount_roll_deg').value}, {p('mount_pitch_deg').value}, "
            f"{p('mount_yaw_deg').value}) deg | offset={offset} m | "
            f"latency={self.camera_latency * 1000:.0f} ms")

    # ------------------------------------------------------------------
    def on_mavros_pose(self, msg):
        q = msg.pose.orientation
        pos = msg.pose.position
        self._push_attitude(stamp_to_sec(msg.header.stamp),
                            (q.x, q.y, q.z, q.w), 
                            (pos.x, pos.y, pos.z))
        

    def _push_attitude(self, t, q_xyzw, drone_pos):
        self._attitude.append((t, q_xyzw, drone_pos))

        cutoff = t - self.buffer_s
        while self._attitude and self._attitude[0][0] < cutoff:
            self._attitude.popleft()
        

    def on_imu(self, msg):
        q = msg.orientation
        self._push_attitude(stamp_to_sec(msg.header.stamp),(q.x, q.y, q.z, q.w), (0.0, 0.0, 0.0))

    def _lookup(self, t):
        """Mau attitude gan t nhat, hoac None neu qua cu / chua co."""
        if not self._attitude:
            return None
     
        best = min(self._attitude, key=lambda s: abs(s[0] - t))

        return best if abs(best[0] - t) <= self.max_attitude_age else None

    def on_marker_pose(self, msg):
        #Bu do tre: pose mang dau thoi diem XU LY, canh anh duoc CHUP som hon
        
        t_capture = stamp_to_sec(msg.header.stamp) #- self.camera_latency
        sample = self._lookup(t_capture)

        if sample is None:
            self.pub_valid.publish(Bool(data=False))
            self.get_logger().warn(
                "Khong co attitude khop timestamp -> bo pose " f"(buffer {len(self._attitude)} mau)", 
                throttle_duration_sec=2.0)
            return

        _, q_xyzw, drone_pos = sample

        t = msg.pose.position



        p_rel, _ = marker_relative_enu((t.x, t.y, t.z), q_xyzw, self.extrinsic)
        self._last_marker = time.monotonic()

        header_level = msg.header
        header_level.frame_id = self.level_frame_id
        rel = PointStamped()
        rel.header = header_level
        rel.point.x, rel.point.y, rel.point.z = map(float, p_rel)
        self.pub_rel.publish(rel)

        out = PoseStamped()
        out.header.stamp = msg.header.stamp
        out.header.frame_id = self.map_frame_id
        out.pose.position.x = float(drone_pos[0] + p_rel[0])
        out.pose.position.y = float(drone_pos[1] + p_rel[1])
        out.pose.position.z = float(drone_pos[2] + p_rel[2])
        # Huong cua MARKER, khong phai cua drone: phai di qua ca chuoi
        # camera -> than -> ENU. Chi lay yaw, bo phan nghieng vi ambiguity.
        o = msg.pose.orientation
        yaw = marker_yaw_enu(quat_to_rotmat(o.x, o.y, o.z, o.w),
                             q_xyzw, self.extrinsic)
        out.pose.orientation.z = math.sin(yaw / 2.0)
        out.pose.orientation.w = math.cos(yaw / 2.0)
        self.pub_pose.publish(out)

        # p_rel[2] am khi marker o duoi drone -> doi dau thanh do cao duong
        self.pub_height.publish(Float32(data=float(-p_rel[2])))
        self.pub_valid.publish(Bool(data=True))


    def _check_marker_fresh(self):
        """Khong co pose moi trong marker_timeout -> bao mat marker.
        ~/valid=False co HAI nguyen nhan khac han nhau, va bo dieu khien phai
        xu ly khac nhau: mat marker (khuat tam nhin, thuong tam thoi) va mat
        attitude (MAVROS co van de, nghiem trong hon nhieu).
        """
        if self._last_marker is None:
            return
        if time.monotonic() - self._last_marker > self.marker_timeout:
            self.pub_valid.publish(Bool(data=False))
            self.get_logger().warn("Mat marker", throttle_duration_sec=2.0)


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = MarkerLocalizer()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
