from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package="aruco_detection",
            executable="aruco_node.py",
            name="aruco_pose_node",
            output="screen",
            parameters=[{
                "camera_source": "2",       # USB index, rtsp:// URL, or file path
                "threaded_capture": True,   # background reader, never blocks the executor
                "calib_side": "left",
                "marker_size": 0.267,        # meters, outer black edge
                "dictionary": "DICT_4X4_50",
                "target_id": 1,              # -1 accepts any marker
                "frame_id": "camera_optical_frame",
                "publish_rate": 30.0,
                "use_ssr": False,
                "lpf_alpha": 1.0,            # 1.0 = no smoothing
                "use_reliable_qos": False,   # True to debug with ros2 topic hz
                "publish_debug_image": False,
            }],
        ),
    ])
