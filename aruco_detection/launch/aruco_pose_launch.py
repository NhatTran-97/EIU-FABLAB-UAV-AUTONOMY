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
                "camera_index": 2,
                "calib_side": "left",
                "marker_size": 0.267,        # met, canh den ngoai cung
                "dictionary": "DICT_4X4_50",
                "target_id": 1,              # -1 = nhan moi marker
                "frame_id": "camera_optical_frame",
                "publish_rate": 30.0,
                "use_ssr": False,
                "use_lpf": False,
                "publish_debug_image": False,
            }],
        ),
    ])
