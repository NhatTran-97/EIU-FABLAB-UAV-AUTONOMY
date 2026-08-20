from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    camera_source = LaunchConfiguration("camera_source")
    detector_type = LaunchConfiguration("detector_type")
    marker_size = LaunchConfiguration("marker_size")
    dictionary = LaunchConfiguration("dictionary")
    fractal_config = LaunchConfiguration("fractal_config")
    target_id = LaunchConfiguration("target_id")

    return LaunchDescription([
        DeclareLaunchArgument("camera_source", default_value="2"),
        DeclareLaunchArgument("detector_type", default_value="aruco"),
        DeclareLaunchArgument("marker_size", default_value="0.267"),
        DeclareLaunchArgument("dictionary", default_value="DICT_4X4_50"),
        DeclareLaunchArgument("fractal_config", default_value="FRACTAL_5L_6"),
        DeclareLaunchArgument("target_id", default_value="1"),
        Node(
            package="aruco_detection",
            executable="aruco_node.py",
            name="aruco_pose_node",
            output="screen",
            parameters=[{
                "camera_source": ParameterValue(camera_source, value_type=str),
                "threaded_capture": True,   # background reader, never blocks the executor
                "calib_side": "left",
                "marker_size": ParameterValue(marker_size, value_type=float),
                "detector_type": detector_type,
                "dictionary": dictionary,
                "fractal_config": fractal_config,
                "target_id": ParameterValue(target_id, value_type=int),
                "frame_id": "camera_optical_frame",
                "publish_rate": 30.0,
                "use_ssr": False,
                "use_reliable_qos": False,   # True to debug with ros2 topic hz
                "publish_debug_image": False,
            }],
        ),
    ])
