from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('skydroid'),
        'config',
        'camera.yaml'
    )

    camera_node = Node(
        package='skydroid',
        executable='rtsp_cam',
        name='camera_node',
        output='screen',
        parameters=[config],
    )

    return LaunchDescription([camera_node])
