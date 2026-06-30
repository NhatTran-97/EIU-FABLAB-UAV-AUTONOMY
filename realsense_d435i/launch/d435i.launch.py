from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('realsense_d435i'),
        'config', 'd435i_params.yaml')

    return LaunchDescription([
        Node(
            package='realsense_d435i',
            executable='d435i_node',
            name='d435i_driver',
            parameters=[config],
            output='screen'
        )
    ])
