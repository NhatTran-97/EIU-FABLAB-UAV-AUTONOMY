import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(get_package_share_directory('drone_telemetry'),
                          'config', 'telemetry.yaml')

    telemetry_node = Node(
        package='drone_telemetry',
        executable='telemetry_node.py',
        name='drone_telemetry',
        output='screen',
        parameters=[config],
        respawn=True,
        respawn_delay=2.0,
    )

    return LaunchDescription([telemetry_node])
