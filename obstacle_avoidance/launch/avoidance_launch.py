import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(get_package_share_directory('obstacle_avoidance'),
                          'config', 'avoidance.yaml')

    avoidance_node = Node(
        package='obstacle_avoidance',
        executable='avoidance_node.py',
        name='obstacle_avoidance',
        output='screen',
        parameters=[config],
    )

    return LaunchDescription([avoidance_node])
