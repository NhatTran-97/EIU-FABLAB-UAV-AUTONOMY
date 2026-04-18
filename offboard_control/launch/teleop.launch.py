from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    config = get_package_share_directory("offboard_control")

    teleop_node = Node(
            package='offboard_control',
            executable='teleop_executor',
            name='teleop_executor',
            output='screen',
            parameters=[os.path.join(config, "config", "params.yaml")],
        )

    return LaunchDescription([
        teleop_node

    ])
