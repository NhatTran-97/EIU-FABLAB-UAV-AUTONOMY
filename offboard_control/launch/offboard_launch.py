from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    offboard_pkg = get_package_share_directory("offboard_control")

    offboard_node = Node(
            package='offboard_control',
            executable='test_mode',
            name='test_mode',
            output='screen',
            # parameters=[os.path.join(offboard_pkg, "config", "offboard.yaml")],
        )

    return LaunchDescription([
        offboard_node

    ])
