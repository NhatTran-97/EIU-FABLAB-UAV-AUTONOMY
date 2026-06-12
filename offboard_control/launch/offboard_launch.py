from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    offboard_pkg = get_package_share_directory("offboard_control")

    offboard_node = Node(
            package='offboard_control',
            executable='offboard_control',
            name='offboard_control',
            output='screen',
            # parameters=[os.path.join(offboard_pkg, "config", "offboard.yaml")],
            parameters=[{
                # Write the mission/flight debug log into the bind-mounted workspace
                # (/home/drone_ws inside the container == /home/fablab01/drone_ws on the
                # Jetson host == ~/nhatbot_remote on the laptop via sshfs). Default would be
                # $HOME/mission_debug.log = /root/... inside the container — invisible on host.
                'mission_log_file': '/home/drone_ws/mission_debug.log',
            }],
        )

    return LaunchDescription([
        offboard_node

    ])
