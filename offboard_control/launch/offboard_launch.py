"""Legacy launch name, kept for launch files that still include it (e.g. lora_drone_package/bringup.launch.py).

Includes px4_offboard.launch.py, which starts the Python nodes mission_manager and offboard_control and holds
their parameters. The C++ (MAVROS) version in src/ is no longer built.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    path = os.path.join(get_package_share_directory('offboard_control'), 'launch', 'px4_offboard.launch.py')
    return LaunchDescription([IncludeLaunchDescription(PythonLaunchDescriptionSource(path))])
