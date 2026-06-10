from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import (
    PythonLaunchDescriptionSource,
    AnyLaunchDescriptionSource,
)
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # ---- Launch arguments -------------------------------------------------
    # FCU connection URL. Default = REAL drone over the PX4 USB serial link
    # (/dev/ttyACM0 @ 57600). For PX4 SITL instead, override on the CLI:
    #   ros2 launch lora_drone_package bringup.launch.py \
    #       fcu_url:=udp://:14540@127.0.0.1:14557
    fcu_url = LaunchConfiguration('fcu_url')
    fcu_url_arg = DeclareLaunchArgument(
        'fcu_url',
        default_value='serial:///dev/ttyACM0:57600',
        description='MAVROS FCU URL (serial:///dev/ttyACM0:57600 for real FCU, udp://... for SITL)',
    )

    # ---- MAVROS (PX4) -----------------------------------------------------
    # px4.launch is an XML launch file, so use AnyLaunchDescriptionSource
    # (PythonLaunchDescriptionSource only loads .py launch files).
    mavros_pkg_share = get_package_share_directory('mavros')
    mavros_launch = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            os.path.join(mavros_pkg_share, 'launch', 'px4.launch')
        ),
        launch_arguments={'fcu_url': fcu_url}.items(),
    )

    # ---- offboard_control (test_mode) -------------------------------------
    offboard_pkg_share = get_package_share_directory('offboard_control')
    offboard_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(offboard_pkg_share, 'launch', 'offboard_launch.py')
        )
    )

    # ---- LoRa bridge (Python node) ---------------------------------------
    lora_drone_node = Node(
        package='lora_drone_package',
        executable='lora_drone',   # trùng console_scripts trong setup.py
        name='lora_drone',
        output='screen',
    )

    return LaunchDescription([
        fcu_url_arg,
        mavros_launch,      # bring MAVROS up first so the FCU link is ready
        offboard_launch,
        lora_drone_node,
    ])
