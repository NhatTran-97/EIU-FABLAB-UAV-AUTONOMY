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

                # --- Pre-takeoff EKF gate (see ekf_ready in offboard_control.cpp) ---
                # Don't arm/climb until the estimator is trustworthy: requires a GPS 3D fix
                # AND local-z holding within ekf_z_stable_thresh (m) for ekf_z_stable_sec (s).
                # This is the fix for "takeoff, hover, land — never flies the mission", caused
                # by arming before the EKF converges. Set ekf_gate_enable False to disable.
                'ekf_gate_enable': True,
                'ekf_z_stable_thresh': 0.30,
                'ekf_z_stable_sec': 3.0,
                # Altitude-reach timeout after ARM (s). Was a hard-coded 30s; raised to 60s
                # so a slow-but-fine climb isn't aborted prematurely.
                'climb_timeout_sec': 60.0,
                # "Reached altitude" tolerance (m). The drone hovers ~0.2-0.4m below the
                # commanded altitude (normal position-hold droop), so a 0.1m gate was never
                # satisfied -> climb, hover short, timeout. 0.5m = real droop + safe margin.
                'alt_reached_tol': 0.5,
            }],
        )

    return LaunchDescription([
        offboard_node

    ])
