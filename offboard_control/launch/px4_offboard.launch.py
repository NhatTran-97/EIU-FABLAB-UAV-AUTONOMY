"""Launch the OFFBOARD flight nodes: mission_manager and offboard_control.

Command flow from the ground station:

    GCS --433--> drone_telemetry --/gcs/command--> mission_manager --/mission/*--> offboard_control --> PX4
                                 <-/gcs/command_ack-

mission_manager waits for commands and never takes off on its own. Take-off climbs in place to the altitude
entered on the GCS and hovers; Land lands and returns to IDLE. Requires MicroXRCEAgent and drone_telemetry
(commands.takeoff_land_via: mission).
"""
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    mission_manager = Node(
        package='offboard_control',
        executable='mission_manager.py',
        name='mission_manager',
        output='screen',
        parameters=[{
            'takeoff_altitude': -5.0,       # NED (negative = up); used when the command has no altitude
            # m; higher requests are rejected. Keep below the GF_MAX_VER_DIST geofence (outdoor.params: 20 m,
            # GF_ACTION = Hold), otherwise PX4 interrupts the flight with Hold.
            'max_takeoff_altitude': 15.0,
            'start_timeout': 3.0,           # s for offboard_control to start after /mission/start
            'position_timeout': 1.0,        # s; reject Take-off if the position is older
            'cruise_speed': 2.0,            # m/s between waypoints when the GCS sends no speed
            'max_cruise_speed': 5.0,        # m/s; speeds chosen on the GCS are limited to this
                                            # (PX4 MPC_XY_VEL_MAX stays the hard limit)
            'yaw_mode': 'waypoint',         # nose towards the next waypoint ('hold': keep take-off heading)
            'yaw_rate_deg': 45.0,           # deg/s, maximum commanded turn rate
            # Obstacle avoidance is chosen per mission on the GCS; false rejects those missions until the
            # obstacle_avoidance node has been tested.
            'allow_avoidance': False,
            'marker_topic': '/marker_localizer_node/marker_rel',   # ArUco offset for pick tasks
            'payload_service': '/payload/hook',                    # std_srvs/Trigger, automatic hook
        }],
    )
    offboard_control = Node(
        package='offboard_control',
        executable='offboard_control.py',
        name='offboard_control_node',
        output='screen',
        parameters=[{
            'setpoint_rate_hz': 10.0,
            'offboard_warmup_s': 1.0,       # s of setpoint streaming before requesting OFFBOARD + ARM
            'arm_timeout_s': 3.0,           # s to wait for PX4 to arm before aborting take-off
            'velocity_timeout_s': 0.5,      # s; velocity setpoints older than this -> hold position
            # PX4 topic names are derived from px4_msgs MESSAGE_VERSION; set *_topic only to override.
        }],
    )
    return LaunchDescription([mission_manager, offboard_control])
