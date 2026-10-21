# offboard_control

Autonomous flight on the Jetson for a PX4 drone, using OFFBOARD mode over uXRCE-DDS (`px4_msgs`). The package contains two Python nodes:

- **`mission_manager`** decides what to fly. It receives commands from the ground station, runs the flight state machine and the mission pipeline, and sends targets.
- **`offboard_control`** is the only node that talks to PX4. It streams setpoints, requests OFFBOARD and ARM, sends LAND, handles a pilot override and reports the vehicle state back.

The drone never takes off on its own: every flight starts from a ground station command, at the drone's current position.

```
GCS --433--> drone_telemetry --/gcs/command--> mission_manager --/mission/*--> offboard_control --> PX4
```

Diagrams, state machines and interfaces: **[docs/ARCHITECTURE.md](docs/ARCHITECTURE.md)**. Mission steps, pick tasks and the manual hand-over: **[docs/MISSION_PIPELINE.md](docs/MISSION_PIPELINE.md)**.

## What it does

| Command (from the GCS) | Result |
|---|---|
| Take-off *alt* | climbs vertically to *alt* metres at the current position, then hovers |
| Start mission | flies the uploaded waypoints and their tasks, then hovers at the last waypoint |
| Start mission + ☑ Obstacle avoidance | the same, with every leg flown through the obstacle_avoidance node |
| Continue | ends a manual task at a waypoint, or resumes after the pilot took control |
| Mission speed / tolerance changed | applied at once to the running or paused mission |
| Land | lands in place; both nodes return to idle on touchdown |

Each waypoint can carry a task. **Pick** descends to a search height, finds the ArUco marker, aligns over it and descends to the hook height, then hooks the load automatically (payload node) or hands over to the pilot, and finally climbs back to the waypoint altitude. If the marker is not found or the automatic hook fails, the pilot is asked to take over instead of the mission stopping.

If the pilot moves the sticks during a mission, the mission pauses. It resumes from where the drone is when the pilot flips the RC switch back to Offboard or the operator presses Continue.

Commands that do not fit the current state, or that need a node that is not running, are refused with a reason.

## Parameters

Values set in `launch/px4_offboard.launch.py`; every other parameter keeps its default from the node.

| Node | Parameter | Value | Meaning |
|---|---|---|---|
| mission_manager | `takeoff_altitude` | `-5.0` | NED altitude used when the command has none |
| | `max_takeoff_altitude` | `15.0` m | higher take-off or waypoint altitudes are rejected; keep below the PX4 geofence `GF_MAX_VER_DIST` |
| | `start_timeout` | `3.0` s | time for `offboard_control` to start after `/mission/start` |
| | `position_timeout` | `1.0` s | maximum age of the position for Take-off |
| | `cruise_speed` | `2.0` m/s | speed between waypoints when the ground station sends none |
| | `max_cruise_speed` | `5.0` m/s | upper limit for the mission speed chosen on the ground station |
| | `xy_tolerance`, `z_tolerance` | `0.75`, `0.5` m | arrival tolerances when the ground station sends none (it can set them at start and during the mission) |
| | `yaw_mode` | `waypoint` | nose towards the next waypoint; `hold` keeps the take-off heading |
| | `yaw_rate_deg` | `45` deg/s | maximum commanded turn rate |
| | `turn_first_deg` | `10` deg | at each waypoint, a larger heading error makes the drone turn in place before moving |
| | `allow_avoidance` | `false` | safety switch: missions asking for obstacle avoidance are refused while false |
| | `hover_keep_distance` | `0.0` m | while hovering, an obstacle closer than this moves the drone away (0.5 m/s, at most `hover_max_drift` 3 m); needs `allow_avoidance` and the avoidance node; 0 = off |
| | `marker_topic` | `/marker_localizer_node/marker_rel` | marker offset (PointStamped, ENU) for pick tasks |
| | `payload_service` | `/payload/hook` | `std_srvs/Trigger` server for the automatic hook |
| offboard_control | `setpoint_rate_hz` | `10.0` Hz | setpoint stream rate (PX4 needs > 2 Hz) |
| | `offboard_warmup_s` | `1.0` s | streaming time before OFFBOARD + ARM |
| | `arm_timeout_s` | `3.0` s | time for PX4 to arm before the take-off is aborted |
| | `velocity_timeout_s` | `0.5` s | velocity setpoints older than this fall back to holding position |

Pick heights, tolerances and timeouts are listed in [docs/MISSION_PIPELINE.md](docs/MISSION_PIPELINE.md).

### PX4 parameters to check

| Parameter | Why |
|---|---|
| `MPC_XY_VEL_MAX`, `MPC_Z_VEL_MAX_UP`, `MPC_Z_VEL_MAX_DN` | hard speed limits, also in OFFBOARD |
| `COM_RC_OVERRIDE` (offboard bit) | moving the sticks gives control to the pilot immediately |
| an RC mode-switch position set to Offboard | lets the pilot hand control back after a manual task |
| `COM_OF_LOSS_T`, `COM_OBL_RC_ACT` | what PX4 does if the setpoint stream stops |

## Build and run

Requires `px4_msgs` (matching the firmware), `custom_msgs`, `std_srvs`, and MicroXRCEAgent connected to PX4. Pick tasks also need a rangefinder in PX4 (`dist_bottom`) and the ArUco localizer.

```bash
cd ~/drone_ws && colcon build --packages-select offboard_control
source install/setup.bash
ros2 launch offboard_control px4_offboard.launch.py
```

`drone_telemetry` must run with `commands.takeoff_land_via: mission` to forward ground station commands. `launch/offboard_launch.py` is kept as a legacy name and simply includes `px4_offboard.launch.py`.

Quick test without the ground station (propellers removed):

```bash
ros2 topic pub --once /gcs/command std_msgs/msg/String '{data: "{\"id\": 1, \"cmd\": \"takeoff\", \"alt\": 2.0}"}'
ros2 topic echo /gcs/command_ack
ros2 topic echo /mission/state
ros2 topic pub --once /gcs/command std_msgs/msg/String '{data: "{\"id\": 2, \"cmd\": \"land\"}"}'
```

## Tests

```bash
cd ~/drone_ws/src/offboard_control
python3 -m pytest test/test_mission_pipeline.py                # pipeline, no ROS
python3 -m pytest test/test_mission_manager_node.py            # node, needs a sourced ROS 2 workspace
python3 -m pytest test/test_offboard_control_node.py           # node, needs px4_msgs
```

## Safety

- Nothing flies on startup; every flight starts from an explicit command at the current position.
- Take-off needs a valid, fresh EKF position; GPS missions need the EKF GPS origin.
- If the setpoint stream stops, PX4 leaves OFFBOARD and applies its failsafe.
- The RC transmitter always has priority: moving the sticks pauses the mission, the kill switch ends the flight.
- Pick tasks fly close to the ground: test them in SITL first, then with the propellers removed.
