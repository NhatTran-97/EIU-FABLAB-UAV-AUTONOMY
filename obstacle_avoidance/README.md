# obstacle_avoidance

Obstacle avoidance for a PX4 drone with a 360° 2D lidar. The node runs VFH+ on every scan and **proposes** a safe setpoint on `/avoidance/status`. It never commands PX4: `mission_manager` decides whether to use the setpoint and stays the only node that writes `/mission/target_position`.

```
/scan, PX4 pose + attitude ──► obstacle_avoidance ──/avoidance/status──► mission_manager ──► offboard_control ──► PX4
                                       ▲                                       │
                                       └──── /mission/goal, /avoidance/enable ─┘
```

The package also contains the driver for the STL-50B2 lidar (`stl_50b2_node`, C++), which publishes `/scan`.

More detail:
- **[docs/BEHAVIOR.md](docs/BEHAVIOR.md)**: what the drone does when someone stands in its way, when it is boxed in, and while hovering.
- **[docs/ARCHITECTURE.md](docs/ARCHITECTURE.md)**: modules, the VFH+ steps, speed limits, timing and the interface with `mission_manager`.

## Features

| Feature | Where | Default |
|---|---|---|
| Avoidance during mission legs: the drone goes around obstacles on the way to each waypoint | Start mission with ☑ Obstacle avoidance on the ground station | off (`allow_avoidance: false` in `mission_manager`) |
| Speed limited by the stopping distance, from the measured lidar latency | `obstacle_avoidance` | always on while avoiding |
| Hold when boxed in, stuck, or without fresh lidar data; then hold for the pilot or land | `mission_manager` (`blocked_action`) | `hold` |
| Keep distance while hovering: an obstacle closer than X m moves the drone away | `mission_manager` (`hover_keep_distance`) | off (`0.0`) |
| PX4 Collision Prevention: stops the pilot from flying into obstacles in Position mode | `publish_obstacle_distance` + PX4 `CP_DIST` | off |

## Build

```bash
conda deactivate                       # colcon needs the system Python with catkin_pkg
cd ~/drone_ws
colcon build --packages-select custom_msgs obstacle_avoidance offboard_control
source install/setup.bash
```

## Use

**1. Start the lidar and the avoidance node** on the Jetson:

```bash
ros2 launch obstacle_avoidance stl_50b2.launch.py      # lidar -> /scan
ros2 launch obstacle_avoidance avoidance_launch.py     # VFH+ -> /avoidance/status
```

**2. Allow avoidance in `mission_manager`** (`offboard_control/launch/px4_offboard.launch.py`):

```python
'allow_avoidance': True,
'hover_keep_distance': 2.0,    # optional: keep 2 m away from obstacles while hovering; 0 = off
```

**3. Fly:** Take-off, Send WP, then Start mission with ☑ Obstacle avoidance. Without the tick the mission flies straight lines and the lidar is not used.

**4. Watch it work:**

```bash
ros2 topic echo /avoidance/status      # state, safe_setpoint, min_distance, speed_limit
ros2 topic echo /mission/nav_mode      # NOMINAL / AVOIDING / HOLDING
```

Every `diagnostics_period` seconds the node logs a `[diag]` line with the measured rate of each input, the latency, and the speeds it allows (`v_open` in free space, `v@block` at `block_distance`).

**Optional, PX4 Collision Prevention:** set `publish_obstacle_distance: true` in `config/avoidance.yaml`, then set `CP_DIST` (for example 1.5 m) in QGroundControl. PX4 then refuses to let the pilot fly into an obstacle in Position mode. It does not push the drone away, and it has no effect in OFFBOARD.

## Configuration

Everything is in `config/avoidance.yaml`. The values that must match the hardware:

| Parameter | Current | Must match |
|---|---|---|
| `lidar_yaw_offset_deg`, `lidar_upside_down` | `0`, `false` | how the lidar is mounted (here: facing the nose) |
| `self_mask_deg` | empty | angles hidden by the frame or legs |
| `max_range` | `12.0` m | the driver's `range_max`; VFH+ only uses points within `window_radius` (8 m) |
| `drone_radius` | `0.5` m | half the propeller-tip diagonal plus a little margin (80 cm diagonal → 0.4 m) |
| `px4_xy_p` | `0.95` | PX4 `MPC_XY_P` |
| `max_decel` | `2.0` m/s² | at most PX4 `MPC_ACC_HOR` |
| `max_speed` | `2.0` m/s | at most PX4 `MPC_XY_VEL_MAX` |

The PX4 values are in QGroundControl (Vehicle Setup → Parameters, search the name) or in the MAVLink console (`param show MPC_XY_P`). Rates and timeouts are not configured: the node measures them while running.

## Tests

```bash
cd ~/drone_ws/src/obstacle_avoidance && python3 -m pytest test/
```

The tests are pure numpy (no ROS): VFH+ decisions, the lidar → NED conversion, timing, speed limits, and a closed-loop simulation with latency and limited acceleration.

## Before the first real flight

1. **Static frame test** (the most dangerous error): hold an object in front of the nose and echo `/avoidance/status`. `min_distance_bearing` must be close to the drone heading. Move the object to the **left**: the bearing must **decrease** by 90° (NED turns clockwise). Otherwise fix `lidar_yaw_offset_deg` / `lidar_upside_down`.
2. Look at `/scan` with the drone on the ground and mask the beams that hit the legs or frame (`self_mask_deg`).
3. SITL with a 2D lidar: a pole, a wall, a corridor and a U-shaped trap. The U trap must end in HOLD.
4. Check that `px4_xy_p`, `max_decel` and `max_speed` match PX4, and read the `[diag]` line for plausible rates and latency.

## Known limits

- A 2D lidar only sees obstacles in its scan plane. Wires or branches at another height are invisible.
- VFH+ is reactive and has no map, so it can get stuck in a local minimum. `mission_manager` detects this (no progress for `stuck_timeout`) and holds, but does not find a way out.
- The lidar offset from the drone centre is not compensated. If it is more than 10 cm, add it to `drone_radius`.
- The STL-50B2 driver reports `scan_time = 0.15 s`; the node measures the real scan period (7–10 Hz) and uses that.
