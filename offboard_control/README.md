# offboard_control

ROS2 C++ node — autonomous flight controller for a PX4 drone via MAVROS.

Two independent triggers, kept strictly separate:

- **OFFBOARD** (`/mode_signal`) → arm → takeoff → **hover** → land. Ignores any pending mission.
- **Mission** (`/mission_path`, while idle) → auto-starts: arm → takeoff → **fly waypoints** → land.

Position setpoints are streamed to PX4 at a stable rate on a dedicated thread.

Architecture details (modules, FSM, threading, safety): **[docs/ARCHITECTURE.md](docs/ARCHITECTURE.md)**

---

## Role in the system

```
Ground GUI ──LoRa──► lora_drone (Python) ──┬─ /mode_signal  (srv) ─► offboard_control ──► MAVROS ──► PX4
                                            └─ /mission_path (topic)─►
```

`offboard_control` is the sole node that talks to MAVROS. `lora_drone` only forwards commands; all flight decisions are made here.

The package is split into modules (see [docs/ARCHITECTURE.md](docs/ARCHITECTURE.md) §1): the node (`offboard_control`) orchestrates (FSM + ROS + MAVROS calls); `carrot_follower` is the pure waypoint-following geometry (no ROS, unit-testable); `setpoint_streamer` owns the >2 Hz streamer thread; `battery_monitor` is the pure battery sustained-low debounce (no ROS, unit-testable).

---

## Build

> Build on **Jetson Nano inside Docker** only — not on the laptop (missing MAVROS/PX4 headers).

```bash
# Inside container:
cd /home/drone_ws
colcon build --packages-select offboard_control

# Clean build (after renaming files or targets):
rm -rf build/offboard_control install/offboard_control
colcon build --packages-select offboard_control
```

> The executable is built from multiple source files (`offboard_control.cpp`, `carrot_follower.cpp`, `setpoint_streamer.cpp`, `battery_monitor.cpp`). New modules must be added to `add_executable(...)` in `CMakeLists.txt`.

---

## Run

```bash
# Full stack (MAVROS + offboard_control + lora_drone):
ros2 launch lora_drone_package bringup.launch.py

# Node only (MAVROS must be running):
ros2 launch offboard_control offboard_launch.py

# SITL:
ros2 launch lora_drone_package bringup.launch.py \
    fcu_url:=udp://:14540@127.0.0.1:14557
```

---

## ROS interfaces

| Direction | Name | Type | Description |
|---|---|---|---|
| Service (server) | `/mode_signal` | `custom_msgs/srv/ModeSignal` | OFFBOARD (hover flight) or LAND |
| Subscribe | `/mission_path` | `nav_msgs/Path` | Waypoint list — auto-starts a mission flight when idle. z = relative to takeoff ground. **VOLATILE QoS** (ignores latched/old missions at startup). |
| Subscribe | `/mavros/state` | `mavros_msgs/State` | PX4 mode + armed status |
| Subscribe | `/mavros/local_position/pose` | `geometry_msgs/PoseStamped` | EKF position |
| Subscribe | `/mavros/battery` | `sensor_msgs/BatteryState` | Voltage (safety) |
| Subscribe | `/mavros/extended_state` | `mavros_msgs/ExtendedState` | Landed/in-air status |
| Publish | `/mavros/setpoint_position/local` | `geometry_msgs/PoseStamped` | Position setpoint (20 Hz) |
| Client | `/mavros/cmd/arming` | `mavros_msgs/srv/CommandBool` | Arm / disarm |
| Client | `/mavros/set_mode` | `mavros_msgs/srv/SetMode` | OFFBOARD / AUTO.LAND |

---

## Parameters

| Param | Default | Description |
|---|---|---|
| `takeoff_height_` | `5.0` m | Takeoff altitude above ground |
| `takeoff_delay_sec` | `5.0` s | Setpoint prime time before entering OFFBOARD |
| `hover_seconds` | `10.0` s | Hover duration when no mission (≤ 0 = hold until LAND command) |
| `arm_timeout_sec` | `15.0` s | Abort if ARM not achieved within this time |
| `min_battery_` | `13.2` V | Low-battery threshold (3.3 V/cell, 4S) |
| `low_batt_hold_sec` | `5.0` s | Voltage must stay below threshold for this long before emergency land |
| `wp_reach_radius` | `1.5` m | Waypoint reached radius |
| `wp_timeout_sec` | `30.0` s | Skip waypoint if not reached within this time |
| `cruise_speed` | `2.0` m/s | Carrot advance speed along the mission path |
| `carrot_lead` | `2.5` m | Max carrot lead distance ahead of the drone |
| `setpoint_rate_hz` | `20.0` Hz | Setpoint stream rate (must be > 2 Hz for PX4 OFFBOARD) |
| `mission_log_file` | `$HOME/mission_debug.log` | Flight event log file |

---

## Log format

Events are appended to `mission_log_file`:

```
STATE: CLIMBING -> MISSION
MISSION_RX: received 5 waypoints
ARMED
ALTITUDE reached -> deciding mission vs hover
DECISION: FLY MISSION (5 waypoints)
WP 1/5 reached -> next
MISSION COMPLETE at WP5 -> landing
LANDED & disarmed -> idle
```

The launch file sets `mission_log_file` to `/home/drone_ws/mission_debug.log` (Docker bind-mount → visible on host and laptop via sshfs).

---

## Safety

- **Altitude timeout:** no altitude reached within 30 s after ARM → emergency land.
- **Low battery (debounced):** voltage < `min_battery_` for ≥ `low_batt_hold_sec` continuously → emergency land. Transient sag on takeoff does not trigger.
- **Stale FCU / pose:** setpoint stream stops; PX4 exits OFFBOARD automatically.
- **External override:** PX4 leaves OFFBOARD for > 3 s → ABORT → MANUAL.

---

*Last updated: 2026-06-11*
