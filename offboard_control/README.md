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
| Subscribe | `/mavros/global_position/raw/fix` | `sensor_msgs/NavSatFix` | GPS fix — pre-takeoff EKF gate |
| Subscribe | `/mavros/battery` | `sensor_msgs/BatteryState` | Voltage (safety) |
| Subscribe | `/mavros/extended_state` | `mavros_msgs/ExtendedState` | Landed/in-air status |
| Publish | `/mavros/setpoint_position/local` | `geometry_msgs/PoseStamped` | Position setpoint (20 Hz) |
| Client | `/mavros/cmd/arming` | `mavros_msgs/srv/CommandBool` | Arm / disarm |
| Client | `/mavros/set_mode` | `mavros_msgs/srv/SetMode` | OFFBOARD / AUTO.LAND |

---

## Parameters

| Param | Default | Description |
|---|---|---|
| `takeoff_height_` | `5.0` m | Takeoff altitude for an **OFFBOARD hover**. A **mission** instead takes off to its first waypoint's altitude. |
| `takeoff_delay_sec` | `5.0` s | Setpoint prime time before entering OFFBOARD |
| `hover_seconds` | `10.0` s | Hover duration when no mission (≤ 0 = hold until LAND command) |
| `arm_timeout_sec` | `15.0` s | Abort if ARM not achieved within this time |
| `climb_timeout_sec` | `60.0` s | Emergency land if takeoff altitude not reached within this time after ARM |
| `alt_reached_tol` | `0.5` m | "Altitude reached" when `z ≥ target − tol` (matches real hover droop; 0.1 m was unreachable) |
| `ekf_gate_enable` | `true` | Pre-takeoff EKF gate: require GPS 3D fix + stable local-z before ARM. `false` = arm immediately (bench/GPS-denied) |
| `ekf_z_stable_thresh` | `0.30` m | local-z must hold within ±this of a reference… |
| `ekf_z_stable_sec` | `3.0` s | …for this long before the EKF is considered settled |
| `min_battery_` | `13.2` V | Low-battery threshold (3.3 V/cell, 4S) |
| `low_batt_hold_sec` | `5.0` s | Voltage must stay below threshold for this long before emergency land |
| `wp_reach_radius` | `1.5` m | Waypoint reached radius |
| `wp_timeout_sec` | `30.0` s | **Final** waypoint only: land if not reached within this time. Intermediate WPs are never skipped by time (every WP is visited). |
| `cruise_speed` | `2.0` m/s | Carrot advance speed along the mission path |
| `carrot_lead` | `2.5` m | Max carrot lead distance ahead of the drone |
| `setpoint_rate_hz` | `20.0` Hz | Setpoint stream rate (must be > 2 Hz for PX4 OFFBOARD) |
| `mission_log_file` | `$HOME/mission_debug.log` | Flight event log file |

---

## Log format

Events are appended to `mission_log_file`:

```
MISSION_RX: received 5 waypoints
EKF gate WAIT: z settling 1.2/3.0s (z=0.05 ref=0.04 drift=0.01m)
EKF gate passed (GPS 3D fix + stable local-z z=0.05m) -> proceeding to takeoff
ARMED
CLIMB: z=2.10/6.00 (gap=3.90m) mode=OFFBOARD armed=1 t=4/60s
ALTITUDE reached -> deciding mission vs hover
DECISION: FLY MISSION (5 waypoints)
MISSION WP1/5: drone(1.2,3.4,6.0) carrot(2.1,4.0,6.0) dist=0.95 mode=OFFBOARD
WP 1/5 reached -> next
MISSION COMPLETE at WP5 -> landing
LANDED & disarmed -> idle
```

The `CLIMB:`/`MISSION:` lines are throttled (~2 s); their `mode=` column makes an external
override (e.g. `POSCTL`) obvious. `EKF gate WAIT:` lines show why takeoff is being held (GPS vs z).

The launch file sets `mission_log_file` to `/home/drone_ws/mission_debug.log` (Docker bind-mount → visible on host and laptop via sshfs).

---

## Safety

- **EKF gate (pre-takeoff):** won't ARM until a fresh GPS 3D fix + stable local-z (`ekf_z_stable_thresh` for `ekf_z_stable_sec`). Prevents the "arm before EKF converges → never reaches altitude → land" failure. Latched so it never re-trips mid-flight. Disable with `ekf_gate_enable:=false`.
- **Altitude timeout:** no altitude reached within `climb_timeout_sec` (60 s) after ARM → emergency land. "Reached" = within `alt_reached_tol` (0.5 m) of target.
- **Low battery (debounced):** voltage < `min_battery_` for ≥ `low_batt_hold_sec` continuously → emergency land. Transient sag on takeoff does not trigger.
- **Stale FCU / pose:** setpoint stream stops; PX4 exits OFFBOARD automatically.
- **External override:** PX4 leaves OFFBOARD for > 3 s → ABORT → MANUAL.

---

*Last updated: 2026-06-11*
