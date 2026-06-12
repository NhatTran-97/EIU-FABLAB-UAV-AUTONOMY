# EIU-FABLAB-UAV-AUTONOMY

Autonomous UAV system built on ROS2 + PX4 MAVROS. The drone receives waypoint missions over a LoRa radio link from a PyQt6 ground station, executes them using a carrot-follower trajectory tracker, and streams live telemetry back to the operator.

> Developed by the EIU FabLab team for autonomous UAV research and education.

## Authors

| Name | Role |
|---|---|
| Tran Duy Nhat | Project Lead, UAV Autonomy, ROS2 Integration |
| Vo Pham Mai Uyen | MAVROS & Control Logic, Vision & Camera System |
| Vo Hoang Yen Nhi | MAVROS & Control Logic, Vision & Camera System |

---

## System Overview

```
[Ground Station (PyQt6)] ──LoRa E32──> [lora_drone_package] ──ROS2──> [offboard_control]
        ^                                      |                               |
        |                                telemetry                         MAVROS
        └────────────────────────────────── LoRa ──────────────────────> [PX4 FCU]

[skydroid] ──> /image_raw   [gimbal_ground_station] <──UART──> gimbal hardware
[skydroid_msgs] — shared message definitions for gimbal interface
```

---

## Packages

### `offboard_control`

C++ ROS2 node that drives the drone through a finite-state machine (IDLE → ARMING → CLIMBING → HOVER/MISSION → LANDING). Subscribes to `/mission_path` (geometry_msgs/PoseArray) published by `lora_drone_package` and executes waypoint missions using a carrot-follower algorithm.

**Modules:**

| File | Role |
|---|---|
| `offboard_control.cpp` | Main node + FSM dispatcher. Subscribes to MAVROS state/battery/pose; publishes setpoints. |
| `carrot_follower.cpp` | Pure C++ waypoint tracker. Advances a moving target point along the path; emits `WAYPOINT_PASSED` / `MISSION_COMPLETE` events. |
| `setpoint_streamer.cpp` | SCHED_FIFO thread (20 Hz) that keeps the FCU in OFFBOARD mode with a continuous setpoint heartbeat. |
| `battery_monitor.cpp` | Pure C++ debounce filter. Requires sustained low voltage for `hold_sec` before triggering emergency land; suppresses transient takeoff sags. |

Architecture detail: [offboard_control/docs/ARCHITECTURE.md](offboard_control/docs/ARCHITECTURE.md)

---

### `lora_drone_package`

Python ROS2 node (`lora_drone.py`) that bridges the LoRa serial link to the ROS2 graph. Decodes CRC32-framed JSON commands from the ground station, publishes `/mission_path`, and streams telemetry back over LoRa.

**Modules:**

| File | Role |
|---|---|
| `lora_drone.py` | Main node + event loop. Wires modules together; handles ROS2 topics and serial I/O scheduling. |
| `protocol.py` | CRC32 frame encode/decode, JSON sanitisation, session/mission ID generation, chunk helpers. |
| `idempotency.py` | `IdempotencyCache` — deduplicates repeated packets by sequence number; resets on new session ID. |
| `telemetry.py` | `TelemetryEncoder` — change-detection encoder that skips unchanged fields to stay within LoRa goodput budget (~73 B/s). |
| `mission_assembler.py` | `MissionAssembler` — stateful chunk reassembly; validates begin/chunk/commit sequence and returns parsed waypoints. |
| `serial_link.py` | `SerialLink` — thread-safe serial writer + reconnect logic for both ground and drone sides. |

Architecture detail: [lora_drone_package/docs/ARCHITECTURE.md](lora_drone_package/docs/ARCHITECTURE.md)

---

### `gimbal_ground_station`

Qt/QML desktop application for real-time gimbal control. Communicates with gimbal hardware over UART and visualises attitude feedback.

**Key files:**

| File | Role |
|---|---|
| `qml/main.qml` | Main QML UI — attitude display, control sliders, connection status. |
| `scripts/test_gimbal_rx.py` | CLI diagnostic tool for raw gimbal UART data. |

Depends on custom messages from `skydroid_msgs`.

---

### `skydroid`

ROS2 package for camera integration with the Skydroid payload. Publishes camera frames to the ROS2 image pipeline.

**Key files:**

| File | Role |
|---|---|
| `scripts/camera_launch.py` | Entry point — initialises the camera driver and publishes `/image_raw`. |

---

### `skydroid_msgs`

Custom ROS2 message definitions shared between `skydroid` and `gimbal_ground_station`.

| Message | Fields |
|---|---|
| `GimbalAttitude` | Current roll/pitch/yaw of the gimbal (feedback from hardware). |
| `GimbalCommand` | Target roll/pitch/yaw command sent to the gimbal. |
| `GimbalState` | Connection state and mode flags. |

---

## Build

```bash
cd ~/drone_ws
colcon build --symlink-install
source install/setup.bash
```

> **Note:** `offboard_control` must be built on the **Jetson Nano inside Docker** — do not build locally.
> Edit files in `~/drone_ws/src/`, then sync to `~/nhatbot_remote/` and build on target.

---

## Launch

```bash
# Full drone stack (offboard + LoRa bridge):
ros2 launch offboard_control offboard_control.launch.py

# LoRa bridge only:
ros2 run lora_drone_package lora_drone

# Gimbal ground station:
ros2 run gimbal_ground_station gimbal_ground_station

# Camera:
ros2 run skydroid camera_launch
```

---

## Key ROS2 Topics

| Topic | Type | Direction |
|---|---|---|
| `/mission_path` | `geometry_msgs/PoseArray` | lora_drone → offboard_control |
| `/mavros/state` | `mavros_msgs/State` | FCU → offboard_control |
| `/mavros/local_position/pose` | `geometry_msgs/PoseStamped` | FCU → offboard_control |
| `/mavros/battery` | `sensor_msgs/BatteryState` | FCU → offboard_control |
| `/mavros/setpoint_position/local` | `geometry_msgs/PoseStamped` | offboard_control → FCU |
| `/mavros/cmd/arming` | service | offboard_control → FCU |
| `/mavros/set_mode` | service | offboard_control → FCU |
