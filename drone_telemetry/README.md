# drone_telemetry

ROS 2 node on the Jetson that links the drone to the ground station over a SiK 433 MHz radio. It sends component health, pose, position and gimbal angles down to the ground station, and it receives flight commands and waypoints from it. Messages use MAVLink v2 with a small custom dialect (`mavlink/drone_telemetry.xml`).

```
ground station <--433 MHz--> drone_telemetry <--ROS 2--> sensor drivers, mission_manager, PX4 (uXRCE-DDS)
```

Diagrams, threads, the telemetry and command paths, and the acknowledgement protocol: **[docs/ARCHITECTURE.md](docs/ARCHITECTURE.md)**.

## What it does

Sent to the ground station:

| Message | Content | Rate |
|---|---|---|
| `HEARTBEAT` | link alive | 1 Hz |
| `DRONE_COMPONENT_STATUS` | level, state and measured rate of lidar, gimbal, camera, PX4 link and radio | 1 s, and at once on a change |
| `DRONE_LOCAL_POSE` | PX4 odometry for the 3D view | 5 Hz |
| `GLOBAL_POSITION_INT` | map position and heading, used when the ground station has no 915 link | 2 Hz |
| `DRONE_GIMBAL_STATE` | gimbal angles | 2 Hz |
| `MISSION_CURRENT` | mission progress from `mission_manager`: current waypoint, count, state (active / paused / complete) | 1 Hz, and at once on a change |
| `STATUSTEXT` | command results and `mission_manager` messages (e.g. "WP2 pick: take control, then Continue") | on demand |

Received from the ground station:

| Command | Handled by |
|---|---|
| Take-off, Land | forwarded to `mission_manager` on `/gcs/command` |
| Start mission (`MISSION_START`, `param3` bit 0 = obstacle avoidance, `param4` = speed in m/s, `param5` / `param6` = arrival tolerance xy / z in m) | forwarded with its options |
| Continue (`DO_PAUSE_CONTINUE`, `param1 = 1`) | forwarded; ends a manual task or a pause |
| Live settings (`MAV_CMD_USER_2`, `param1` = speed, `param2` / `param3` = tolerance xy / z, 0 = unchanged) | forwarded as `{"cmd": "settings"}` and acknowledged at once; does not replace a running command |
| Hold, RTL | sent to PX4 as `VehicleCommand` |
| Send WP (mission protocol) | stored with each item's command and params, published on `/gcs/mission` |
| Ping (Test button) | answered directly, PX4 not involved |

Commands are checked first: no fresh PX4 data, Take-off in the air, or Land / Hold / RTL on the ground are refused with a reason. A command resent after a lost ACK is never executed twice.

## Configuration

Everything is set in `config/telemetry.yaml`:

| Key | Default | Meaning |
|---|---|---|
| `port`, `baud` | `/dev/ttyUSB0`, `57600` | radio serial port; on the Jetson use the udev name `/dev/telem_radio` |
| `system_id`, `component_id` | `1`, `191` | MAVLink identity (onboard computer) |
| `send_rate_hz`, `stale_timeout_s` | `1.0`, `3.0` | component status period and staleness limit |
| `components` | `[lidar, gimbal, camera, px4]` | monitored components, each with its own block (`id`, `source`, ...) |
| `pose.*`, `global_position.*`, `gimbal_state.*` | enabled | topic, `rate_hz` and `max_age_s` of each stream |
| `commands.enabled` | `true` | accept commands from the ground station |
| `commands.takeoff_land_via` | `mission` | `mission` (via `mission_manager`) or `px4` (direct `VehicleCommand`) |

## Build and run

```bash
cd ~/drone_ws && colcon build --packages-select drone_telemetry
source install/setup.bash
ros2 launch drone_telemetry telemetry.launch.py
```

The launch file loads `config/telemetry.yaml` and restarts the node if it exits. MicroXRCEAgent must be running for the PX4 topics, and `mission_manager` / `offboard_control` (package `offboard_control`) for Take-off, Land and missions.

## Tools

- `scripts/telemetry_monitor.py`: ground-side monitor without ROS. It opens the ground radio and prints the heartbeat age, radio RSSI and component table: `python3 scripts/telemetry_monitor.py --port /dev/ttyUSB0`
- `scripts/gen_dialect.sh`: regenerates `drone_telemetry/drone_dialect.py` after editing `mavlink/drone_telemetry.xml`. Copy the result to `ground_station_ui/ground_station_ui/core/mavlink/` so both ends use the same dialect.

## Tests

```bash
cd ~/drone_ws/src/drone_telemetry && python3 -m pytest test/
```

`test_health.py` covers component monitoring and `test_commands.py` covers the command rules, mission commands, live settings and the mission protocol. Neither needs ROS or hardware.
