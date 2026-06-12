# lora_drone_package

ROS2 package running on the **Jetson Nano** (companion computer) — the bridge between the
LoRa E32-433T20D radio and the ROS2/MAVROS/PX4 system.

The package ships **2 independent nodes** (only `lora_drone` is started by `bringup.launch.py`):

| Executable | Node name | Description |
|---|---|---|
| `lora_drone` | `lora_drone` | Serial ↔ ROS2 bridge: receives commands from the Ground GUI over LoRa → `/mode_signal` / `/mission_path`; streams telemetry from `/mavros/*` every 1s |
| `lora_gimbal_bridge` | `lora_gimbal_bridge` | LoRa ↔ gimbal bridge: receives 32-byte binary packets from Skydroid → `/gimbal_command`; sends 18-byte feedback from `/gimbal_state` |

`lora_drone` is **split into modules** (codec, dedupe, telemetry, mission, serial) — the node only
orchestrates. Full architecture + per-file function + per-module API: **[docs/ARCHITECTURE.md](docs/ARCHITECTURE.md)**.

---

## Build

> **Build on the Jetson Nano inside the Docker container only.** Do not `colcon build` on the laptop.

```bash
# SSH into the Jetson, enter the container:
sudo systemctl stop drone-bringup
cd ~/drone_ws/script && ./launch_drone_container.sh

# Inside the container:
cd /home/drone_ws
colcon build --packages-select lora_drone_package
source install/setup.bash
```

---

## Run

### Automatic (systemd — starts with the system)
```bash
sudo systemctl start drone-bringup      # start
sudo systemctl stop drone-bringup       # stop
sudo systemctl status drone-bringup     # status
journalctl -u drone-bringup -f          # live log
```

### Manual (debug)
```bash
# Full stack (MAVROS + offboard_control + lora_drone):
ros2 launch lora_drone_package bringup.launch.py

# lora_drone only:
ros2 run lora_drone_package lora_drone

# Gimbal bridge (standalone, not in bringup):
ros2 run lora_drone_package lora_gimbal_bridge --ros-args -p port:=/dev/ttyUSB1 -p baud:=9600
```

---

## Topics & Services

### `lora_drone` node
| Direction | Topic / Service | Type | Description |
|---|---|---|---|
| Subscribe | `/mavros/local_position/pose` | `geometry_msgs/PoseStamped` | EKF local position (x,y,z) → telemetry |
| Subscribe | `/mavros/global_position/global` | `sensor_msgs/NavSatFix` | GPS lat/lon/alt → telemetry |
| Subscribe | `/mavros/battery` | `sensor_msgs/BatteryState` | Battery → telemetry |
| Subscribe | `/mavros/local_position/velocity_local` | `geometry_msgs/TwistStamped` | Speed → telemetry |
| Subscribe | `/mavros/state` | `mavros_msgs/State` | PX4 mode → telemetry |
| Publish | `/mission_path` | `nav_msgs/Path` | Waypoints from the Ground GUI → offboard_control |
| Client | `/mode_signal` | `custom_msgs/srv/ModeSignal` | Trigger OFFBOARD or LAND on offboard_control |

### `lora_gimbal_bridge` node
| Direction | Topic | Type | Description |
|---|---|---|---|
| Publish | `/gimbal_command` | `skydroid_msgs/GimbalCommand` | Command received over LoRa (32-byte binary) → gimbal driver |
| Subscribe | `/gimbal_state` | `skydroid_msgs/GimbalState` | Gimbal state → 18-byte feedback to the UI |

---

## Launch file

`launch/bringup.launch.py` starts the whole drone stack:

```
1. MAVROS (px4.launch) — connects to the FCU over serial:///dev/ttyACM0:57600
2. offboard_control (offboard_launch.py) — offboard_control node (C++)
3. lora_drone — Python node
```

Override the FCU URL for SITL testing:
```bash
ros2 launch lora_drone_package bringup.launch.py \
    fcu_url:=udp://:14540@127.0.0.1:14557
```

---

## File layout

```
lora_drone_package/
│
├── launch/
│   └── bringup.launch.py           # Stack launcher: MAVROS + offboard_control + lora_drone
│
├── lora_drone_package/
│   │  # --- lora_drone bridge: node + 5 modules (see docs/ARCHITECTURE.md) ---
│   ├── lora_drone.py               # [LIVE] Node: ROS wiring + callbacks + command routing + telemetry loop
│   ├── protocol.py                 # [LIVE] PURE: LoRa frame codec (CRC32, clean JSON, finite-check)
│   ├── idempotency.py              # [LIVE] PURE: IdempotencyCache — seq dedupe, sid session reset, ACK cache
│   ├── telemetry.py                # [LIVE] PURE: TelemetryEncoder (compact packet + change-detection) + battery % estimate
│   ├── mission_assembler.py        # [LIVE] PURE: MissionAssembler + parse_waypoints — chunked upload begin/chunk/commit
│   ├── serial_link.py              # [LIVE] SerialLink — serial port + writer lock + reconnect + read loop (ROS-agnostic)
│   │  # --- standalone node + tools ---
│   ├── lora_gimbal_bridge_node.py  # [LIVE] lora_gimbal_bridge node: binary protocol gimbal
│   ├── e32_config.py               # [TOOL] CLI to configure the E32 module (not a ROS2 node)
│   ├── test_gimbal_rx.py           # [TOOL] Standalone debug: monitor gimbal LoRa packets
│   └── __init__.py
│
├── docs/
│   └── ARCHITECTURE.md             # Detailed architecture: file map, per-module API, data flow
├── package.xml                     # ROS2 package metadata + dependencies
├── setup.py                        # Registers console_scripts: lora_drone, lora_gimbal_bridge
├── setup.cfg
└── resource/lora_drone_package     # Ament index marker
```

> `find_packages()` in `setup.py` auto-includes every `.py` above → importable as
> `lora_drone_package.<module>` with no extra config. `lora_drone.py` has a `sys.path` shim so it
> runs both via `ros2 run` and as a plain `python3 lora_drone.py`.

---

## Dependencies

Declared in `package.xml`:

| Package | Kind |
|---|---|
| `rclpy` | ROS2 Python client |
| `geometry_msgs`, `nav_msgs`, `sensor_msgs` | Standard ROS2 |
| `mavros_msgs` | MAVROS state, command |
| `custom_msgs` | `/mode_signal` service (`ModeSignal.srv`) |
| `skydroid_msgs` | `GimbalCommand`, `GimbalState` (for the gimbal bridge) |
| `python3-serial` | pyserial |

---

## Protocol

- **lora_drone**: JSON + CRC32 — frame details in [docs/ARCHITECTURE.md §4](docs/ARCHITECTURE.md#4-wire-protocol).
- **lora_gimbal_bridge**: Binary fixed-length (TX 32-byte header `0xAA 0x55`, RX 18-byte header `0xBB 0x66`, CRC-8 poly 0x07).
