# lora_drone_package

ROS2 package chạy trên **Jetson Nano** (companion computer) — cầu nối giữa
radio LoRa E32-433T20D và hệ thống ROS2/MAVROS/PX4.

Package đăng ký **2 node** độc lập (chỉ `lora_drone` được khởi động qua `bringup.launch.py`):

| Executable | Node name | Mô tả |
|---|---|---|
| `lora_drone` | `lora_drone` | Bridge serial ↔ ROS2: nhận lệnh từ Ground GUI qua LoRa → `/mode_signal` / `/mission_path`; gửi telemetry từ `/mavros/*` mỗi 1s |
| `lora_gimbal_bridge` | `lora_gimbal_bridge` | Bridge LoRa ↔ gimbal: nhận gói binary 32-byte từ Skydroid → `/gimbal_command`; gửi feedback 18-byte từ `/gimbal_state` |

---

## Build

> **Chỉ build trên Jetson Nano trong Docker container.** Không `colcon build` trên laptop.

```bash
# SSH vào Jetson, vào container:
sudo systemctl stop drone-bringup
cd ~/drone_ws/script && ./launch_drone_container.sh

# Trong container:
cd /home/drone_ws
colcon build --packages-select lora_drone_package
source install/setup.bash
```

---

## Chạy

### Tự động (systemd — khởi động cùng hệ thống)
```bash
sudo systemctl start drone-bringup      # khởi động
sudo systemctl stop drone-bringup       # dừng
sudo systemctl status drone-bringup     # trạng thái
journalctl -u drone-bringup -f          # xem log live
```

### Thủ công (debug)
```bash
# Toàn bộ stack (MAVROS + offboard_control + lora_drone):
ros2 launch lora_drone_package bringup.launch.py

# Chỉ lora_drone:
ros2 run lora_drone_package lora_drone

# Gimbal bridge (độc lập, không trong bringup):
ros2 run lora_drone_package lora_gimbal_bridge --ros-args -p port:=/dev/ttyUSB1 -p baud:=9600
```

---

## Topics & Services

### `lora_drone` node
| Direction | Topic / Service | Type | Mô tả |
|---|---|---|---|
| Subscribe | `/mavros/local_position/pose` | `geometry_msgs/PoseStamped` | Vị trí EKF local (x,y,z) → telemetry |
| Subscribe | `/mavros/global_position/global` | `sensor_msgs/NavSatFix` | GPS lat/lon/alt → telemetry |
| Subscribe | `/mavros/battery` | `sensor_msgs/BatteryState` | Pin → telemetry |
| Subscribe | `/mavros/local_position/velocity_local` | `geometry_msgs/TwistStamped` | Tốc độ → telemetry |
| Subscribe | `/mavros/state` | `mavros_msgs/State` | Mode PX4 → telemetry |
| Publish | `/mission_path` | `nav_msgs/Path` | Waypoints gửi từ Ground GUI → test_mode |
| Client | `/mode_signal` | `custom_msgs/srv/ModeSignal` | Trigger OFFBOARD hoặc LAND tới test_mode |

### `lora_gimbal_bridge` node
| Direction | Topic | Type | Mô tả |
|---|---|---|---|
| Publish | `/gimbal_command` | `skydroid_msgs/GimbalCommand` | Lệnh nhận từ LoRa (binary 32-byte) → gimbal driver |
| Subscribe | `/gimbal_state` | `skydroid_msgs/GimbalState` | Trạng thái gimbal → gửi feedback 18-byte về UI |

---

## Launch file

`launch/bringup.launch.py` khởi động toàn bộ drone stack:

```
1. MAVROS (px4.launch) — kết nối FCU qua serial:///dev/ttyACM0:57600
2. offboard_control (offboard_launch.py) — node test_mode (C++)
3. lora_drone — Python node
```

Ghi đè FCU URL khi test với SITL:
```bash
ros2 launch lora_drone_package bringup.launch.py \
    fcu_url:=udp://:14540@127.0.0.1:14557
```

---

## Cấu trúc file

```
lora_drone_package/
│
├── launch/
│   └── bringup.launch.py           # Stack launcher: MAVROS + offboard_control + lora_drone
│
├── lora_drone_package/
│   ├── lora_drone.py               # [LIVE] Node lora_drone: serial ↔ ROS2 bridge
│   ├── lora_gimbal_bridge_node.py  # [LIVE] Node lora_gimbal_bridge: binary protocol gimbal
│   ├── e32_config.py               # [TOOL] CLI cấu hình module E32 (không phải ROS2 node)
│   ├── test_gimbal_rx.py           # [TOOL] Standalone debug: monitor gimbal LoRa packets
│   ├── lora_drone_draft.py         # [DEAD] Phiên bản cũ, không build
│   ├── lora_drone_draft_2.py       # [DEAD] Phiên bản cũ, không build
│   └── __init__.py
│
├── package.xml                     # ROS2 package metadata + dependencies
├── setup.py                        # Đăng ký console_scripts: lora_drone, lora_gimbal_bridge
├── setup.cfg
└── resource/lora_drone_package     # Ament index marker
```

> **Bản review working**: `lora_drone.py` và `e32_config.py` có bản sao tại
> `~/ground_gui/` trên laptop. Sửa ở đó (qua sshfs), build ở đây.

---

## Phụ thuộc

Khai báo trong `package.xml`:

| Package | Loại |
|---|---|
| `rclpy` | ROS2 Python client |
| `geometry_msgs`, `nav_msgs`, `sensor_msgs` | Chuẩn ROS2 |
| `mavros_msgs` | MAVROS state, command |
| `custom_msgs` | `/mode_signal` service (`ModeSignal.srv`) |
| `skydroid_msgs` | `GimbalCommand`, `GimbalState` (cho gimbal bridge) |
| `python3-serial` | pyserial |

---

## Giao thức

- **lora_drone**: JSON + CRC32, xem chi tiết tại [ground_gui/ARCHITECTURE.md](../../../ground_gui/ARCHITECTURE.md).
- **lora_gimbal_bridge**: Binary fixed-length (TX 32-byte header `0xAA 0x55`, RX 18-byte header `0xBB 0x66`, CRC-8 poly 0x07).
