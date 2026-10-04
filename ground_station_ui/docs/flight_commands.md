# Lệnh bay từ Ground Station → Jetson → PX4

Tài liệu mô tả cách các nút **Take-off / Land / Hold / RTL / Send WP** trên app EIU Drone GCS
đi tới PX4, các luật an toàn, giao thức xác nhận và cách kiểm tra.

> **Trạng thái:** đã kiểm tra bằng test tự động và bài chạy đầu‑cuối app ↔ Jetson với PX4 giả lập.
> **Chưa thử trên drone thật** — lần đầu làm theo mục [Thử nghiệm](#6-thử-nghiệm).

---

## 1. Tổng quan

```
┌──────────── Laptop ────────────┐      ┌─────────────── Jetson (container) ───────────────┐      ┌── Pixhawk ──┐
│ EIU Drone GCS                  │      │ drone_telemetry                                    │      │ PX4 1.17    │
│  FlightCommands (QML)          │ 433  │  RadioLink ─► hàng đợi ─► CommandBridge ───────────┼─DDS─►│ commander   │
│  CommandSender / Uploader      │─────►│                          MissionReceiver ─► /gcs/mission   │ navigator   │
│                                │◄─────│  COMMAND_ACK / MISSION_* / STATUSTEXT              │◄─DDS─│             │
└────────────────────────────────┘ MHz  └────────────────────────────────────────────────────┘      └─────────────┘
```

| Đoạn | Kênh | Định dạng |
|---|---|---|
| App ↔ Jetson | Radio SiK 433 MHz (`/dev/telem_radio` trên Jetson) | MAVLink v2, dialect `drone_dialect` |
| Jetson ↔ PX4 | uXRCE‑DDS qua `/dev/ttyTHS0` (MicroXRCEAgent) | ROS 2 topic `/fmu/in/*`, `/fmu/out/*` (`px4_msgs`) |
| App ↔ PX4 (song song) | Radio SiK 915 MHz | MAVLink chuẩn — chỉ đọc trạng thái, **không** gửi lệnh bay |

Lệnh bay **chỉ** đi qua Jetson. Jetson là nơi áp luật an toàn và sau này là nơi chạy logic tự hành
(Offboard, tránh vật cản, bay theo waypoint).

**Mã nguồn**

| Phần | File |
|---|---|
| Nút, hộp xác nhận, pre‑flight check | `ground_station_ui/qml/components/FlightCommands.qml` |
| Gửi lệnh + chờ ACK + gửi lại | `ground_station_ui/ground_station_ui/core/commands/sender.py` |
| Gửi waypoint (giao thức mission) | `ground_station_ui/ground_station_ui/core/mission/uploader.py` |
| Nhận lệnh, luật an toàn, chuyển cho PX4 | `drone_telemetry/drone_telemetry/commands.py` (`CommandBridge`) |
| Nhận waypoint | `drone_telemetry/drone_telemetry/commands.py` (`MissionReceiver`) |
| Nối ROS (topic, radio, timer) | `drone_telemetry/drone_telemetry/telemetry_node.py` (`_setup_commands`) |

---

## 2. Các lệnh

| Nút | App gửi (`COMMAND_LONG`, tới sysid 1 / compid 191) | Jetson gửi PX4 (`VehicleCommand`) | PX4 làm gì |
|---|---|---|---|
| **Take‑off** | `MAV_CMD_NAV_TAKEOFF` (22), `param7` = độ cao **so với điểm cất cánh** (m) | `COMPONENT_ARM_DISARM` (400, p1 = 1) nếu chưa arm → `NAV_TAKEOFF` (22) | Lên độ cao rồi **tự Hold** — không tự hạ |
| **Land** | `MAV_CMD_NAV_LAND` (21) | `NAV_LAND` (21), lat/lon NaN = tại chỗ | Hạ cánh tại chỗ, chạm đất tự disarm |
| **Hold** | `MAV_CMD_NAV_LOITER_UNLIM` (17) | `DO_SET_MODE` (176): p1 = 1 (custom), p2 = 4 (AUTO), p3 = 3 (LOITER) | Dừng, đứng yên |
| **RTL** | `MAV_CMD_NAV_RETURN_TO_LAUNCH` (20) | `NAV_RETURN_TO_LAUNCH` (20) | Lên `RTL_RETURN_ALT`, bay về Home, hạ |
| **Send WP** | Giao thức mission (mục 5) | — (chưa gửi PX4) | — |

**Độ cao cất cánh.** PX4 hiểu `param7` của `NAV_TAKEOFF` là độ cao **AMSL**. Jetson đổi:

* Có độ cao toàn cầu (`vehicle_global_position.alt_valid`): `AMSL = alt hiện tại + độ cao app gửi`.
* Không có (bay trong nhà, không GPS): gửi `NaN` → PX4 dùng tham số `MIS_TAKEOFF_ALT`
  (1.5 m trong `config/px4/indoor.params`) và Jetson báo về app
  *"Take-off: no global altitude, PX4 uses MIS_TAKEOFF_ALT"*.

---

## 3. Luật an toàn

Áp ở **Jetson** (`CommandBridge.handle`) — đúng cả khi app có lỗi hoặc lệnh đến từ nơi khác:

| Điều kiện | Kết quả |
|---|---|
| Không có dữ liệu PX4 trong 2 s (agent DDS chết, chưa có `vehicle_land_detected`) | `TEMPORARILY_REJECTED` — *"no PX4 data (DDS agent?)"* |
| Take‑off khi **đang bay** (`landed = false`) | `DENIED` — *"already flying"* |
| Land / Hold / RTL khi **ở mặt đất** | `DENIED` — *"vehicle is on the ground"* |
| Lệnh khác đến khi lệnh trước chưa xong | Lệnh mới **thay** lệnh cũ (vd Land khi đang cất cánh), báo *"… superseded by …"* |
| PX4 từ chối (pre‑arm check, không có vị trí, RTL không có GPS…) | Trả đúng kết quả PX4 (`DENIED` / `FAILED`…) + lý do |

Áp thêm ở **app** (`FlightCommands.qml`) để người vận hành khó bấm nhầm:

* Thanh nút đổi theo trạng thái (`EXTENDED_SYS_STATE` của PX4):
  mặt đất → **Take‑off · Send WP**; đang bay → **Send WP · Hold · RTL · Land**.
* Take‑off / Land / RTL phải **giữ nút 1 s** trong hộp xác nhận. Hold bấm là chạy (lệnh dừng khẩn).
* Take‑off có **Pre‑flight check** (`core/alerts/health.py`): có lỗi nghiêm trọng (mất PX4, cảm biến
  lỗi, pin ≤ 15 %, ngoài trời chưa có GPS fix) → khóa nút. Land không bao giờ bị khóa.
* RTL bị khóa khi môi trường bay là **Trong nhà** (RTL cần GPS).
* Mất kết nối Jetson (radio 433) → khóa toàn bộ nút.

Tay điều khiển RC **luôn ưu tiên**: gạt công tắc chế độ (Stabilized / Position) hoặc Kill vẫn có
hiệu lực bất kể lệnh từ app.

---

## 4. Giao thức xác nhận (ACK)

```
App                         Jetson                                PX4
 │ COMMAND_LONG (conf=0) ──►│                                      │
 │◄── ACK IN_PROGRESS ──────│ VehicleCommand ARM ─────────────────►│
 │                          │◄────────────── vehicle_command_ack ──│ ACCEPTED
 │◄── ACK IN_PROGRESS (0.5s)│ VehicleCommand NAV_TAKEOFF ─────────►│
 │                          │◄────────────── vehicle_command_ack ──│ ACCEPTED
 │◄── ACK ACCEPTED ─────────│ STATUSTEXT "Take-off: accepted by PX4"
```

| Bên | Thời gian chờ | Khi hết giờ |
|---|---|---|
| App chờ ACK từ Jetson | 1.5 s (`commands.ack_timeout_s` trong `gcs.yaml`) | Gửi lại, tối đa 3 lần (`retries`), rồi báo *"Jetson not responding"* |
| Jetson giữ chỗ cho app | Gửi lại `IN_PROGRESS` mỗi 0.5 s khi đang chờ PX4 | — |
| Jetson chờ ACK từ PX4 cho mỗi bước | 3 s | Gửi lại 1 lần, rồi trả `FAILED` *"no ACK from PX4"* |

* App gửi lại cùng lệnh (vì chưa thấy ACK) → Jetson **không chạy lại**, chỉ trả `IN_PROGRESS`.
* Lý do từ chối / kết quả gửi kèm bằng `STATUSTEXT` (component 191) → hiện ở tab
  **Diagnostics → PX4 messages**; trạng thái lệnh hiện ngay dưới thanh nút trên Map.

Giá trị kết quả (MAVLink `MAV_RESULT` = PX4 `VEHICLE_CMD_RESULT`):
`0 ACCEPTED · 1 TEMPORARILY_REJECTED · 2 DENIED · 3 UNSUPPORTED · 4 FAILED · 5 IN_PROGRESS`.

---

## 5. Gửi waypoint (Send WP)

Giao thức mission chuẩn MAVLink, Jetson chủ động xin từng điểm:

```
App ── MISSION_COUNT(n) ─────────────► Jetson
    ◄─ MISSION_REQUEST_INT(0) ────────┘
    ── MISSION_ITEM_INT(0) ──────────►
    …                                   (mất gói → Jetson xin lại sau 1.5 s, tối đa 5 lần)
    ◄─ MISSION_ACK(ACCEPTED) ─────────  + STATUSTEXT "Mission: n waypoints stored on Jetson"
```

* Mỗi điểm: `MAV_FRAME_GLOBAL_RELATIVE_ALT_INT`, `MAV_CMD_NAV_WAYPOINT`, lat/lon × 1e7, độ cao (m).
* Jetson **chỉ lưu** và publish lên `/gcs/mission` (`std_msgs/String`, JSON, QoS transient‑local —
  node vào sau vẫn nhận được bản cuối):
  ```json
  [{"seq": 0, "lat": 11.0529, "lon": 106.6660, "alt": 20.0, "command": 16, "frame": 6}, ...]
  ```
* **Chưa bay theo waypoint.** Hai hướng cho bước sau: node Offboard trên Jetson đọc `/gcs/mission`
  và gửi `trajectory_setpoint`, hoặc nạp mission vào PX4 qua MAVLink (`/dev/ttyACM0`) —
  uXRCE‑DDS không nạp mission được.

---

## 6. Thử nghiệm

**Lần đầu — THÁO CÁNH QUẠT**, drone trên sàn, một người cầm RC, tay sẵn trên công tắc **Kill**.

1. Build và chạy trên Jetson:
   ```bash
   cd /home/drone_ws && colcon build --packages-select drone_telemetry && source install/setup.bash
   ros2 launch drone_bringup bringup.launch.py           # terminal 1 (có MicroXRCEAgent)
   ros2 launch drone_telemetry telemetry.launch.py       # terminal 2
   ```
   Log phải có `commands: Take-off / Land / Hold / RTL + waypoint -> PX4 (DDS)`.
2. **Từ chối đúng luật:** bấm Land khi drone nằm yên → app báo bị từ chối, PX4 messages hiện
   *"Land: vehicle is on the ground"*; log Jetson **không** có dòng `-> PX4 cmd`.
3. **Take‑off:** bấm Take‑off, giữ 1 s → motor arm và tăng ga; log Jetson
   `-> PX4 cmd 400 …` rồi `-> PX4 cmd 22 …`; app báo *accepted*. Gạt Kill / disarm bằng RC.
4. **Send WP:** thêm 2–3 điểm trên Map → Send WP → app *"Uploaded n waypoints"*; trên Jetson:
   `ros2 topic echo /gcs/mission`.
5. Sau đó mới lắp cánh, bay thử thấp, sẵn sàng gạt về Stabilized.

**Test tự động**

```bash
cd ~/drone_ws/src/ground_station_ui && python3 -m pytest test/        # app
cd ~/drone_ws/src/drone_telemetry   && python3 -m pytest test/         # Jetson (test_commands.py)
```

---

## 7. Cấu hình

**Jetson — `drone_telemetry/config/telemetry.yaml`**

```yaml
commands:
  enabled: true                                   # false: bỏ qua mọi lệnh từ ground
  px4_system_id: 1
  command_topic: /fmu/in/vehicle_command
  ack_topic: /fmu/out/vehicle_command_ack
  control_mode_topic: /fmu/out/vehicle_control_mode   # trạng thái armed
  land_topic: /fmu/out/vehicle_land_detected
  global_topic: /fmu/out/vehicle_global_position
  mission_topic: /gcs/mission
```

**App — `ground_station_ui/config/gcs.yaml`**

```yaml
commands:
  link: jetson
  target_system: 1
  target_component: 191
  ack_timeout_s: 1.5
  retries: 3
  takeoff_altitude_indoor: 1.5
  takeoff_altitude_outdoor: 5
```

---

## 8. Phiên bản message PX4 (quan trọng khi cập nhật firmware)

Firmware trên drone là **PX4 1.17**; `px4_msgs` trong workspace Jetson mới hơn (~1.18). DDS chỉ ghép
được khi **tên kiểu và cấu trúc** giống nhau:

| Topic | Firmware | `px4_msgs` workspace | Dùng được? |
|---|---|---|---|
| `vehicle_command` | v0 | v0 | ✓ |
| `vehicle_command_ack` | v0 | v1 (cùng trường) | ✓ |
| `vehicle_land_detected` | v0 | v0 | ✓ |
| `vehicle_global_position` | v0 | v0 | ✓ |
| `vehicle_control_mode` | v0 | v0 | ✓ (lấy `flag_armed`) |
| `vehicle_status` (`_v1`) | v1 | **v4** | ✗ — không dùng |

Khi nâng firmware: so lại `MESSAGE_VERSION` trong `px4_msgs/msg/*.msg` với hậu tố topic
(`ros2 topic list | grep fmu`), hoặc chạy `translation_node` của PX4.

---

## 9. Xử lý sự cố

| Hiện tượng | Nguyên nhân thường gặp | Kiểm tra |
|---|---|---|
| App: *"Jetson not responding"* | `drone_telemetry` chưa chạy / chưa build bản mới / radio 433 rời | Log Jetson có dòng `commands: …`; tab Connections link Jetson |
| *"no PX4 data (DDS agent?)"* | MicroXRCEAgent không chạy hoặc mất `/dev/ttyTHS0` | `ros2 topic hz /fmu/out/vehicle_land_detected` |
| PX4 từ chối Take‑off | Pre‑arm check (pin, cảm biến, chưa có vị trí, chế độ RC) | Tab Diagnostics → PX4 messages; QGC |
| Take‑off *"no ACK from PX4"* | Lệnh không tới PX4 (sai topic / lệch phiên bản message) | `ros2 topic echo /fmu/out/vehicle_command_ack` |
| RTL bị từ chối | Không có GPS / Home | Chỉ dùng ngoài trời có GPS fix 3D |
