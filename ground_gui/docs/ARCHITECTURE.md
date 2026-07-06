# Ground Control Station ↔ Drone (LoRa) — Architecture & Protocol

Tài liệu mô tả toàn bộ pipeline điều khiển/giám sát drone qua LoRa: kiến trúc,
giao thức truyền/nhận, mô hình threading, và các cơ chế reliability đã áp dụng.

> Phạm vi: hệ thống gồm **Ground GUI** (PyQt6 + Azure Maps) và **Drone side**
> (ROS2 + MAVROS/PX4), nối với nhau bằng một đường **LoRa serial** điểm-điểm.

---

## Mục lục
1. [Tổng quan pipeline](#1-tổng-quan-pipeline)
2. [Thành phần & vai trò file](#2-thành-phần--vai-trò-file)
3. [Giao thức truyền/nhận (wire protocol)](#3-giao-thức-truyềnnhận-wire-protocol)
4. [Mô hình threading](#4-mô-hình-threading)
5. [Cơ chế reliability](#5-cơ-chế-reliability)
6. [Vòng đời kết nối (lifecycle)](#6-vòng-đời-kết-nối-lifecycle)
7. [Các kịch bản bay](#7-các-kịch-bản-bay)
8. [Cấu hình (env vars)](#8-cấu-hình-env-vars)
9. [Testing](#9-testing)
10. [Bản đồ file & deploy](#10-bản-đồ-file--deploy)
11. [Giới hạn đã biết & việc tương lai](#11-giới-hạn-đã-biết--việc-tương-lai)

---

## 1. Tổng quan pipeline

```
┌─────────────────────────── GROUND (laptop) ───────────────────────────┐
│                                                                         │
│   index/map.html  ──QWebChannel──►  LoraBridge  ──►  GroundController    │
│   (UI, Azure Maps)   signal/slot    (lora_bridge)     (control.py)       │
│        ▲                                                   │ serial      │
│        └───────────────── telemetry (Qt signals) ──────────┤ /dev/ttyUSB0│
└────────────────────────────────────────────────────────────┼───────────┘
                                                               │  LoRa
                                                               │  9600 8N1
┌──────────────────────────── DRONE (companion) ──────────────┼───────────┐
│                                                               │ /dev/lora_drone
│   lora_drone.py (ROS2 node)  ◄── serial ──►                   ▼          │
│      • read_serial  → /mode_signal (service client)                     │
│      • send_pose_loop ← /mavros/*  (telemetry 0.4s)                      │
│                                   │                                       │
│                                   ▼  /mode_signal (ModeSignal srv)        │
│   offboard_control.cpp  (executable "offboard_control")                  │
│      • FSM: arm → takeoff → hover/mission → land                          │
│      • ──► /mavros/setpoint_position/local, /mavros/cmd/arming, set_mode  │
│                                   │                                       │
│                                   ▼                                       │
│                              MAVROS ──► PX4 (FCU)                         │
└─────────────────────────────────────────────────────────────────────────┘
```

**Hai luồng song song, độc lập:**
- **Lệnh (command):** Ground → LoRa → `lora_drone.py` → ROS2 service `/mode_signal` → `offboard_control.cpp` → MAVROS/PX4.
- **Telemetry:** MAVROS → `lora_drone.py` (0.4s) → LoRa → Ground → UI.

---

## 2. Thành phần & vai trò file

### Ground (`~/ground_gui/`)

`control.py` được **tách thành module** (2026-06-12): controller chỉ điều phối; *policy* (codec, parse,
lệnh) nằm trong các module **thuần** (test offline được). `control.py` **re-export** các helper protocol
để `test/` + `main.py` vẫn `from control import ...` được.

| File | Vai trò |
|---|---|
| `main.py` | Cửa sổ PyQt6, nhúng `QWebEngineView` (map.html), chạy HTTP server tĩnh phục vụ map, khởi tạo `GroundController` + `LoraBridge`, dọn tài nguyên khi đóng (`closeEvent`). |
| `lora_bridge.py` | `LoraBridge(QObject)` — cầu nối JS ↔ Python qua **QWebChannel**. Slot nhận lệnh từ JS (`startConnection`, `offBoardConnect`, `receivedTargetWaypoint`…), signal đẩy telemetry lên JS (`positionUpdated`, `batteryUpdated`, `headingUpdated`, `modePushed`, `missionAck`…). |
| `control.py` | `GroundController` — **orchestrator**: vòng đời serial (connect + auto-reconnect lái bởi heartbeat), gửi lệnh có ACK + retry + dedupe seq, vòng RX, gating GPS, heartbeat watch. Điều phối các module dưới; chỉ gọi bridge qua duck-typing (không phụ thuộc Qt). |
| `protocol.py` | **[PURE]** Codec khung LoRa: CRC32 `wrap/unwrap`, `clean_json`, `_is_num`/`_as_true`, sinh `session_id`/`mission_id`, `_chunk_list`. Đối xứng với `protocol.py` phía drone. **Đổi khung/CRC = sửa ở đây.** |
| `telemetry_parser.py` | **[PURE]** `parse_frame(data) → ParsedFrame`: tách field từ 1 frame telemetry/ACK (pos local/global, battery, speed, heading, state, gps_ok, hb, ack). **Thêm field telemetry mới = sửa 1 chỗ ở đây.** |
| `commands.py` | **[PURE]** Builder lệnh ground→drone: `cmd_offboard`, `cmd_land`, `mission_begin/chunk/commit`. Schema lệnh wire ở 1 nơi. **Thêm lệnh mới = thêm 1 builder.** |
| `index/map.html` | UI bản đồ (Azure Maps SDK từ CDN), điều khiển bay, vẽ waypoint, hiển thị telemetry. Gồm 4 layer: flight trail (vết bay thực tế), WP path (polygon nối waypoint), line/polygon overlay, HtmlMarker drone image (xoay theo heading thực từ `headingUpdated`). |
| `run_silent.py` | Chạy `main.py` với stderr/stdout → /dev/null (ẩn log libva/Chromium). |
| `test/test_logic.py` | Unit tests (stdlib `unittest`): CRC, seq/ACK, waypoints, GPS gating, lifecycle, auto-reconnect. Import protocol helper + `GroundController` từ `control` (re-export giữ nguyên đường import). |
| `test/test_protocol_integration.py` | Integration tests ground↔drone: ACK-retry sau mất gói, chunked mission round-trip, dedupe seq drone-side, bad-CRC rejection, session-id reset. Cần `import lora_drone` (xem §9). |
| `lora_config/e32_config.py` | CLI đọc/ghi cấu hình EBYTE E32 (air data rate, channel) qua UART ở CONFIG mode (M0=M1=3.3V). |
| `lora_link_check.py` | Đo goodput, CRC health, heartbeat rate trực tiếp từ serial LoRa (đóng main.py trước). |
| `rules/install_lora_rules.sh` | Cài `99-lora-ground.rules` vào `/etc/udev/rules.d/`, reload udev, trigger `action=add`, báo kết quả symlink, cảnh báo dialout. |
| `rules/99-lora-ground.rules` | udev rules: PL2303 → `/dev/lora_ground_drone`; CP210x serial `0001` → `/dev/lora_ground`; FTDI `D30JUEQN` → `/dev/lora_telem`. |

### Drone — `lora_drone_package` (ROS2 package trên Jetson Nano)

Package path: `~/drone_ws/src/lora_drone_package/` (bản local laptop) ↔ Jetson qua sshfs. **`lora_drone`
cũng đã tách module** (node + protocol/idempotency/telemetry/mission_assembler/serial_link) — chi tiết
ở **`lora_drone_package/docs/ARCHITECTURE.md`**. Tóm tắt file LIVE:

| File | Vai trò |
|---|---|
| `launch/bringup.launch.py` | Launcher chính: MAVROS (`px4.launch`) + offboard_control + lora_drone. FCU default `serial:///dev/ttyACM0:57600`; override `fcu_url:=udp://...` cho SITL. |
| `lora_drone_package/lora_drone.py` | `LoraDrone(Node)` — **[LIVE]** node cầu nối serial ↔ ROS2 (verify CRC32 → `/mode_signal` / `/mission_path`; telemetry từ `/mavros/*`). Điều phối 5 module (`protocol`, `idempotency`, `telemetry`, `mission_assembler`, `serial_link`). |
| `lora_drone_package/lora_gimbal_bridge_node.py` | **[LIVE]** bridge LoRa ↔ gimbal (binary 32-byte header `0xAA 0x55`, CRC-8). |
| `lora_drone_package/{e32_config,test_gimbal_rx}.py` | **[TOOL]** cấu hình E32 / debug gimbal. |

### Drone flight — `offboard_control` (C++ package)

Đã đổi tên + tách module — chi tiết ở **`offboard_control/docs/ARCHITECTURE.md`**.

| File | Vai trò |
|---|---|
| `src/offboard_control.cpp` | `OffboardMode` — **[LIVE]** node bay (executable `offboard_control`, đổi tên từ `test.cpp`/`test_mode`). FSM arm→takeoff→hover/mission→land; carrot follower; SCHED_FIFO setpoint; an toàn pin + timeout. |
| `src/{carrot_follower,setpoint_streamer,battery_monitor}.cpp` | **[LIVE]** module tách ra: thuật toán bám waypoint / thread setpoint / debounce pin. |

---

## 3. Giao thức truyền/nhận (wire protocol)

### 3.1. Khung tin (frame) — hai chiều giống nhau
Mỗi gói là **một dòng JSON kết thúc bằng `\n`**, payload được bọc CRC32:

```json
{"payload":{...},"crc32":"A1B2C3D4"}\n
```

- **Canonical bytes để tính CRC:** `json.dumps(payload, separators=(",",":"), sort_keys=True)` → encode UTF-8.
- **CRC32:** `zlib.crc32(canonical) & 0xFFFFFFFF` → 8 ký tự **hex hoa**.
- Hàm: `_wrap_with_crc()`, `_unwrap_and_verify_crc()` — **byte-identical** giữa `control.py` và `lora_drone.py` (đã test round-trip).

**Trạng thái khi unwrap:**
| status | Ý nghĩa | Hành xử |
|---|---|---|
| `ok_crc` | Có wrapper, CRC khớp | Xử lý payload |
| `bad_crc` | Có wrapper, CRC sai / payload không phải dict | **Drop** |
| `plain` | Không có wrapper (legacy) | Chấp nhận payload nguyên trạng |
| `invalid` | Không phải dict | Drop |

> Phía nhận còn `_clean_json_str()` trích đúng `{...}` ngoài cùng → chịu được nhiễu rác đầu/cuối dòng từ sóng LoRa trước khi `json.loads`.

### 3.2. Ground → Drone (lệnh)
Tất cả đều thêm trường `seq` (số nguyên tăng dần) trước khi bọc CRC.

| Lệnh | Payload | Sinh ra từ |
|---|---|---|
| OFFBOARD | `{"cmd":"offboard","seq":N}` | `offboard_req()` |
| LAND | `{"cmd":"land","seq":N}` | `land_req()` |
| Mission begin | `{"op":"begin","mi":"<id>","tc":<n_chunks>,"tn":<n_wps>,"seq":N}` | `send_waypoints_to_drone()` — worker bước 1 |
| Mission chunk | `{"op":"chunk","mi":"<id>","ci":<idx>,"tc":<n_chunks>,"wps":[{"x":…,"y":…,"z":…},…],"seq":N}` | worker bước 2 (lặp từng chunk) |
| Mission commit | `{"op":"commit","mi":"<id>","seq":N}` | worker bước 3 |

> **Giao thức 3 bước (begin → chunk × N → commit)** được dùng để chia mission lớn thành
> các frame nhỏ phù hợp giới hạn LoRa. Mỗi bước dùng `_send_payload_wait_ack()` — chờ ACK
> tương ứng rồi mới gửi bước tiếp. `mi` (mission_id) dùng để ghép lại đúng mission khi
> có retry. Số waypoint mỗi chunk = `GROUND_GUI_MISSION_CHUNK_SIZE` (mặc định 4).

**Control token (KHÔNG bọc CRC, KHÔNG seq):** `ON\n` / `OFF\n` — bật/tắt module LoRa, drone **bỏ qua** (`handle_command` lọc trước).

### 3.3. Drone → Ground

**Telemetry (KHÔNG seq, KHÔNG ACK):**
```json
{"hb":1,"state":"OFFBOARD","gps_ok":1,
 "x":1.23,"y":4.56,"z":7.89,"hdg":-37,
 "lat":11.0529,"lon":106.6661,"alt":12.34,
 "battery":{"percent":85.0,"voltage":16.2},"speed":1.5}
```
Các nhóm trường xuất hiện **có điều kiện** (chỉ gửi khi có dữ liệu tương ứng hoặc khi thay đổi): `x/y/z` (local pose), `hdg` (yaw độ, ENU frame: 0°=East CCW+, chỉ gửi khi thay đổi ≥1°), `lat/lon/alt` (GPS), `battery` (chỉ gửi khi thay đổi), `speed`. `hb` luôn có; `state`, `gps_ok` gửi khi thay đổi hoặc full-refresh (15s).

**ACK `mode_push`** (đáp lệnh offboard/land):
```json
{"event":"mode_push","status":true,"mode":"OFFBOARD","msg":"...","ack_seq":N}
```
Nhánh lỗi dùng `"error"` thay `"msg"`. `ack_seq` = `seq` của lệnh.

**ACK `mission_begin`** (đáp `op=begin`):
```json
{"event":"mission_begin","status":true,"mission_id":"<id>","total_chunks":N,"total_count":M,"msg":"mission begin accepted","ack_seq":N}
```
Nhánh lỗi: `status:false`, kèm `"error":"..."`.

**ACK `mission_chunk`** (đáp `op=chunk`):
```json
{"event":"mission_chunk","status":true,"mission_id":"<id>","chunk_index":I,"received_chunks":K,"total_chunks":T,"msg":"chunk accepted","ack_seq":N}
```
Nhánh lỗi: `status:false`, kèm `"chunk_index"` và `"error":"..."`.

**ACK `uploaded`** (đáp `op=commit`):
```json
{"event":"uploaded","status":true,"mission_id":"<id>","count":3,"total":3,"msg":"mission uploaded","ack_seq":N}
```
Nhánh từ chối: `status:false`, kèm `"error":"missing chunks: [N]"` hoặc `"waypoint count mismatch"` v.v.

### 3.4. Ground RX dispatch (`control.py::_read_loop`)
| Trường nhận | Hành động |
|---|---|
| `hb` (truthy) | Cập nhật `_last_hb` → link **UP** |
| `state` | `update_state()` → `stateUpdated` |
| `gps_ok` | `update_gps()` → `gpsStatusUpdated` |
| `x,y,z` (finite) | `update_position()` → `positionUpdated` |
| `lat,lon,alt` (finite) | `update_global_position()` + cập nhật `_last_global_fix_at` |
| `battery`/`percent`/`voltage` | `update_battery()` |
| `speed`/`vel` | `update_speed()` |
| `hdg` (finite) | `update_heading()` → `headingUpdated` → JS xoay drone marker theo yaw thực |
| `event=mode_push` | set ACK snapshot + `mode_push()` → `modePushed` |
| `event=mission_begin` | set ACK snapshot; `_send_payload_wait_ack` trả về → worker gửi chunk tiếp |
| `event=mission_chunk` | set ACK snapshot; `_send_payload_wait_ack` trả về → worker gửi chunk tiếp (hoặc commit) |
| `event=uploaded` | set ACK snapshot + `mission_ack()` → `missionAck` |

---

## 4. Mô hình threading

### 4.1. Ground
| Thread | Nhiệm vụ |
|---|---|
| **Qt main** | UI + QWebChannel slot (lệnh từ JS). CONNECT/DISCONNECT được đẩy sang thread nền để **không freeze UI**. |
| **Reader** (`_read_loop`) | Đọc serial (`in_waiting` → drain), parse, bắn telemetry qua Qt signal (auto-queue sang main thread). 1 instance duy nhất (`_reader_lock` + `_reader_running`). |
| **Heartbeat** (`_hb_watch`) | Theo dõi `_last_hb`; phát `linkUpdated(True/False)`. |
| **TX worker** (ephemeral) | Mỗi lệnh spawn 1 thread retry; **ghi serial tuần tự** qua `_tx_lock`; tự huỷ khi `_tx_epoch` đổi. |

**Locks:** `_tx_lock` (ghi), `_ack_lock` (snapshot ACK), `_conn_lock` (mở serial atomic), `_reader_lock` (1 reader), `_seq_lock` (seq), `_tx_epoch` (cờ huỷ TX).

> Đọc và ghi serial là **độc lập** (full-duplex) → reader không cần `_tx_lock`; `_tx_lock` chỉ tuần tự hoá nhiều **writer**.

### 4.2. Drone (`lora_drone.py`)
| Thread | Nhiệm vụ |
|---|---|
| **ROS executor** (`rclpy.spin`) | Callback subscription (`/mavros/*`) + done-callback của service client (bắn ACK `mode_push`). |
| **Reader** (`read_serial`) | `readline()` **không ôm lock** (đọc độc lập) → không chặn writer. Parse → `handle_command`. |
| **Telemetry** (`send_pose_loop`) | 0.4s; ghi serial qua `serial_lock`. |

**Locks:** `serial_lock` (mọi writer: telemetry + ACK), `_proc_lock` (bảng dedupe seq).

> **Bug đã sửa:** trước đây `read_serial` ôm `serial_lock` quanh `readline()` (timeout 1s) → chặn telemetry/ACK tới 1 giây. Nay đã bỏ.

### 4.3. Drone flight (`test.cpp`)
SingleThreadedExecutor; timer 50ms (`control_loop`); service `/mode_signal`. Không có data race giữa callback (cùng 1 thread).

---

## 5. Cơ chế reliability

Mô hình tổng: **at-least-once (ground retry) + idempotent receiver (drone dedupe)**.

| # | Cơ chế | Phía | Mô tả |
|---|---|---|---|
| 1 | **CRC32** | cả hai | Phát hiện hỏng gói → drop. Không bao giờ nhận nhầm dữ liệu sai. |
| 2 | **seq / ack_seq** | cả hai | Ground gắn `seq`; drone echo `ack_seq`. Ground khớp **chính xác** (`_ack_matches`: `last_seq == sent_seq`), **bỏ wildcard legacy**. |
| 3 | **Retry** | ground | `_send_with_retry`: `tries=2, interval=2.0s` → tối đa 3 lần (~6s) trước khi báo FAILED. |
| 4 | **Idempotency (dedupe)** | drone | `handle_command` nhớ `seq` đã xử lý + ACK đã sinh. Gói trùng → **re-ACK từ cache, KHÔNG thực thi lại** (không double publish mission / double mode). Cache chỉ giữ ACK đầu tiên (auto-OFFBOARD không ghi đè ACK "uploaded" cùng seq). |
| 5 | **TX epoch** | ground | `connect()/stop()` tăng `_tx_epoch`; worker chụp epoch lúc gửi, đổi epoch → **tự huỷ**, không ghi vào serial đã đóng/mới. Không bắn FAILED giả khi link reset. |
| 6 | **Heartbeat watch** | ground | `_hb_watch(timeout=30s)`: link DOWN nếu im >30s; UP ngay khi có gói. |
| 7 | **Recent-GPS gating** | ground | OFFBOARD & mission bị chặn nếu ground chưa nhận `lat/lon` trong `3.0s` (`_has_recent_global_position`). LAND không gating. |
| 8 | **Mission validation** | drone | Chỉ chấp nhận khi `total>0 và saved==total`; còn lại **reject** `status=False` + lý do, **không publish, không auto-offboard**. |
| 9 | **Non-blocking service** | drone | `service_is_ready()` thay `wait_for_service(3.0)` → reader không bị kẹt 3s. |
| 10 | **Serial open retry** | drone | Mở serial retry 5× (USB-LoRa enumerate trễ), thất bại → `raise` → `main()` shutdown sạch (không spin node chết). |
| 11 | **Resource cleanup** | ground | `closeEvent`: `controller.stop()` + tắt HTTP server (không rò thread / zombie port 8000). |

### Ví dụ: vì sao retry không gây double-mission
```
Ground: gửi mission seq=10  ──►  Drone: publish /mission_path, gửi ACK uploaded(ack_seq=10)
        (ACK bị mất trên đường về)
Ground: hết interval → retry mission seq=10  ──►  Drone: seq=10 ĐÃ xử lý
                                                  → re-ACK uploaded(ack_seq=10) từ cache
                                                  → KHÔNG publish lại
Ground: nhận ACK, khớp seq=10 → dừng retry, GUI báo OK
```

---

## 6. Vòng đời kết nối (lifecycle)

### Khởi động (`main.py`)
1. Chạy HTTP server (`python3 -m http.server <HTTP_PORT>`) phục vụ `index/`.
2. Nạp UI, tạo `QWebEngineView` + `QWebChannel(bridge)`.
3. Tải `map.html` (sau 1s để server kịp sẵn sàng).
4. Tạo `GroundController(port, baudrate, bridge)` và `controller.connect()` (mở serial sớm, **chưa** đọc telemetry).

### CONNECT (người dùng bấm trên UI)
`startConnection` (chạy thread nền, không freeze UI) → `controller.start()` (mở serial nếu cần + gửi `ON`) → `read_position_from_drone()` (spawn reader + heartbeat). Link báo UP khi gói `hb` đầu tiên về.

### DISCONNECT
`stopConnection` (thread nền) → `controller.stop()`: `received=False`, `_tx_epoch += 1` (huỷ TX worker), join reader/heartbeat, gửi `OFF`, đóng serial, phát `linkUpdated(False)`.

### Đóng app
`MainWindow.closeEvent` → `controller.stop()` + `http_process.terminate()`.

---

## 7. Các kịch bản bay

### 7.1. OFFBOARD (1 trigger → cả chuỗi tự động)
```
GUI OFFBOARD → control.offboard_req() [gating GPS] → LoRa {"cmd":"offboard","seq":N}
 → lora_drone → /mode_signal(OFFBOARD) → test.cpp.on_mode_signal: start_offboard_=true
 → test.cpp.control_loop: chờ 5s → set_mode(OFFBOARD)+ARM → bay tới takeoff_height_(5m)
 → giữ vị trí 10s → AUTO.LAND → auto-disarm
 ↩ lora_drone gửi mode_push(status,OFFBOARD,ack_seq=N) → GUI hiện kết quả
```
> Ground **chỉ gửi trigger**; độ cao (5m, param `takeoff_height_`) và hover (10s, hardcode) nằm ở `test.cpp`. Nút **LAND** = hạ cánh sớm thủ công.

### 7.2. LAND
`GUI LAND → {"cmd":"land","seq":N} → /mode_signal(LAND) → test.cpp.land_vehicle() → AUTO.LAND`.

### 7.3. Mission (gửi waypoint → drone bay theo carrot/lookahead)
```
GUI Send Mission → control.send_waypoints_to_drone() [gating vị trí]
 → begin → chunk×N → commit  (3-phase, mỗi bước chờ ACK)
 → lora_drone: commit OK → _publish_mission(/mission_path) + ACK uploaded + auto-OFFBOARD
 → test.cpp mission_cb: nhận /mission_path → lưu vào mission_wps_ (inbox)
 → control_loop: khi đã arm+takeoff → consume inbox (copy → active_wps_, xóa inbox)
 → flying_mission_=true: mỗi 50ms advance carrot @ cruise_speed_ m/s về hướng WP hiện tại
   carrot không vượt quá carrot_lead_ m trước drone → drone chạy theo carrot mượt mà
   WP trung gian: chuyển sang wp_index_+1 khi carrot_at_wp (carrot đã chạm WP)
   WP cuối: land_vehicle() khi drone trong wp_reach_radius_ hoặc timeout
 ↩ GUI: missionAck → hiện OK/FAILED
```
> **Mission consume pattern** (không replay): `active_wps_` được nạp MỘT LẦN lúc bắt đầu
> bay — sau khi hạ cánh, `mission_wps_` (inbox) đã trống nên lần nhấn OFFBOARD tiếp theo
> sẽ chỉ hover chứ không replay lại mission cũ.

### 7.4. An toàn (test.cpp)
- **Timeout độ cao:** chưa đạt độ cao trong **30s sau ARM** (neo theo `arm_time`) → emergency land.
- **Pin yếu:** khi **đang armed**, điện áp < `min_battery_`(15.5V) liên tiếp ≥3 mẫu → emergency land (bỏ qua đọc 0/NaN).
- **Mất FCU / pose cũ:** dừng gửi setpoint.

---

## 8. Cấu hình (env vars)

| Biến | Mặc định | Tác dụng |
|---|---|---|
| `GROUND_GUI_SERIAL_PORT` | `/dev/ttyUSB0` | Cổng serial LoRa phía ground |
| `GROUND_GUI_BAUDRATE` | `9600` | Baudrate |
| `GROUND_GUI_HTTP_PORT` | `8000` | Cổng HTTP phục vụ map.html |
| `GROUND_GUI_DEBUG` | `0` | `1` = bật log per-packet (`[RAW]`, `[DATA]`, vị trí…) |
| `QTWEBENGINE_CHROMIUM_FLAGS` | `--disable-features=VaapiVideoDecoder,VaapiVideoEncoder` | Tắt dò VA-API (ẩn "libva error") |

Ví dụ:
```bash
GROUND_GUI_SERIAL_PORT=/dev/lora_ground GROUND_GUI_DEBUG=1 python3 main.py
```

> Drone side: cổng `/dev/lora_drone` @9600 đang hardcode trong `lora_drone.py`.

---

## 9. Testing

`test_logic.py` — **~52 unit tests** (stdlib, không cần hardware/Qt/ROS):
```bash
cd ~/ground_gui
python3 -m unittest test_logic -v     # chi tiết
python3 test_logic.py                  # gọn
```
Phủ: CRC framing (wrap/unwrap/bad_crc/tamper/plain/invalid/case), seq + `_prepare_frame`, session_id, `_chunk_list`, `_ack_matches` (khớp/sai event/stale/sai seq/null seq bị từ chối/sai mode/uploaded), waypoints (default z, skip lỗi, remove index, out-of-range), recent-GPS gating (global + local), `_reject_mission` → bridge callback, lifecycle (`stop()` tăng `_tx_epoch`, link timestamps, OFF skip, hb_watch reconnect, reconnect_loop, epoch change).

`test_protocol_integration.py` — **~6 integration tests** (không cần hardware):
```bash
python3 -m unittest test_protocol_integration -v
```
Phủ: ground retry sau drop ACK (`drop_first_events`), full chunked upload 5-waypoint sequence, drone publish-once + re-ACK duplicate commit, bad-CRC drop + plain-frame reject, session-id change → dedupe cache reset, commit với chunk thiếu → reject.

> Test CRC round-trip + tamper là "khoá" tương thích: nếu ai sửa nhầm `separators`/`sort_keys`/wrapper một phía → test đỏ ngay, tránh CRC drift âm thầm.

---

## 10. Bản đồ file & deploy

| Vai trò | Live (đang chạy) | Code chết (đừng đụng) |
|---|---|---|
| Ground app | `main.py`, `lora_bridge.py`, `control.py`, `index/map.html` | — |
| Drone bridge serial↔ROS | `lora_drone.py` → `lora_drone_package/lora_drone_package/lora_drone.py` | `lora_drone_draft.py`, `lora_drone_draft_2.py` |
| Drone gimbal bridge | `lora_gimbal_bridge_node.py` (không trong bringup.launch, chạy riêng khi cần) | — |
| Drone bay | `test.cpp` → executable **`test_mode`** (package `offboard_control`, launch `offboard_launch.py`) | `offboard_control.cpp`, `nhi.py` (comment trong CMakeLists) |
| Stack launcher | `launch/bringup.launch.py` → systemd `drone-bringup.service` | — |

**Luồng deploy (sshfs → Jetson → Docker → build):**
```
Sửa file trên laptop (~/drone_ws/src/... )
  → sync sang Jetson (sshfs / scp)
  → sudo systemctl stop drone-bringup
  → cd ~/drone_ws/script && ./launch_drone_container.sh
  → (trong container) colcon build --packages-select lora_drone_package offboard_control
  → exit
  → sudo systemctl start drone-bringup
```

**Quan trọng khi deploy:**
- `lora_drone.py` **không còn** ở `~/ground_gui/` — bản LIVE duy nhất ở `~/drone_ws/src/lora_drone_package/lora_drone_package/lora_drone.py` (đã tách module). Sửa ở đó rồi build/sync sang Jetson.
- `mavros` + `lora_drone` + `offboard_control` phải **cùng `ROS_DOMAIN_ID`** (Jetson dùng `ROS_DOMAIN_ID=7`). Khác domain → `/mode_signal` không thông → OFFBOARD báo "service not ready".
- `index/atlas.min.js|css` local **không dùng** (map.html nạp SDK từ CDN). Muốn chạy offline thì trỏ `<script src>` về file local.
- `lora_gimbal_bridge` **không** nằm trong `bringup.launch.py` — khởi động riêng khi cần dùng gimbal.

---

## 11. Giới hạn đã biết & việc tương lai

| Mục | Mô tả | Ưu tiên |
|---|---|---|
| **Auto-reconnect serial** | Ground có auto-reconnect (`_reconnect_loop`); drone chỉ retry lúc khởi động (`_reopen_serial`). Nếu rớt cáp giữa bay, drone node sẽ tự reconnect nhưng ground cũng cần điều chỉnh thời điểm trigger. | Field hardening |
| **LoRa airtime** | Telemetry mặc định 1s; nếu air-rate module thấp (< 19.2k), buffer đầy → `write()` chặn → ACK trễ. Giảm `LORA_DRONE_TELEMETRY_INTERVAL` hoặc cắt trường. | Tuning |
| **Protocol duplication** | Codec CRC giờ ở `protocol.py` **hai phía** (`ground_gui/protocol.py` và `lora_drone_package/.../protocol.py`) — vẫn là 2 file khác repo, phải **giữ đồng bộ**; test CRC bắt được drift. | Maintainability |
| **test.cpp warm-up** | `wait_for_fcu_connection()` + 100 setpoint chạy trong constructor → block ~5s, `/mode_signal` chưa phục vụ trong lúc đó. Nên đưa vào state machine của timer. | Polish |
| **OFFBOARD ACK semantics** | `on_mode_signal` trả `accepted=true` ngay (là "đã nhận trigger", chưa phải đã vào OFFBOARD/đã arm). Ground hiển thị ACK này ngay không phản ánh trạng thái thật của drone. | Tùy UX |
| **test_protocol_integration field names** | `AckingSerial._ack_for_payload` dùng key cũ `mission_op` (legacy), nhưng `control.py` hiện gửi key mới `op`. Test `test_chunked_upload_sequence_emits_success` cần cập nhật để khớp format thật. | Test correctness |
| **Carrot tuning** | `cruise_speed` và `carrot_lead` hiện hardcode qua ROS params (mặc định 2.0/2.5). Chưa có UI để điều chỉnh live. `carrot_lead ≈ cruise_speed × 1.0–1.3` là tỉ lệ lý tưởng. | UX |

---

*Cập nhật lần cuối: 2026-06-12. `control.py` đã tách module (`protocol.py`, `telemetry_parser.py`,
`commands.py`) — xem §2. Mọi thay đổi protocol phải đồng bộ `ground_gui/protocol.py` và
`lora_drone_package/.../protocol.py` (hai bản khác repo), và chạy lại `test/`.*
