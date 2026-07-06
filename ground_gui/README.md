# Ground Control Station — LoRa Drone

Ứng dụng điều khiển và giám sát drone qua radio **LoRa E32-433T20D** (9600 baud UART,
19.2 kbps OTA). Giao diện desktop PyQt6 với bản đồ Azure Maps tích hợp, gửi lệnh qua LoRa
về companion computer (Jetson Nano), nhận telemetry vị trí + pin mỗi 1s.

Kiến trúc đầy đủ (pipeline, giao thức wire, threading, reliability): [ARCHITECTURE.md](ARCHITECTURE.md).

---

## Yêu cầu

| Thành phần | Phiên bản |
|---|---|
| Ubuntu | 22.04 LTS |
| Python | 3.10+ |
| PyQt6 + WebEngine | `pip install PyQt6 PyQt6-WebEngine pyserial` |
| Radio LoRa | EBYTE E32-433T20D trên `/dev/lora_ground` |

---

## Cài đặt udev rules (một lần duy nhất)

Tạo symlink ổn định cho USB radio — không bị đổi tên sau reboot/cắm lại:

```bash
cd ~/ground_gui
./install_lora_rules.sh          # tự xin sudo, cài vào /etc/udev/rules.d/
# Cắm lại cáp USB radio sau khi chạy xong
ls -la /dev/lora_ground /dev/lora_telem 2>/dev/null   # kiểm tra symlink
```

Chi tiết mapping USB → symlink: [99-lora-ground.rules](99-lora-ground.rules).

---

## Chạy ứng dụng

```bash
cd ~/ground_gui

# Chạy bình thường (mở /dev/lora_ground @9600 mặc định):
python3 main.py

# Ẩn log libva / Chromium GPU (khuyên dùng trên desktop):
python3 run_silent.py

# Cổng / baudrate tùy chỉnh:
GROUND_GUI_SERIAL_PORT=/dev/ttyUSB0 GROUND_GUI_BAUDRATE=9600 python3 main.py

# Bật log từng packet để debug:
GROUND_GUI_DEBUG=1 python3 main.py
```

**Luồng sau khi mở app:**
1. Bấm **CONNECT** → ứng dụng mở serial, gửi `ON`, bắt đầu đọc telemetry.
2. Khi nhận được heartbeat đầu tiên → thanh trạng thái báo **Link UP**.
3. Bấm **OFFBOARD** → drone arm → takeoff → hover → tự land sau `hover_seconds`.
4. Vẽ waypoint trên bản đồ → **Send Mission** → drone fly-through từng điểm theo carrot/lookahead.
5. Bấm **LAND** bất kỳ lúc nào → drone hạ cánh ngay.

---

## Cấu hình (env vars)

| Biến | Mặc định | Tác dụng |
|---|---|---|
| `GROUND_GUI_SERIAL_PORT` | `/dev/lora_ground` | Cổng serial LoRa ground |
| `GROUND_GUI_BAUDRATE` | `9600` | Baudrate |
| `GROUND_GUI_HTTP_PORT` | `8000` | Cổng HTTP phục vụ `index/map.html` |
| `GROUND_GUI_DEBUG` | `0` | `1` = log từng packet (RAW/DATA/CRC) |
| `GROUND_GUI_HB_TIMEOUT` | `15.0` | Giây không có heartbeat → link DOWN + auto-reconnect |
| `GROUND_GUI_MISSION_CHUNK_SIZE` | `4` | Số waypoint mỗi chunk (tăng nếu link khoẻ) |
| `GROUND_GUI_AUTO_RECONNECT` | `1` | `0` = tắt auto-reconnect khi link mất |
| `GROUND_GUI_SEND_OFF_ON_STOP` | `0` | `1` = gửi token `OFF` khi DISCONNECT |
| `LORA_DRONE_DEBUG` | `0` | `1` = log packet ở drone side (ROS2) |
| `LORA_DRONE_TELEMETRY_INTERVAL` | `1.0` | Giây giữa các telemetry packet của drone |

---

## Test

Không cần phần cứng, không cần Qt, không cần ROS2:

```bash
cd ~/ground_gui

# Toàn bộ (kỳ vọng: 56 passed, 4 skipped):
python3 -m pytest test/ -q

# Hoặc riêng từng file:
python3 -m unittest test.test_logic -v
python3 -m unittest test.test_protocol_integration -v
```

> 4 ca trong `test_protocol_integration` đang **skip**: chúng poke internal của `lora_drone`
> bản monolith cũ — logic đó đã chuyển vào các module của `lora_drone_package` và có unit-test
> riêng ở đó (idempotency / mission_assembler / protocol). Xem chú thích trong file test.

---

## Tiện ích

### `e32_config.py` — Cấu hình module LoRa

Đọc hoặc thay đổi air data rate của module E32. Cần **M0=M1=3.3V** (CONFIG mode) trước khi chạy.

```bash
# Đọc cấu hình hiện tại (an toàn, không ghi):
python3 e32_config.py --port /dev/lora_ground

# Đặt air rate 19.2 kbps (tốc độ cao nhất, tầm ngắn hơn):
python3 e32_config.py --port /dev/lora_ground --air 19.2k

# Sau khi xong: M0=M1=GND rồi power-cycle module.
# Cấu hình cả HAI module (drone + ground) CÙNG air rate + channel!
```

### `lora_link_check.py` — Đo sức khoẻ LoRa link

Nghe trực tiếp trên serial và báo goodput, CRC, heartbeat rate. **Đóng `main.py` trước** (app đang giữ cổng).

```bash
python3 lora_link_check.py                          # /dev/lora_ground, 14 giây
python3 lora_link_check.py --secs 30                # đo lâu hơn
python3 lora_link_check.py --port /dev/ttyUSB0      # cổng khác
```

---

## Cấu trúc file

`control.py` được tách module (controller chỉ điều phối; codec/parse/lệnh ở module thuần). Chức năng
chi tiết từng file: [ARCHITECTURE.md §2](ARCHITECTURE.md#2-thành-phần--vai-trò-file).

```
ground_gui/
│
├── main.py                       # Cửa sổ PyQt6: HTTP server, QWebEngineView, QWebChannel
├── lora_bridge.py                # LoraBridge(QObject): signals JS→Py, slots Py→JS
├── control.py                    # GroundController: orchestrator (serial, reconnect, seq/ACK, retry, RX)
│   #  --- module tách ra (THUẦN, test offline được) ---
├── protocol.py                   # Codec khung LoRa: CRC32 wrap/unwrap, clean JSON, id/chunk helpers
├── telemetry_parser.py           # parse_frame(data) → ParsedFrame (pos/battery/speed/state/ack)
├── commands.py                   # Builder lệnh: cmd_offboard/land, mission_begin/chunk/commit
├── run_silent.py                 # Chạy main.py không log GPU/libva
│
├── index/
│   ├── map.html                  # UI bản đồ Azure Maps, controls, telemetry overlay
│   ├── atlas.min.js              # Azure Maps SDK local (hiện dùng CDN trong map.html)
│   └── atlas.min.css             # Azure Maps styles local
│
├── ui/main.ui                    # Qt Designer layout (nạp bằng uic.loadUi)
│
├── rules/
│   ├── 99-lora-ground.rules      # udev: USB serial → /dev/lora_ground, lora_telem, lora_ground_drone
│   └── install_lora_rules.sh     # Cài udev rules + reload + kiểm tra symlink
│
├── lora_config/e32_config.py     # CLI cấu hình module E32 (air rate, channel)
├── lora_link_check.py            # Đo goodput/CRC/heartbeat từ serial
│
├── docs/ARCHITECTURE.md          # Kiến trúc đầy đủ: pipeline, protocol, threading, vai trò từng file
└── test/
    ├── test_logic.py             # Unit tests: CRC, seq/ACK, waypoints, GPS gating, lifecycle
    └── test_protocol_integration.py  # Integration tests ground↔drone (4 ca drone-side đã skip — xem §2)
```

> **Lưu ý deploy:** `lora_drone.py` (node drone) **không còn ở đây** — bản LIVE ở package ROS2
> `lora_drone_package` (`~/drone_ws/...`), đã tách module. Xem
> [ARCHITECTURE.md §10](ARCHITECTURE.md#10-bản-đồ-file--deploy).
