# ground_station_ui — Drone GCS

App vận hành drone trong lúc bay (PySide6 + QML). Gom vào một app những gì trước đây phải mở
QGC + GUI LoRa cùng lúc. **Nạp firmware, cấu hình, hiệu chuẩn PX4 vẫn dùng QGroundControl**
(tắt app này trước — mỗi cổng serial chỉ một app mở được).

| Link | Đường truyền | Nội dung |
|---|---|---|
| `px4` | radio SiK 915 MHz | MAVLink chuẩn PX4: heartbeat, mode, GPS, pin, sensor, độ cao, STATUSTEXT |
| `jetson` | radio SiK 433 MHz (NETID 77) | `drone_telemetry`: trạng thái + tần số lidar / gimbal / camera / PX4-DDS |
| `sitl` | UDP 14550 | PX4 SITL trên laptop — test giao diện không cần drone |

Thêm / bớt link, lớp bản đồ: [config/gcs.yaml](config/gcs.yaml), không sửa code.

## Bản đồ offline + waypoint

Cách giống QGC: bản đồ online + **cache tải sẵn**. QtLocation lấy ô bản đồ từ tile server cục bộ
trong app; tile server trả ô từ file `.mbtiles` (SQLite) trong `data/maps/` trong package,
ô chưa có thì tải từ nguồn online (nếu có mạng) rồi lưu lại. Mất mạng → chỉ dùng cache.

- **Lớp** (chọn ở tab **Cài đặt**, được ghi nhớ): *Hybrid* (mặc định: ảnh Google — cùng nguồn QGC, nét nhất ở khu bay — + nhãn Esri),
  *Azure* (ảnh Maxar/Bing, cần key, xem dưới). Nút ⚑: ẩn/hiện danh sách waypoint.
  Lớp ghép (Hybrid = ảnh Google + nhãn đường/địa danh Esri; Azure) do tile server ghép từ nhiều nguồn, mỗi nguồn cache riêng.
- **Azure Maps**: key trong `config/keys.yaml` (`AZURE_MAPS_KEY: "..."`, đã .gitignore) hoặc biến môi trường `AZURE_MAPS_KEY`.
  Không có key → lớp chỉ đọc cache; cache cũng trống → lớp bị ẩn.
- **Zoom sâu hơn 19**: QtLocation tự phóng to ô z19 — chi tiết bị giới hạn bởi ảnh gốc của nguồn.
- **Trước khi ra hiện trường** (còn mạng): tab Map → kéo tới vùng bay → nút ⬇ → chọn zoom → *Tải về*.
  Làm cho từng lớp (Hybrid tự tải cả 3 nguồn). Vùng 2×2 km zoom tới 19 ≈ vài chục MB / lớp.
- **Ảnh drone tự chụp** (nét nhất, hợp lệ): WebODM → `orthophoto.tif` → `gdal2tiles.py --xyz`
  / `rio mbtiles` → thêm lớp `file: ortho.mbtiles` (không có `url`) trong `gcs.yaml`.
- **Tìm vị trí**: ô góc trên trái, nhập `11.052919, 106.666057` (copy từ Google Maps được) → Enter:
  bản đồ bay tới + ghim đỏ; *＋ Waypoint* để thêm luôn điểm đó. App nhớ vùng xem cuối cùng.
- **Môi trường bay** (tab Cài đặt: Tự động / Trong nhà / Ngoài trời): chọn nguồn độ cao
  (trong nhà = cảm biến khoảng cách qua EKF, ngoài trời = so với Home), độ cao cất cánh mặc định,
  màu cảnh báo GPS; app đọc `EKF2_GPS_CTRL` / `EKF2_HGT_REF` từ PX4 và **cảnh báo nếu PX4 đang nạp
  bộ tham số không khớp** (vd ra ngoài trời mà quên nạp `outdoor.params`).
- **Đồng hồ** (góc trên phải): tư thế (roll / pitch), la bàn, độ cao, tốc độ lên/xuống và ngang — từ `ATTITUDE`, `VFR_HUD`, `LOCAL_POSITION_NED` của PX4.
- **Waypoint**: *＋ Thêm điểm* rồi bấm lên bản đồ; kéo điểm để di chuyển; chuột phải để xóa;
  sửa độ cao trong danh sách bên phải.
- **Vùng cấm / hạn chế bay** (nút ⛔: bật/tắt, nhấn giữ = nạp lại file): chép `.geojson` hoặc
  OpenAir (`.txt`) vào `data/airspace/`. Tô màu giống cambay.mod.gov.vn: đỏ (cấm), cam (hạn chế),
  vàng (nguy hiểm), xanh (CTR sân bay); tick trong chú thích để ẩn/hiện từng loại; waypoint / ghim nằm trong vùng hiện ⚠. GeoJSON:
  Polygon / MultiPolygon, hoặc Point + `radius_m`; `properties: name, kind, lower, upper`.
  Nguồn chính thức của Bộ Quốc phòng: tra cứu trên http://cambay.mod.gov.vn (chưa có API công khai)
  → vẽ lại vùng quanh khu bay bằng geojson.io. App chỉ **cảnh báo**, không thay giấy phép bay.
- Điều khoản: Esri / Google / Bing hạn chế tải hàng loạt — tự cân nhắc nguồn trong `gcs.yaml`.

## Trang 3D (thay RViz)

Lưới sàn, trục `map` (gốc EKF của PX4), drone + trục `base_link`, vệt bay; chuột trái xoay, cuộn
zoom, chuột phải kéo; *Bám drone* / *Xóa vệt*. Nguồn: Jetson subscribe `/fmu/out/vehicle_odometry`
(PX4 qua uXRCE-DDS) → `DRONE_LOCAL_POSE` (id 45001, ~57 B, 5 Hz ≈ 300 B/s) → radio 433 → app.

Quy ước: truyền **NED** (giữ nguyên PX4); app đổi **một lần** sang **ENU / FLU** như ROS
(`core/geometry/frames.py`, giống `px4_ros_com`) để lidar, ArUco, TF từ Jetson dùng thẳng.
Qt Quick 3D là Y-lên: scene = (E, U, −N) × 100 (1 m = 100), chỉ ở `qt/geometry3d.py` và
`View3DPage.qml`.

Sau này: marker ArUco (`Axes3D`).

## Nhận diện EIU

`images/logo.png` (logo gốc) · `images/logo_dark.png` (bản nền tối: phần navy → trắng, vạch vàng giữ
nguyên — sinh từ logo gốc) · `images/eiu.png` (ảnh campus). Logo nhỏ ở đầu menu; splash ~7 s lúc mở
app (ảnh campus + logo, bấm để bỏ qua, `holdMs` trong `qml/components/Splash.qml`); Settings → About.
Không đặt ảnh / logo lớn trong các trang vận hành (Map, 3D, Telemetry, Diagnostics).

**Icon ứng dụng:** `images/icons/drone-gcs-*.png` (sinh từ `images/logo_desktop.png`, nền ngoài khung
→ trong suốt). Cửa sổ dùng icon này; lối tắt trong menu ứng dụng / dock:
`scripts/install_desktop.sh` (chạy trong `drone_env`; gỡ: `--uninstall`).

## Cảnh báo & an toàn

- **Thanh trên cùng:** chế độ bay chữ to + huy hiệu **ARMED** (đỏ) / DISARMED; pin có biểu tượng
  ⚠ (≤ 25 %) / ⛔ (≤ 15 %), không chỉ đổi màu.
- **Alert Center** (nút ⚠ n góc phải): danh sách vấn đề hiện tại — cùng nguồn với **System Health**
  đầu tab Diagnostics và **Pre-flight check** trong hộp xác nhận Take-off (có lỗi nghiêm trọng →
  khóa nút cất cánh). Quy tắc: `core/alerts/health.py`.
- **Giọng nói** (tiếng Anh, tắt / bật ở Settings): đổi chế độ, arm / disarm, mất kết nối, pin 30/20/15/10 %.

## Take-off / Land / Gửi waypoint (→ Jetson)

Chi tiết cách hoạt động, luật an toàn, giao thức ACK, cách thử: [docs/flight_commands.md](docs/flight_commands.md).

Thanh nút giữa dưới tab Map, chỉ bật khi có heartbeat Jetson (radio 433). Take-off / Land phải
**giữ 1 giây** để xác nhận; gửi waypoint cảnh báo nếu có điểm nằm trong vùng cấm / hạn chế.

| Nút | MAVLink gửi tới sysid 1 / compid 191 (link `jetson`) | Jetson trả về |
|---|---|---|
| Take-off | `COMMAND_LONG` `MAV_CMD_NAV_TAKEOFF` (22), `param7` = độ cao m | `COMMAND_ACK` |
| Land | `COMMAND_LONG` `MAV_CMD_NAV_LAND` (21) | `COMMAND_ACK` |
| Hold | `COMMAND_LONG` `MAV_CMD_NAV_LOITER_UNLIM` (17) → Jetson: `DO_SET_MODE` AUTO.LOITER | `COMMAND_ACK` |
| RTL | `COMMAND_LONG` `MAV_CMD_NAV_RETURN_TO_LAUNCH` (20) — chỉ khi ngoài trời (cần GPS) | `COMMAND_ACK` |
| Gửi WP | `MISSION_COUNT` → (Jetson xin) `MISSION_REQUEST_INT` → `MISSION_ITEM_INT` (`GLOBAL_RELATIVE_ALT_INT`, `NAV_WAYPOINT`) | `MISSION_ACK` |

Không có phản hồi → gửi lại (`commands.ack_timeout_s`, `retries` trong `gcs.yaml`), hết lượt → báo
"Jetson not responding". Phía Jetson: `drone_telemetry/commands.py` chuyển lệnh thành PX4
`/fmu/in/vehicle_command` (Take-off = ARM rồi NAV_TAKEOFF, PX4 lên độ cao rồi tự Hold), chờ
`vehicle_command_ack`, trả `COMMAND_ACK` (IN_PROGRESS giữ chỗ, rồi kết quả) + `STATUSTEXT` lý do.
Luật an toàn ở Jetson: Take-off chỉ khi ở mặt đất; Land / Hold / RTL chỉ khi đang bay; không có dữ
liệu PX4 → từ chối. Waypoint: Jetson **chỉ lưu** (publish `/gcs/mission`, JSON) — chưa bay theo.
Thanh nút đổi theo trạng thái: mặt đất → Take-off / Send WP; đang bay → Send WP / Hold / RTL / Land.

## Dữ liệu nằm trong package

`data/maps/*.mbtiles` (cache bản đồ offline), `data/airspace/` (vùng cấm bay), `config/keys.yaml`
(API key). Chép cả thư mục `ground_station_ui` sang máy khác là chạy được với bản đồ đã tải.
`*.mbtiles` và `keys.yaml` nằm trong `.gitignore` (nặng / bí mật) — gửi qua git thì không đi kèm.

## Chạy

```bash
pip install -r requirements.txt     # PySide6, pyserial, pyyaml, Pillow (PySide6 không có trong rosdep)

# Build như package ROS 2
cd ~/drone_ws && colcon build --packages-select ground_station_ui
source install/setup.bash
ros2 run ground_station_ui gcs

# Hoặc chạy thẳng từ source (sửa QML/Python xong chạy lại ngay)
python3 ~/drone_ws/src/ground_station_ui/ground_station_ui/main.py
```

Lần đầu: tab **Links** → chọn cổng → Connect. Cổng được ghi nhớ, tự kết nối lại lần sau.
Thoát: đóng cửa sổ hoặc `Ctrl+C` trong terminal.

Test với SITL: `cd ~/px4/PX4-Autopilot && make px4_sitl gz_x500` → tab Links → `PX4 SITL` → Connect.

## Kiến trúc

```
┌─ QML ── mỗi tính năng 1 trang (qml/pages), menu sinh từ qt/features.py ─────────────┐
├─ qt/ ── adapter: FactGroup -> QML (values / items), cập nhật tối đa ui_rate_hz ─────┤
├─ core/ (Python thuần, test bằng pytest, không Qt) ──────────────────────────────────┤
│  vehicle/  Vehicle = các FactGroup: status, gps, home, battery, sensors, flight,     │
│            messages, companion. Mỗi group tự đăng ký message nó cần.                 │
│  mission/  Mission: danh sách waypoint đang soạn                                    │
│  maps/     MBTiles cache, TileServer (localhost), Prefetcher (tải sẵn vùng)          │
│  router.py MessageRouter: subscribe('GPS_RAW_INT', handler) — không có if/elif dài   │
│  links/    Link (lớp chung) -> SerialLink, UdpLink; LinkManager tạo từ gcs.yaml      │
└─────────────────────────────────────────────────────────────────────────────────────┘
```

**Luồng dữ liệu:** thread của mỗi link đọc + parse → gửi **cả lô** message về main thread →
`MessageRouter.dispatch` → các FactGroup cập nhật `values` / `items` và bật cờ `dirty` →
`UiTicker` (5 Hz) gọi `tick()` (tuổi heartbeat, STALE...) rồi `flush()` adapter → QML vẽ lại.
Toàn bộ domain chạy trên main thread nên không cần khóa; thread chỉ có ở tầng link.

**Mở rộng:**

| Muốn thêm | Làm |
|---|---|
| Link mới (serial / UDP) | thêm vào `config/gcs.yaml` |
| Loại link mới (TCP, replay .tlog...) | lớp con của `core/links/base.py:Link` (4 hàm I/O) + đăng ký trong `LINK_TYPES` |
| Dữ liệu mới từ PX4 | 1 `FactGroup` trong `core/vehicle/groups.py`, thêm vào `Vehicle.groups` + 1 property trong `qt/adapters.py:VehicleAdapter` |
| Tính năng / trang mới | 1 dòng trong `qt/features.py` + 1 file `qml/pages/*.qml` |
| Nhiều drone | tạo nhiều `Vehicle` theo sysid — các group đã lọc theo `vehicle.is_autopilot()` |

**Hiệu năng:** parse trên thread link, chuyển message theo lô; UI chỉ cập nhật khi có thay đổi
và tối đa `ui_rate_hz`; Repeater trong QML dùng **số lượng** làm model để không tạo lại delegate
mỗi lần cập nhật (giữ nguyên lựa chọn trong ComboBox, ít tốn CPU).

## Dialect MAVLink

`ground_station_ui/core/mavlink/drone_dialect.py` là bản copy của
`drone_telemetry/drone_telemetry/drone_dialect.py` (sinh từ `drone_telemetry/mavlink/drone_telemetry.xml`).
Sửa XML → chạy `scripts/gen_dialect.sh` bên `drone_telemetry` → copy file `.py` sang đây.
Hai đầu phải dùng **cùng** một file.

## An toàn

Mỗi link gửi HEARTBEAT của GCS trên thread riêng. Nếu app tắt/treo khi đang bay, PX4 mất
heartbeat GCS và xử lý theo tham số `NAV_DLL_ACT` — kiểm tra tham số này bằng QGC trước khi bay.

## Test

```bash
cd ~/drone_ws/src/ground_station_ui && python3 -m pytest test/
```

## Lộ trình

1. ✅ Links, thanh trạng thái, Diagnostics, thông số bay; kiến trúc tầng
2. ✅ Map offline (cache MBTiles, tải sẵn vùng), vị trí drone, Home, soạn waypoint trên map
3. ✅ Take-off / Land / Gửi waypoint xuống Jetson (ACK + gửi lại) — ⏳ Jetson xử lý lệnh; RTL
4. HUD, video RTSP
5. Ghi / phát lại `.tlog` (link replay) để debug không cần drone
