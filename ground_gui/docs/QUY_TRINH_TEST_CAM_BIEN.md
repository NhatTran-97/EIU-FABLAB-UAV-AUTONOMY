# Quy trình test cảm biến & telemetry — Ground GUI ↔ Drone (LoRa)

Tài liệu mô tả cách kiểm tra toàn bộ luồng cảm biến từ drone về Ground GUI qua link
LoRa: vị trí, GPS, pin, tốc độ, trạng thái bay, heartbeat — và đường lệnh điều khiển.

---

## 1. Mục đích

Xác nhận **từng cảm biến** truyền đúng, đủ, đúng nhịp từ drone về UI; link ổn định,
không mất gói; và lệnh điều khiển (OFFBOARD/LAND/mission) có phản hồi ACK.

## 2. Luồng dữ liệu

```
MAVROS topics ──> lora_drone.py ──> LoRa(E32 19.2k) ──> lora_ground ──> control.py ──> LoraBridge ──> UI (map.html)
   (cảm biến)      (đóng gói JSON)      (RF 420MHz)        (serial)        (parse)        (Qt signal)     (hiển thị)
```

## 3. Bảng cảm biến — nguồn, gói tin, hiển thị, cách verify

| Cảm biến | ROS topic (drone) | Field trên wire | Hiển thị UI | Giá trị kỳ vọng |
|----------|-------------------|-----------------|-------------|-----------------|
| **Heartbeat** | (sinh mỗi chu kỳ) | `hb:1` | "Connected" + chấm xanh | có đều mỗi 0.5s |
| **Vị trí local** | `/mavros/local_position/pose` | `x,y,z` | marker local / panel | số hữu hạn, đổi khi drone di chuyển |
| **GPS** | `/mavros/global_position/global` | `lat,lon,alt`,`gps_ok` | marker trên map + "3D FIX" | lat/lon đúng vùng bay; gps_ok=1 khi có fix |
| **Pin** | `/mavros/battery` | `battery{percent,voltage}` | thanh pin % + V | percent 0–100, voltage hợp lý (vd 4S ~14–16.8V) |
| **Tốc độ** | `/mavros/local_position/velocity_local` | `speed` | ô tốc độ (m/s) | ≥0, tăng khi bay |
| **Trạng thái bay** | `/mavros/state` | `state` | top bar (MANUAL/STABILIZED/OFFBOARD/…) | khớp mode thực của FCU |

> Lưu ý nén: `state` và `battery` chỉ gửi **khi thay đổi** (hoặc refresh mỗi 15s) để tiết
> kiệm băng thông. Gói thường chỉ có `hb,gps_ok,x,y,z,speed` — đây là hành vi đúng.

## 4. Chuẩn bị

- [ ] Drone cấp nguồn, FCU + MAVROS chạy (các topic ở Bảng mục 3 có dữ liệu).
- [ ] 2 module LoRa E32 cùng **air-rate 19.2k** + cùng **channel 10 (420MHz)**
      (kiểm tra: `python3 e32_config.py --port <dev>` — xem [e32_config.py](e32_config.py)).
- [ ] Cổng cố định đã có: `/dev/lora_ground` (ground), `/dev/lora_drone` (drone)
      — udev [99-lora-ground.rules](99-lora-ground.rules) đã cài.
- [ ] Anten gắn đủ cho cả 2 module (KHÔNG cấp nguồn module thiếu anten).

## 5. Quy trình test

### Bước 1 — Kiểm tra cổng (chống nhầm radio)
```bash
ls -l /dev/lora_ground /dev/lora_telem
```
✔️ Đạt: `lora_ground -> ttyUSBx`. Nếu thiếu → cài lại udev (mục 7).

### Bước 2 — Khởi động drone node
Trên drone:
```bash
python3 lora_drone.py            # mặc định 2Hz (interval=0.5)
```
✔️ Đạt: log `✅ Serial connected at /dev/lora_drone` và `[STATE] MAVROS mode: ... -> ...`.

### Bước 3 — Đo sức khỏe link (KHÔNG cần mở UI)
Trên ground, **đảm bảo `main.py` đang TẮT** (nó độc chiếm cổng), rồi:
```bash
cd ~/ground_gui
python3 lora_link_check.py --secs 20
```
Đọc báo cáo — ✔️ **Đạt** khi:
- `valid packets` ≥ ~2×số giây, `malformed` thấp, **`loss rate < 5%`**
- `hb cadence` ≈ 0.50s (2Hz), ổn định (min/max sát nhau)
- `CRC status` = `plain` (telemetry không CRC — đúng thiết kế)
- Dòng cuối: `✅ LINK HEALTHY`

Mẫu gói giải mã in ra phải thấy `x,y,z`, thỉnh thoảng có `state`, `battery`.

### Bước 4 — Kiểm tra từng cảm biến trên UI
Tắt `link_check`, chạy:
```bash
python3 main.py
```
Đối chiếu theo Bảng mục 3:
- [ ] Top bar: "Connected" + flight state đúng mode FCU.
- [ ] GPS: "3D FIX" khi có fix; marker nằm đúng vị trí trên map.
- [ ] Pin: % và điện áp hợp lý, cập nhật khi pin đổi.
- [ ] Tốc độ: 0 khi đứng yên, tăng khi bay.
- [ ] Vị trí local: x/y/z đổi khi drone di chuyển.
- [ ] Rút/cắm lại nguồn module (hoặc tắt drone node) → UI báo mất kết nối, rồi
      tự kết nối lại khi có heartbeat trở lại.

### Bước 5 — Kiểm tra đường lệnh (command path)
- [ ] Bấm **OFFBOARD** trên UI → drone log `✅ Mode OFFBOARD -> accepted`, UI nhận ACK.
- [ ] Bấm **LAND** → tương tự.
- [ ] Gửi **mission** (nếu dùng) → UI báo "uploaded", drone log số waypoint đúng.

> Đường lệnh **bắt buộc có CRC**; telemetry thì không. Đây là khác biệt có chủ đích.

## 6. Tiêu chí Đạt / Không đạt

| Hạng mục | Đạt | Không đạt → xem mục 7 |
|----------|-----|----------------------|
| Cổng | symlink đúng | thiếu `lora_ground` |
| Heartbeat | đều, đúng nhịp | "Waiting Telemetry" kéo dài |
| Loss rate | < 5% | > 10% hoặc nhiều `malformed` |
| Cảm biến | tất cả khớp Bảng mục 3 | thiếu field / giá trị vô lý |
| Lệnh | có ACK | timeout không ACK |

## 7. Xử lý sự cố thường gặp

| Triệu chứng | Nguyên nhân | Cách xử lý |
|-------------|-------------|-----------|
| "Waiting Telemetry" mãi | telemetry vỡ gói / sai port / lệch air-rate | chạy `lora_link_check.py`; xem loss; kiểm tra 2 module cùng air-rate+channel |
| `loss rate` cao, nhiều `malformed` | quá tải air-rate (interval quá nhỏ) hoặc nhiễu/xa | tăng `LORA_DRONE_TELEMETRY_INTERVAL`; kiểm tra anten/khoảng cách |
| Mở `lora_ground` báo busy | `main.py` đang giữ cổng | tắt `main.py` trước khi sniff |
| Mất `/dev/lora_ground` | udev chưa nạp | `sudo udevadm control --reload-rules && sudo udevadm trigger --subsystem-match=tty` |
| Reconnect loop / nhầm radio | mở trúng FTDI MAVLink thay vì LoRa | đảm bảo `main.py` trỏ `/dev/lora_ground` (đã set) |
| Lệnh không ACK | drone bận / service chưa sẵn | kiểm tra log drone; `service not ready` → chờ MAVROS |

## 8. Phụ lục — chỉnh tham số

**Tần suất telemetry** (drone) — `LORA_DRONE_TELEMETRY_INTERVAL` (giây/gói):
| Giá trị | Tần suất | Ghi chú |
|---------|----------|---------|
| `1.0` | 1 Hz | tiết kiệm airtime, bay xa |
| `0.5` ⭐ | 2 Hz | **mặc định — sạch nhất, 0% mất** |
| `0.4` | 2.5 Hz | ~3% mất, jitter nhẹ |
| `0.3` | ~3 Hz | sàn an toàn; đừng thấp hơn (gói GPS chạm trần) |

**Air-rate LoRa** (cả 2 module phải giống nhau) — xem [e32_config.py](e32_config.py):
```bash
python3 e32_config.py --port /dev/lora_ground              # đọc config
python3 e32_config.py --port /dev/lora_ground --air 19.2k  # đặt (cần M0=M1=HIGH)
```

**Công cụ:**
- [lora_link_check.py](lora_link_check.py) — đo link/telemetry (dùng đúng parser của ground).
- [e32_config.py](e32_config.py) — đọc/đặt air-rate module E32.
- [99-lora-ground.rules](99-lora-ground.rules) — udev đặt tên cổng cố định.
