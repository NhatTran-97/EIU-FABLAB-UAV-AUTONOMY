# aruco_detection

Nhìn thấy marker → biết marker ở đâu so với drone. Đầu vào của bộ hạ cánh chính xác.

## Kiến trúc

Hai node, mỗi node một việc, nối bằng topic:

```
camera ──► aruco_node ──/pose (hệ quang học)──┐
                                              ├──► marker_localizer ──► ~/marker_rel  (ENU, đã khử nghiêng)
MAVROS ──/mavros/local_position/pose──────────┘                        ~/height
```

Tách làm hai vì **kích thước rosbag**: ghi hai topic pose là ~3 MB cho 10 phút
bay, replay bao nhiêu lần cũng được khi chỉnh bộ lọc và bộ điều khiển. Gộp lại
thì đầu vào là ảnh — 10 phút 1080p là ~100 GB, và Jetson không ghi nổi.

`aruco_node` chạy được một mình (không cần MAVROS) để hiệu chuẩn camera và
chỉnh detect. `marker_localizer` thì cần cả hai nguồn.

## Vì sao cần node thứ hai

`tvec` trả lời "marker ở đâu so với ống kính". Hai tình huống cho ra **cùng
một `tvec`**:

- drone thăng bằng, marker lệch phải 52 cm
- drone **ở ngay trên marker**, đang nghiêng 10°

Camera không phân biệt được — chỉ attitude từ FCU mới tách được. Đưa thẳng
`tvec` vào bộ điều khiển thì ở tình huống thứ hai drone nghiêng thêm để sửa một
sai số không tồn tại, càng nghiêng càng lệch.

Xem `test/test_geometry.py::test_tilt_does_not_move_a_stationary_marker`.

## Cấu trúc

```
aruco_detection/          module python, chỉ thứ node import
├── aruco_node.py             node 1 — detect + pose (hệ quang học)
├── marker_localizer_node.py  node 2 — + attitude → ENU khử nghiêng
├── detector.py               ArUco/Fractal + solvePnP
├── load_camera.py            nguồn USB / RTSP / file
├── calibration.py            đọc nội tham số
├── geometry.py               toàn bộ toán hệ toạ độ
└── visualization.py          overlay debug
scripts/                  công cụ chạy tay, KHÔNG phải node
test/                     unit test (không cần ROS)
```

`geometry.py` chia ba tầng: biểu diễn phép quay → hệ quang học → hệ bay MAVROS.
Chỉ tầng 3 chạm vào quy ước drone. Đổi sang px4_msgs thì viết file **khác**,
đừng sửa tầng 3 — NED/FRD và ENU/FLU lệch nhau 180° quanh trục x cộng thứ tự
phần tử quaternion, trộn lẫn thì số trông hợp lý nhưng sai dấu y và z.

## Chạy

```bash
colcon build --packages-select aruco_detection    # nhớ conda deactivate trước
source install/setup.bash

ros2 launch aruco_detection aruco_pose_launch.py \
    camera_source:=2 detector_type:=fractal marker_size:=0.282

ros2 run aruco_detection marker_localizer_node.py     # cần MAVROS đang chạy
```

## Topic

**aruco_node**

| Topic | Kiểu | Ghi chú |
|---|---|---|
| `~/pose` | PoseStamped | MÉT, hệ quang học camera |
| `~/detected` | Bool | |
| `~/marker_id` | Int32 | marker nào đang được dùng |
| `~/ambiguity` | Float32 | gần 1.0 = **xoay không tin được** |
| `~/reprojection_error` | Float32 | RMS pixel (Fractal) |
| `~/image` | Image | overlay debug |

**marker_localizer**

| Topic | Kiểu | Ghi chú |
|---|---|---|
| `~/marker_rel` | PointStamped | so với drone, ENU, đã khử nghiêng |
| `~/marker_pose` | PoseStamped | trong hệ local ENU của MAVROS |
| `~/height` | Float32 | khoảng cách **thẳng đứng** [m] |
| `~/valid` | Bool | |

## Tham số đáng chú ý

| Tham số | Mặc định | |
|---|---|---|
| `publish_rate` | 60.0 | Poll **nhanh hơn** camera. Hai đồng hồ 30 Hz không đồng bộ; hai frame về giữa hai lần poll thì frame đầu mất luôn. |
| `camera_source` | `"2"` | `"2"`=webcam · `rtsp://192.168.144.108:554/stream=1`=Skydroid · `/path/clip.mp4`=replay. **Một nguồn tại một thời điểm** — đừng để node `skydroid` mở cùng lúc, gimbal sẽ có hai kết nối RTSP. |
| `rtsp_latency_ms` | 100 | Jitter buffer, chỉ áp dụng cho RTSP. Là **một phần đã biết** của `pipeline_latency_s`. |
| `pipeline_latency_s` | 0.0 | Trễ trước khi frame tới `read()`. **Phải đo** — chĩa camera vào đồng hồ bấm giây. Ở 2 m/s, 100 ms = 20 cm sai vị trí. |
| `camera_timeout_s` | 1.0 | Ngưỡng tách "chưa có frame mới" (bình thường) khỏi "camera chết". |
| `debug_image_every` | 1 | Publish mỗi frame (~30 FPS). Tăng lên 2–6 với ảnh lớn để giảm băng thông DDS. |
| `mount_roll_deg` / `mount_pitch_deg` | 0.0 | Sai số góc lắp. Lệch 2° ở 5 m = **17 cm** sai số hệ thống, không bộ lọc nào khử được. |

## Hiệu chuẩn góc lắp — làm trước khi bay

1. Đặt marker dưới sàn, cầm drone **thăng bằng** ngay trên nó. Đo bằng thước,
   so với `~/height`.
2. Giữ nguyên chỗ đứng, **nghiêng drone** 10–15° các hướng. `~/height` phải
   không đổi, `~/marker_rel` x,y phải giữ nguyên.
3. Nghiêng làm x,y trôi → góc lắp sai. Chỉnh `mount_roll_deg`/`mount_pitch_deg`
   đến khi hết trôi.

Bước 3 là cách duy nhất tách sai số lắp khỏi các sai số khác, và không làm được
trên không.

## Test

```bash
colcon test --packages-select aruco_detection
```

`test/test_geometry.py` thuần numpy, không cần ROS. Sai hệ toạ độ là loại lỗi
bay thử gần như không phát hiện được — số trông hợp lý, drone chỉ lệch sai hướng.

## Chưa làm

Ước lượng vận tốc marker, lọc pose nhảy, xử lý mất marker, state machine hạ
cánh, giới hạn setpoint. `pipeline_latency_s` cũng chưa có số đo thật.
