# Mô hình 3D drone EIU (trang 3D của GCS)

Dựng bằng script Blender theo tham số → `.glb` → `pyside6-balsam` → `qml/models/eiu_drone/EiuDrone.qml`.
Trang 3D (`qml/pages/View3DPage.qml`) dùng `EiuDrone { scale: 100 }` trong node `base_link`.

| File | Vai trò |
|---|---|
| `build_drone.py` | dựng drone theo `PARAMS` (mm đầu file): khung, tay, động cơ SunnySky, cánh, chân, hộp, lidar, GPS |
| `build.sh` | chạy lại tất cả: Blender → glb → balsam → `qml/models/eiu_drone/` + ảnh xem trước |
| `c12.glb` | gimbal **Skydroid C12** chính hãng (từ STEP GrabCAD, đã giảm tam giác) |
| `x2216.glb` | động cơ **SunnySky X2216** chính hãng (STEP GrabCAD) — đặt ở 4 động cơ |
| `gps_m9n.glb` | GPS **Holybro M9N** (cùng vỏ với M10 trên drone) — trên cột phía sau, mũi tên về mũi drone |
| `step2stl.py`, `convert_part.py` | đổi STEP → `.glb` (preset `c12`, `x2216`, `gps_m9n`); chỉ chạy lại khi đổi file STEP |
| `add_halo.py` | thêm viền sáng cho `images/drone_top.png` |

File sinh ra khi chạy `build.sh`, không commit (`.gitignore`): `eiu_drone.blend` (cảnh Blender để chỉnh tay),
`preview*.png` (ảnh xem nhanh). File STEP gốc không nằm trong repo (bản quyền của tác giả trên GrabCAD).

**Hệ trục**: X = mũi (phía gimbal; mũi tên GPS chỉ về đây), Y = trái, Z = lên, mét, **gốc = mạch PX4 (`base_link`)**.
Lidar: tâm quét cao hơn PX4 115 mm (khớp TF `drone_bringup/launch/static_tf.launch.py`).

## Dựng lại sau khi sửa `PARAMS`

```bash
cd ~/drone_ws/src/ground_station_ui && ./tools/drone_model/build.sh
```
Dùng **Blender 5.x ở `/snap/bin/blender`** (bản apt 3.0 ở `/usr/bin/blender` quá cũ). Mở lại app để thấy thay đổi.

## Đổi file STEP linh kiện (hiếm khi cần)

Tải STEP từ GrabCAD vào `tools/drone_model/` (Skydroid C12, SunnySky X2216, Holybro M9N). Blender không đọc
STEP → dùng gmsh (OpenCASCADE) chia lưới thành STL từng khối:
```bash
pip download --no-deps gmsh -d /tmp/gmsh && cd /tmp/gmsh && unzip -q gmsh-*.whl -d x
python3 tools/drone_model/step2stl.py /tmp/gmsh/x "tools/drone_model/Камера Skydroid C12.stp" /tmp/c12_parts
/snap/bin/blender --background --python tools/drone_model/convert_part.py -- c12 /tmp/c12_parts tools/drone_model/c12.glb
python3 tools/drone_model/step2stl.py /tmp/gmsh/x "tools/drone_model/Sunnysky X2216 Motor.STEP" /tmp/x2216_parts
/snap/bin/blender --background --python tools/drone_model/convert_part.py -- x2216 /tmp/x2216_parts tools/drone_model/x2216.glb
python3 tools/drone_model/step2stl.py /tmp/gmsh/x "tools/drone_model/GPS Holybro M9N.step" /tmp/gps_parts
/snap/bin/blender --background --python tools/drone_model/convert_part.py -- gps_m9n /tmp/gps_parts tools/drone_model/gps_m9n.glb
./tools/drone_model/build.sh
```
`convert_part.py`: xoay STEP về hệ drone (trước = +X, lên = +Z), mm → m, đặt gốc (C12: mặt trên tấm treo;
X2216: tâm đáy đế), gán màu theo số khối (`step2stl.py` in bbox từng khối), giảm tam giác. Thiếu `.glb` thì
`build_drone.py` tự dùng bản tự dựng.
