# obstacle_avoidance

Né vật cản bằng **VFH+** từ lidar 2D 360° (`/scan`) cho drone PX4. Node **chỉ đề xuất**
setpoint (`/avoidance/status`). `mission_manager` quyết định có dùng hay không, và là node
duy nhất publish `/mission/target_position`.

```
/scan ─────────────────────────────┐
/fmu/out/vehicle_local_position_v1 ┤                      /avoidance/status
/fmu/out/vehicle_attitude ─────────┼─► avoidance_node ──────────────────────► mission_manager ─► /mission/target_position ─► offboard_control
/mission/goal  ◄───────────────────┤                                               │
/avoidance/enable ◄────────────────┘◄──────────────────────────────────────────────┘
                     (tùy chọn) ──► /fmu/in/obstacle_distance  (PX4 Collision Prevention)
```

| File | Vai trò |
|---|---|
| `vfh_plus.py` | Thuật toán VFH+ thuần numpy (histogram, hysteresis, valley, cost) |
| `scan_processing.py` | Lọc scan, đổi hệ lidar → FLU → FRD → NED bằng quaternion, lọc tia chạm đất |
| `avoidance_node.py` | ROS wrapper, timeout/STALE, tính carrot, ObstacleDistance |
| `custom_msgs/msg/AvoidanceStatus.msg` | `CLEAR / AVOIDING / BLOCKED / STALE` + `safe_setpoint` |

## Build, test, chạy

```bash
conda deactivate                       # colcon cần catkin_pkg của python hệ thống
colcon build --packages-select custom_msgs obstacle_avoidance offboard_control
source install/setup.bash
colcon test --packages-select obstacle_avoidance && colcon test-result --verbose

ros2 launch obstacle_avoidance avoidance_launch.py
ros2 run offboard_control mission_manager.py
ros2 topic echo /avoidance/status
ros2 topic echo /mission/nav_mode      # NOMINAL / AVOIDING / HOLDING
```

## Hành vi của mission_manager

| `/avoidance/status` | Manager |
|---|---|
| CLEAR / AVOIDING | bay theo `safe_setpoint` (carrot cách vị trí hiện tại `lookahead × speed_scale`) |
| BLOCKED, STALE, hoặc không nhận được status > `avoidance_timeout` | **HOLD** tại vị trí chốt lúc bắt đầu hold |
| Khoảng cách tới goal không giảm `stuck_min_progress` trong `stuck_timeout` | **HOLD** (bẫy chữ U: VFH+ dao động mà không báo BLOCKED) |
| HOLD quá `blocked_timeout` | `blocked_action`: `hold` (pilot lấy lái) hoặc `land` |

Avoidance chỉ bật trong MISSION/LANDING_SEARCH (khi bay tới điểm staging). Lúc TAKEOFF và khi
đã khoá landing mode thì tắt, vì lidar sẽ thấy mặt đất hoặc bãi đáp. Ở tick đầu mỗi lần bật,
manager HOLD một nhịp do chưa có status. Log `no fresh /avoidance/status` lúc đó là bình thường.

`avoidance_enabled:=false` sẽ quay về hành vi cũ, bay thẳng tới waypoint.

## Checklist trước khi bay thật

1. **Test tĩnh chiều đổi hệ** (lỗi nguy hiểm nhất): cầm vật đứng trước mũi drone, rồi
   `ros2 topic echo /avoidance/status`. Kiểm tra `min_distance_bearing` ≈ heading của drone.
   Đưa vật sang **trái** thì bearing phải **giảm** 90° (NED quay theo chiều kim đồng hồ). Sai
   thì chỉnh `lidar_yaw_offset_deg` / `lidar_upside_down`.
2. `self_mask_deg`: xem `/scan` khi drone đứng yên. Tia nào dính chân/khung thì che lại.
3. SITL Gazebo có lidar 2D: thử cột, tường, hành lang, chữ U (chữ U phải kết thúc bằng HOLD).
4. Giới hạn tốc độ: `MPC_XY_VEL_MAX` khoảng 2 m/s khi thử. Tốc độ bay thực tế ≈
   `MPC_XY_P × lookahead_distance`.
5. Nếu bật `publish_obstacle_distance`: xác nhận trong PX4 shell `listener obstacle_distance`
   thấy dữ liệu và timestamp không bị coi là cũ (timestamp đang lấy theo clock ROS, giống
   `offboard_control.py`), sau đó mới đặt `CP_DIST`.

## Giới hạn đã biết

- Lidar 2D chỉ thấy vật cản trong mặt phẳng quét. Dây điện, cành cây ở độ cao khác sẽ không thấy.
- VFH+ là planner phản ứng, không có bản đồ, nên dễ kẹt ở local minimum. Watchdog của manager
  bắt được trường hợp này nhưng không tự thoát ra. Muốn tự thoát thì cần VFH* hoặc một global planner.
- Không bù offset tịnh tiến của lidar so với tâm drone. Nếu lệch > 10 cm, hãy cộng phần lệch vào `drone_radius`.
