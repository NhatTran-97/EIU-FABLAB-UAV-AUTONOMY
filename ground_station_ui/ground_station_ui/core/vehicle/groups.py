"""Cac nhom du lieu cua drone. Moi nhom tu dang ky message minh can (attach) -> them nhom moi
khong phai sua code cu.

Message cua PX4 chi duoc nhan khi den tu autopilot da nhan dien (vehicle.is_autopilot):
tranh lan du lieu cua GCS khac / thiet bi phu tren cung link.
"""

import math
import time

from ground_station_ui.core.facts import FactGroup
from ground_station_ui.core.geometry.frames import ned_to_enu, q_ned_frd_to_enu_flu
from ground_station_ui.core.mavlink import drone_dialect as mavlink
from ground_station_ui.core.vehicle import px4_modes

LINK_TIMEOUT_S = 3.0
COMPONENT_STALE_S = 3.0
MAX_MESSAGES = 50

GPS_FIX = {0: 'No GPS', 1: 'No Fix', 2: '2D Fix', 3: '3D Fix', 4: 'DGPS', 5: 'RTK Float',
           6: 'RTK Fixed', 7: 'Static', 8: 'PPP'}
SEVERITY = ['EMERGENCY', 'ALERT', 'CRITICAL', 'ERROR', 'WARNING', 'NOTICE', 'INFO', 'DEBUG']
SENSORS = [   # GPS / pin co nhom rieng nen khong lap lai
    ('Gyro', mavlink.MAV_SYS_STATUS_SENSOR_3D_GYRO),
    ('Accel', mavlink.MAV_SYS_STATUS_SENSOR_3D_ACCEL),
    ('Compass', mavlink.MAV_SYS_STATUS_SENSOR_3D_MAG),
    ('Barometer', mavlink.MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE),
    ('RC', mavlink.MAV_SYS_STATUS_SENSOR_RC_RECEIVER),
]
COMPONENT_NAMES = {
    mavlink.DRONE_COMPONENT_LIDAR: 'Lidar',
    mavlink.DRONE_COMPONENT_GIMBAL: 'Gimbal',
    mavlink.DRONE_COMPONENT_CAMERA: 'Camera',
    mavlink.DRONE_COMPONENT_PX4: 'PX4 (DDS)',
    mavlink.DRONE_COMPONENT_RADIO: 'Radio 433 (drone)',
}
# don vi cua rate_hz: radio gui byte/s xuong (khong phai Hz)
COMPONENT_UNITS = {mavlink.DRONE_COMPONENT_RADIO: 'B/s'}
LANDED = {0: '—', 1: 'ON GROUND', 2: 'IN AIR', 3: 'TAKING OFF', 4: 'LANDING'}
COMPONENT_LEVELS = {0: 'ok', 1: 'warn', 2: 'error', 3: 'stale'}


class _VehicleGroup(FactGroup):
    def __init__(self, vehicle, **defaults):
        super().__init__(**defaults)
        self.vehicle = vehicle


class StatusGroup(_VehicleGroup):
    """HEARTBEAT cua autopilot: ket noi, che do bay, armed, thoi gian bay."""

    def __init__(self, vehicle):
        super().__init__(vehicle, linkAlive=False, heartbeatAge=-1.0, armed=False,
                         mode='—', flightTime=0, landed='—', failsafe=False)
        self._last_hb = None
        self._armed_at = None
        self._flight_prev = 0.0

    def attach(self, router):
        router.subscribe('HEARTBEAT', self.on_heartbeat)
        router.subscribe('EXTENDED_SYS_STATE', self.on_ext_state)

    def on_ext_state(self, m, _link):
        if self.vehicle.is_autopilot(m):
            self.set(landed=LANDED.get(m.landed_state, '—'))

    def on_heartbeat(self, m, _link):
        if m.autopilot == mavlink.MAV_AUTOPILOT_INVALID or m.type == mavlink.MAV_TYPE_GCS:
            return                       # GCS / companion / thiet bi phu, khong phai autopilot
        self.vehicle.identify(m.get_srcSystem(), m.get_srcComponent())
        if not self.vehicle.is_autopilot(m):
            return
        now = time.monotonic()
        self._last_hb = now
        armed = bool(m.base_mode & mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
        if armed and self._armed_at is None:
            self._armed_at = now
        elif not armed and self._armed_at is not None:
            self._flight_prev += now - self._armed_at
            self._armed_at = None
        mode = (px4_modes.decode(m.custom_mode)
                if m.base_mode & mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED else '—')
        # PX4 bao CRITICAL / EMERGENCY trong system_status khi dang failsafe
        self.set(armed=armed, mode=mode,
                 failsafe=m.system_status in (mavlink.MAV_STATE_CRITICAL, mavlink.MAV_STATE_EMERGENCY))

    def tick(self, now):
        running = now - self._armed_at if self._armed_at is not None else 0.0
        self.set(flightTime=int(self._flight_prev + running))
        if self._last_hb is not None:
            age = now - self._last_hb
            self.set(heartbeatAge=round(age, 1), linkAlive=age < LINK_TIMEOUT_S)


def _gps_val(raw, scale, max_ok):
    """So GPS_RAW_INT -> don vi that; 0 / UINT_MAX / qua lon (= khong co) -> None."""
    if not raw or raw in (65535, 0xFFFFFFFF):
        return None
    v = raw / scale
    return round(v, 2) if v <= max_ok else None


class GpsGroup(_VehicleGroup):
    def __init__(self, vehicle):
        super().__init__(vehicle, fix='—', fixType=0, satellites=0, lat=None, lon=None,
                         hdop=None, hAcc=None, vAcc=None)

    def attach(self, router):
        router.subscribe('GPS_RAW_INT', self.on_gps)
        router.subscribe('GLOBAL_POSITION_INT', self.on_position)

    def on_gps(self, m, _link):
        if self.vehicle.is_autopilot(m):
            self.set(fix=GPS_FIX.get(m.fix_type, str(m.fix_type)), fixType=m.fix_type,
                     satellites=m.satellites_visible if m.satellites_visible != 255 else 0,
                     # PX4 bao "khong co" bang gia tri dac biet (UINT16/32_MAX, 9999...) khi chua fix
                     hdop=_gps_val(m.eph, 100.0, 50.0) if m.fix_type >= 2 else None,
                     hAcc=_gps_val(getattr(m, 'h_acc', 0), 1000.0, 1000.0) if m.fix_type >= 2 else None,
                     vAcc=_gps_val(getattr(m, 'v_acc', 0), 1000.0, 1000.0) if m.fix_type >= 2 else None)

    def on_position(self, m, _link):
        if self.vehicle.is_autopilot(m):
            self.set(lat=m.lat / 1e7, lon=m.lon / 1e7)


class HomeGroup(_VehicleGroup):
    """Diem Home (PX4 dat khi arm / co GPS lan dau)."""

    def __init__(self, vehicle):
        super().__init__(vehicle, valid=False, lat=None, lon=None, alt=None)

    def attach(self, router):
        router.subscribe('HOME_POSITION', self.on_home)

    def on_home(self, m, _link):
        if self.vehicle.is_autopilot(m):
            self.set(valid=True, lat=m.latitude / 1e7, lon=m.longitude / 1e7,
                     alt=round(m.altitude / 1000.0, 1))


class BatteryGroup(_VehicleGroup):
    def __init__(self, vehicle):
        super().__init__(vehicle, percent=-1, voltage=None, current=None)

    def attach(self, router):
        router.subscribe('SYS_STATUS', self.on_sys_status)

    def on_sys_status(self, m, _link):
        if not self.vehicle.is_autopilot(m):
            return
        self.set(
            percent=m.battery_remaining if m.battery_remaining >= 0 else -1,   # <0: PX4 khong uoc luong
            voltage=round(m.voltage_battery / 1000.0, 2) if m.voltage_battery != 65535 else None,
            current=round(m.current_battery / 100.0, 1) if m.current_battery >= 0 else None,
        )


class SensorsGroup(_VehicleGroup):
    """Suc khoe cam bien tu SYS_STATUS (present / enabled / health bitmask)."""

    def __init__(self, vehicle):
        super().__init__(vehicle)
        self.items = [{'name': n, 'status': 'unknown'} for n, _ in SENSORS]

    def attach(self, router):
        router.subscribe('SYS_STATUS', self.on_sys_status)

    def on_sys_status(self, m, _link):
        if not self.vehicle.is_autopilot(m):
            return
        items = []
        for name, bit in SENSORS:
            if not m.onboard_control_sensors_present & bit:
                status = 'none'
            elif not m.onboard_control_sensors_enabled & bit:
                status = 'off'
            elif m.onboard_control_sensors_health & bit:
                status = 'ok'
            else:
                status = 'error'
            items.append({'name': name, 'status': status})
        self.set_items(items)


ALT_MODES = ('auto', 'indoor', 'outdoor')


class FlightGroup(_VehicleGroup):
    """Do cao, toc do, huong. VFR_HUD + LOCAL_POSITION_NED co ca khi khong GPS.

    Hai nguon do cao:
      altHome  = GLOBAL_POSITION_INT.relative_alt (so voi Home) -- dung ngoai troi co GPS
      altLocal = -LOCAL_POSITION_NED.z (EKF; trong nha = cam bien khoang cach MTF-02)
    altRel (so hien thi) chon theo che do: indoor -> local, outdoor -> home,
    auto -> home neu GPS fix >= 3D, nguoc lai local. Trong nha Home bi chot luc bat nguon
    roi EKF dat lai do cao nhieu lan nen relative_alt lech (vd bao 2.3 m khi nam tren san).
    """

    def __init__(self, vehicle):
        super().__init__(vehicle, altRel=None, altSource='—', altHome=None, altLocal=None,
                         altAmsl=None, groundSpeed=None, climbRate=None, heading=None)
        self.mode = 'auto'
        self._fix = 0

    def attach(self, router):
        router.subscribe('GLOBAL_POSITION_INT', self.on_global)
        router.subscribe('LOCAL_POSITION_NED', self.on_local)
        router.subscribe('GPS_RAW_INT', self.on_gps)
        router.subscribe('VFR_HUD', self.on_hud)

    def set_mode(self, mode):
        if mode in ALT_MODES and mode != self.mode:
            self.mode = mode
            self._pick()

    def use_home(self):
        if self.mode == 'outdoor':
            return self.values['altHome'] is not None
        if self.mode == 'indoor':
            return False
        return self._fix >= 3 and self.values['altHome'] is not None

    def _pick(self):
        if self.use_home():
            self.set(altRel=self.values['altHome'], altSource='home')
        elif self.values['altLocal'] is not None:
            self.set(altRel=self.values['altLocal'], altSource='local')
        else:
            self.set(altRel=self.values['altHome'], altSource='home' if self.values['altHome'] is not None else '—')

    def on_global(self, m, _link):
        if self.vehicle.is_autopilot(m):
            self.set(altHome=round(m.relative_alt / 1000.0, 1))
            self._pick()

    def on_local(self, m, _link):
        if self.vehicle.is_autopilot(m):
            self.set(altLocal=round(-m.z, 1))
            self._pick()

    def on_gps(self, m, _link):
        if self.vehicle.is_autopilot(m):
            self._fix = m.fix_type
            self._pick()

    def on_hud(self, m, _link):
        if self.vehicle.is_autopilot(m):
            self.set(groundSpeed=round(m.groundspeed, 1), climbRate=round(m.climb, 1),
                     altAmsl=round(m.alt, 1), heading=m.heading)


class AttitudeGroup(_VehicleGroup):
    """Tu the tu ATTITUDE (do): roll / pitch cho dong ho chan troi, heading 0-360 cho la ban."""

    def __init__(self, vehicle):
        super().__init__(vehicle, valid=False, roll=0.0, pitch=0.0, heading=0.0)

    def attach(self, router):
        router.subscribe('ATTITUDE', self.on_attitude)

    def on_attitude(self, m, _link):
        if not self.vehicle.is_autopilot(m):
            return
        self.set(valid=True,
                 roll=round(math.degrees(m.roll), 1),
                 pitch=round(math.degrees(m.pitch), 1),
                 heading=round(math.degrees(m.yaw) % 360.0, 1))


class LocalPoseGroup(_VehicleGroup):
    """Pose cuc bo cua drone do Jetson gui (DRONE_LOCAL_POSE, tu PX4 vehicle_odometry qua DDS).

    values: valid, e/n/u (m, ENU), q (w,x,y,z: FLU -> ENU), speed, age_s
    items : vet bay [[e, n, u], ...] (them diem khi di > TRAIL_STEP_M, toi da TRAIL_MAX diem;
            xoa khi EKF dat lai vi tri -- reset_counter doi)
    """
    TRAIL_STEP_M = 0.05
    TRAIL_MAX = 3000
    STALE_S = 2.0

    def __init__(self, vehicle):
        super().__init__(vehicle, valid=False, e=0.0, n=0.0, u=0.0, q=[1.0, 0.0, 0.0, 0.0],
                         ve=0.0, vn=0.0, vu=0.0, speed=0.0, age_s=-1.0)
        self._last = None
        self._reset = None

    def attach(self, router):
        router.subscribe('DRONE_LOCAL_POSE', self.on_pose)

    def on_pose(self, m, _link):
        e, n, u = ned_to_enu(m.x, m.y, m.z)
        q = q_ned_frd_to_enu_flu(tuple(m.q))
        if self._reset is not None and m.reset_counter != self._reset:
            self.set_items([])                    # vi tri nhay -> vet cu khong con dung
        self._reset = m.reset_counter
        self._last = time.monotonic()
        self.set(valid=True, e=round(e, 3), n=round(n, 3), u=round(u, 3),
                 q=[round(c, 4) for c in q],
                 ve=round(m.vy, 2), vn=round(m.vx, 2), vu=round(-m.vz, 2),
                 speed=round(math.sqrt(m.vx * m.vx + m.vy * m.vy + m.vz * m.vz), 2), age_s=0.0)
        trail = self.items
        if not trail or math.dist(trail[-1], (e, n, u)) >= self.TRAIL_STEP_M:
            self.set_items((trail + [[round(e, 3), round(n, 3), round(u, 3)]])[-self.TRAIL_MAX:])

    def clear_trail(self):
        self.set_items([])

    def tick(self, now):
        if self._last is not None:
            age = now - self._last
            self.set(age_s=round(age, 1), valid=age < self.STALE_S)


class GimbalGroup(_VehicleGroup):
    """Goc gimbal Skydroid do Jetson gui (DRONE_GIMBAL_STATE), so voi than drone."""
    STALE_S = 3.0

    def __init__(self, vehicle):
        super().__init__(vehicle, valid=False, yaw=None, pitch=None, roll=None, connected=False,
                         mode='—', age_s=-1.0)
        self._last = None

    def attach(self, router):
        router.subscribe('DRONE_GIMBAL_STATE', self.on_gimbal)

    def on_gimbal(self, m, _link):
        self._last = time.monotonic()
        self.set(valid=True, yaw=round(m.yaw, 1), pitch=round(m.pitch, 1), roll=round(m.roll, 1),
                 connected=bool(m.connected), mode='AUTO' if m.control_mode == 1 else 'MANUAL', age_s=0.0)

    def tick(self, now):
        if self._last is not None:
            age = now - self._last
            self.set(age_s=round(age, 1), valid=age < self.STALE_S)


class MessagesGroup(_VehicleGroup):
    """STATUSTEXT: nhan ca truoc heartbeat dau tien (PX4 gui canh bao preflight rat som)."""

    def attach(self, router):
        router.subscribe('STATUSTEXT', self.on_text)

    def on_text(self, m, _link):
        if m.get_srcComponent() == mavlink.MAV_COMP_ID_MISSIONPLANNER:
            return
        item = {
            'time': time.strftime('%H:%M:%S'),
            'severity': SEVERITY[m.severity] if m.severity < len(SEVERITY) else str(m.severity),
            'level': m.severity,
            'text': m.text,
        }
        self.set_items([item] + self.items[:MAX_MESSAGES - 1])


class CompanionGroup(_VehicleGroup):
    """Jetson: heartbeat cua drone_telemetry + DRONE_COMPONENT_STATUS tung thiet bi."""

    def __init__(self, vehicle):
        super().__init__(vehicle, linkAlive=False, heartbeatAge=-1.0)
        self._last_hb = None
        self._comps = {}     # component_id -> (t_nhan, msg)

    def attach(self, router):
        router.subscribe('HEARTBEAT', self.on_heartbeat)
        router.subscribe('DRONE_COMPONENT_STATUS', self.on_component)

    def on_heartbeat(self, m, _link):
        if m.type == mavlink.MAV_TYPE_ONBOARD_CONTROLLER:
            self._last_hb = time.monotonic()

    def on_component(self, m, _link):
        self._comps[m.component_id] = (time.monotonic(), m)

    def tick(self, now):
        if self._last_hb is not None:
            age = now - self._last_hb
            self.set(heartbeatAge=round(age, 1), linkAlive=age < LINK_TIMEOUT_S)
        items = []
        for cid in sorted(self._comps):
            t, m = self._comps[cid]
            stale = now - t > COMPONENT_STALE_S
            items.append({
                'id': cid,
                'name': COMPONENT_NAMES.get(cid, f'#{cid}'),
                'level': 'stale' if stale else COMPONENT_LEVELS.get(m.level, 'error'),
                'state': 'NO_DATA' if stale else m.state,
                'rateHz': 0.0 if stale else round(m.rate_hz, 1),
                'unit': COMPONENT_UNITS.get(cid, 'Hz'),
            })
        self.set_items(items)
