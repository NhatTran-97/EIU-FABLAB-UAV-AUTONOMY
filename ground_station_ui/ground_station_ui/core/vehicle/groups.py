"""Vehicle data groups. Each group subscribes to the messages it needs in attach(router).

Source filtering: PX4 messages are accepted only from the identified autopilot on an allowed link
(vehicle.is_autopilot); DRONE_* messages only from the companion (vehicle.is_companion).
"""

import math
import time

from ground_station_ui.core.facts import FactGroup, PathLog
from ground_station_ui.core.geometry.frames import ned_to_enu, q_ned_frd_to_enu_flu
from ground_station_ui.core.mavlink import drone_dialect as mavlink
from ground_station_ui.core.vehicle import px4_modes

LINK_TIMEOUT_S = 3.0
COMPONENT_STALE_S = 3.0
MAX_MESSAGES = 50

GPS_FIX = {0: 'No GPS', 1: 'No Fix', 2: '2D Fix', 3: '3D Fix', 4: 'DGPS', 5: 'RTK Float',
           6: 'RTK Fixed', 7: 'Static', 8: 'PPP'}
SEVERITY = ['EMERGENCY', 'ALERT', 'CRITICAL', 'ERROR', 'WARNING', 'NOTICE', 'INFO', 'DEBUG']
SENSORS = [   # GPS and battery have their own groups
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
# rate_hz unit per component (radio: bytes/s)
COMPONENT_UNITS = {mavlink.DRONE_COMPONENT_RADIO: 'B/s'}
LANDED = {0: '—', 1: 'ON GROUND', 2: 'IN AIR', 3: 'TAKING OFF', 4: 'LANDING'}
COMPONENT_LEVELS = {0: 'ok', 1: 'warn', 2: 'error', 3: 'stale'}


class _VehicleGroup(FactGroup):
    def __init__(self, vehicle, **defaults):
        super().__init__(**defaults)
        self.vehicle = vehicle


class StatusGroup(_VehicleGroup):
    """Autopilot HEARTBEAT: link, flight mode, armed state, flight time."""

    def __init__(self, vehicle):
        super().__init__(vehicle, linkAlive=False, heartbeatAge=-1.0, armed=False,
                         mode='—', flightTime=0, landed='—', failsafe=False, source='—')
        self._last_hb = None
        self._armed_at = None
        self._flight_prev = 0.0

    def attach(self, router):
        router.subscribe('HEARTBEAT', self.on_heartbeat)
        router.subscribe('EXTENDED_SYS_STATE', self.on_ext_state)

    def on_ext_state(self, m, _link):
        if self.vehicle.is_autopilot(m, _link):
            self.set(landed=LANDED.get(m.landed_state, '—'))

    def on_heartbeat(self, m, _link):
        if m.autopilot == mavlink.MAV_AUTOPILOT_INVALID or m.type == mavlink.MAV_TYPE_GCS:
            return                       # not an autopilot
        if not self.vehicle.identify(m.get_srcSystem(), m.get_srcComponent(), _link):
            return                       # another autopilot or a link that is not allowed
        self.set(source=f'{_link} · sys {m.get_srcSystem()}')
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
        # failsafe: system_status is CRITICAL / EMERGENCY
        self.set(armed=armed, mode=mode,
                 failsafe=m.system_status in (mavlink.MAV_STATE_CRITICAL, mavlink.MAV_STATE_EMERGENCY))

    def tick(self, now):
        running = now - self._armed_at if self._armed_at is not None else 0.0
        self.set(flightTime=int(self._flight_prev + running))
        if self._last_hb is not None:
            age = now - self._last_hb
            self.set(heartbeatAge=round(age, 1), linkAlive=age < LINK_TIMEOUT_S)


def _gps_val(raw, scale, max_ok):
    """Convert GPS_RAW_INT fields to real units; 0 / UINT_MAX / out-of-range (= unknown) -> None."""
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
        if self.vehicle.is_autopilot(m, _link):
            self.set(fix=GPS_FIX.get(m.fix_type, str(m.fix_type)), fixType=m.fix_type,
                     satellites=m.satellites_visible if m.satellites_visible != 255 else 0,
                     # sentinel values (UINT16/32_MAX, 9999) mean unknown
                     hdop=_gps_val(m.eph, 100.0, 50.0) if m.fix_type >= 2 else None,
                     hAcc=_gps_val(getattr(m, 'h_acc', 0), 1000.0, 1000.0) if m.fix_type >= 2 else None,
                     vAcc=_gps_val(getattr(m, 'v_acc', 0), 1000.0, 1000.0) if m.fix_type >= 2 else None)

    def on_position(self, m, _link):
        if self.vehicle.accept_global(m, _link):          # autopilot; companion when the autopilot is stale
            self.set(lat=m.lat / 1e7, lon=m.lon / 1e7)


class HomeGroup(_VehicleGroup):
    """Home position (set by PX4 on arming or first GPS fix)."""

    def __init__(self, vehicle):
        super().__init__(vehicle, valid=False, lat=None, lon=None, alt=None)

    def attach(self, router):
        router.subscribe('HOME_POSITION', self.on_home)

    def on_home(self, m, _link):
        if self.vehicle.is_autopilot(m, _link):
            self.set(valid=True, lat=m.latitude / 1e7, lon=m.longitude / 1e7,
                     alt=round(m.altitude / 1000.0, 1))


class BatteryGroup(_VehicleGroup):
    def __init__(self, vehicle):
        super().__init__(vehicle, percent=-1, voltage=None, current=None)

    def attach(self, router):
        router.subscribe('SYS_STATUS', self.on_sys_status)

    def on_sys_status(self, m, _link):
        if not self.vehicle.is_autopilot(m, _link):
            return
        self.set(
            percent=m.battery_remaining if m.battery_remaining >= 0 else -1,   # -1: not estimated
            voltage=round(m.voltage_battery / 1000.0, 2) if m.voltage_battery != 65535 else None,
            current=round(m.current_battery / 100.0, 1) if m.current_battery >= 0 else None,
        )


class SensorsGroup(_VehicleGroup):
    """Sensor health from SYS_STATUS (present / enabled / health bitmasks)."""

    def __init__(self, vehicle):
        super().__init__(vehicle)
        self.items = [{'name': n, 'status': 'unknown'} for n, _ in SENSORS]

    def attach(self, router):
        router.subscribe('SYS_STATUS', self.on_sys_status)

    def on_sys_status(self, m, _link):
        if not self.vehicle.is_autopilot(m, _link):
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
    """Altitude, speed and heading.

    altHome  = GLOBAL_POSITION_INT.relative_alt (above Home, needs GPS)
    altLocal = -LOCAL_POSITION_NED.z (EKF; indoors from the MTF-02 rangefinder)
    altRel   = displayed value: indoor -> altLocal, outdoor -> altHome, auto -> altHome with fix >= 3D.
    altHome is not used indoors: Home is latched at power-up and EKF height resets make relative_alt drift.
    """

    def __init__(self, vehicle):
        super().__init__(vehicle, altRel=None, altSource='—', altHome=None, altLocal=None,
                         altAmsl=None, altGlobal=None, groundSpeed=None, climbRate=None, heading=None)
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
        if not self.vehicle.accept_global(m, _link):
            return
        # altGlobal (m AMSL): NAV_TAKEOFF param7
        self.set(altHome=round(m.relative_alt / 1000.0, 1), altGlobal=round(m.alt / 1000.0, 2))
        if not self.vehicle.is_autopilot(m, _link) and m.hdg != 65535:
            self.set(heading=round(m.hdg / 100.0, 1))     # hdg is taken from the companion only
        self._pick()

    def on_local(self, m, _link):
        if self.vehicle.is_autopilot(m, _link):
            self.set(altLocal=round(-m.z, 1))
            self._pick()

    def on_gps(self, m, _link):
        if self.vehicle.is_autopilot(m, _link):
            self._fix = m.fix_type
            self._pick()

    def on_hud(self, m, _link):
        if self.vehicle.is_autopilot(m, _link):
            self.set(groundSpeed=round(m.groundspeed, 1), climbRate=round(m.climb, 1),
                     altAmsl=round(m.alt, 1), heading=m.heading)


def q_ned_to_euler(q):
    """Quaternion (w, x, y, z) FRD -> NED to (roll, pitch, yaw) in rad (yaw from North, clockwise)."""
    w, x, y, z = q
    roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    pitch = math.asin(max(-1.0, min(1.0, 2 * (w * y - z * x))))
    yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    return roll, pitch, yaw


class AttitudeGroup(_VehicleGroup):
    """Attitude in degrees: roll, pitch, heading 0-360 (from North, clockwise).

    Sources in priority order:
      px4    : ATTITUDE from the autopilot (915 / SITL)
      jetson : DRONE_LOCAL_POSE.q (vehicle_odometry over DDS), used when px4 is stale > STALE_S
    All sources stale -> valid = False.
    values: valid, source ('px4' | 'jetson' | ''), roll, pitch, heading
    """
    STALE_S = 2.0

    def __init__(self, vehicle):
        super().__init__(vehicle, valid=False, source='', roll=0.0, pitch=0.0, heading=0.0)
        self._seen = {'px4': None, 'jetson': None}

    def attach(self, router):
        router.subscribe('ATTITUDE', self.on_attitude)
        router.subscribe('DRONE_LOCAL_POSE', self.on_companion_pose)

    def _apply(self, source, roll, pitch, yaw):
        self._seen[source] = time.monotonic()
        self.set(valid=True, source=source,
                 roll=round(math.degrees(roll), 1),
                 pitch=round(math.degrees(pitch), 1),
                 heading=round(math.degrees(yaw) % 360.0, 1))

    def on_attitude(self, m, _link):
        if self.vehicle.is_autopilot(m, _link):
            self._apply('px4', m.roll, m.pitch, m.yaw)

    def on_companion_pose(self, m, _link):
        if not self.vehicle.is_companion(m, _link):
            return
        t = self._seen['px4']
        if t is not None and time.monotonic() - t < self.STALE_S:
            return                                    # px4 still fresh: it has priority
        self._apply('jetson', *q_ned_to_euler(tuple(m.q)))

    def tick(self, now):
        seen = [t for t in self._seen.values() if t is not None]
        if seen and now - max(seen) >= self.STALE_S:
            self.set(valid=False)


def euler_ned_to_q(roll, pitch, yaw):
    """roll / pitch / yaw (rad, FRD relative to NED) -> quaternion (w, x, y, z), as in ATTITUDE_QUATERNION."""
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    return (cr * cp * cy + sr * sp * sy, sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy, cr * cp * sy - sr * sp * cy)


class LocalPoseGroup(_VehicleGroup):
    """Local pose (PX4 EKF frame) for the 3D page.

    Sources in priority order (switches when the higher one is stale > STALE_S):
      jetson : DRONE_LOCAL_POSE (vehicle_odometry over DDS, radio 433)
      px4    : LOCAL_POSITION_NED + ATTITUDE_QUATERNION / ATTITUDE (radio 915 / SITL UDP)

    values: valid, source ('jetson' | 'px4' | ''), e/n/u (m, ENU), q (w,x,y,z: FLU -> ENU),
            ve/vn/vu, speed, age_s, trailPoints,
            extentM (largest horizontal distance from the EKF origin reached since the trail was cleared,
            rounded up to whole metres; sizes the 3D grid)
    trail : PathLog of [e, n, u]; step TRAIL_STEP_M, at most TRAIL_MAX (drops the first TRAIL_TRIM).
            Cleared when reset_counter changes; kept across source switches (same EKF frame).
    """
    TRAIL_STEP_M = 0.05
    TRAIL_MAX = 3000
    TRAIL_TRIM = 500
    STALE_S = 2.0

    def __init__(self, vehicle):
        super().__init__(vehicle, valid=False, source='', e=0.0, n=0.0, u=0.0, q=[1.0, 0.0, 0.0, 0.0],
                         ve=0.0, vn=0.0, vu=0.0, speed=0.0, age_s=-1.0, trailPoints=0, extentM=0.0)
        self.trail = PathLog()
        self._seen = {'jetson': None, 'px4': None}      # source -> last receive time (monotonic)
        self._reset = None
        self._att = None                                # (t, q NED/FRD, 'q' | 'euler')

    def attach(self, router):
        router.subscribe('DRONE_LOCAL_POSE', self.on_pose)
        router.subscribe('LOCAL_POSITION_NED', self.on_px4_position)
        router.subscribe('ATTITUDE_QUATERNION', self.on_px4_attitude_q)
        router.subscribe('ATTITUDE', self.on_px4_attitude)

    def _fresh(self, source, now):
        t = self._seen[source]
        return t is not None and now - t < self.STALE_S

    # --- Jetson source (preferred) ---------------------------------------------------------
    def on_pose(self, m, _link):
        if not self.vehicle.is_companion(m, _link):
            return
        if self._reset is not None and m.reset_counter != self._reset:
            self.clear_trail()                    # EKF position reset
        self._reset = m.reset_counter
        self._seen['jetson'] = time.monotonic()
        self._apply('jetson', m.x, m.y, m.z, tuple(m.q), m.vx, m.vy, m.vz)

    # --- direct PX4 source (fallback) -----------------------------------------------------
    def on_px4_attitude_q(self, m, _link):
        if self.vehicle.is_autopilot(m, _link):
            self._att = (time.monotonic(), (m.q1, m.q2, m.q3, m.q4), 'q')

    def on_px4_attitude(self, m, _link):
        if not self.vehicle.is_autopilot(m, _link):
            return
        if self._att is not None and self._att[2] == 'q' and time.monotonic() - self._att[0] < self.STALE_S:
            return                                # ATTITUDE_QUATERNION still fresh: it has priority
        self._att = (time.monotonic(), euler_ned_to_q(m.roll, m.pitch, m.yaw), 'euler')

    def on_px4_position(self, m, _link):
        if not self.vehicle.is_autopilot(m, _link):
            return
        now = time.monotonic()
        self._seen['px4'] = now
        if self._fresh('jetson', now):
            return                                # jetson still fresh: it has priority
        q = self._att[1] if self._att is not None and now - self._att[0] < self.STALE_S else (1.0, 0.0, 0.0, 0.0)
        self._apply('px4', m.x, m.y, m.z, q, m.vx, m.vy, m.vz)

    # --- common ----------------------------------------------------------------------------
    def _apply(self, source, x, y, z, q_ned, vx, vy, vz):
        e, n, u = ned_to_enu(x, y, z)
        q = q_ned_frd_to_enu_flu(tuple(q_ned))
        self.set(valid=True, source=source, e=round(e, 3), n=round(n, 3), u=round(u, 3),
                 q=[round(c, 4) for c in q],
                 ve=round(vy, 2), vn=round(vx, 2), vu=round(-vz, 2),
                 speed=round(math.sqrt(vx * vx + vy * vy + vz * vz), 2), age_s=0.0)
        reach = math.ceil(max(abs(e), abs(n)))
        if reach > self.values['extentM']:
            self.set(extentM=float(reach))
        last = self.trail.last()
        if last is None or math.dist(last, (e, n, u)) >= self.TRAIL_STEP_M:
            self.trail.add((round(e, 3), round(n, 3), round(u, 3)))
            if self.trail.count > self.TRAIL_MAX:
                self.trail.trim_front(self.TRAIL_TRIM)
            self.set(trailPoints=self.trail.count)

    def clear_trail(self):
        self.trail.clear()
        self.set(trailPoints=0, extentM=0.0)

    def tick(self, now):
        src = self.values['source']
        if not src:
            return
        t = self._seen[src]
        age = now - t
        if age >= self.STALE_S and src == 'jetson' and self._fresh('px4', now):
            self.set(source='px4')                # jetson stale -> px4
            age = now - self._seen['px4']
        self.set(age_s=round(age, 1), valid=age < self.STALE_S)


def geo_dist_m(lat1, lon1, lat2, lon2):
    """Distance (m) between two nearby GPS points, equirectangular approximation."""
    k = math.pi / 180 * 6371000.0
    dx = (lon2 - lon1) * k * math.cos(math.radians((lat1 + lat2) / 2))
    dy = (lat2 - lat1) * k
    return math.hypot(dx, dy)


def write_gpx(path, segments, name='EIU drone flight'):
    """Write segments [[[lat, lon, alt_amsl, t_unix], ...], ...] to a GPX 1.1 file."""
    import os
    os.makedirs(os.path.dirname(path) or '.', exist_ok=True)
    out = ['<?xml version="1.0" encoding="UTF-8"?>',
           '<gpx version="1.1" creator="EIU Drone GCS" xmlns="http://www.topografix.com/GPX/1/1">',
           f'<trk><name>{name}</name>']
    for seg in segments:
        out.append('<trkseg>')
        for lat, lon, alt, t in seg:
            ts = time.strftime('%Y-%m-%dT%H:%M:%SZ', time.gmtime(t))
            out.append(f'<trkpt lat="{lat:.7f}" lon="{lon:.7f}"><ele>{alt:.1f}</ele><time>{ts}</time></trkpt>')
        out.append('</trkseg>')
    out += ['</trk>', '</gpx>']
    with open(path, 'w') as f:
        f.write('\n'.join(out) + '\n')


class TrackGroup(_VehicleGroup):
    """GPS flight track, recorded while flying; one segment per flight.

    Positions come from GLOBAL_POSITION_INT of the autopilot, or of the companion when the autopilot position is
    stale (vehicle.accept_global). Flying = armed while the autopilot heartbeat is alive; without it (Jetson
    only), flying = more than AIRBORNE_ALT_M above Home.

    _flight : full resolution, step STEP_M; written to save_dir as GPX on disarm.
    path    : PathLog for display, step DISPLAY_STEP_M; above DISPLAY_MAX, thin() halves the points.
    values  : recording, points, lengthM, lastFile
    """
    STEP_M = 0.5
    DISPLAY_STEP_M = 2.0
    DISPLAY_MAX = 6000
    MAX_FLIGHT_POINTS = 200000          # ~100 km at 0.5 m per point
    AIRBORNE_ALT_M = 0.5                # m above Home, flight detection without the autopilot heartbeat

    def __init__(self, vehicle, save_dir=None):
        super().__init__(vehicle, recording=False, points=0, lengthM=0.0, lastFile='')
        self.save_dir = save_dir
        self.path = PathLog()
        self._armed = False
        self._flight = []               # [lat, lon, alt_amsl, t_unix]

    def attach(self, router):
        router.subscribe('GLOBAL_POSITION_INT', self.on_global)

    def on_global(self, m, _link):
        if not self.vehicle.accept_global(m, _link):
            return
        status = self.vehicle.groups['status'].values
        if self.vehicle.is_autopilot(m, _link) or status.get('linkAlive'):
            armed = bool(status.get('armed'))
        else:                                       # companion only: no arming state available
            armed = m.relative_alt / 1000.0 > self.AIRBORNE_ALT_M
        if armed != self._armed:
            self._armed = armed
            if armed:
                self._flight = []
                self.path.start_segment()
            else:
                self._save_flight()
            self.set(recording=armed)
        if not armed or (m.lat == 0 and m.lon == 0):
            return
        lat, lon = m.lat / 1e7, m.lon / 1e7
        if self._flight:
            d = geo_dist_m(self._flight[-1][0], self._flight[-1][1], lat, lon)
            if d < self.STEP_M:
                return
            self.set(lengthM=round(self.values['lengthM'] + d, 1))
        if len(self._flight) < self.MAX_FLIGHT_POINTS:
            self._flight.append([round(lat, 7), round(lon, 7), round(m.alt / 1000.0, 1), round(time.time(), 1)])
            self.set(points=self.values['points'] + 1)
        last = self.path.last()
        if last is None or geo_dist_m(last[0], last[1], lat, lon) >= self.DISPLAY_STEP_M:
            self.path.add((round(lat, 7), round(lon, 7)))
            if self.path.count > self.DISPLAY_MAX:
                self.path.thin()

    def _save_flight(self):
        if not self.save_dir or len(self._flight) < 2:
            return
        name = time.strftime('track_%Y%m%d_%H%M%S.gpx', time.localtime(self._flight[0][3]))
        import os
        path = os.path.join(self.save_dir, name)
        try:
            write_gpx(path, [self._flight])
            self.set(lastFile=path)
        except OSError as e:
            self.set(lastFile=f'error: {e}')

    def clear(self):
        """Clear the displayed path; _flight and saved GPX files are kept."""
        self.path.clear()
        if self._armed:
            self.path.start_segment()
        self.set(points=0, lengthM=0.0)


class GimbalGroup(_VehicleGroup):
    """Gimbal angles (DRONE_GIMBAL_STATE from the companion), relative to the drone body."""
    STALE_S = 3.0

    def __init__(self, vehicle):
        super().__init__(vehicle, valid=False, yaw=None, pitch=None, roll=None, connected=False,
                         mode='—', age_s=-1.0)
        self._last = None

    def attach(self, router):
        router.subscribe('DRONE_GIMBAL_STATE', self.on_gimbal)

    def on_gimbal(self, m, _link):
        if not self.vehicle.is_companion(m, _link):
            return
        self._last = time.monotonic()
        self.set(valid=True, yaw=round(m.yaw, 1), pitch=round(m.pitch, 1), roll=round(m.roll, 1),
                 connected=bool(m.connected), mode='AUTO' if m.control_mode == 1 else 'MANUAL', age_s=0.0)

    def tick(self, now):
        if self._last is not None:
            age = now - self._last
            self.set(age_s=round(age, 1), valid=age < self.STALE_S)


class MessagesGroup(_VehicleGroup):
    """STATUSTEXT. Accepted even before the first heartbeat (PX4 sends preflight warnings early)."""

    def attach(self, router):
        router.subscribe('STATUSTEXT', self.on_text)

    def on_text(self, m, _link):
        if m.get_srcComponent() == mavlink.MAV_COMP_ID_MISSIONPLANNER:
            return
        v = self.vehicle
        # autopilot, companion, or an allowed autopilot link before identification
        if not (v.is_autopilot(m, _link) or v.is_companion(m, _link)
                or (v.sysid is None and v.link_allowed(_link))):
            return
        item = {
            'time': time.strftime('%H:%M:%S'),
            'severity': SEVERITY[m.severity] if m.severity < len(SEVERITY) else str(m.severity),
            'level': m.severity,
            'text': m.text,
        }
        self.set_items([item] + self.items[:MAX_MESSAGES - 1])


class MissionProgressGroup(_VehicleGroup):
    """Mission progress reported by the Jetson (mission_manager via drone_telemetry).

    MISSION_CURRENT from the companion: current item, item count and mission state. While the state is
    `paused` the mission waits for the operator: a planned manual task at a waypoint, or a pause after the
    pilot took control. `text` is the latest STATUSTEXT from the companion and explains which.
    values: valid, seq, total, state ('idle' | 'active' | 'paused' | 'complete' | ''), text
    """
    STALE_S = 3.0
    STATES = {2: 'idle', 3: 'active', 4: 'paused', 5: 'complete'}   # MAVLink MISSION_STATE

    def __init__(self, vehicle):
        super().__init__(vehicle, valid=False, seq=0, total=0, state='', text='')
        self._last = None

    def attach(self, router):
        router.subscribe('MISSION_CURRENT', self.on_current)
        router.subscribe('STATUSTEXT', self.on_text)

    def on_current(self, m, _link):
        if not self.vehicle.is_companion(m, _link):
            return
        self._last = time.monotonic()
        self.set(valid=True, seq=int(m.seq), total=int(getattr(m, 'total', 0)),
                 state=self.STATES.get(int(getattr(m, 'mission_state', 0)), ''))

    def on_text(self, m, _link):
        if self.vehicle.is_companion(m, _link):
            self.set(text=m.text)

    def tick(self, now):
        if self._last is not None and now - self._last > self.STALE_S:
            self.set(valid=False)


class CompanionGroup(_VehicleGroup):
    """Companion: drone_telemetry HEARTBEAT and DRONE_COMPONENT_STATUS."""

    def __init__(self, vehicle):
        super().__init__(vehicle, linkAlive=False, heartbeatAge=-1.0)
        self._last_hb = None
        self._comps = {}     # component_id -> (t, msg)

    def attach(self, router):
        router.subscribe('HEARTBEAT', self.on_heartbeat)
        router.subscribe('DRONE_COMPONENT_STATUS', self.on_component)

    def on_heartbeat(self, m, _link):
        if m.type == mavlink.MAV_TYPE_ONBOARD_CONTROLLER and self.vehicle.is_companion(m, _link):
            self._last_hb = time.monotonic()
            self.vehicle.companion_link = _link        # link used for commands / waypoints

    def on_component(self, m, _link):
        if self.vehicle.is_companion(m, _link):
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
