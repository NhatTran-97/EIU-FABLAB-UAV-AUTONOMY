"""Lenh bay: 2 duong rieng + KHOA CHEO + dieu kien tung nut (Python thuan).

    px4    : o PX4 tren thanh tren cung -> Take-off / Land GUI THANG PX4 qua radio 915.
             ACK do chinh PX4 tra. Take-off = [ARM neu chua arm] -> NAV_TAKEOFF.
    jetson : 3 nut duoi ban do -> radio 433 -> Jetson (drone_telemetry) -> uXRCE-DDS -> PX4.
             Send WP cung di duong nay.

Khoa cheo (CommandLock): duong nao da gui lenh bay (Take-off / Land / Hold / RTL) thi GIU QUYEN den
khi drone ve dat + disarm; duong con lai bi chan. Ngoai le: duong giu quyen MAT KET NOI -> duong kia
duoc dung (khong de drone khong ai dieu khien). Tay RC luon uu tien hon ca hai.

Che do TEST (nut 🧪 canh 3 nut duoi, trong phong thi nghiem): ping Jetson (MAV_CMD_USER_1, Jetson tra
ACK + STATUSTEXT, khong dung PX4) va gui Take-off / Land qua Jetson BO QUA dieu kien phia GCS (DDS, GPS,
khoa cheo, trang thai bay) -- de kiem tra lenh co len toi Jetson khong. Jetson VAN ap luat an toan rieng
(khong co PX4 -> tu choi 'no PX4 data'; co PX4 + o mat dat -> Take-off SE ARM).

Dieu kien (blockers): duong con song; Jetson phai co du lieu PX4 qua DDS (lenh bay); bat dau viec moi
(Take-off / Send WP qua Jetson) khi ngoai troi can GPS 3D fix, RTL luon can GPS; Land / Hold khong bao
gio bi chan vi GPS. Trang thai bay (EXTENDED_SYS_STATE): Take-off khi dang bay / Land, Hold, RTL khi
o mat dat -> chan; chua biet -> de PX4 / Jetson quyet.
"""

import math

from ground_station_ui.core.commands.sender import CommandTarget
from ground_station_ui.core.mavlink import drone_dialect as mavlink
from ground_station_ui.core.vehicle import px4_modes
from ground_station_ui.core.vehicle.groups import LINK_TIMEOUT_S

ROUTES = ('px4', 'jetson')
ROUTE_NAMES = {'px4': 'PX4 · 915', 'jetson': 'Jetson · 433'}
ACTIONS = {'px4': ('takeoff', 'land'), 'jetson': ('takeoff', 'land', 'hold', 'rtl', 'upload')}
FLIGHT_ACTIONS = ('takeoff', 'land', 'hold', 'rtl')
LABELS = {'takeoff': 'Take-off', 'land': 'Land', 'hold': 'Hold', 'rtl': 'RTL', 'upload': 'Send WP'}
IN_AIR = ('IN AIR', 'TAKING OFF', 'LANDING')
NAN = float('nan')
AUTO_LOITER = 3     # PX4_CUSTOM_SUB_MODE_AUTO_LOITER
PENDING = ('sending', 'in_progress')


def check_state(action, landed):
    """'' = cho gui; nguoc lai = ly do chan. landed: StatusGroup.values['landed']."""
    if action == 'takeoff' and landed in IN_AIR:
        return 'already flying'
    if action in ('land', 'hold', 'rtl') and landed == 'ON GROUND':
        return 'vehicle is on the ground'
    return ''


def px4_steps(action, altitude, armed, amsl):
    """Cac COMMAND_LONG gui thang PX4. altitude: m so voi diem cat canh; amsl: do cao AMSL hien tai
    (None = khong co vi tri toan cuc -> NaN, PX4 dung MIS_TAKEOFF_ALT)."""
    if action == 'takeoff':
        steps = [] if armed else [(mavlink.MAV_CMD_COMPONENT_ARM_DISARM, [1.0])]
        alt = amsl + altitude if amsl is not None and math.isfinite(altitude) else NAN
        # PX4: param7 = AMSL; lat / lon / yaw NaN = vi tri, huong hien tai
        steps.append((mavlink.MAV_CMD_NAV_TAKEOFF, [NAN, 0, 0, NAN, NAN, NAN, alt]))
        return steps
    if action == 'land':
        return [(mavlink.MAV_CMD_NAV_LAND, [0, 0, 0, NAN, NAN, NAN, NAN])]
    if action == 'hold':
        return [(mavlink.MAV_CMD_DO_SET_MODE, [mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                                               px4_modes.MAIN_AUTO, AUTO_LOITER])]
    return [(mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH, [])]


def jetson_steps(action, altitude):
    """Lenh gui Jetson (1 lenh; Jetson tu tach thanh cac buoc cho PX4)."""
    if action == 'takeoff':
        return [(mavlink.MAV_CMD_NAV_TAKEOFF, [0, 0, 0, NAN, NAN, NAN, float(altitude)])]
    if action == 'land':
        return [(mavlink.MAV_CMD_NAV_LAND, [0, 0, 0, NAN, NAN, NAN, 0])]
    if action == 'hold':
        return [(mavlink.MAV_CMD_NAV_LOITER_UNLIM, [])]
    return [(mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH, [])]


class CommandLock:
    """Duong nao dang giu quyen lenh bay. Nha quyen khi:
      * drone da bay (armed / trong khong) roi ve dat + disarm, hoac
      * lenh cua duong do khong thanh (bi tu choi / het han), hoac duoc chap nhan nhung drone khong
        arm trong LINK_TIMEOUT_S (vd PX4 tu disarm)."""

    def __init__(self):
        self.owner = None
        self._flew = False
        self._idle_since = None

    def claim(self, route):
        if route != self.owner:
            self.owner, self._flew = route, False
        self._idle_since = None

    def update(self, now, armed, landed, owner_state):
        if self.owner is None:
            return
        if armed or landed in IN_AIR:
            self._flew, self._idle_since = True, None
            return
        if self._flew:                              # da bay, nay ve dat + disarm
            self.owner = None
            return
        if owner_state in PENDING:
            self._idle_since = None
            return
        if owner_state != 'accepted':               # tu choi / het han / bi chan: chua bay
            self.owner = None
            return
        self._idle_since = self._idle_since if self._idle_since is not None else now
        if now - self._idle_since > LINK_TIMEOUT_S:  # chap nhan nhung khong arm
            self.owner = None


class FlightCommander:
    """UI -> kiem tra -> CommandSender cua tung duong. Trang thai lenh hien o senders[route].values."""

    def __init__(self, senders, vehicle, jetson_target, environment=lambda: 'indoor'):
        self.senders = senders                  # {'px4': CommandSender, 'jetson': CommandSender}
        self.vehicle = vehicle
        self.jetson_target = jetson_target
        self.environment = environment          # -> 'indoor' | 'outdoor' (Settings / PX4 config)
        self.lock = CommandLock()

    # --- dieu kien -------------------------------------------------------------------------
    def available(self):
        v = self.vehicle
        return {'px4': v.sysid is not None and v.link is not None
                       and bool(v.groups['status'].values['linkAlive']),
                'jetson': bool(v.groups['companion'].values['linkAlive'])}

    def jetson_has_px4(self):
        """Jetson bao nhan duoc PX4 qua DDS (DRONE_COMPONENT_STATUS 'PX4 (DDS)' ok / warn)."""
        for c in self.vehicle.groups['companion'].items:
            if c.get('id') == mavlink.DRONE_COMPONENT_PX4:
                return c.get('level') in ('ok', 'warn')
        return False

    def blocker(self, route, action):
        """'' = nut dung duoc; nguoc lai = ly do khoa (hien tren UI)."""
        if not self.available()[route]:
            return 'PX4 not connected (radio 915)' if route == 'px4' else 'Jetson not connected (radio 433)'
        if action in FLIGHT_ACTIONS:
            owner = self.lock.owner
            if owner is not None and owner != route and self.available()[owner]:
                return f'flight controlled from {ROUTE_NAMES[owner]}'
            if route == 'jetson' and not self.jetson_has_px4():
                return 'Jetson has no PX4 data (DDS)'
        if route == 'jetson':
            fix = self.vehicle.groups['gps'].values.get('fixType') or 0
            if action == 'rtl' and fix < 3:
                return 'RTL needs GPS fix'
            if action in ('takeoff', 'upload') and self.environment() == 'outdoor' and fix < 3:
                return 'no GPS fix'
        return ''

    def gates(self):
        """{route: {action: ly do khoa ('' = mo)}} cho QML."""
        return {r: {a: self.blocker(r, a) for a in ACTIONS[r]} for r in ROUTES}

    # --- thuc hien -------------------------------------------------------------------------
    def execute(self, route, action, altitude=NAN, test=False):
        """test=True (chi duong jetson): bo qua moi dieu kien phia GCS, Jetson tu quyet."""
        if action not in FLIGHT_ACTIONS or action not in ACTIONS[route] or (test and route != 'jetson'):
            raise ValueError(f'{route}/{action}')
        sender = self.senders[route]
        label = LABELS[action] + (f' {altitude:g} m' if action == 'takeoff' and math.isfinite(altitude) else '')
        status = self.vehicle.groups['status'].values
        if test:
            label += ' [TEST]'
        why = '' if test else (self.blocker(route, action) or check_state(action, status['landed']))
        if why:
            sender.block(label, ROUTE_NAMES[route], why)
            return False
        if route == 'px4':
            target = CommandTarget(self.vehicle.link, self.vehicle.sysid, self.vehicle.compid,
                                   self.jetson_target.ack_timeout_s, self.jetson_target.retries,
                                   name=ROUTE_NAMES['px4'])
            sender.send_steps(target, px4_steps(action, altitude, status['armed'], self._amsl()), label)
        else:
            sender.send_steps(self.jetson_target, jetson_steps(action, altitude), label)
        self.lock.claim(route)
        return True

    def ping(self):
        """Kiem tra duong ground <-> Jetson: Jetson tra ACK (khong dung PX4). Ket qua o senders['jetson']."""
        self.senders['jetson'].send_steps(self.jetson_target, [(mavlink.MAV_CMD_USER_1, [])], 'Link test')

    def tick(self, now):
        st = self.vehicle.groups['status'].values
        owner = self.lock.owner
        self.lock.update(now, st['armed'], st['landed'],
                         self.senders[owner].values['state'] if owner else 'idle')

    def _amsl(self):
        """Do cao AMSL hien tai neu co vi tri toan cuc tin duoc (GPS 3D fix), nguoc lai None."""
        g = self.vehicle.groups
        if (g['gps'].values.get('fixType') or 0) < 3:
            return None
        return g['flight'].values.get('altGlobal')
