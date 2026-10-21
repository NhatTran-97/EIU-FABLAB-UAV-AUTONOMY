"""Flight commands: two routes, a cross-route lock and per-command preconditions.

Routes:
    px4    : radio 915 -> PX4. Take-off = [ARM if disarmed] -> NAV_TAKEOFF; ACK from PX4.
    jetson : radio 433 -> drone_telemetry -> uXRCE-DDS -> PX4. Also carries Send WP.

Cross-route lock (CommandLock): the route that sent a flight command (Take-off / Land / Hold / RTL) owns
control until the drone has landed and disarmed; the other route is blocked unless the owner loses its
link. RC always has priority.

Test mode: ping the Jetson (MAV_CMD_USER_1 -> ACK + STATUSTEXT, PX4 not involved); Take-off / Land via
the Jetson skip the GCS-side checks (DDS, GPS, lock, flight state). The Jetson still applies its own
safety checks.

Blockers:
  * link alive; flight commands via the Jetson need PX4 over DDS;
  * outdoors: Take-off / Send WP via the Jetson need a 3D GPS fix; RTL always needs GPS; Land / Hold never do;
  * EXTENDED_SYS_STATE: Take-off is blocked in the air, Land / Hold / RTL on the ground; unknown -> not blocked.
"""

import math

from ground_station_ui.core.commands.sender import CommandTarget
from ground_station_ui.core.mavlink import drone_dialect as mavlink
from ground_station_ui.core.vehicle import px4_modes
from ground_station_ui.core.vehicle.groups import LINK_TIMEOUT_S

ROUTES = ('px4', 'jetson')
ROUTE_NAMES = {'px4': 'PX4 · 915', 'jetson': 'Jetson · 433'}
ACTIONS = {'px4': ('takeoff', 'land'),
           'jetson': ('takeoff', 'land', 'hold', 'rtl', 'upload', 'mission', 'continue')}
FLIGHT_ACTIONS = ('takeoff', 'land', 'hold', 'rtl', 'mission', 'continue')
LABELS = {'takeoff': 'Take-off', 'land': 'Land', 'hold': 'Hold', 'rtl': 'RTL', 'upload': 'Send WP',
          'mission': 'Start mission', 'continue': 'Continue'}
OPT_AVOIDANCE = 1   # MISSION_START param3 bit 0 (project convention, see drone_telemetry)
IN_AIR = ('IN AIR', 'TAKING OFF', 'LANDING')
NAN = float('nan')
AUTO_LOITER = 3     # PX4_CUSTOM_SUB_MODE_AUTO_LOITER
PENDING = ('sending', 'in_progress')


def check_state(action, landed):
    """'' = allowed; otherwise the blocking reason. landed: StatusGroup.values['landed']."""
    if action == 'takeoff' and landed in IN_AIR:
        return 'already flying'
    if action in ('land', 'hold', 'rtl', 'mission', 'continue') and landed == 'ON GROUND':
        return 'vehicle is on the ground'
    return ''


def px4_steps(action, altitude, armed, amsl):
    """COMMAND_LONGs sent directly to PX4. altitude: m above the take-off point; amsl: current AMSL
    altitude (None without a global position -> NaN, PX4 then uses MIS_TAKEOFF_ALT)."""
    if action == 'takeoff':
        steps = [] if armed else [(mavlink.MAV_CMD_COMPONENT_ARM_DISARM, [1.0])]
        alt = amsl + altitude if amsl is not None and math.isfinite(altitude) else NAN
        # param7 = AMSL; lat / lon / yaw = NaN keep the current position / heading
        steps.append((mavlink.MAV_CMD_NAV_TAKEOFF, [NAN, 0, 0, NAN, NAN, NAN, alt]))
        return steps
    if action == 'land':
        return [(mavlink.MAV_CMD_NAV_LAND, [0, 0, 0, NAN, NAN, NAN, NAN])]
    if action == 'hold':
        return [(mavlink.MAV_CMD_DO_SET_MODE, [mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                                               px4_modes.MAIN_AUTO, AUTO_LOITER])]
    return [(mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH, [])]


def jetson_steps(action, altitude, options=None):
    """Command sent to the Jetson (one command; the Jetson splits it into PX4 steps).
    options for Start mission: {'avoidance': bool} -> bitmask in param3, {'speed': m/s} -> param4,
    {'xy_tolerance', 'z_tolerance': m} -> param5 / param6 (0 = Jetson default)."""
    if action == 'takeoff':
        return [(mavlink.MAV_CMD_NAV_TAKEOFF, [0, 0, 0, NAN, NAN, NAN, float(altitude)])]
    if action == 'land':
        return [(mavlink.MAV_CMD_NAV_LAND, [0, 0, 0, NAN, NAN, NAN, 0])]
    if action == 'hold':
        return [(mavlink.MAV_CMD_NAV_LOITER_UNLIM, [])]
    if action == 'mission':     # mission_manager flies the uploaded waypoints (from HOVER)
        options = options or {}
        bits = OPT_AVOIDANCE if options.get('avoidance') else 0
        return [(mavlink.MAV_CMD_MISSION_START, [0, 0, bits, float(options.get('speed') or 0.0),
                                                 float(options.get('xy_tolerance') or 0.0),
                                                 float(options.get('z_tolerance') or 0.0)])]
    if action == 'continue':    # after a manual task or a pause
        return [(mavlink.MAV_CMD_DO_PAUSE_CONTINUE, [1])]
    return [(mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH, [])]


class CommandLock:
    """Tracks which route owns flight control. Ownership is released when:
      * the drone has flown (armed / in air) and is back on the ground and disarmed, or
      * the route's command failed (rejected / timed out), or was accepted but the drone did not arm
        within LINK_TIMEOUT_S (e.g. PX4 auto-disarmed)."""

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
        if self._flew:                              # flight over: landed and disarmed
            self.owner = None
            return
        if owner_state in PENDING:
            self._idle_since = None
            return
        if owner_state != 'accepted':               # rejected / timeout / denied
            self.owner = None
            return
        self._idle_since = self._idle_since if self._idle_since is not None else now
        if now - self._idle_since > LINK_TIMEOUT_S:  # accepted but never armed
            self.owner = None


class FlightCommander:
    """UI -> checks -> per-route CommandSender. Command state is exposed in senders[route].values."""

    def __init__(self, senders, vehicle, jetson_target, environment=lambda: 'indoor'):
        self.senders = senders                  # {'px4': CommandSender, 'jetson': CommandSender}
        self.vehicle = vehicle
        self.jetson_target = jetson_target
        self.environment = environment          # -> 'indoor' | 'outdoor' (Settings / PX4 config)
        self.lock = CommandLock()

    # --- preconditions ------------------------------------------------------------------
    def available(self):
        v = self.vehicle
        return {'px4': v.sysid is not None and v.link is not None
                       and bool(v.groups['status'].values['linkAlive']),
                'jetson': bool(v.groups['companion'].values['linkAlive'])}

    def jetson_has_px4(self):
        """Jetson reports PX4 over DDS (DRONE_COMPONENT_STATUS 'PX4 (DDS)' ok / warn)."""
        for c in self.vehicle.groups['companion'].items:
            if c.get('id') == mavlink.DRONE_COMPONENT_PX4:
                return c.get('level') in ('ok', 'warn')
        return False

    def blocker(self, route, action):
        """'' = button enabled; otherwise the lock reason shown in the UI."""
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
            if action == 'mission' and fix < 3:        # waypoints are GPS coordinates
                return 'mission needs GPS fix'
            if action in ('takeoff', 'upload') and self.environment() == 'outdoor' and fix < 3:
                return 'no GPS fix'
        return ''

    def gates(self):
        """{route: {action: lock reason ('' = enabled)}} for QML."""
        return {r: {a: self.blocker(r, a) for a in ACTIONS[r]} for r in ROUTES}

    # --- execution ----------------------------------------------------------------------
    def _jetson(self):
        """Jetson target on companion_link (433 / SITL UDP)."""
        self.jetson_target.link = self.vehicle.companion_link
        return self.jetson_target

    def execute(self, route, action, altitude=NAN, test=False, options=None):
        """test=True (jetson route only): skip all GCS-side checks and let the Jetson decide."""
        if action not in FLIGHT_ACTIONS or action not in ACTIONS[route] or (test and route != 'jetson'):
            raise ValueError(f'{route}/{action}')
        sender = self.senders[route]
        label = LABELS[action] + (f' {altitude:g} m' if action == 'takeoff' and math.isfinite(altitude) else '')
        if action == 'mission' and (options or {}).get('speed'):
            label += f" {options['speed']:g} m/s"
        if action == 'mission' and (options or {}).get('avoidance'):
            label += ' + avoidance'
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
            sender.send_steps(self._jetson(), jetson_steps(action, altitude, options), label)
        self.lock.claim(route)
        return True

    def ping(self):
        """Ping the Jetson (ACK without PX4). The result is in senders['jetson']."""
        self.senders['jetson'].send_steps(self._jetson(), [(mavlink.MAV_CMD_USER_1, [])], 'Link test')

    def tick(self, now):
        st = self.vehicle.groups['status'].values
        owner = self.lock.owner
        self.lock.update(now, st['armed'], st['landed'],
                         self.senders[owner].values['state'] if owner else 'idle')

    def _amsl(self):
        """Current AMSL altitude with a 3D GPS fix or better, otherwise None."""
        g = self.vehicle.groups
        if (g['gps'].values.get('fixType') or 0) < 3:
            return None
        return g['flight'].values.get('altGlobal')
