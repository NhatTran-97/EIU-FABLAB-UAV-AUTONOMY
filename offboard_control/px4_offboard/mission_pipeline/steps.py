"""Mission steps. Each step is one unit of work with start / tick / abort.

A step talks to the vehicle only through the context object (`ctx`) provided by mission_manager:

    ctx.now()                      node time (s)
    ctx.position                   (x, y, z) local NED (m), or None
    ctx.agl                        height above ground from the rangefinder (m), NaN when invalid
    ctx.heading                    current heading (rad, NED: 0 = north, clockwise), NaN when unknown
    ctx.marker()                   (north, east) offset from the drone to the marker (m), or None if not fresh
    ctx.offboard_active            PX4 is in OFFBOARD
    ctx.set_target(x, y, z)        position setpoint (NED)
    ctx.set_velocity(vn, ve, vd)   velocity setpoint (NED, m/s)
    ctx.set_yaw(yaw)               heading setpoint (rad); NaN keeps the take-off heading
    ctx.nav                        navigation strategy for Navigate steps
    ctx.payload_request()          start a hook request; returns a token, or None if no payload node
    ctx.payload_result(token)      None while pending, else (ok: bool, reason: str)
    ctx.take_continue()            True once if the operator pressed Continue
    ctx.request_resume()           ask offboard_control to request OFFBOARD again
    ctx.say(text, warn=False)      short status message for the ground station (<= 50 characters)
    ctx.log(text, warn=False)      node log only
    ctx.cfg                        StepConfig
"""

import math
from dataclasses import dataclass

RUNNING = 'running'
SUCCEEDED = 'succeeded'


class Failed:
    """Step failure with a reason. land=True asks mission_manager to land (e.g. avoidance blocked_action)."""

    def __init__(self, reason, land=False):
        self.reason = reason
        self.land = land

    def __repr__(self):
        return f'Failed({self.reason!r})'


@dataclass
class StepConfig:
    xy_tolerance: float = 0.5           # m, goal reached horizontally
    z_tolerance: float = 0.5            # m, goal reached vertically
    goal_hold_time: float = 0.5         # s inside the tolerance before a goal counts as reached
    agl_tolerance: float = 0.2          # m, height above ground reached
    agl_lost_timeout: float = 2.0       # s without a valid rangefinder before Descend / Align fail
    descend_timeout: float = 30.0       # s
    search_timeout: float = 20.0        # s to find the marker
    marker_seen_time: float = 0.5       # s of continuous detection before Search succeeds
    marker_lost_timeout: float = 3.0    # s without the marker before Align fails
    align_gain: float = 0.6             # 1/s, horizontal velocity per metre of offset
    align_max_speed: float = 0.4        # m/s
    align_tolerance: float = 0.15       # m, horizontal offset counted as aligned
    align_vz_gain: float = 0.5          # 1/s, vertical velocity per metre of height error
    align_max_vz: float = 0.3           # m/s
    align_timeout: float = 60.0         # s
    payload_timeout: float = 30.0       # s for the payload node to report the hook result
    manual_reminder: float = 15.0       # s between reminders while waiting for the pilot
    yaw_mode: str = 'waypoint'          # Navigate heading: 'waypoint' (face the waypoint) | 'hold' (keep heading)
    yaw_rate: float = math.radians(45)  # rad/s, maximum turn rate commanded by Navigate
    turn_first: float = math.radians(10)  # rad; larger heading error at a leg start -> turn in place first (0 = off)
    turn_timeout: float = 10.0          # s; if the heading is not reached, move on anyway (logged)
    yaw_freeze_radius: float = 1.5      # m; closer to the waypoint the heading is kept (bearing gets unstable)


def _clamp(v, limit):
    return max(-limit, min(limit, v))


def wrap_angle(a):
    """Angle in (-pi, pi]."""
    return math.atan2(math.sin(a), math.cos(a))


def bearing(frm, to):
    """Heading (rad, NED: 0 = north, clockwise) from point frm to point to."""
    return math.atan2(to[1] - frm[1], to[0] - frm[0])


class Step:
    """Base class. `name` is shown in progress reports; `uses_avoidance` enables the avoidance node;
    `expects_manual` tells mission_manager that losing OFFBOARD during this step is planned."""
    name = 'step'
    uses_avoidance = False
    expects_manual = False

    def start(self, ctx):
        pass

    def tick(self, ctx):
        return SUCCEEDED

    def abort(self, ctx):
        pass


class _HoldHere(Step):
    """Shared helper: latch the position at start and hold it."""

    def _latch(self, ctx):
        self.hold = ctx.position

    def _hold(self, ctx):
        if self.hold is not None:
            ctx.set_target(*self.hold)


class _GoalHold:
    """Tracks how long a condition has been true continuously."""

    def __init__(self):
        self.since = None

    def update(self, ok, now, needed):
        if not ok:
            self.since = None
            return False
        if self.since is None:
            self.since = now
        return now - self.since >= needed


class Navigate(Step):
    """Fly to a point using the navigation strategy in ctx.nav (direct or obstacle avoidance).

    Heading (cfg.yaw_mode = 'waypoint'): at the start of every leg the heading is checked; if the error exceeds
    cfg.turn_first the drone turns in place and moves only once the error is below half of it (or after
    cfg.turn_timeout). During the leg the nose keeps pointing at the waypoint, turning at most cfg.yaw_rate.
    Within cfg.yaw_freeze_radius of the waypoint the heading is kept, since the bearing becomes unstable.
    A finite `yaw` overrides this with a fixed heading; yaw_mode 'hold' keeps the take-off heading (NaN).
    """
    name = 'Navigate'

    def __init__(self, point, yaw=math.nan):
        self.point = point
        self.yaw = yaw
        self.uses_avoidance = True       # only effective when the strategy is Avoidance

    def start(self, ctx):
        self.done = _GoalHold()
        self.hold = ctx.position
        self.t = ctx.now()
        heading = ctx.heading
        self.yaw_cmd = heading if math.isfinite(heading) else None
        self.yaw_target = None
        target = self._target_heading(ctx)
        self.turning = (target is not None and self.yaw_cmd is not None and ctx.cfg.turn_first > 0
                        and abs(wrap_angle(target - self.yaw_cmd)) > ctx.cfg.turn_first)
        self.turn_start = ctx.now()
        if not self.turning:
            ctx.nav.begin(ctx, self.point)

    def _target_heading(self, ctx):
        """Heading to fly with, or None to leave the heading unchanged (NaN = keep take-off heading)."""
        if math.isfinite(self.yaw):
            return self.yaw
        if ctx.cfg.yaw_mode != 'waypoint':
            return math.nan
        pos = ctx.position
        if pos is not None and math.hypot(self.point[0] - pos[0], self.point[1] - pos[1]) > ctx.cfg.yaw_freeze_radius:
            self.yaw_target = bearing(pos, self.point)
        return self.yaw_target

    def _steer(self, ctx):
        """Command the heading towards the target heading, limited to cfg.yaw_rate."""
        now = ctx.now()
        dt, self.t = max(0.0, now - self.t), now
        target = self._target_heading(ctx)
        if target is None:
            if self.yaw_cmd is not None:
                ctx.set_yaw(self.yaw_cmd)
            return
        if not math.isfinite(target):
            ctx.set_yaw(target)
            return
        if self.yaw_cmd is None:
            self.yaw_cmd = target
        step = _clamp(wrap_angle(target - self.yaw_cmd), ctx.cfg.yaw_rate * dt)
        self.yaw_cmd = wrap_angle(self.yaw_cmd + step)
        ctx.set_yaw(self.yaw_cmd)

    def tick(self, ctx):
        self._steer(ctx)
        if self.turning:
            if self.hold is not None:
                ctx.set_target(*self.hold)
            target = self.yaw if math.isfinite(self.yaw) else self.yaw_target
            heading = ctx.heading
            aligned = (target is None or not math.isfinite(heading)
                       or abs(wrap_angle(target - heading)) < ctx.cfg.turn_first / 2)
            if not aligned and ctx.now() - self.turn_start > ctx.cfg.turn_timeout:
                ctx.log(f'Heading not reached within {ctx.cfg.turn_timeout:.0f} s, moving on', warn=True)
                aligned = True
            if aligned:
                self.turning = False
                ctx.nav.begin(ctx, self.point)
            return RUNNING
        result = ctx.nav.setpoint(ctx, self.point)
        if isinstance(result, Failed):
            return result
        ctx.set_target(*result)
        pos = ctx.position
        if pos is None:
            return RUNNING
        x, y, z = self.point
        ok = (math.hypot(x - pos[0], y - pos[1]) < ctx.cfg.xy_tolerance
              and abs(z - pos[2]) < ctx.cfg.z_tolerance)
        return SUCCEEDED if self.done.update(ok, ctx.now(), ctx.cfg.goal_hold_time) else RUNNING

    def abort(self, ctx):
        ctx.nav.end(ctx)


class HoldTime(_HoldHere):
    """Hold the current position for a number of seconds (NAV_LOITER_TIME)."""
    name = 'Hold'

    def __init__(self, seconds):
        self.seconds = max(0.0, float(seconds))

    def start(self, ctx):
        self._latch(ctx)
        self.t0 = ctx.now()

    def tick(self, ctx):
        self._hold(ctx)
        return SUCCEEDED if ctx.now() - self.t0 >= self.seconds else RUNNING


class Descend(Step):
    """Move vertically to a height above ground measured by the rangefinder (works up or down)."""
    name = 'Descend'

    def __init__(self, agl):
        self.agl = float(agl)

    def start(self, ctx):
        pos = ctx.position
        self.xy = (pos[0], pos[1]) if pos else None
        self.t0 = ctx.now()
        self.bad_since = None
        self.done = _GoalHold()

    def tick(self, ctx):
        now, pos, agl = ctx.now(), ctx.position, ctx.agl
        if pos is None or self.xy is None:
            return Failed('no position')
        if now - self.t0 > ctx.cfg.descend_timeout:
            return Failed(f'{self.agl:.1f} m AGL not reached')
        if not math.isfinite(agl):
            if self.bad_since is None:
                self.bad_since = now
            ctx.set_target(self.xy[0], self.xy[1], pos[2])
            if now - self.bad_since > ctx.cfg.agl_lost_timeout:
                return Failed('no rangefinder height')
            return RUNNING
        self.bad_since = None
        # NED: z grows downwards, so a lower target height means a larger z
        ctx.set_target(self.xy[0], self.xy[1], pos[2] + (agl - self.agl))
        ok = abs(agl - self.agl) < ctx.cfg.agl_tolerance
        return SUCCEEDED if self.done.update(ok, now, ctx.cfg.goal_hold_time) else RUNNING


class Search(_HoldHere):
    """Hold position until the marker has been seen continuously for marker_seen_time."""
    name = 'Search'

    def start(self, ctx):
        self._latch(ctx)
        self.t0 = ctx.now()
        self.seen = _GoalHold()

    def tick(self, ctx):
        self._hold(ctx)
        now = ctx.now()
        if self.seen.update(ctx.marker() is not None, now, ctx.cfg.marker_seen_time):
            return SUCCEEDED
        if now - self.t0 > ctx.cfg.search_timeout:
            return Failed('marker not found')
        return RUNNING


class Align(Step):
    """Centre over the marker with velocity setpoints, descending to the hook height once roughly centred."""
    name = 'Align'

    def __init__(self, agl):
        self.agl = float(agl)

    def start(self, ctx):
        self.t0 = ctx.now()
        self.lost_since = None
        self.done = _GoalHold()

    def tick(self, ctx):
        cfg, now = ctx.cfg, ctx.now()
        if now - self.t0 > cfg.align_timeout:
            return Failed('alignment timeout')
        offset = ctx.marker()
        if offset is None:
            if self.lost_since is None:
                self.lost_since = now
            if ctx.position is not None:
                ctx.set_target(*ctx.position)
            if now - self.lost_since > cfg.marker_lost_timeout:
                return Failed('marker lost')
            return RUNNING
        self.lost_since = None
        dn, de = offset
        err = math.hypot(dn, de)
        agl = ctx.agl
        vd = 0.0
        if err < 2 * cfg.align_tolerance and math.isfinite(agl):
            vd = _clamp(cfg.align_vz_gain * (agl - self.agl), cfg.align_max_vz)   # positive = down
        ctx.set_velocity(_clamp(cfg.align_gain * dn, cfg.align_max_speed),
                         _clamp(cfg.align_gain * de, cfg.align_max_speed), vd)
        ok = err < cfg.align_tolerance and math.isfinite(agl) and abs(agl - self.agl) < cfg.agl_tolerance
        return SUCCEEDED if self.done.update(ok, now, cfg.goal_hold_time) else RUNNING


class PayloadHook(_HoldHere):
    """Ask the payload node to hook the load and wait for its result, holding position."""
    name = 'Hook'

    def start(self, ctx):
        self._latch(ctx)
        self.t0 = ctx.now()
        self.token = ctx.payload_request()

    def tick(self, ctx):
        self._hold(ctx)
        if self.token is None:
            return Failed('payload node not available')
        result = ctx.payload_result(self.token)
        if result is None:
            return Failed('payload timeout') if ctx.now() - self.t0 > ctx.cfg.payload_timeout else RUNNING
        ok, reason = result
        return SUCCEEDED if ok else Failed(f'hook failed: {reason}')


class ManualHandover(_HoldHere):
    """Planned hand-over to the pilot. Holds position and waits until the pilot has finished.

    Finished means: the pilot took control and switched back to Offboard, or the operator pressed Continue.
    Losing OFFBOARD during this step is expected, so mission_manager does not pause.
    """
    name = 'Manual'
    expects_manual = True

    def __init__(self, label):
        self.label = label

    def start(self, ctx):
        self._latch(ctx)
        self.left_offboard = False
        self.reminded = ctx.now()
        ctx.say(f'{self.label}: take control, then Continue', warn=True)

    def tick(self, ctx):
        self._hold(ctx)
        now = ctx.now()
        if not ctx.offboard_active:
            self.left_offboard = True
        elif self.left_offboard:
            return SUCCEEDED                      # pilot switched back to Offboard
        if ctx.take_continue():
            if not ctx.offboard_active:
                ctx.request_resume()
            return SUCCEEDED
        if now - self.reminded > ctx.cfg.manual_reminder:
            self.reminded = now
            ctx.say(f'{self.label}: waiting for pilot / Continue', warn=True)
        return RUNNING


class Ascend(Step):
    """Climb (or descend) vertically to an absolute NED z, e.g. back to the waypoint altitude."""
    name = 'Ascend'

    def __init__(self, z):
        self.z = float(z)

    def start(self, ctx):
        pos = ctx.position
        self.xy = (pos[0], pos[1]) if pos else None
        self.done = _GoalHold()

    def tick(self, ctx):
        pos = ctx.position
        if pos is None or self.xy is None:
            return Failed('no position')
        ctx.set_target(self.xy[0], self.xy[1], self.z)
        ok = abs(pos[2] - self.z) < ctx.cfg.z_tolerance
        return SUCCEEDED if self.done.update(ok, ctx.now(), ctx.cfg.goal_hold_time) else RUNNING
