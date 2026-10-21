"""Navigation strategies used by Navigate steps: how to get from here to a goal.

    DirectNav     the setpoint is a carrot moving towards the goal at the mission speed.
    AvoidanceNav  the setpoint is the safe point proposed by the obstacle_avoidance node (VFH+ carrot),
                  with hold logic when its data is stale, the path is blocked or no progress is made.

Extra context used by AvoidanceNav:
    ctx.avoidance_status()   (status, receive_time) of the latest /avoidance/status, or (None, None)
    ctx.publish_goal(point)  publish the real goal on /mission/goal
"""

import math

from mission_pipeline.steps import Failed

NOMINAL, AVOIDING, HOLDING = 'NOMINAL', 'AVOIDING', 'HOLDING'

# AvoidanceStatus.state values (custom_msgs/AvoidanceStatus)
CLEAR, STATUS_AVOIDING, BLOCKED, STALE = 0, 1, 2, 3


class DirectNav:
    """Straight line to the goal at a limited speed.

    The setpoint is a carrot that moves from the start point towards the goal at cruise_speed (m/s), so PX4
    never sees a far-away setpoint and flies the leg at that speed instead of its maximum. The carrot stays
    at most max_lead metres ahead of the drone, so it waits when the drone lags (wind, slow climb).
    cruise_speed <= 0 disables the limit: the setpoint is the goal itself.
    """
    name = 'direct'
    mode = NOMINAL

    def __init__(self, cruise_speed=0.0, max_lead=3.0):
        self.cruise_speed = cruise_speed
        self.max_lead = max_lead
        self.carrot = None

    def begin(self, ctx, goal):
        self.carrot = ctx.position
        self.t = ctx.now()

    def setpoint(self, ctx, goal):
        if self.cruise_speed <= 0.0 or self.carrot is None:
            return goal
        now = ctx.now()
        dt, self.t = max(0.0, now - self.t), now
        remaining = math.dist(self.carrot, goal)
        pos = ctx.position
        if remaining > 1e-6 and (pos is None or math.dist(self.carrot, pos) < self.max_lead):
            k = min(1.0, self.cruise_speed * dt / remaining)
            self.carrot = tuple(c + k * (g - c) for c, g in zip(self.carrot, goal))
        return self.carrot

    def end(self, ctx):
        self.carrot = None


class AvoidanceConfig:
    def __init__(self, timeout_periods=3.0, max_timeout=1.0, clear_time=0.5, blocked_timeout=5.0,
                 blocked_action='hold', stuck_timeout=15.0, stuck_min_progress=1.0):
        self.timeout_periods = timeout_periods    # status timeout = periods x measured scan period
        self.max_timeout = max_timeout            # s; cap, also used until the scan period is known
        self.clear_time = clear_time              # s of clear path before AVOIDING -> NOMINAL
        self.blocked_timeout = blocked_timeout    # s in HOLDING before blocked_action
        self.blocked_action = blocked_action      # 'hold' (pilot takes over) | 'land'
        self.stuck_timeout = stuck_timeout        # s without stuck_min_progress m of progress -> HOLDING
        self.stuck_min_progress = stuck_min_progress


def status_timeout(cfg, status):
    """Maximum age of an avoidance status: periods x measured scan period, capped at max_timeout."""
    if status is None or not status.scan_period > 0.0:
        return cfg.max_timeout
    return min(cfg.max_timeout, cfg.timeout_periods * status.scan_period)


class AvoidanceNav:
    """Follow the avoidance node's safe setpoint; hold position when it cannot be trusted.

    HOLDING is entered when the status is stale, the node reports STALE or BLOCKED, or the distance to the
    goal has not shrunk by stuck_min_progress within stuck_timeout (VFH+ can oscillate in a U-shaped trap
    without reporting BLOCKED). After blocked_timeout in HOLDING, blocked_action applies: keep holding for the
    pilot, or fail with land=True.
    """
    name = 'avoidance'

    def __init__(self, cfg):
        self.cfg = cfg
        self.mode = NOMINAL
        self._reset_goal()

    def _reset_goal(self):
        self.goal_key = None
        self.best_distance = math.inf
        self.last_progress = 0.0
        self.stuck = False
        self.clear_since = None
        self._release()
        self.blocked_done = False

    def _release(self):
        self.hold_point = None
        self.hold_start = None
        self.hold_reason = ''

    def begin(self, ctx, goal):
        self._reset_goal()
        self.mode = NOMINAL

    def end(self, ctx):
        self._release()
        self.mode = NOMINAL

    def _set_mode(self, ctx, mode, reason):
        if mode == self.mode:
            return
        ctx.log(f'Nav mode: {self.mode} --> {mode} ({reason})', warn=(mode == HOLDING))
        self.mode = mode
        if mode != NOMINAL:
            self.clear_since = None

    def _track_progress(self, ctx, goal, now):
        key = tuple(round(v, 3) for v in goal)
        if key != self.goal_key:
            self._reset_goal()
            self.goal_key = key
            self.last_progress = now
        pos = ctx.position
        if pos is None:
            return
        distance = math.hypot(goal[0] - pos[0], goal[1] - pos[1])
        if distance < self.best_distance - self.cfg.stuck_min_progress:
            self.best_distance = distance
            self.last_progress = now

    def _hold(self, ctx, reason, now, pause_progress=False):
        if pause_progress:
            self.last_progress = now
        if self.hold_point is None:
            self.hold_point = ctx.position
            self.hold_start = now
            self.hold_reason = reason
            self._set_mode(ctx, HOLDING, reason)
        if not self.blocked_done and now - self.hold_start > self.cfg.blocked_timeout:
            self.blocked_done = True
            if self.cfg.blocked_action == 'land':
                ctx.log(f'Holding {self.cfg.blocked_timeout:.0f} s ({self.hold_reason}) -> LAND', warn=True)
                return Failed(f'avoidance: {self.hold_reason}', land=True)
            ctx.log(f'Holding {self.cfg.blocked_timeout:.0f} s ({self.hold_reason}) '
                    '-> staying in HOLD, pilot must take over', warn=True)
            ctx.say('Avoidance blocked: holding', warn=True)
        return self.hold_point

    def setpoint(self, ctx, goal):
        now = ctx.now()
        ctx.publish_goal(goal)
        self._track_progress(ctx, goal, now)
        status, received = ctx.avoidance_status()
        fresh = status is not None and received is not None and now - received <= status_timeout(self.cfg, status)

        if self.stuck or now - self.last_progress > self.cfg.stuck_timeout:
            self.stuck = True
            return self._hold(ctx, f'no progress: goal distance did not shrink {self.cfg.stuck_min_progress:.1f} m '
                                   f'in {self.cfg.stuck_timeout:.0f} s', now)
        if not fresh:
            return self._hold(ctx, 'no fresh /avoidance/status', now, pause_progress=True)
        if status.state == STALE:
            return self._hold(ctx, 'avoidance STALE (lost scan/pose/goal)', now, pause_progress=True)
        if status.state == BLOCKED:
            return self._hold(ctx, f'avoidance BLOCKED (nearest obstacle {status.min_distance:.2f} m)', now,
                              pause_progress=True)

        self._release()
        if status.state == STATUS_AVOIDING:
            self.clear_since = None
            self._set_mode(ctx, AVOIDING, f'obstacle {status.min_distance:.2f} m')
        else:
            if self.mode == HOLDING:
                self._set_mode(ctx, AVOIDING, 'path clear again, resuming')
            if self.clear_since is None:
                self.clear_since = now
            if self.mode == AVOIDING and now - self.clear_since >= self.cfg.clear_time:
                self._set_mode(ctx, NOMINAL, 'path to goal clear')
        return (float(status.safe_setpoint.x), float(status.safe_setpoint.y), float(goal[2]))
