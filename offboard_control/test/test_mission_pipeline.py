"""Mission pipeline tests with a simulated vehicle (no ROS)."""

import math
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'px4_offboard'))

from mission_pipeline.compiler import (CMD_LOITER_TIME, CMD_PICK, CompileConfig, MissionOptions,  # noqa: E402
                                       compile_mission)
from mission_pipeline.executor import DONE, MissionExecutor  # noqa: E402
from mission_pipeline.navigation import AvoidanceConfig, AvoidanceNav, DirectNav  # noqa: E402
from mission_pipeline.steps import RUNNING, Failed, StepConfig  # noqa: E402

REF = (11.0, 106.0)
DEG_PER_M = 1.0 / 111195.0


class SimCtx:
    """Point-mass vehicle: flies to the position setpoint at 2 m/s or follows the velocity setpoint.
    Ground at z = 0 (agl = -z). The marker is seen within 4 m horizontally when below 6 m."""

    def __init__(self, nav=None, marker_at=(0.0, 0.0), marker_visible=True, payload_ok=True):
        self.t = 0.0
        self.pos = [0.0, 0.0, -5.0]
        self.mode, self.target, self.vel = 'pos', list(self.pos), [0.0, 0.0, 0.0]
        self.nav = nav or DirectNav()
        self.cfg = StepConfig()
        self.offboard_active = True
        self.marker_at = marker_at
        self.marker_visible = marker_visible
        self.payload_ok = payload_ok
        self.continue_pressed = False
        self.resume_requested = False
        self.said = []
        self.payload_calls = 0
        self.status = (None, None)
        self.heading = 0.0                 # rad, follows the yaw command at 90 deg/s
        self.yaw_cmd = math.nan
        self.yaw_log = []

    # context API
    def now(self):
        return self.t

    @property
    def position(self):
        return tuple(self.pos)

    @property
    def agl(self):
        return -self.pos[2]

    def marker(self):
        dn, de = self.marker_at[0] - self.pos[0], self.marker_at[1] - self.pos[1]
        if self.marker_visible and math.hypot(dn, de) < 4.0 and self.agl < 6.0:
            return dn, de
        return None

    def set_target(self, x, y, z):
        self.mode, self.target = 'pos', [x, y, z]

    def set_velocity(self, vn, ve, vd):
        self.mode, self.vel = 'vel', [vn, ve, vd]

    def set_yaw(self, yaw):
        self.yaw_cmd = yaw
        self.yaw_log.append((self.t, yaw))

    def payload_request(self):
        self.payload_calls += 1
        self.payload_t0 = self.t
        return 'token'

    def payload_result(self, token):
        if self.t - self.payload_t0 < 1.0:
            return None
        return (True, '') if self.payload_ok else (False, 'jammed')

    def take_continue(self):
        pressed, self.continue_pressed = self.continue_pressed, False
        return pressed

    def request_resume(self):
        self.resume_requested = True

    def say(self, text, warn=False):
        self.said.append(text)

    def log(self, text, warn=False):
        pass

    def avoidance_status(self):
        return self.status

    def publish_goal(self, goal):
        pass

    # simulation
    def step(self, dt=0.05):
        self.t += dt
        if math.isfinite(self.yaw_cmd):
            err = math.atan2(math.sin(self.yaw_cmd - self.heading), math.cos(self.yaw_cmd - self.heading))
            self.heading += max(-math.radians(90) * dt, min(math.radians(90) * dt, err))
        if self.mode == 'vel':
            self.pos = [p + v * dt for p, v in zip(self.pos, self.vel)]
            return
        d = [t - p for t, p in zip(self.target, self.pos)]
        n = math.sqrt(sum(v * v for v in d))
        if n > 1e-9:
            k = min(1.0, 2.0 * dt / n)
            self.pos = [p + k * v for p, v in zip(self.pos, d)]


def wp(north, east, alt=5.0, command=16, params=None):
    return {'seq': 0, 'lat': REF[0] + north * DEG_PER_M, 'lon': REF[1] + east * DEG_PER_M / math.cos(math.radians(REF[0])),
            'alt': alt, 'command': command, 'frame': 6, 'params': params or []}


CFG = CompileConfig(max_altitude=15.0, max_distance=300.0)


def build(items, ctx, avoidance=False):
    plan, needs, why = compile_mission(items, REF, 0.0, ctx.position, MissionOptions(avoidance), CFG)
    assert why == '', why
    return MissionExecutor(plan, len(items)), needs


def run(ex, ctx, seconds, until=None):
    for _ in range(int(seconds / 0.05)):
        r = ex.tick(ctx)
        if r != RUNNING or (until and until()):
            return r
        ctx.step()
    return RUNNING


def test_default_mission_flies_waypoints_and_finishes():
    ctx = SimCtx()
    ex, needs = build([wp(10, 0), wp(10, 10)], ctx)
    assert needs == set()
    ex.start(ctx)
    assert run(ex, ctx, 60) == DONE
    assert math.hypot(ctx.pos[0] - 10, ctx.pos[1] - 10) < 0.8


def test_loiter_holds_for_param_seconds():
    ctx = SimCtx()
    ex, _ = build([wp(5, 0, command=CMD_LOITER_TIME, params=[3.0])], ctx)
    ex.start(ctx)
    t_arrive = None
    while ex.tick(ctx) == RUNNING:
        if ex.current_step.name == 'Hold' and t_arrive is None:
            t_arrive = ctx.t
        ctx.step()
    assert ctx.t - t_arrive >= 3.0


def test_manual_pick_descends_aligns_waits_for_pilot_then_ascends():
    ctx = SimCtx(marker_at=(10.3, -0.2))
    ex, needs = build([wp(10, 0, alt=8.0, command=CMD_PICK, params=[1, 3.0, 1.0])], ctx)
    assert needs == {'rangefinder', 'marker'}
    ex.start(ctx)
    assert run(ex, ctx, 90, until=lambda: ex.current_step.name == 'Manual') == RUNNING
    assert ex.current_step.name == 'Manual'
    assert abs(ctx.agl - 1.0) < 0.3                                  # at hook height
    assert math.hypot(ctx.pos[0] - 10.3, ctx.pos[1] + 0.2) < 0.2      # centred on the marker
    assert any('take control' in s for s in ctx.said)
    # pilot takes over, then switches back to Offboard
    ctx.offboard_active = False
    run(ex, ctx, 2)
    assert ex.current_step.name == 'Manual'
    ctx.offboard_active = True
    assert run(ex, ctx, 30) == DONE
    assert abs(ctx.pos[2] + 8.0) < 0.5                                # back at waypoint altitude


def test_continue_button_ends_handover_and_requests_offboard():
    ctx = SimCtx()
    ex, _ = build([wp(5, 0, alt=8.0, command=CMD_PICK, params=[1, 3.0, 1.0])], ctx)
    ctx.marker_at = (5.0, 0.0)
    ex.start(ctx)
    run(ex, ctx, 90, until=lambda: ex.current_step.name == 'Manual')
    ctx.offboard_active = False                                     # pilot still in Position mode
    run(ex, ctx, 1)
    ctx.continue_pressed = True
    ex.tick(ctx)
    assert ctx.resume_requested
    assert ex.current_step.name == 'Ascend'


def test_auto_pick_calls_payload():
    ctx = SimCtx(marker_at=(5.0, 0.0))
    ex, needs = build([wp(5, 0, alt=8.0, command=CMD_PICK, params=[0, 3.0, 1.0])], ctx)
    assert 'payload' in needs
    ex.start(ctx)
    assert run(ex, ctx, 120) == DONE
    assert ctx.payload_calls == 1
    assert not any('take control' in s for s in ctx.said)


def test_marker_not_found_falls_back_to_manual_and_skips_align_and_hook():
    ctx = SimCtx(marker_visible=False)
    ex, _ = build([wp(5, 0, alt=8.0, command=CMD_PICK, params=[0, 3.0, 1.0])], ctx)
    ex.start(ctx)
    run(ex, ctx, 120, until=lambda: ex.current_step.name == 'Manual')
    assert ex.current_step.name == 'Manual'
    assert any('marker not found' in s for s in ctx.said)
    ctx.continue_pressed = True
    ex.tick(ctx)
    assert ex.current_step.name == 'Ascend'                          # Align and Hook skipped
    assert run(ex, ctx, 30) == DONE
    assert ctx.payload_calls == 0


def test_compiler_rejects_bad_missions():
    ctx = SimCtx()
    bad = [
        [wp(5, 0, command=178)],                                     # unsupported command
        [wp(5, 0, alt=20.0)],                                        # above max altitude
        [wp(5, 0, alt=8.0, command=CMD_PICK, params=[1, 2.0, 3.0])], # hook above search height
        [wp(500, 0)],                                                # too far
    ]
    for items in bad:
        plan, _, why = compile_mission(items, REF, 0.0, ctx.position, MissionOptions(), CFG)
        assert plan is None and why
    _, _, why = compile_mission([wp(5, 0)], None, 0.0, ctx.position, MissionOptions(), CFG)
    assert 'GPS' in why


def test_avoidance_option_adds_capability_and_holds_without_status():
    nav = AvoidanceNav(AvoidanceConfig(blocked_timeout=2.0, blocked_action='land'))
    ctx = SimCtx(nav=nav)
    ex, needs = build([wp(10, 0)], ctx, avoidance=True)
    assert needs == {'avoidance'}
    ex.start(ctx)
    start = ctx.position
    r = run(ex, ctx, 5)
    assert isinstance(r, Failed) and r.land                           # held, then blocked_action = land
    assert math.dist(start, ctx.position) < 0.1                       # never moved without avoidance data


def test_pause_and_resume_restart_current_step():
    ctx = SimCtx()
    ex, _ = build([wp(10, 0)], ctx)
    ex.start(ctx)
    run(ex, ctx, 1)
    ex.pause(ctx)
    assert ex.tick(ctx) == RUNNING
    ex.resume(ctx)
    assert run(ex, ctx, 30) == DONE


def test_direct_nav_limits_speed_to_cruise_speed():
    ctx = SimCtx(nav=DirectNav(cruise_speed=1.0))
    ex, _ = build([wp(10, 0)], ctx)
    ex.start(ctx)
    run(ex, ctx, 4.0)
    assert 3.0 < ctx.pos[0] < 4.5          # ~1 m/s (the sim vehicle could fly 2 m/s)
    assert run(ex, ctx, 30) == DONE


def test_navigate_faces_the_waypoint_and_turns_first():
    ctx = SimCtx()
    ex, _ = build([wp(0, 10)], ctx)                                   # due east, nose starts north
    ex.start(ctx)
    run(ex, ctx, 0.5)
    assert math.hypot(ctx.pos[0], ctx.pos[1]) < 0.05                  # turning in place, not moving yet
    run(ex, ctx, 3.0)
    assert abs(math.degrees(ctx.heading) - 90) < 5                    # facing east
    assert ctx.pos[1] > 1.0                                           # now flying
    assert run(ex, ctx, 30) == DONE


def test_heading_rate_is_limited_and_kept_near_the_waypoint():
    ctx = SimCtx()
    ex, _ = build([wp(-10, 0)], ctx)                                  # due south: 180 deg turn
    ex.start(ctx)
    assert run(ex, ctx, 30) == DONE
    cmds = [(t, y) for t, y in ctx.yaw_log if math.isfinite(y)]
    for (t0, y0), (t1, y1) in zip(cmds, cmds[1:]):
        dy = abs(math.atan2(math.sin(y1 - y0), math.cos(y1 - y0)))
        assert dy <= math.radians(45) * (t1 - t0) + 1e-6             # never faster than yaw_rate
    assert abs(abs(math.degrees(cmds[-1][1])) - 180) < 5              # still facing south at arrival


def test_yaw_mode_hold_keeps_take_off_heading():
    ctx = SimCtx()
    ctx.cfg.yaw_mode = 'hold'
    ex, _ = build([wp(0, 10)], ctx)
    ex.start(ctx)
    assert run(ex, ctx, 30) == DONE
    assert all(math.isnan(y) for _, y in ctx.yaw_log)


def test_direct_nav_reaches_cruise_speeds_above_3_m_s_with_a_px4_like_controller():
    """PX4 position control flies ~MPC_XY_P x distance: a fixed 3 m carrot lead capped this at ~2.9 m/s."""
    nav = DirectNav(cruise_speed=5.0, xy_p=0.95)

    class Ctx:
        t, pos = 0.0, (0.0, 0.0, -5.0)

        def now(self):
            return self.t

        @property
        def position(self):
            return self.pos

    ctx = Ctx()
    goal = (200.0, 0.0, -5.0)
    nav.begin(ctx, goal)
    dt, v = 0.05, 0.0
    for _ in range(int(20 / dt)):
        sp = nav.setpoint(ctx, goal)
        v = min(12.0, 0.95 * (sp[0] - ctx.pos[0]))           # P controller, MPC_XY_VEL_MAX 12
        ctx.t += dt
        ctx.pos = (ctx.pos[0] + v * dt, 0.0, -5.0)
    assert 4.7 < v <= 5.05
