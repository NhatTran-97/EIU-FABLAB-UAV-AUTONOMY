"""mission_manager node tests: the real node with a simulated offboard_control / vehicle.

Callbacks are called directly and publishers are intercepted; nothing is spun. The test runs in an isolated
DDS domain on localhost so it can never talk to a real drone.
"""

import json
import math
import os
import sys

import pytest

os.environ['ROS_LOCALHOST_ONLY'] = '1'
os.environ['ROS_DOMAIN_ID'] = '199'
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'px4_offboard'))

rclpy = pytest.importorskip('rclpy')
pytest.importorskip('custom_msgs.msg')
pytest.importorskip('std_srvs.srv')

from geometry_msgs.msg import Point, PointStamped  # noqa: E402
from sensor_msgs.msg import NavSatFix, NavSatStatus  # noqa: E402
from std_msgs.msg import Bool, Float32, String  # noqa: E402

import mission_manager as mm  # noqa: E402

REF = (11.0, 106.0)
DEG_PER_M = 1.0 / 111195.0


@pytest.fixture(scope='module', autouse=True)
def ros():
    rclpy.init()
    yield
    rclpy.shutdown()


NODES = []


@pytest.fixture(autouse=True)
def destroy_nodes():
    """Each test starts with a clean ROS graph (no publishers left over from earlier tests)."""
    yield
    while NODES:
        NODES.pop().destroy_node()


class Sim:
    """Vehicle + offboard_control stand-in around a MissionManagerNode."""

    def __init__(self):
        self.node = mm.MissionManagerNode()
        NODES.append(self.node)
        self.t = 100.0
        self.node.now_sec = lambda: self.t
        self.pos = [0.0, 0.0, 0.0]
        self.target = None
        self.vel = None
        self.acks, self.texts, self.progress = [], [], []
        self.resume_requests = 0
        n = self.node
        n.target_position_pub.publish = lambda m: self._set_target(m)
        n.target_velocity_pub.publish = lambda m: self._set_vel(m)
        self.heading, self.yaw_cmd = 0.0, math.nan
        n.target_yaw_pub.publish = lambda m: setattr(self, 'yaw_cmd', m.data)
        n.command_ack_pub.publish = lambda m: self.acks.append(json.loads(m.data))
        n.status_text_pub.publish = lambda m: self.texts.append(json.loads(m.data)['text'])
        n.progress_pub.publish = lambda m: self.progress.append(json.loads(m.data))
        n.resume_pub.publish = lambda m: setattr(self, 'resume_requests', self.resume_requests + 1)
        n.start_pub.publish = lambda m: self._started()
        n.land_pub.publish = lambda m: setattr(self, 'landing', True)
        self.marker_at = None
        self.offboard = False
        self.cmd_id = 0
        self.landing = False
        ref = NavSatFix()
        ref.status.status = NavSatStatus.STATUS_FIX
        ref.latitude, ref.longitude = REF
        n.geo_reference_callback(ref)
        self.feed()

    def _set_target(self, m):
        self.target, self.vel = [m.x, m.y, m.z], None

    def _set_vel(self, m):
        self.vel = [m.x, m.y, m.z]

    def _started(self):
        self.node.flight_active_callback(Bool(data=True))

    def feed(self):
        n = self.node
        n.current_position_callback(Point(x=self.pos[0], y=self.pos[1], z=self.pos[2]))
        n.agl_callback(Float32(data=-self.pos[2]))
        n.heading_callback(Float32(data=self.heading))
        if self.marker_at is not None and -self.pos[2] < 6.0:
            dn, de = self.marker_at[0] - self.pos[0], self.marker_at[1] - self.pos[1]
            if math.hypot(dn, de) < 4.0:
                m = PointStamped()
                m.point.x, m.point.y = de, dn          # ENU offset
                n.marker_callback(m)

    def add_marker_publisher(self):
        """The pick capability check needs a publisher on the marker topic."""
        self.marker_pub = self.node.create_publisher(PointStamped, self.node.marker_topic, 10)

    def set_offboard(self, on):
        self.offboard = on
        self.node.offboard_active_callback(Bool(data=on))

    def command(self, cmd, **kw):
        self.cmd_id += 1
        self.node.gcs_command_callback(String(data=json.dumps({'id': self.cmd_id, 'cmd': cmd, **kw})))
        return self.acks[-1] if self.acks and self.acks[-1]['id'] == self.cmd_id else None

    def run(self, seconds, until=None, dt=0.05):
        for _ in range(int(seconds / dt)):
            self.t += dt
            if self.offboard and math.isfinite(self.yaw_cmd):
                err = math.atan2(math.sin(self.yaw_cmd - self.heading), math.cos(self.yaw_cmd - self.heading))
                self.heading += max(-1.6 * dt, min(1.6 * dt, err))
            if self.offboard and self.vel is not None:
                self.pos = [p + v * dt for p, v in zip(self.pos, self.vel)]
            elif self.offboard and self.target is not None:
                d = [a - b for a, b in zip(self.target, self.pos)]
                dist = math.sqrt(sum(v * v for v in d))
                if dist > 1e-9:
                    k = min(1.0, 3.0 * dt / dist)
                    self.pos = [p + k * v for p, v in zip(self.pos, d)]
            self.feed()
            self.node.timer_callback()
            if until and until():
                return True
        return False

    def state(self):
        return self.node.state.name

    def upload(self, items):
        self.node.gcs_mission_callback(String(data=json.dumps(items)))

    def takeoff(self, alt=5.0):
        self.run(0.1)                                   # WAIT_FOR_POSITION -> IDLE
        ack = self.command('takeoff', alt=alt)
        assert ack['result'] == 'accepted', ack
        self.run(0.2)
        self.set_offboard(True)
        assert self.run(20, until=lambda: self.state() == 'HOVER')


def wp(north, east, alt=5.0, command=16, params=None):
    return {'seq': 0, 'lat': REF[0] + north * DEG_PER_M,
            'lon': REF[1] + east * DEG_PER_M / math.cos(math.radians(REF[0])),
            'alt': alt, 'command': command, 'frame': 6, 'params': params or []}


def test_default_mission_flies_and_hovers_at_the_end():
    sim = Sim()
    sim.takeoff()
    sim.upload([wp(8, 0), wp(8, 8)])
    assert sim.command('mission')['result'] == 'accepted'
    assert sim.run(60, until=lambda: sim.state() == 'HOVER')
    assert math.hypot(sim.pos[0] - 8, sim.pos[1] - 8) < 1.0
    assert sim.progress[-1]['state'] == 'complete'
    assert any('Mission complete' in t for t in sim.texts)


def test_manual_pick_hands_over_to_pilot_and_continues():
    sim = Sim()
    sim.takeoff(8.0)
    sim.marker_at = (6.2, 0.3)
    sim.add_marker_publisher()
    sim.upload([wp(6, 0, alt=8.0, command=31012, params=[1, 3.0, 1.0]), wp(0, 0, alt=8.0)])
    assert sim.command('mission')['result'] == 'accepted'
    assert sim.run(120, until=lambda: sim.node.plan_runner and sim.node.plan_runner.current_step.name == 'Manual')
    assert abs(-sim.pos[2] - 1.0) < 0.3
    assert sim.progress[-1]['state'] == 'paused'
    assert any('take control' in t for t in sim.texts)
    # pilot takes over: planned, so no PAUSED state
    sim.set_offboard(False)
    sim.run(2)
    assert sim.state() == 'MISSION'
    # operator presses Continue while the pilot is still in Position mode
    assert sim.command('continue')['result'] == 'accepted'
    sim.run(0.2)
    assert sim.resume_requests == 1
    sim.set_offboard(True)
    assert sim.run(60, until=lambda: sim.state() == 'HOVER')
    assert math.hypot(sim.pos[0], sim.pos[1]) < 1.0                  # flew on to the second waypoint


def test_unplanned_override_pauses_and_switch_back_resumes():
    sim = Sim()
    sim.takeoff()
    sim.upload([wp(20, 0)])
    sim.command('mission')
    sim.run(2)
    sim.set_offboard(False)                                           # pilot moves the sticks
    sim.run(0.2)
    assert sim.state() == 'PAUSED'
    assert sim.progress[-1]['state'] == 'paused'
    sim.pos[1] = 5.0                                                  # pilot flew sideways
    sim.set_offboard(True)                                            # RC switch back to Offboard
    assert sim.state() == 'MISSION'
    assert sim.run(60, until=lambda: sim.state() == 'HOVER')
    assert math.hypot(sim.pos[0] - 20, sim.pos[1]) < 1.0


def test_continue_from_pause_requests_offboard():
    sim = Sim()
    sim.takeoff()
    sim.set_offboard(False)                                           # e.g. GCS Hold while hovering
    sim.run(0.2)
    assert sim.state() == 'PAUSED'
    assert sim.command('continue')['result'] == 'accepted'
    assert sim.resume_requests == 1
    sim.set_offboard(True)
    assert sim.state() == 'HOVER'


def test_avoidance_rejected_when_not_allowed_and_pick_needs_its_nodes():
    sim = Sim()
    sim.takeoff()
    sim.upload([wp(5, 0)])
    ack = sim.command('mission', options={'avoidance': True})
    assert ack['result'] == 'denied' and 'allow_avoidance' in ack['text']
    sim.upload([wp(5, 0, alt=5.0, command=31012, params=[0, 3.0, 1.0])])
    ack = sim.command('mission')
    assert ack['result'] == 'denied' and 'marker localizer' in ack['text']
    assert sim.state() == 'HOVER'


def test_land_aborts_a_running_mission():
    sim = Sim()
    sim.takeoff()
    sim.upload([wp(20, 0)])
    sim.command('mission')
    sim.run(1)
    assert sim.command('land')['result'] == 'accepted'
    assert sim.state() == 'LANDING' and sim.landing and sim.node.plan_runner is None


def test_nose_points_at_the_waypoint_and_hover_keeps_that_heading():
    sim = Sim()
    sim.takeoff()
    sim.upload([wp(0, 15)])                                           # due east
    sim.command('mission')
    assert sim.run(60, until=lambda: sim.state() == 'HOVER')
    assert abs(math.degrees(sim.heading) - 90) < 5
    sim.run(3)
    assert abs(math.degrees(sim.yaw_cmd) - 90) < 5                    # hover does not turn back north


def test_mission_speed_from_the_ground_station_is_used_and_limited():
    sim = Sim()
    sim.takeoff()
    sim.upload([wp(30, 0)])
    ack = sim.command('mission', options={'speed': 1.0})
    assert ack['result'] == 'accepted' and '1.0 m/s' in ack['text']
    sim.run(5.0)
    assert 2.5 < sim.pos[0] < 6.5                                     # ~1 m/s (after turning / settling)
    sim.command('land')
    sim2 = Sim()
    sim2.takeoff()
    sim2.upload([wp(10, 0)])
    ack = sim2.command('mission', options={'speed': 9.0})
    assert '5.0 m/s (limited)' in ack['text']


def test_arrival_tolerances_from_the_ground_station():
    sim = Sim()
    sim.takeoff()
    sim.upload([wp(10, 0), wp(10, 10)])
    ack = sim.command('mission', options={'xy_tolerance': 3.0, 'z_tolerance': 9.0})
    assert ack['result'] == 'accepted'
    assert 'tolerance xy 3.00 / z 3.00 m (clamped)' in ack['text']     # z limited to 3 m
    assert sim.node.ctx.cfg.xy_tolerance == 3.0
    # waypoint 1 counts as reached ~3 m before the point
    assert sim.run(30, until=lambda: sim.node.plan_runner and sim.node.plan_runner.current_item == 2)
    assert 6.0 < sim.pos[0] < 8.5
    assert sim.node.step_cfg.xy_tolerance == 0.75                     # node default unchanged


def test_live_settings_change_a_running_mission_only():
    sim = Sim()
    sim.takeoff()
    ack = sim.command('settings', options={'speed': 1.0})
    assert ack['result'] == 'denied' and 'no mission running' in ack['text']
    sim.upload([wp(40, 0)])
    sim.command('mission', options={'speed': 3.0})
    sim.run(2.0)
    ack = sim.command('settings', options={'speed': 1.0, 'xy_tolerance': 1.5})
    assert ack['result'] == 'accepted'
    assert sim.node.ctx.nav.cruise_speed == 1.0
    assert sim.node.ctx.cfg.xy_tolerance == 1.5 and sim.node.ctx.cfg.z_tolerance == sim.node.step_cfg.z_tolerance


def avoidance_sim():
    """Mission node with avoidance allowed and a stand-in avoidance node that, like the real one, reports
    STALE until it has received a goal and then CLEAR with a carrot towards that goal."""
    from custom_msgs.msg import AvoidanceStatus
    sim = Sim()
    n = sim.node
    n.allow_avoidance = True
    sim.enable_sub = n.create_subscription(Bool, '/avoidance/enable', lambda m: None, 10)
    sim.goal = None
    n.goal_pub.publish = lambda m: setattr(sim, 'goal', (m.x, m.y, m.z))
    sim.carrots = 0

    def scan():
        st = AvoidanceStatus()
        st.scan_period = 0.1
        if sim.goal is None:
            st.state = AvoidanceStatus.STALE
        else:
            st.state = AvoidanceStatus.CLEAR
            dn, de = sim.goal[0] - sim.pos[0], sim.goal[1] - sim.pos[1]
            d = max(math.hypot(dn, de), 1e-9)
            k = min(1.0, 2.0 / d)
            st.safe_setpoint.x, st.safe_setpoint.y = sim.pos[0] + k * dn, sim.pos[1] + k * de
            sim.carrots += 1
        n.avoidance_status_callback(st)
    sim.scan = scan
    return sim


def test_avoidance_mission_starts_once_the_node_has_the_first_goal():
    sim = avoidance_sim()
    sim.takeoff()
    sim.upload([wp(10, 0)])
    sim.command('mission', options={'avoidance': True})
    assert sim.state() == 'STARTING'
    for _ in range(10):                        # 10 Hz scans while STARTING
        sim.run(0.1)
        sim.scan()
        if sim.state() == 'MISSION':
            break
    assert sim.state() == 'MISSION' and sim.acks[-1]['result'] == 'accepted'
    assert abs(sim.goal[0] - 10.0) < 0.1


def test_avoidance_setpoint_ignored_while_turning_in_place():
    sim = avoidance_sim()
    sim.takeoff()
    sim.upload([wp(0, 10)])                    # due east: 90 deg turn first
    sim.command('mission', options={'avoidance': True})
    sim.run(0.1)
    sim.scan()
    sim.run(0.05)
    assert sim.state() == 'MISSION'
    step = sim.node.plan_runner.current_step
    assert step.turning
    start = list(sim.pos)
    for _ in range(5):
        sim.run(0.1)
        sim.scan()
    assert step.turning and math.hypot(sim.pos[0] - start[0], sim.pos[1] - start[1]) < 0.05


def test_hover_keeps_distance_from_a_close_obstacle():
    from custom_msgs.msg import AvoidanceStatus
    sim = avoidance_sim()
    sim.node.hover_keep_distance = 2.0
    sim.takeoff()
    start = list(sim.pos)

    def obstacle(dist):
        st = AvoidanceStatus()
        st.state, st.scan_period = AvoidanceStatus.CLEAR, 0.1
        st.min_distance, st.min_distance_bearing = dist, 0.0      # obstacle due north
        sim.node.avoidance_status_callback(st)

    for _ in range(20):                       # person 1 m north of the drone for 2 s
        obstacle(1.0 + (start[0] - sim.pos[0]))
        sim.run(0.1)
    assert sim.pos[0] < start[0] - 0.5 and abs(sim.pos[1] - start[1]) < 0.05    # moved south
    assert any('moving away' in t for t in sim.texts)
    for _ in range(100):                      # obstacle follows: drift stops at hover_max_drift
        obstacle(1.0)
        sim.run(0.1)
    assert abs(math.hypot(sim.pos[0] - start[0], sim.pos[1] - start[1]) - 3.0) < 0.2


def test_hover_keep_distance_is_off_by_default():
    from custom_msgs.msg import AvoidanceStatus
    sim = avoidance_sim()
    sim.takeoff()
    start = list(sim.pos)
    for _ in range(20):
        st = AvoidanceStatus()
        st.state, st.scan_period, st.min_distance = AvoidanceStatus.CLEAR, 0.1, 0.8
        sim.node.avoidance_status_callback(st)
        sim.run(0.1)
    assert math.hypot(sim.pos[0] - start[0], sim.pos[1] - start[1]) < 0.05
