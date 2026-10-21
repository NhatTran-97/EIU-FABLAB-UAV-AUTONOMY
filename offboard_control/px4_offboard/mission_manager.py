#!/usr/bin/env python3
"""Mission manager: turns ground station commands into targets for offboard_control via /mission/*.

This node makes the flight decisions. It never takes off on its own: it waits for commands from the GCS
(radio 433 -> drone_telemetry) and then feeds targets to offboard_control. The mission itself is compiled
into a plan of steps and run by the mission pipeline (package mission_pipeline, docs/MISSION_PIPELINE.md).

GCS interface (via drone_telemetry):
    /gcs/command      JSON command: {"id": n, "cmd": "takeoff", "alt": 5.0}, {"cmd": "land"},
                      {"cmd": "mission", "options": {"avoidance": false, "speed": 2.0, "xy_tolerance": 0.75,
                      "z_tolerance": 0.5}}, {"cmd": "continue"}, or {"cmd": "settings", "options": {"speed",
                      "xy_tolerance", "z_tolerance"}} to change a running mission (0 = unchanged)
    /gcs/command_ack  JSON result: {"id": n, "cmd": ..., "result": "accepted" | "denied", "text": ...}
    /gcs/mission      JSON item list [{"seq", "lat", "lon", "alt", "command", "frame", "params"}], alt in metres
                      above Home. Latched; the list is only stored and flown on the "mission" command.
    /mission/status_text  JSON {"severity", "text"}: short messages for the ground station
    /mission/progress     JSON {"seq", "total", "state", "step"}: mission progress for the ground station

Core states:
    WAIT_FOR_POSITION  wait for the first position from offboard_control, then IDLE.
    IDLE               on the ground, waiting for Take-off.
    TAKEOFF            climb vertically to the requested altitude, then HOVER.
    HOVER              hold position, waiting for Land or Start mission.
    STARTING           Start mission accepted, waiting for the selected options to be ready (e.g. avoidance).
    MISSION            the mission plan is running; at the end, HOVER at the last target.
    PAUSED             PX4 left OFFBOARD outside a planned hand-over (pilot or GCS took over); Continue or an
                       RC switch back to Offboard resumes where the drone is.
    LANDING            landing requested; back to IDLE when offboard_control reports touchdown.

Waypoint altitude is relative to the ground at take-off (Home = arming point): z = z_ground - alt.
"""

import dataclasses
import json
import math
from enum import Enum

import rclpy
from geometry_msgs.msg import Point, PointStamped, Vector3
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import NavSatFix, NavSatStatus
from std_msgs.msg import Bool, Float32, String
from std_srvs.srv import Trigger
from custom_msgs.msg import AvoidanceStatus

from mission_pipeline.compiler import CompileConfig, MissionOptions, compile_mission
from mission_pipeline.executor import DONE, MissionExecutor
from mission_pipeline.navigation import CLEAR, STATUS_AVOIDING, AvoidanceConfig, AvoidanceNav, DirectNav, \
    status_timeout
from mission_pipeline.steps import RUNNING, Failed, Navigate, StepConfig

SEV_WARNING, SEV_INFO = 4, 6


class MissionState(Enum):
    WAIT_FOR_POSITION = 0
    TAKEOFF = 1
    MISSION = 2
    IDLE = 5        # position known, on the ground, waiting for a GCS command
    HOVER = 6       # at altitude, holding position, waiting for a command
    LANDING = 7     # /mission/land sent, waiting for touchdown from offboard_control
    STARTING = 8    # Start mission accepted, checking that the selected options are ready
    PAUSED = 9      # PX4 left OFFBOARD outside a planned hand-over


FLYING_STATES = (MissionState.TAKEOFF, MissionState.HOVER, MissionState.STARTING, MissionState.MISSION,
                 MissionState.PAUSED)


class MissionContext:
    """The step API of mission_pipeline (see mission_pipeline/steps.py), backed by the node."""

    def __init__(self, node):
        self.node = node
        self.cfg = node.step_cfg
        self.nav = DirectNav()
        self.continue_pressed = False
        self.last_target = None
        self.last_yaw = math.nan

    def now(self):
        return self.node.now_sec()

    @property
    def position(self):
        p = self.node.current_position
        return (p.x, p.y, p.z) if self.node.have_position else None

    @property
    def agl(self):
        return self.node.agl_value()

    @property
    def heading(self):
        return self.node.heading_value()

    @property
    def offboard_active(self):
        return self.node.offboard_active

    def marker(self):
        return self.node.marker_offset()

    def set_target(self, x, y, z):
        self.last_target = (x, y, z)
        self.node.publish_target_position(Point(x=float(x), y=float(y), z=float(z)))

    def set_velocity(self, vn, ve, vd):
        self.node.target_velocity_pub.publish(Vector3(x=float(vn), y=float(ve), z=float(vd)))

    def set_yaw(self, yaw):
        if math.isfinite(yaw):
            self.last_yaw = yaw
        self.node.publish_target_yaw(yaw)

    def payload_request(self):
        return self.node.payload_request()

    def payload_result(self, token):
        return self.node.payload_result(token)

    def take_continue(self):
        pressed, self.continue_pressed = self.continue_pressed, False
        return pressed

    def request_resume(self):
        self.node.resume_pub.publish(Bool(data=True))

    def say(self, text, warn=False):
        self.node.say(text, warn)

    def log(self, text, warn=False):
        self.node.log(text, warn)

    def avoidance_status(self):
        return self.node.avoidance_status, self.node.avoidance_status_time

    def publish_goal(self, goal):
        self.node.goal_pub.publish(Point(x=float(goal[0]), y=float(goal[1]), z=float(goal[2])))


class MissionManagerNode(Node):
    """Flight state machine: accepts GCS commands, runs the mission plan and reports results."""

    def __init__(self) -> None:
        super().__init__('mission_manager')

        self.declare_parameter('takeoff_altitude', -5.0)   # NED (negative = up); used when the command has no altitude
        self.declare_parameter('max_takeoff_altitude', 15.0)   # m; higher requests are rejected
        # After /mission/start, offboard_control must report flight_active within this time; otherwise
        # (node not running or no position) Take-off is cancelled and the state returns to IDLE.
        self.declare_parameter('start_timeout', 3.0)
        # Reject Take-off if /mission/current_position is older than this (no valid position from
        # offboard_control / PX4).
        self.declare_parameter('position_timeout', 1.0)
        # Defaults for reaching a target; the ground station can override them per mission (Start mission).
        self.declare_parameter('xy_tolerance', 0.75)
        self.declare_parameter('z_tolerance', 0.5)
        # Hold times are in seconds, not timer ticks, so changing control_rate_hz does not change behaviour.
        self.declare_parameter('goal_hold_time', 0.5)
        self.declare_parameter('control_rate_hz', 20.0)
        self.declare_parameter('mission_yaw', math.nan)   # NaN keeps the heading at take-off
        # Reject the whole mission if any GCS waypoint is farther than this (horizontally) from the current
        # position: most likely a misplaced point or a wrong origin.
        self.declare_parameter('max_mission_distance', 300.0)
        # Horizontal speed between waypoints without avoidance (m/s); 0 = PX4 limits only (MPC_XY_VEL_MAX).
        self.declare_parameter('cruise_speed', 2.0)
        # Upper limit for the speed chosen on the ground station (m/s); faster requests are reduced to it.
        self.declare_parameter('max_cruise_speed', 5.0)
        # Heading while flying to a waypoint: 'waypoint' (nose towards the waypoint) or 'hold' (take-off heading).
        # A finite mission_yaw overrides both with a fixed heading.
        self.declare_parameter('yaw_mode', 'waypoint')
        self.declare_parameter('yaw_rate_deg', 45.0)         # deg/s, maximum commanded turn rate
        self.declare_parameter('turn_first_deg', 10.0)       # heading error at a leg start that makes the drone turn first
        self.declare_parameter('yaw_freeze_radius', 1.5)     # m; heading kept this close to the waypoint

        # Obstacle avoidance (obstacle_avoidance package), selected per mission from the ground station.
        # allow_avoidance is a safety switch: false rejects every mission that asks for avoidance.
        self.declare_parameter('allow_avoidance', False)
        self.declare_parameter('avoidance_ready_timeout', 1.5)  # s to get a valid status after enabling
        # The status timeout is derived from the scan period measured by the avoidance node
        # (AvoidanceStatus.scan_period): timeout = periods * scan_period, capped.
        self.declare_parameter('avoidance_timeout_periods', 3.0)
        self.declare_parameter('avoidance_max_timeout', 1.0)  # s; also used until scan_period is known
        self.declare_parameter('avoidance_clear_time', 0.5)   # s of clear path before AVOIDING -> NOMINAL
        self.declare_parameter('blocked_timeout', 5.0)        # s in HOLD before blocked_action
        self.declare_parameter('blocked_action', 'hold')      # 'hold' (pilot takes over) or 'land'
        # VFH+ is reactive: in a U-shaped trap it can oscillate without ever reporting BLOCKED. If the goal
        # distance does not shrink by stuck_min_progress within stuck_timeout, treat it as blocked.
        self.declare_parameter('stuck_timeout', 15.0)
        self.declare_parameter('stuck_min_progress', 1.0)

        # Pick tasks at waypoints: marker from the ArUco localizer, hook through the payload node.
        self.declare_parameter('marker_topic', '/marker_localizer_node/marker_rel')  # PointStamped, ENU offset
        self.declare_parameter('marker_timeout', 0.5)          # s; older detections count as not seen
        self.declare_parameter('payload_service', '/payload/hook')   # std_srvs/Trigger
        self.declare_parameter('pick_search_agl', 3.0)         # m, default search height above ground
        self.declare_parameter('pick_hook_agl', 1.0)           # m, default hook height above ground
        self.declare_parameter('agl_timeout', 0.5)             # s; older rangefinder values count as invalid

        g = lambda name: self.get_parameter(name).value  # noqa: E731
        self.takeoff_altitude = float(g('takeoff_altitude'))
        self.max_takeoff_altitude = float(g('max_takeoff_altitude'))
        self.start_timeout = float(g('start_timeout'))
        self.position_timeout = float(g('position_timeout'))
        self.control_rate_hz = float(g('control_rate_hz'))
        if self.control_rate_hz <= 0.0:
            raise ValueError('Parameter "control_rate_hz" must be > 0.0')
        self.mission_yaw = float(g('mission_yaw'))
        self.cruise_speed = float(g('cruise_speed'))
        self.max_cruise_speed = float(g('max_cruise_speed'))
        self.allow_avoidance = bool(g('allow_avoidance'))
        self.avoidance_ready_timeout = float(g('avoidance_ready_timeout'))
        self.avoidance_cfg = AvoidanceConfig(
            timeout_periods=float(g('avoidance_timeout_periods')), max_timeout=float(g('avoidance_max_timeout')),
            clear_time=float(g('avoidance_clear_time')), blocked_timeout=float(g('blocked_timeout')),
            blocked_action=str(g('blocked_action')), stuck_timeout=float(g('stuck_timeout')),
            stuck_min_progress=float(g('stuck_min_progress')))
        if self.avoidance_cfg.blocked_action not in ('hold', 'land'):
            raise ValueError('Parameter "blocked_action" must be "hold" or "land"')
        self.marker_timeout = float(g('marker_timeout'))
        self.agl_timeout = float(g('agl_timeout'))
        yaw_mode = str(g('yaw_mode'))
        if yaw_mode not in ('waypoint', 'hold'):
            raise ValueError('Parameter "yaw_mode" must be "waypoint" or "hold"')
        self.step_cfg = StepConfig(xy_tolerance=float(g('xy_tolerance')), z_tolerance=float(g('z_tolerance')),
                                   goal_hold_time=float(g('goal_hold_time')), yaw_mode=yaw_mode,
                                   yaw_rate=math.radians(float(g('yaw_rate_deg'))),
                                   turn_first=math.radians(float(g('turn_first_deg'))),
                                   yaw_freeze_radius=float(g('yaw_freeze_radius')))
        self.compile_cfg = CompileConfig(max_altitude=self.max_takeoff_altitude,
                                         max_distance=float(g('max_mission_distance')),
                                         default_search_agl=float(g('pick_search_agl')),
                                         default_hook_agl=float(g('pick_hook_agl')))

        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)

        # To offboard_control
        self.target_position_pub = self.create_publisher(Point, '/mission/target_position', 10)
        self.target_velocity_pub = self.create_publisher(Vector3, '/mission/target_velocity', 10)
        self.target_yaw_pub = self.create_publisher(Float32, '/mission/target_yaw', 10)
        self.start_pub = self.create_publisher(Bool, '/mission/start', 10)
        self.land_pub = self.create_publisher(Bool, '/mission/land', 10)
        self.resume_pub = self.create_publisher(Bool, '/mission/resume', 10)
        # From offboard_control
        self.create_subscription(Point, '/mission/current_position', self.current_position_callback, 10)
        self.create_subscription(Float32, '/mission/agl', self.agl_callback, 10)
        self.create_subscription(Float32, '/mission/heading', self.heading_callback, 10)
        self.create_subscription(Bool, '/mission/flight_active', self.flight_active_callback, latched)
        self.create_subscription(Bool, '/mission/offboard_active', self.offboard_active_callback, latched)
        self.create_subscription(NavSatFix, '/mission/geo_reference', self.geo_reference_callback, latched)

        # Ground station (via drone_telemetry)
        self.create_subscription(String, '/gcs/command', self.gcs_command_callback, 10)
        self.command_ack_pub = self.create_publisher(String, '/gcs/command_ack', 10)
        self.create_subscription(String, '/gcs/mission', self.gcs_mission_callback, latched)
        self.status_text_pub = self.create_publisher(String, '/mission/status_text', 10)
        self.progress_pub = self.create_publisher(String, '/mission/progress', latched)

        # Monitoring
        self.state_pub = self.create_publisher(String, '/mission/state', 10)
        self.nav_mode_pub = self.create_publisher(String, '/mission/nav_mode', 10)

        # Obstacle avoidance: the node only proposes a setpoint; this node stays the single writer of
        # /mission/target_position. The real goal goes on /mission/goal.
        self.goal_pub = self.create_publisher(Point, '/mission/goal', 10)
        self.avoidance_enable_pub = self.create_publisher(Bool, '/avoidance/enable', 10)
        self.create_subscription(AvoidanceStatus, '/avoidance/status', self.avoidance_status_callback, 10)

        # Pick tasks
        self.marker_topic = str(g('marker_topic'))
        self.create_subscription(PointStamped, self.marker_topic, self.marker_callback, 10)
        self.payload_client = self.create_client(Trigger, str(g('payload_service')))

        # State
        self.state = MissionState.WAIT_FOR_POSITION
        self.current_position = Point()
        self.have_position = False
        self.position_time = None
        self.agl = math.nan
        self.agl_time = None
        self.heading = math.nan
        self.heading_time = None
        self.hover_yaw = math.nan        # NaN = take-off heading; set to the current heading after a mission
        self.flight_active = False
        self.offboard_active = False
        self.offboard_seen = False       # OFFBOARD was reached during the current flight
        self.takeoff_target = None
        self.hover_target = None
        self.ground_z = None
        self.start_sent = False
        self.start_sent_time = None
        self.goal_hold_start = None
        self.gcs_mission = []
        self.geo_reference = None
        self.last_command_id = None
        self.last_ack = None
        self.pending_ack = None          # (id, cmd) answered when STARTING ends
        self.avoidance_status = None
        self.avoidance_status_time = None
        self.marker = None               # (north, east, receive time)
        self.ctx = MissionContext(self)
        self.plan_runner = None
        self.options = MissionOptions()
        self.starting_since = None
        self.resume_to = None            # state to return to from PAUSED
        self.avoidance_was_on = False
        self.mission_complete = False    # last mission finished; reported until the next mission or landing
        self.last_progress = None
        self.last_progress_time = 0.0
        self.last_status_log_time = 0.0

        self.timer = self.create_timer(1.0 / self.control_rate_hz, self.timer_callback)
        self.get_logger().info('Mission manager node started')
        self.get_logger().info(
            f'takeoff_altitude={self.takeoff_altitude:.2f}, max_takeoff_altitude={self.max_takeoff_altitude:.1f}, '
            f'cruise_speed={self.cruise_speed:.1f} m/s, allow_avoidance={self.allow_avoidance}, '
            f'control_rate_hz={self.control_rate_hz:.1f}')

    # ---- Main loop -----------------------------------------------------------------------------

    def timer_callback(self):
        """Main loop at control_rate_hz: run the current state, then publish state, avoidance and progress."""
        self.check_override()
        if self.state == MissionState.WAIT_FOR_POSITION:
            self.handle_wait_for_position()
        elif self.state == MissionState.IDLE:
            self.maybe_log_status('State=IDLE | waiting for GCS command', 5.0)
        elif self.state == MissionState.TAKEOFF:
            self.handle_takeoff()
        elif self.state == MissionState.HOVER:
            self.handle_hover()
        elif self.state == MissionState.STARTING:
            self.handle_starting()
        elif self.state == MissionState.MISSION:
            self.handle_mission()
        elif self.state == MissionState.PAUSED:
            self.maybe_log_status('State=PAUSED | pilot / GCS in control, waiting for Offboard or Continue', 5.0)
        elif self.state == MissionState.LANDING:
            self.maybe_log_status('State=LANDING | waiting for touchdown', 2.0)

        self.state_pub.publish(String(data=self.state.name))
        self.nav_mode_pub.publish(String(data=self.ctx.nav.mode))
        self.publish_avoidance_enable()
        self.publish_progress()

    def check_override(self):
        """Pause when PX4 has left OFFBOARD during a flight, unless a planned hand-over is running."""
        if self.state not in FLYING_STATES or self.state == MissionState.PAUSED:
            return
        if not (self.flight_active and self.offboard_seen) or self.offboard_active:
            return
        step = self.plan_runner.current_step if self.plan_runner and self.state == MissionState.MISSION else None
        if step is not None and step.expects_manual:
            return
        self.resume_to = MissionState.MISSION if self.state == MissionState.MISSION else MissionState.HOVER
        if self.state == MissionState.MISSION:
            self.plan_runner.pause(self.ctx)
        if self.state == MissionState.STARTING:
            self.finish_starting(False, 'pilot took control')
            self.plan_runner = None
        self.transition_to(MissionState.PAUSED)
        self.say('Paused: pilot in control. Continue to resume', warn=True)

    def resume_from_pause(self):
        """OFFBOARD is back: continue the mission step, or hover where the pilot left the drone."""
        pos = self.ctx.position
        if self.resume_to == MissionState.MISSION and self.plan_runner is not None:
            self.plan_runner.resume(self.ctx)
            self.transition_to(MissionState.MISSION)
        else:
            self.hover_target = Point(x=pos[0], y=pos[1], z=pos[2]) if pos else self.hover_target
            self.transition_to(MissionState.HOVER)
        self.say('Resumed')

    # ---- State handlers ------------------------------------------------------------------------

    def handle_wait_for_position(self) -> None:
        """Wait for the first position, then go to IDLE to wait for Take-off."""
        self.maybe_log_status('State=WAIT_FOR_POSITION')
        if self.have_position:
            self.transition_to(MissionState.IDLE)

    def handle_takeoff(self) -> None:
        """Take-off: send /mission/start once, then hold the target until the altitude is reached.

        If offboard_control does not report flight_active within start_timeout, return to IDLE. Once the
        target is held for goal_hold_time, switch to HOVER and wait; waypoints are not flown automatically.
        """
        target = self.takeoff_target
        self.publish_target_position(target)
        self.publish_target_yaw(math.nan)     # NaN: offboard_control keeps the take-off heading
        if not self.start_sent:
            self.start_pub.publish(Bool(data=True))
            self.start_sent = True
            self.start_sent_time = self.now_sec()
            self.get_logger().info('Takeoff start request sent')
        elif not self.flight_active and self.now_sec() - self.start_sent_time > self.start_timeout:
            self.get_logger().error(f'offboard_control did not start within {self.start_timeout:.1f} s '
                                    '(node running? local position?) -> back to IDLE')
            self.transition_to(MissionState.IDLE)
            return
        self.maybe_log_status(
            f'State=TAKEOFF | z={self.current_position.z:.2f} target_z={target.z:.2f} '
            f'| xy_error={self.xy_distance(target):.2f}, held={self.held_time():.2f}s')
        if self.update_goal_hold(target):
            self.get_logger().info('Takeoff complete. Hovering, waiting for GCS command.')
            self.hover_target = target
            self.transition_to(MissionState.HOVER)

    def handle_hover(self) -> None:
        """Hold the HOVER point (take-off point, last mission target or where a pause ended)."""
        self.publish_target_position(self.hover_target)
        self.publish_target_yaw(self.hover_yaw)
        self.maybe_log_status(
            f'State=HOVER | current=({self.current_position.x:.2f}, {self.current_position.y:.2f}, '
            f'{self.current_position.z:.2f}) | target=({self.hover_target.x:.2f}, {self.hover_target.y:.2f}, '
            f'{self.hover_target.z:.2f})', 2.0)

    def handle_starting(self) -> None:
        """Hover while the selected options get ready; then start the plan or reject the command."""
        self.handle_hover()
        status, received = self.avoidance_status, self.avoidance_status_time
        now = self.now_sec()
        fresh = (status is not None and received is not None and received >= self.starting_since
                 and now - received <= status_timeout(self.avoidance_cfg, status))
        if fresh and status.state in (CLEAR, STATUS_AVOIDING):
            self.finish_starting(True, f'mission started: {self.plan_runner.total} waypoints, avoidance on')
            self.plan_runner.start(self.ctx)
            self.transition_to(MissionState.MISSION)
        elif now - self.starting_since > self.avoidance_ready_timeout:
            self.finish_starting(False, 'avoidance not ready (no valid /avoidance/status)')
            self.plan_runner = None
            self.ctx.nav = DirectNav()
            self.transition_to(MissionState.HOVER)

    def handle_mission(self) -> None:
        """Run the plan; on completion hover at the last target, on failure hover here (or land)."""
        result = self.plan_runner.tick(self.ctx)
        if result == RUNNING:
            step = self.plan_runner.current_step
            self.maybe_log_status(
                f'State=MISSION | item {self.plan_runner.current_item}/{self.plan_runner.total} '
                f'{step.name if step else ""} | nav={self.ctx.nav.mode} '
                f'| pos=({self.current_position.x:.2f}, {self.current_position.y:.2f}, {self.current_position.z:.2f})'
                f' agl={self.agl_value():.2f}', 2.0)
            return
        if result == DONE:
            self.get_logger().info('Mission complete. Hovering at the last target.')
            self.say('Mission complete: hovering')
            self.mission_complete = True
            self.hover_at_last_target()
            return
        # Failed
        self.get_logger().error(f'Mission stopped at item {self.plan_runner.current_item}: {result.reason}')
        self.say(f'Mission stopped: {result.reason}', warn=True)
        self.plan_runner.abort(self.ctx)
        if result.land:
            self.start_landing()
            return
        pos = self.ctx.position
        self.hover_target = Point(x=pos[0], y=pos[1], z=pos[2]) if pos else self.hover_target
        self.transition_to(MissionState.HOVER)

    def hover_at_last_target(self):
        t = self.ctx.last_target or self.ctx.position
        if t is not None:
            self.hover_target = Point(x=float(t[0]), y=float(t[1]), z=float(t[2]))
        self.plan_runner = None
        self.transition_to(MissionState.HOVER)

    # ---- Ground station commands ---------------------------------------------------------------

    def gcs_command_callback(self, msg: String) -> None:
        """Execute a JSON command from the GCS and reply on /gcs/command_ack.

        A repeated command id (the GCS resends when the ACK was lost on the radio) only resends the
        previous ACK; the command is not executed again. Start mission with options may answer later,
        once the options are ready (STARTING).
        """
        try:
            req = json.loads(msg.data)
            cmd, cid = str(req['cmd']), req.get('id')
        except (ValueError, KeyError, TypeError) as e:
            self.get_logger().error(f'invalid /gcs/command: {msg.data!r} ({e})')
            return
        if cid is not None and cid == self.last_command_id:
            if self.last_ack is not None and self.pending_ack is None:
                self.command_ack_pub.publish(self.last_ack)
            return
        self.last_command_id = cid
        if cmd == 'takeoff':
            ok, text = self.start_takeoff(req.get('alt'))
        elif cmd == 'land':
            ok, text = self.start_landing()
        elif cmd == 'mission':
            ok, text = self.start_mission(MissionOptions.from_dict(req.get('options')))
            if ok is None:                    # STARTING: answered by finish_starting()
                self.pending_ack = (cid, cmd)
                return
        elif cmd == 'continue':
            ok, text = self.continue_mission()
        elif cmd == 'settings':
            ok, text = self.apply_settings(MissionOptions.from_dict(req.get('options')))
        else:
            ok, text = False, f'unknown command {cmd}'
        self.send_ack(cid, cmd, ok, text)

    def send_ack(self, cid, cmd, ok, text):
        # Separate log calls: rclpy binds one severity to each call site and raises ValueError if it changes.
        if ok:
            self.get_logger().info(f'GCS {cmd}: {text}')
        else:
            self.get_logger().warning(f'GCS {cmd}: {text}')
        ack = String(data=json.dumps({'id': cid, 'cmd': cmd, 'result': 'accepted' if ok else 'denied',
                                      'text': text}))
        self.last_ack = ack
        self.command_ack_pub.publish(ack)

    def start_takeoff(self, alt):
        """Validate and start a vertical take-off to alt metres at the current position."""
        if self.state == MissionState.WAIT_FOR_POSITION:
            return False, 'no local position yet'
        if self.state != MissionState.IDLE or self.flight_active:
            return False, f'not on ground (state {self.state.name})'
        if self.position_time is None or self.now_sec() - self.position_time > self.position_timeout:
            return False, 'no valid local position (EKF xy/z invalid or not publishing)'
        try:
            alt = float(alt)
        except (TypeError, ValueError):
            alt = math.nan
        if not math.isfinite(alt) or alt <= 0.0:
            alt = abs(self.takeoff_altitude)
        if alt > self.max_takeoff_altitude:
            return False, f'altitude {alt:.1f} m > max {self.max_takeoff_altitude:.1f} m'
        self.takeoff_target = Point(x=self.current_position.x, y=self.current_position.y,
                                    z=self.current_position.z - alt)     # straight up (NED: negative z = up)
        self.ground_z = self.current_position.z       # ground level at take-off, reference for waypoints
        self.start_sent = False
        self.offboard_seen = False
        self.transition_to(MissionState.TAKEOFF)
        return True, f'take-off to {alt:.1f} m'

    def start_mission(self, options):
        """Compile the uploaded mission, check the required capabilities, then run it.

        Returns (True, text), (False, reason), or (None, '') when the answer waits for STARTING.
        """
        if self.state != MissionState.HOVER:
            return False, f'take off and hover first (state {self.state.name})'
        plan, needs, why = compile_mission(self.gcs_mission, self.geo_reference, self.ground_z,
                                           self.ctx.position, options, self.compile_cfg, self.mission_yaw)
        if plan is None:
            return False, why
        why = self.check_capabilities(needs)
        if why:
            return False, why
        self.options = options
        self.mission_complete = False
        self.ctx.cfg, tol_text = self.mission_step_config(options)
        self.plan_runner = MissionExecutor(plan, len(self.gcs_mission))
        if options.avoidance:
            self.ctx.nav = AvoidanceNav(self.avoidance_cfg)
            self.starting_since = self.now_sec()
            self.transition_to(MissionState.STARTING)
            return None, ''
        speed = options.speed or self.cruise_speed
        limited = speed > self.max_cruise_speed
        speed = min(speed, self.max_cruise_speed)
        self.ctx.nav = DirectNav(speed)
        self.plan_runner.start(self.ctx)
        self.transition_to(MissionState.MISSION)
        return True, (f'mission started: {len(self.gcs_mission)} waypoints at {speed:.1f} m/s'
                      + (' (limited)' if limited else '') + tol_text)

    # Ranges accepted for the per-mission arrival tolerances (m); values outside are clamped.
    XY_TOLERANCE_RANGE = (0.2, 5.0)
    Z_TOLERANCE_RANGE = (0.1, 3.0)

    def apply_settings(self, options):
        """Live update of the running mission: speed (Direct navigation) and arrival tolerances. Values of 0
        are left unchanged. Without a running mission there is nothing to change: the next Start mission
        carries its own settings."""
        if self.plan_runner is None or self.state not in (MissionState.STARTING, MissionState.MISSION,
                                                          MissionState.PAUSED):
            return False, 'no mission running (settings are sent with Start mission)'
        parts = []
        if options.speed:
            speed = min(options.speed, self.max_cruise_speed)
            if isinstance(self.ctx.nav, DirectNav):
                self.ctx.nav.cruise_speed = speed
                parts.append(f'{speed:.1f} m/s' + (' (limited)' if speed < options.speed else ''))
            else:
                parts.append('speed set by avoidance')
        if options.xy_tolerance or options.z_tolerance:
            self.ctx.cfg, tol_text = self.mission_step_config(options, base=self.ctx.cfg)
            parts.append(tol_text.lstrip(', '))
        if not parts:
            return False, 'no settings given'
        text = 'settings: ' + ', '.join(parts)
        self.say(text)
        return True, text

    def mission_step_config(self, options, base=None):
        """Step configuration for a mission: `base` (default: the node defaults) with the tolerances from the
        ground station, clamped to XY_TOLERANCE_RANGE / Z_TOLERANCE_RANGE. Returns (config, text for the ACK)."""
        cfg = base or self.step_cfg
        if not (options.xy_tolerance or options.z_tolerance):
            return cfg, ''
        xy = min(max(options.xy_tolerance or cfg.xy_tolerance, self.XY_TOLERANCE_RANGE[0]), self.XY_TOLERANCE_RANGE[1])
        z = min(max(options.z_tolerance or cfg.z_tolerance, self.Z_TOLERANCE_RANGE[0]), self.Z_TOLERANCE_RANGE[1])
        clamped = ((options.xy_tolerance and xy != options.xy_tolerance)
                   or (options.z_tolerance and z != options.z_tolerance))
        return (dataclasses.replace(cfg, xy_tolerance=xy, z_tolerance=z),
                f', tolerance xy {xy:.2f} / z {z:.2f} m' + (' (clamped)' if clamped else ''))

    def check_capabilities(self, needs):
        """'' if every capability needed by the plan is allowed and present, otherwise the reason."""
        if 'avoidance' in needs:
            if not self.allow_avoidance:
                return 'avoidance disabled on the drone (allow_avoidance)'
            if self.count_subscribers('/avoidance/enable') == 0:
                return 'avoidance node not running'
        if 'rangefinder' in needs and not math.isfinite(self.agl_value()):
            return 'pick needs a rangefinder (no valid height above ground)'
        if 'marker' in needs and self.count_publishers(self.marker_topic) == 0:
            return f'pick needs the marker localizer ({self.marker_topic})'
        if 'payload' in needs and not self.payload_client.service_is_ready():
            return 'automatic pick needs the payload node (use manual)'
        return ''

    def finish_starting(self, ok, text):
        if self.pending_ack is not None:
            cid, cmd = self.pending_ack
            self.pending_ack = None
            self.send_ack(cid, cmd, ok, text)

    def continue_mission(self):
        """Continue after a planned hand-over (pilot finished the task) or after a pause."""
        step = self.plan_runner.current_step if self.plan_runner else None
        if self.state == MissionState.MISSION and step is not None and step.expects_manual:
            self.ctx.continue_pressed = True
            return True, 'continuing mission'
        if self.state == MissionState.PAUSED:
            if self.offboard_active:
                self.resume_from_pause()
            else:
                self.ctx.request_resume()      # offboard_control requests OFFBOARD; resume on confirmation
            return True, 'resuming'
        return False, 'nothing to continue'

    def start_landing(self):
        """Request landing. Accepted in any flying state; a running mission is aborted."""
        if self.state not in FLYING_STATES:
            return False, f'not flying (state {self.state.name})'
        if self.state == MissionState.STARTING:
            self.finish_starting(False, 'landing requested')
        if self.plan_runner is not None:
            self.plan_runner.abort(self.ctx)
            self.plan_runner = None
        self.mission_complete = False
        self.land_pub.publish(Bool(data=True))
        self.get_logger().info('Land request sent')
        self.transition_to(MissionState.LANDING)
        return True, 'landing'

    # ---- Inputs --------------------------------------------------------------------------------

    def gcs_mission_callback(self, msg: String) -> None:
        """Store the uploaded GCS mission items (Send WP). They are flown only on Start mission."""
        try:
            items = json.loads(msg.data)
            assert isinstance(items, list)
        except (ValueError, AssertionError) as e:
            self.get_logger().error(f'invalid /gcs/mission ({e})')
            return
        self.gcs_mission = items
        tasks = sum(1 for it in items if isinstance(it, dict) and it.get('command', 16) != 16)
        self.get_logger().info(f'GCS mission received: {len(items)} items ({tasks} with tasks)')

    def geo_reference_callback(self, msg: NavSatFix) -> None:
        """Store the EKF GPS origin; None when the EKF reports no global origin."""
        ok = msg.status.status >= NavSatStatus.STATUS_FIX and math.isfinite(msg.latitude)
        self.geo_reference = (msg.latitude, msg.longitude) if ok else None

    def current_position_callback(self, msg: Point) -> None:
        """Store the current position (NED) from offboard_control and its receive time."""
        self.current_position = msg
        self.position_time = self.now_sec()
        if not self.have_position:
            self.have_position = True
            self.get_logger().info(f'First position received: x={msg.x:.2f}, y={msg.y:.2f}, z={msg.z:.2f}')

    def agl_callback(self, msg: Float32) -> None:
        self.agl = float(msg.data)
        self.agl_time = self.now_sec()

    def heading_callback(self, msg: Float32) -> None:
        self.heading = float(msg.data)
        self.heading_time = self.now_sec()

    def heading_value(self):
        """Current heading (rad, NED), NaN when unknown or older than agl_timeout."""
        if self.heading_time is None or self.now_sec() - self.heading_time > self.agl_timeout:
            return math.nan
        return self.heading

    def agl_value(self):
        """Height above ground (m), NaN when invalid or older than agl_timeout."""
        if self.agl_time is None or self.now_sec() - self.agl_time > self.agl_timeout:
            return math.nan
        return self.agl

    def marker_callback(self, msg: PointStamped) -> None:
        """Marker offset from the drone in ENU (x east, y north), stored as (north, east)."""
        self.marker = (float(msg.point.y), float(msg.point.x), self.now_sec())

    def marker_offset(self):
        """(north, east) offset to the marker, or None if not seen within marker_timeout."""
        m = self.marker
        if m is None or self.now_sec() - m[2] > self.marker_timeout:
            return None
        return m[0], m[1]

    def avoidance_status_callback(self, msg: AvoidanceStatus) -> None:
        """Store the avoidance status; while navigating with avoidance, apply the new setpoint immediately."""
        self.avoidance_status = msg
        self.avoidance_status_time = self.now_sec()
        step = self.plan_runner.current_step if self.plan_runner else None
        if self.avoidance_in_use() and isinstance(step, Navigate):
            setpoint = self.ctx.nav.setpoint(self.ctx, step.point)
            if not isinstance(setpoint, Failed):        # a failure is handled on the next timer tick
                self.ctx.set_target(*setpoint)

    def flight_active_callback(self, msg: Bool) -> None:
        """Handle the end of a flight reported by offboard_control (touchdown, refused ARM, disarm)."""
        was, self.flight_active = self.flight_active, bool(msg.data)
        if not was or self.flight_active:
            return
        if self.state == MissionState.LANDING:
            self.get_logger().info('Touchdown reported by offboard_control')
        elif self.state in FLYING_STATES:
            self.get_logger().error(f'Flight ended by offboard_control during {self.state.name} '
                                    '(take-off aborted / PX4 disarmed) -> IDLE')
            self.finish_starting(False, 'flight ended')
            if self.plan_runner is not None:
                self.plan_runner.abort(self.ctx)
        else:
            return
        self.plan_runner = None
        self.ctx.nav = DirectNav()
        self.transition_to(MissionState.IDLE)

    def offboard_active_callback(self, msg: Bool) -> None:
        """Track OFFBOARD; leaving it is handled by check_override(), returning to it resumes a pause."""
        self.offboard_active = bool(msg.data)
        if not self.flight_active:
            return
        if self.offboard_active:
            self.offboard_seen = True
            if self.state == MissionState.PAUSED:
                self.resume_from_pause()

    # ---- Payload -------------------------------------------------------------------------------

    def payload_request(self):
        """Start a hook request on the payload node; returns the future, or None if it is not available."""
        if not self.payload_client.service_is_ready():
            return None
        return self.payload_client.call_async(Trigger.Request())

    def payload_result(self, future):
        """None while pending, else (success, message)."""
        if not future.done():
            return None
        try:
            res = future.result()
        except Exception as e:  # noqa: BLE001 -- any transport error fails the hook
            return False, str(e)
        return bool(res.success), str(res.message)

    # ---- Outputs -------------------------------------------------------------------------------

    def avoidance_in_use(self):
        step = self.plan_runner.current_step if self.plan_runner else None
        return (isinstance(self.ctx.nav, AvoidanceNav) and step is not None and step.uses_avoidance
                and self.state == MissionState.MISSION)

    def publish_avoidance_enable(self):
        """Enable the avoidance node while it is needed; nothing is sent for missions without avoidance."""
        on = self.state == MissionState.STARTING or self.avoidance_in_use()
        if on or self.avoidance_was_on:
            self.avoidance_enable_pub.publish(Bool(data=on))
        self.avoidance_was_on = on

    def publish_progress(self, state=None):
        """Publish mission progress on change and once per second (drone_telemetry -> MISSION_CURRENT)."""
        ex = self.plan_runner
        if state is None:
            if self.mission_complete and ex is None:
                state = 'complete'
            elif ex is None or self.state not in (MissionState.MISSION, MissionState.PAUSED, MissionState.STARTING):
                state = 'idle'
            elif self.state == MissionState.PAUSED or (ex.current_step and ex.current_step.expects_manual):
                state = 'paused'
            else:
                state = 'active'
        step = ex.current_step.name if ex and ex.current_step else ''
        progress = {'seq': ex.current_item if ex else 0, 'total': ex.total if ex else len(self.gcs_mission),
                    'state': state, 'step': step}
        now = self.now_sec()
        if progress != self.last_progress or now - self.last_progress_time >= 1.0:
            self.last_progress, self.last_progress_time = progress, now
            self.progress_pub.publish(String(data=json.dumps(progress)))

    def say(self, text, warn=False):
        """Short message for the ground station (STATUSTEXT, 50 characters) and the log."""
        self.status_text_pub.publish(String(data=json.dumps({'severity': SEV_WARNING if warn else SEV_INFO,
                                                             'text': text[:50]})))
        self.log(text, warn)

    def log(self, text, warn=False):
        if warn:
            self.get_logger().warning(text)
        else:
            self.get_logger().info(text)

    def publish_target_position(self, target: Point) -> None:
        """Send the target (NED) to offboard_control."""
        self.target_position_pub.publish(target)

    def publish_target_yaw(self, yaw: float) -> None:
        """Send the target heading (rad) to offboard_control; NaN keeps the take-off heading."""
        self.target_yaw_pub.publish(Float32(data=float(yaw)))

    # ---- Helpers -------------------------------------------------------------------------------

    def transition_to(self, new_state: MissionState) -> None:
        """Switch to new_state and reset the goal-hold timer. HOVER after a mission or a pause keeps the
        current heading instead of turning back to the take-off heading."""
        old_state = self.state
        self.state = new_state
        if new_state == MissionState.HOVER and old_state != MissionState.TAKEOFF:
            yaw = self.heading_value()
            self.hover_yaw = yaw if math.isfinite(yaw) else self.ctx.last_yaw
        elif new_state in (MissionState.IDLE, MissionState.TAKEOFF):
            self.hover_yaw = math.nan
        self.goal_hold_start = None
        self.get_logger().info(f'State transition: {old_state.name} --> {new_state.name}')

    def update_goal_hold(self, goal: Point) -> bool:
        """True once the drone has stayed within tolerance of the goal for goal_hold_time."""
        cfg = self.step_cfg
        ok = (self.xy_distance(goal) < cfg.xy_tolerance
              and abs(goal.z - self.current_position.z) < cfg.z_tolerance)
        if not ok:
            self.goal_hold_start = None
            return False
        if self.goal_hold_start is None:
            self.goal_hold_start = self.now_sec()
        return self.now_sec() - self.goal_hold_start >= cfg.goal_hold_time

    def held_time(self) -> float:
        """Seconds the goal has been held so far (0 when outside the tolerance)."""
        return 0.0 if self.goal_hold_start is None else self.now_sec() - self.goal_hold_start

    def xy_distance(self, goal: Point) -> float:
        """Horizontal distance (m) from the current position to goal."""
        return math.hypot(goal.x - self.current_position.x, goal.y - self.current_position.y)

    def maybe_log_status(self, text: str, period_sec: float = 1.0) -> None:
        """Log a status line at most once every period_sec seconds."""
        now = self.now_sec()
        if now - self.last_status_log_time >= period_sec:
            self.get_logger().info(text)
            self.last_status_log_time = now

    def now_sec(self) -> float:
        """Current node time in seconds."""
        return self.get_clock().now().nanoseconds * 1e-9


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MissionManagerNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
