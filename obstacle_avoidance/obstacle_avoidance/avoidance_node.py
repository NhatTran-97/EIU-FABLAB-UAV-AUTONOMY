#!/usr/bin/env python3
"""ROS wrapper for VFH+: /scan + PX4 pose + goal -> /avoidance/status.

This node does NOT publish setpoints for offboard_control. It only proposes one
(AvoidanceStatus.safe_setpoint); mission_manager decides whether to use it.

    /scan                              (sensor_msgs/LaserScan)  ┐
    /fmu/out/vehicle_local_position_v1 (px4_msgs)               ├─> /avoidance/status
    /fmu/out/vehicle_attitude          (px4_msgs)               │   (custom_msgs/AvoidanceStatus)
    /mission/goal                      (geometry_msgs/Point)    │
    /avoidance/enable                  (std_msgs/Bool)          ┘
                                                     (optional) -> /fmu/in/obstacle_distance

No hardcoded rates:
  - Each input has a RateMonitor that measures its real period; timeout = timeout_periods * period
    (capped by max_timeout).
  - The scan -> status latency is measured on every scan (LaserScan header.stamp is the time of the
    FIRST beam). Total latency + actuation_latency gives the speed that can still stop in time
    (motion_limits.py): a slower or more delayed lidar makes the drone slow down on its own.
  - Carrot = speed / MPC_XY_P, so PX4 flies at the computed speed.
"""
import math

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy, qos_profile_sensor_data)
from geometry_msgs.msg import Point
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool
from px4_msgs.msg import ObstacleDistance, VehicleAttitude, VehicleLocalPosition
from custom_msgs.msg import AvoidanceStatus

from obstacle_avoidance.motion_limits import safe_speed, stopping_distance, stopping_speed
from obstacle_avoidance.scan_processing import ScanConfig, process_scan
from obstacle_avoidance.timing import RateMonitor, RecentStats
from obstacle_avoidance.vfh_plus import BLOCKED, CLEAR, VfhParams, VfhPlus

STATE_NAMES = {AvoidanceStatus.CLEAR: 'CLEAR', AvoidanceStatus.AVOIDING: 'AVOIDING',
               AvoidanceStatus.BLOCKED: 'BLOCKED', AvoidanceStatus.STALE: 'STALE'}

# |now - header.stamp| above this means the lidar uses another clock (e.g. the driver's internal time),
# not that the data is really that old.
CLOCK_MISMATCH_SEC = 10.0


def parse_arcs_deg(text: str):
    """"135:225, 350:10" -> [(a, b), ...] (rad). Empty string -> []."""
    arcs = []
    for part in text.replace(';', ',').split(','):
        part = part.strip()
        if not part:
            continue
        a, b = part.split(':')
        arcs.append((math.radians(float(a)), math.radians(float(b))))
    return arcs


class ObstacleAvoidanceNode(Node):
    def __init__(self) -> None:
        super().__init__('obstacle_avoidance')

        # ---------------- Topics
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('local_position_topic', '/fmu/out/vehicle_local_position_v1')
        self.declare_parameter('attitude_topic', '/fmu/out/vehicle_attitude')
        self.declare_parameter('goal_topic', '/mission/goal')
        self.declare_parameter('enable_topic', '/avoidance/enable')
        self.declare_parameter('status_topic', '/avoidance/status')
        self.declare_parameter('obstacle_distance_topic', '/fmu/in/obstacle_distance')

        # ---------------- Lidar / scan filtering
        self.declare_parameter('lidar_yaw_offset_deg', 0.0)
        self.declare_parameter('lidar_upside_down', False)
        self.declare_parameter('min_range', 0.3)
        self.declare_parameter('max_range', 12.0)
        self.declare_parameter('self_mask_deg', '')      # e.g. "135:225, 300:310"
        self.declare_parameter('despeckle_tol', 0.3)
        self.declare_parameter('ground_filter', True)
        self.declare_parameter('ground_margin', 0.3)
        self.declare_parameter('ground_z', 0.0)           # NED z of the ground when dist_bottom is missing

        # ---------------- VFH+
        defaults = VfhParams()
        for name in VfhParams.__dataclass_fields__:
            self.declare_parameter(name, getattr(defaults, name))

        # ---------------- Dynamics: physical constants that ROS cannot measure
        self.declare_parameter('max_speed', 2.0)          # m/s, <= MPC_XY_VEL_MAX
        self.declare_parameter('max_decel', 2.0)          # m/s^2, <= MPC_ACC_HOR
        self.declare_parameter('px4_xy_p', 0.95)          # = MPC_XY_P
        self.declare_parameter('actuation_latency', 0.15) # s, status -> PX4 starts braking
        self.declare_parameter('motion_speed_threshold', 0.3)

        # ---------------- Timing: only factors; rates are measured at runtime
        self.declare_parameter('timeout_periods', 3.0)    # timeout = N * measured period
        self.declare_parameter('max_timeout', 1.0)        # s, blind for longer than this is always STALE
        self.declare_parameter('diagnostics_period', 5.0) # s, period of the rate / latency log

        self.declare_parameter('start_enabled', False)
        self.declare_parameter('publish_obstacle_distance', False)

        gp = lambda n: self.get_parameter(n).value  # noqa: E731

        self.scan_cfg = ScanConfig(
            lidar_yaw_offset=math.radians(float(gp('lidar_yaw_offset_deg'))),
            upside_down=bool(gp('lidar_upside_down')),
            min_range=float(gp('min_range')),
            max_range=float(gp('max_range')),
            self_mask=parse_arcs_deg(str(gp('self_mask_deg'))),
            despeckle_tol=float(gp('despeckle_tol')),
            ground_margin=float(gp('ground_margin')),
        )
        self.ground_filter = bool(gp('ground_filter'))
        self.ground_z = float(gp('ground_z'))

        self.vfh = VfhParams(**{
            name: type(getattr(defaults, name))(gp(name))
            for name in VfhParams.__dataclass_fields__})
        self.planner = VfhPlus(self.vfh)
        self.r_rs = self.vfh.drone_radius + self.vfh.safety_margin
        # Obstacles beyond this range are not seen yet -> the drone must be able to stop within it.
        self.sensing_horizon = min(self.vfh.window_radius, self.scan_cfg.max_range) - self.r_rs

        self.max_speed = float(gp('max_speed'))
        self.max_decel = float(gp('max_decel'))
        self.px4_xy_p = float(gp('px4_xy_p'))
        self.actuation_latency = float(gp('actuation_latency'))
        self.motion_speed_threshold = float(gp('motion_speed_threshold'))
        if min(self.max_speed, self.max_decel, self.px4_xy_p) <= 0.0:
            raise ValueError('max_speed, max_decel, px4_xy_p must be > 0')

        periods = float(gp('timeout_periods'))
        self.max_timeout = float(gp('max_timeout'))
        self.scan_rate = RateMonitor(periods, self.max_timeout)
        self.pose_rate = RateMonitor(periods, self.max_timeout)
        self.attitude_rate = RateMonitor(periods, self.max_timeout)
        self.goal_rate = RateMonitor(periods, self.max_timeout)
        self.latency_stats = RecentStats()
        self.stamp_trusted = None

        self.enabled = bool(gp('start_enabled'))
        self.publish_obstacle_distance = bool(gp('publish_obstacle_distance'))

        # ---------------- QoS: same as offboard_control.py
        px4_qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             history=HistoryPolicy.KEEP_LAST, depth=1)

        self.create_subscription(LaserScan, gp('scan_topic'), self.scan_callback,
                                 qos_profile_sensor_data)
        self.create_subscription(VehicleLocalPosition, gp('local_position_topic'),
                                 self.local_position_callback, px4_qos)
        self.create_subscription(VehicleAttitude, gp('attitude_topic'),
                                 self.attitude_callback, px4_qos)
        self.create_subscription(Point, gp('goal_topic'), self.goal_callback, 10)
        self.create_subscription(Bool, gp('enable_topic'), self.enable_callback, 10)

        self.status_pub = self.create_publisher(AvoidanceStatus, gp('status_topic'), 10)
        self.obstacle_distance_pub = None
        if self.publish_obstacle_distance:
            self.obstacle_distance_pub = self.create_publisher(
                ObstacleDistance, gp('obstacle_distance_topic'), px4_qos)

        self.local_position = None
        self.attitude_q = None
        self.goal = None
        self.last_state = None
        self.last_speed_limit = math.nan

        # The watchdog only has to catch /scan stopping (the planner runs in the scan callback).
        # Period = max_timeout/10 -> detection is late by at most 10 % of max_timeout.
        self.create_timer(self.max_timeout / 10.0, self.watchdog_callback)
        self.create_timer(float(gp('diagnostics_period')), self.diagnostics_callback)

        self.get_logger().info(
            f'obstacle_avoidance started | scan={gp("scan_topic")} '
            f'| sectors={self.planner.n_sectors} | window={self.vfh.window_radius:.1f}m '
            f'| block/free={self.vfh.block_distance:.1f}/{self.vfh.free_distance:.1f}m '
            f'| r_drone+margin={self.r_rs:.2f}m | max_speed={self.max_speed:.1f}m/s '
            f'| decel={self.max_decel:.1f}m/s^2 | enabled={self.enabled}')

    # ------------------------------------------------------------------ #
    def now_sec(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def local_position_callback(self, msg: VehicleLocalPosition) -> None:
        self.local_position = msg
        self.pose_rate.tick(self.now_sec())

    def attitude_callback(self, msg: VehicleAttitude) -> None:
        self.attitude_q = [float(v) for v in msg.q]
        self.attitude_rate.tick(self.now_sec())

    def goal_callback(self, msg: Point) -> None:
        self.goal = msg
        self.goal_rate.tick(self.now_sec())

    def enable_callback(self, msg: Bool) -> None:
        if msg.data and not self.enabled:
            self.planner.reset()
            self.last_state = None
            self.get_logger().info('Avoidance ENABLED')
        elif not msg.data and self.enabled:
            self.get_logger().info('Avoidance disabled')
        self.enabled = bool(msg.data)

    def stale_reason(self, now: float, check_scan: bool):
        checks = [('vehicle_attitude', self.attitude_rate),
                  ('vehicle_local_position', self.pose_rate),
                  ('goal', self.goal_rate)]
        if check_scan:
            checks.insert(0, ('/scan', self.scan_rate))
        for name, monitor in checks:
            if monitor.is_stale(now):
                return f'no {name} for {monitor.age(now):.2f}s (timeout {monitor.timeout():.2f}s)'
        if not self.local_position.xy_valid:
            return 'local position xy invalid'
        return None

    def estimate_agl(self):
        if not self.ground_filter or self.local_position is None:
            return None
        lp = self.local_position
        if lp.dist_bottom_valid:
            return float(lp.dist_bottom)
        return self.ground_z - float(lp.z)

    # ------------------------------------------------------------------ #
    def latency_estimate(self) -> float:
        """p90 scan -> status latency. Without samples: worst case = max_timeout."""
        p90 = self.latency_stats.p90()
        return self.max_timeout if p90 is None else p90

    def total_latency(self) -> float:
        return self.latency_estimate() + self.actuation_latency

    def record_latency(self, stamp: float, t_callback: float) -> None:
        now = self.now_sec()
        age = now - stamp
        trusted = stamp > 0.0 and -0.01 <= age <= CLOCK_MISMATCH_SEC
        if trusted != self.stamp_trusted:
            self.stamp_trusted = trusted
            if not trusted:
                self.get_logger().warning(
                    f'LaserScan header.stamp differs from the node clock by {age:.1f}s -> '
                    f'latency estimated as scan period + processing time')
        # Spinning lidar: when the driver publishes a full turn, the first beam is already >= 1 period old.
        # This lower bound covers drivers that stamp at the END of the turn (one period short) or use
        # another clock.
        floor = (self.scan_rate.period() or self.max_timeout) + (now - t_callback)
        self.latency_stats.add(max(age, floor) if trusted else floor)

    def speed_limit(self, res, points_ne) -> float:
        """Speed that can still stop in time (motion_limits.safe_speed) with the measured latency."""
        if res.state == BLOCKED:
            return 0.0
        lp = self.local_position
        velocity = (float(lp.vx), float(lp.vy)) if lp.v_xy_valid else None
        return safe_speed(points_ne, res.direction, velocity, self.r_rs,
                          self.sensing_horizon, self.total_latency(),
                          self.max_decel, self.max_speed)

    # ------------------------------------------------------------------ #
    def watchdog_callback(self) -> None:
        if not self.enabled:
            return
        reason = self.stale_reason(self.now_sec(), check_scan=True)
        if reason is not None:
            self.publish_stale(reason)

    def scan_callback(self, msg: LaserScan) -> None:
        t_callback = self.now_sec()
        self.scan_rate.tick(t_callback)
        self.scan_rate.set_hint(msg.scan_time)
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if self.attitude_q is None:
            return      # no attitude yet: the frame cannot be converted; the watchdog reports STALE

        processed = process_scan(msg.ranges, msg.angle_min, msg.angle_increment,
                                 msg.range_min, msg.range_max,
                                 self.attitude_q, self.estimate_agl(), self.scan_cfg)

        if self.obstacle_distance_pub is not None:
            self.publish_px4_obstacle_distance(processed)

        if not self.enabled:
            return
        reason = self.stale_reason(t_callback, check_scan=False)
        scan_age = t_callback - stamp
        if reason is None and self.max_timeout < scan_age <= CLOCK_MISMATCH_SEC:
            reason = f'/scan data {scan_age:.2f}s old (> max_timeout)'
        if reason is not None:
            self.publish_stale(reason)
            return
        self.run_planner(processed)
        self.record_latency(stamp, t_callback)

    def run_planner(self, processed) -> None:
        lp, goal = self.local_position, self.goal
        goal_ne = (goal.x - lp.x, goal.y - lp.y)

        motion_heading = None
        if lp.v_xy_valid and math.hypot(lp.vx, lp.vy) > self.motion_speed_threshold:
            motion_heading = math.atan2(lp.vy, lp.vx)

        res = self.planner.plan(processed.points_ne, goal_ne, motion_heading)
        speed = self.speed_limit(res, processed.points_ne)
        self.last_speed_limit = speed

        sp = Point()
        sp.z = float(goal.z)
        if res.state == BLOCKED:
            sp.x, sp.y = float(lp.x), float(lp.y)
        else:
            # PX4: horizontal velocity ~ MPC_XY_P * position error -> carrot = v / MPC_XY_P
            step = speed / self.px4_xy_p
            if res.state == CLEAR and math.hypot(*goal_ne) <= step:
                sp.x, sp.y = float(goal.x), float(goal.y)
            else:
                sp.x = float(lp.x + step * math.cos(res.direction))
                sp.y = float(lp.y + step * math.sin(res.direction))

        msg = self.new_status(res.state)
        msg.safe_setpoint = sp
        msg.selected_heading = float(res.direction) if res.direction is not None else math.nan
        msg.goal_heading = float(res.goal_bearing)
        msg.speed_limit = float(speed)
        msg.clearance = float(res.clearance_ahead)
        msg.min_distance = float(res.min_distance)
        msg.min_distance_bearing = float(res.min_bearing)
        self.status_pub.publish(msg)

        if res.state != self.last_state:
            heading = 'none' if res.direction is None else f'{math.degrees(res.direction):.0f}deg'
            self.get_logger().info(
                f'Avoidance {STATE_NAMES[res.state]} | heading={heading} '
                f'goal={math.degrees(res.goal_bearing):.0f}deg '
                f'| min_dist={res.min_distance:.2f}m @ {math.degrees(res.min_bearing):.0f}deg '
                f'| speed_limit={speed:.2f}m/s')
            self.last_state = res.state

    def new_status(self, state: int) -> AvoidanceStatus:
        msg = AvoidanceStatus()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'local_ned'
        msg.state = state
        period = self.scan_rate.period()
        msg.scan_period = float(period) if period is not None else math.nan
        p90 = self.latency_stats.p90()
        msg.latency = float(p90) if p90 is not None else math.nan
        return msg

    def publish_stale(self, reason: str) -> None:
        msg = self.new_status(AvoidanceStatus.STALE)
        msg.selected_heading = math.nan
        msg.goal_heading = math.nan
        msg.clearance = math.nan
        msg.min_distance = math.nan
        msg.min_distance_bearing = math.nan
        if self.local_position is not None:
            msg.safe_setpoint.x = float(self.local_position.x)
            msg.safe_setpoint.y = float(self.local_position.y)
            msg.safe_setpoint.z = float(self.local_position.z)
        self.status_pub.publish(msg)
        if self.last_state != AvoidanceStatus.STALE:
            self.get_logger().warning(f'Avoidance STALE: {reason}')
            self.last_state = AvoidanceStatus.STALE

    # ------------------------------------------------------------------ #
    def diagnostics_callback(self) -> None:
        """Log the measured rates and latency and the resulting allowed speeds."""
        def hz(m):
            return f'{m.rate():.1f}Hz(to {m.timeout() * 1000:.0f}ms)' if m.period() else 'n/a'

        T = self.total_latency()
        v_open = min(self.max_speed,
                     stopping_speed(self.sensing_horizon, T, self.max_decel))
        v_edge = stopping_speed(self.vfh.block_distance - self.r_rs, T, self.max_decel)
        self.get_logger().info(
            f'[diag] scan {hz(self.scan_rate)} | pose {hz(self.pose_rate)} '
            f'| att {hz(self.attitude_rate)} | goal {hz(self.goal_rate)} '
            f'| latency p90 {self.latency_estimate() * 1000:.0f}ms + actuation '
            f'{self.actuation_latency * 1000:.0f}ms | v_open {v_open:.2f}m/s '
            f'| v@block {v_edge:.2f}m/s | enabled={self.enabled}')

        for name, m in (('/scan', self.scan_rate), ('vehicle_local_position', self.pose_rate),
                        ('vehicle_attitude', self.attitude_rate)):
            if m.saturated():
                self.get_logger().warning(
                    f'{name} {m.rate():.1f}Hz too slow: {m.periods:.0f} periods > max_timeout '
                    f'{self.max_timeout:.2f}s -> frequent STALE expected')
        if v_open < self.max_speed - 1e-3:
            self.get_logger().warning(
                f'max_speed {self.max_speed:.1f}m/s cannot stop within the sensing range '
                f'{self.sensing_horizon:.1f}m with latency {T * 1000:.0f}ms -> limited to '
                f'{v_open:.2f}m/s (stopping distance at max_speed = '
                f'{stopping_distance(self.max_speed, T, self.max_decel):.1f}m)')

    def publish_px4_obstacle_distance(self, processed) -> None:
        """72 bins x 5 degrees, bin 0 = north, in cm -- for PX4 Collision Prevention (CP_DIST)."""
        n_bins, inc = 72, 5.0
        min_cm = int(self.scan_cfg.min_range * 100.0)
        max_cm = int(self.scan_cfg.max_range * 100.0)

        def bins_of(bearings):
            return np.round(np.degrees(np.mod(bearings, 2.0 * math.pi)) / inc).astype(int) % n_bins

        out = np.full(n_bins, 65535, dtype=np.int64)          # UINT16_MAX = unknown
        if processed.free_bearings.size:
            out[bins_of(processed.free_bearings)] = max_cm + 1  # max+1 = no obstacle
        pts = processed.points_ne
        if pts.shape[0]:
            d_cm = np.clip(np.hypot(pts[:, 0], pts[:, 1]) * 100.0, min_cm, max_cm).astype(np.int64)
            nearest = np.full(n_bins, np.iinfo(np.int64).max, dtype=np.int64)
            np.minimum.at(nearest, bins_of(np.arctan2(pts[:, 1], pts[:, 0])), d_cm)
            has = nearest < np.iinfo(np.int64).max
            out[has] = nearest[has]

        msg = ObstacleDistance()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        msg.frame = ObstacleDistance.MAV_FRAME_LOCAL_NED
        msg.sensor_type = ObstacleDistance.MAV_DISTANCE_SENSOR_LASER
        msg.distances = [int(v) for v in out]
        msg.increment = inc
        msg.min_distance = min_cm
        msg.max_distance = max_cm
        msg.angle_offset = 0.0
        self.obstacle_distance_pub.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ObstacleAvoidanceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
