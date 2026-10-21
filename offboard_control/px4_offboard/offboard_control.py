#!/usr/bin/env python3
"""OFFBOARD control node: streams position or velocity setpoints from mission_manager to PX4 over uXRCE-DDS.

The node makes no flight decisions. On /mission/start it takes the current position and heading as the
hold point, streams setpoints for offboard_warmup_s (PX4 only accepts OFFBOARD once a setpoint stream
exists), then requests OFFBOARD and ARM. The continuous setpoint stream is the OFFBOARD heartbeat: if it
stops, PX4 leaves OFFBOARD and runs its failsafe. The flight ends when PX4 reports landed or disarms.

Interface with mission_manager:
    in   /mission/start            (Bool)      start a flight at the current position
         /mission/target_position  (Point)     target in the EKF local NED frame, metres (position mode)
         /mission/target_velocity  (Vector3)   velocity in NED, m/s (velocity mode; falls back to holding
                                               position if not refreshed within velocity_timeout_s)
         /mission/target_yaw       (Float32)   heading in rad; NaN keeps the heading at start
         /mission/land             (Bool)      landing request
         /mission/resume           (Bool)      request OFFBOARD again after the pilot took control
    out  /mission/current_position (Point)     current position, only while the EKF reports it valid
         /mission/agl              (Float32)   height above ground from the rangefinder, NaN when invalid
         /mission/heading          (Float32)   current heading (rad, NED: 0 = north, clockwise)
         /mission/flight_active    (Bool)      flight in progress (latched)
         /mission/offboard_active  (Bool)      PX4 is in OFFBOARD (latched)
         /mission/geo_reference    (NavSatFix) GPS origin of the EKF (latched)

Pilot override: when PX4 leaves OFFBOARD during a flight (sticks moved with COM_RC_OVERRIDE, mode switch,
Hold, RTL), the node keeps streaming with the setpoint following the current position. Switching back to
OFFBOARD (RC mode switch or /mission/resume) therefore starts from where the pilot left the drone.

Quick test without mission_manager:
    ros2 topic pub --once /mission/start std_msgs/msg/Bool "{data: true}"
"""

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from geometry_msgs.msg import Point, Vector3
from std_msgs.msg import Bool, Float32
from sensor_msgs.msg import NavSatFix, NavSatStatus
from px4_msgs.msg import VehicleLandDetected
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand, \
                         VehicleControlMode, VehicleLocalPosition


def px4_out_topic(msg_cls, name: str) -> str:
    """Return the /fmu/out topic name matching the message version in px4_msgs.

    Messages with MESSAGE_VERSION > 0 are published with a _v<N> suffix (e.g. vehicle_local_position_v1),
    so deriving the name lets the node run on firmware that matches the workspace px4_msgs.
    """
    v = int(getattr(msg_cls, 'MESSAGE_VERSION', 0) or 0)
    return f'/fmu/out/{name}' + (f'_v{v}' if v > 0 else '')


class OffboardControlNode(Node):
    """Streams OFFBOARD setpoints to PX4 and manages the flight lifecycle (start, arm, land)."""

    def __init__(self)->None:
        super().__init__("offboard_control_node")

        # Parameters
        self.declare_parameter("default_target_x", 0.0)
        self.declare_parameter("default_target_y", 0.0)
        self.declare_parameter("default_target_z", -5.0)
        self.declare_parameter("default_target_yaw", 0.0)
        self.declare_parameter("setpoint_rate_hz", 10.0)
        self.declare_parameter("flight_active", False)
        # Empty: derive the topic name from px4_msgs. Set only to force a specific name.
        self.declare_parameter("vehicle_status_topic", "")   # unused; kept so old launch files still load
        self.declare_parameter("vehicle_local_position_topic", "")
        self.declare_parameter("vehicle_control_mode_topic", "")
        # Seconds a velocity setpoint stays valid; without a refresh the node holds the current position.
        self.declare_parameter("velocity_timeout_s", 0.5)
        # Seconds of setpoint streaming before requesting OFFBOARD and ARM.
        self.declare_parameter("offboard_warmup_s", 1.0)
        # Seconds to wait for PX4 to arm after the request; on timeout (e.g. failed pre-arm check) abort.
        self.declare_parameter("arm_timeout_s", 3.0)

        self.default_target_x = float(self.get_parameter('default_target_x').value)
        self.default_target_y = float(self.get_parameter('default_target_y').value)
        self.default_target_z = float(self.get_parameter('default_target_z').value)
        self.default_target_yaw = float(self.get_parameter('default_target_yaw').value)
        self.setpoint_rate_hz = float(self.get_parameter('setpoint_rate_hz').value)

        # Flights start only via /mission/start, at the current position. flight_active=True (fly to the
        # default target on startup) is ignored: that target is the EKF origin at power-up and may be far
        # from where the drone is now.
        self.flight_active = False
        if bool(self.get_parameter('flight_active').value):
            self.get_logger().warning('Parameter flight_active=True is ignored: flights start only via '
                                      '/mission/start (Take-off from GCS), at the current position')

        if self.setpoint_rate_hz <= 0.0:
            raise ValueError('Parameter "setpoint_rate_hz" must be > 0.0')

        self.timer_period = 1.0 / self.setpoint_rate_hz
        # Warmup in timer ticks, derived from seconds so that changing setpoint_rate_hz keeps the same delay.
        self.warmup_ticks = max(1, int(math.ceil(float(self.get_parameter('offboard_warmup_s').value)
                                                 * self.setpoint_rate_hz)))
        self.arm_timeout_s = float(self.get_parameter('arm_timeout_s').value)
        self.velocity_timeout_s = float(self.get_parameter('velocity_timeout_s').value)

        # QoS: PX4 topics over uXRCE-DDS are BEST_EFFORT; internal topics use the default (depth 10).
        px4_qos = QoSProfile(reliability = ReliabilityPolicy.BEST_EFFORT,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             history=HistoryPolicy.KEEP_LAST,depth=1)

        ros_qos = 10

        # To PX4
        self.offboard_control_mode_pub = self.create_publisher(OffboardControlMode,
                                                               '/fmu/in/offboard_control_mode', px4_qos)
        self.trajectory_setpoint_pub = self.create_publisher(TrajectorySetpoint,
                                                             '/fmu/in/trajectory_setpoint', px4_qos)
        self.vehicle_command_pub = self.create_publisher(VehicleCommand,
                                                         '/fmu/in/vehicle_command', px4_qos)

        # From PX4
        local_position_topic = (str(self.get_parameter('vehicle_local_position_topic').value)
                                or px4_out_topic(VehicleLocalPosition, 'vehicle_local_position'))
        self.vehicle_local_position_sub = self.create_subscription(VehicleLocalPosition, local_position_topic,
                                                                   self.vehicle_local_position_callback, px4_qos)

        self.land_detected_sub = self.create_subscription(VehicleLandDetected,'/fmu/out/vehicle_land_detected',
                                                        self.vehicle_land_detected_callback, px4_qos)
        # Armed and OFFBOARD state come from vehicle_control_mode (MESSAGE_VERSION 0 on PX4 1.17 and 1.18).
        # vehicle_status is not used: its version differs between firmware and px4_msgs, and a mismatch
        # means no message arrives, which would disable the arm timeout and the end of flight on disarm.
        control_mode_topic = (str(self.get_parameter('vehicle_control_mode_topic').value)
                              or px4_out_topic(VehicleControlMode, 'vehicle_control_mode'))
        self.control_mode_sub = self.create_subscription(VehicleControlMode, control_mode_topic,
                                                         self.vehicle_control_mode_callback, px4_qos)

        # Interface with mission_manager
        self.target_position_sub = self.create_subscription(Point, '/mission/target_position',
                                                            self.target_position_callback, ros_qos)
        self.target_yaw_sub = self.create_subscription(Float32, '/mission/target_yaw', self.target_yaw_callback, ros_qos)
        self.target_velocity_sub = self.create_subscription(Vector3, '/mission/target_velocity',
                                                            self.target_velocity_callback, ros_qos)
        self.resume_sub = self.create_subscription(Bool, '/mission/resume', self.resume_callback, ros_qos)

        self.land_request_sub = self.create_subscription(Bool, '/mission/land', self.land_request_callback, ros_qos)

        self.current_position_pub = self.create_publisher(Point, '/mission/current_position', ros_qos)
        self.agl_pub = self.create_publisher(Float32, '/mission/agl', ros_qos)
        self.heading_pub = self.create_publisher(Float32, '/mission/heading', ros_qos)
        # Latched: late subscribers immediately receive the latest value.
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL, history=HistoryPolicy.KEEP_LAST)
        self.flight_active_pub = self.create_publisher(Bool, '/mission/flight_active', latched)
        self.offboard_active_pub = self.create_publisher(Bool, '/mission/offboard_active', latched)
        # EKF GPS origin, used by mission_manager to convert GCS lat/lon waypoints to local NED.
        self.geo_reference_pub = self.create_publisher(NavSatFix, '/mission/geo_reference', latched)
        self.last_geo_reference = None

        self.start_request_sub = self.create_subscription(Bool,'/mission/start',
                                                          self.start_request_callback, ros_qos)

        # State
        self.vehicle_local_position = VehicleLocalPosition()

        self.have_local_position = False
        self.have_status = False         # vehicle_control_mode received (arming state known)
        self.was_armed = False           # PX4 has armed during the current flight
        self.arm_request_time = None
        self.land_requested = False
        self.land_command_sent = False
        self.offboard_active = False     # PX4 is in OFFBOARD (vehicle_control_mode)
        self.offboard_seen = False       # OFFBOARD was reached during the current flight
        self.setpoint_mode = 'position'  # 'position' | 'velocity'
        self.target_velocity = (0.0, 0.0, 0.0)
        self.velocity_time = None

        self.target_x = self.default_target_x
        self.target_y = self.default_target_y
        self.target_z = self.default_target_z
        self.target_yaw = self.default_target_yaw
        self.hold_yaw = self.default_target_yaw     # heading at flight start, used when the target yaw is NaN

        self.last_logged_target = None
        self.offboard_setpoint_counter = 0
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        self.publish_flight_active()
        self.offboard_active_pub.publish(Bool(data=False))

        self.get_logger().info(f'Offboard control node started (PX4 topics: {local_position_topic}, '
                               f'{control_mode_topic})')
        self.get_logger().info(f'setpoint_rate_hz={self.setpoint_rate_hz:.1f}, warmup {self.warmup_ticks} ticks, '
                               f'arm_timeout_s={self.arm_timeout_s:.1f}')

    # ---- PX4 callbacks -------------------------------------------------------------------------

    def vehicle_local_position_callback(self, msg:VehicleLocalPosition) -> None:
        """Forward the EKF local position to mission_manager while it is valid.

        When xy or z is invalid (e.g. the drone was carried elsewhere or the EKF lost its estimate), the
        position is not forwarded; mission_manager then sees a stale position and refuses Take-off.
        """
        self.vehicle_local_position = msg
        valid = bool(msg.xy_valid and msg.z_valid)
        if valid != self.have_local_position:
            if valid:
                self.get_logger().info(f'Local position valid: x={msg.x:.2f}, y={msg.y:.2f}, z={msg.z:.2f}')
            else:
                self.get_logger().warning('Local position INVALID (EKF xy/z not valid) -- take-off blocked')
        self.have_local_position = valid
        if not valid:
            return

        self.publish_geo_reference(msg)
        agl = float(msg.dist_bottom) if msg.dist_bottom_valid else math.nan
        self.agl_pub.publish(Float32(data=agl))
        if math.isfinite(msg.heading):
            self.heading_pub.publish(Float32(data=float(msg.heading)))

        current_position_msg = Point()
        current_position_msg.x = float(msg.x)
        current_position_msg.y = float(msg.y)
        current_position_msg.z = float(msg.z)
        self.current_position_pub.publish(current_position_msg)

    def update_armed(self, armed: bool) -> None:
        """Track the arming state and end the flight when PX4 disarms after having armed.

        A disarm can come from a completed landing, the kill switch, a failsafe, or the pilot / GCS; the
        flight is over regardless of whether this node sent the LAND command.
        """
        self.have_status = True
        if not self.flight_active:
            return
        if armed:
            self.was_armed = True
        elif self.was_armed:
            self.end_flight('PX4 disarmed after flight')

    def vehicle_land_detected_callback(self, msg:VehicleLandDetected)->None:
        """End the flight when PX4 reports touchdown after a LAND command sent by this node."""
        if msg.landed and self.land_command_sent and self.flight_active:
            self.end_flight('Landing completed')

    def vehicle_control_mode_callback(self, msg: VehicleControlMode) -> None:
        """Track whether PX4 is in OFFBOARD and report changes to mission_manager.

        Leaving OFFBOARD during a flight means the pilot or the ground station took over. From then on the
        setpoint follows the current position, so returning to OFFBOARD does not move the drone.
        """
        self.update_armed(bool(msg.flag_armed))
        active = bool(msg.flag_control_offboard_enabled)
        if active == self.offboard_active:
            return
        self.offboard_active = active
        self.offboard_active_pub.publish(Bool(data=active))
        if not self.flight_active:
            return
        if active:
            self.offboard_seen = True
            self.hold_current_position()
            self.get_logger().info('PX4 in OFFBOARD')
        elif self.offboard_seen and not self.land_command_sent:
            self.get_logger().warning('PX4 left OFFBOARD (pilot / ground station took over) -- tracking position')

    def hold_current_position(self) -> None:
        """Use the current position and heading as the setpoint."""
        pos = self.vehicle_local_position
        if self.have_local_position:
            self.target_x, self.target_y, self.target_z = float(pos.x), float(pos.y), float(pos.z)
            if math.isfinite(pos.heading):
                self.target_yaw = float(pos.heading)
        self.setpoint_mode = 'position'

    def end_flight(self, reason: str) -> None:
        """Reset the flight state and tell mission_manager the flight is over."""
        self.flight_active = False
        self.land_requested = False
        self.land_command_sent = False
        self.offboard_setpoint_counter = 0
        self.was_armed = False
        self.arm_request_time = None
        self.offboard_seen = False
        self.setpoint_mode = 'position'
        self.publish_flight_active()
        self.get_logger().info(f'{reason}; flight state reset')

    # ---- mission_manager callbacks ---------------------------------------------------------------

    def target_position_callback(self, msg:Point)->None:
        """Update the target (NED, metres) and send it to PX4 immediately.

        A changed target is published here instead of waiting for the next timer tick, which reduces latency
        for obstacle avoidance; the timer keeps streaming as the OFFBOARD heartbeat.
        """
        if self.overridden():
            return
        self.target_x = float(msg.x)
        self.target_y = float(msg.y)
        self.target_z = float(msg.z)
        self.setpoint_mode = 'position'

        rounded_target = (
            round(self.target_x, 2),
            round(self.target_y, 2),
            round(self.target_z, 2),
            round(self.target_yaw, 2),
        )
        if rounded_target == self.last_logged_target:
            return              # unchanged (mission_manager repeats it every tick): the timer stream is enough
        self.get_logger().info(f'Updated target position: '
                               f'x={self.target_x:.2f}, y={self.target_y:.2f}, z={self.target_z:.2f}, yaw={self.target_yaw:.2f}')
        self.last_logged_target = rounded_target

        if self.flight_active and not self.land_command_sent and self.offboard_setpoint_counter > self.warmup_ticks:
            self.publish_trajectory_setpoint()

    def target_yaw_callback(self, msg:Float32)->None:
        """Update the target heading (rad). NaN keeps the heading at flight start, so the drone does not
        turn to North on take-off."""
        yaw = float(msg.data)
        self.target_yaw = self.hold_yaw if math.isnan(yaw) else yaw

    def target_velocity_callback(self, msg: Vector3) -> None:
        """Switch to velocity setpoints (NED, m/s), e.g. for visual alignment over a marker."""
        if self.overridden():
            return
        self.target_velocity = (float(msg.x), float(msg.y), float(msg.z))
        self.velocity_time = self.now_sec()
        self.setpoint_mode = 'velocity'
        if self.flight_active and self.offboard_active and not self.land_command_sent:
            self.publish_offboard_control_mode()
            self.publish_trajectory_setpoint()

    def resume_callback(self, msg: Bool) -> None:
        """Request OFFBOARD again after the pilot or the ground station took over."""
        if not (msg.data and self.flight_active and self.was_armed and not self.land_command_sent):
            return
        if self.offboard_active:
            return
        self.hold_current_position()
        self.get_logger().info('Resume requested: requesting OFFBOARD mode...')
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0)

    def overridden(self) -> bool:
        """True while a flight is active but PX4 has left OFFBOARD after reaching it."""
        return self.flight_active and self.offboard_seen and not self.offboard_active

    def land_request_callback(self, msg:Bool)->None:
        """Register a landing request; the LAND command itself is sent from timer_callback.

        Accepted only while flying, with no request pending and LAND not yet sent.
        """
        if (self.flight_active and msg.data and not self.land_requested and not self.land_command_sent):
            self.land_requested = True
            self.get_logger().info('Landing requested by mission system')

    def start_request_callback(self, msg: Bool) -> None:
        """Start a new flight if none is active and the EKF position is valid.

        The initial target is the current position and heading, so the first setpoints hold the drone in
        place until mission_manager sends a new target (e.g. the take-off altitude).
        """
        if msg.data and not self.flight_active:
            if not self.have_local_position:
                self.get_logger().warning('Cannot start: local position unavailable')
                return
            pos = self.vehicle_local_position
            self.target_x, self.target_y, self.target_z = float(pos.x), float(pos.y), float(pos.z)
            self.hold_yaw = float(pos.heading) if math.isfinite(pos.heading) else self.default_target_yaw
            self.target_yaw = self.hold_yaw
            self.flight_active = True
            self.was_armed = False
            self.arm_request_time = None
            self.publish_flight_active()
            self.get_logger().info(f'Starting new flight at current position '
                                   f'({self.target_x:.2f}, {self.target_y:.2f}, {self.target_z:.2f}), '
                                   f'heading {math.degrees(self.hold_yaw):.0f} deg')

    # ---- Outputs to mission_manager --------------------------------------------------------------

    def publish_geo_reference(self, msg: VehicleLocalPosition) -> None:
        """Publish the EKF GPS origin (ref_lat, ref_lon, ref_alt) when it or its validity changes.

        Without a global origin (no GPS, e.g. indoors) the message is sent with status NO_FIX, which makes
        mission_manager reject GPS waypoint missions.
        """
        valid = bool(msg.xy_global and math.isfinite(msg.ref_lat) and math.isfinite(msg.ref_lon))
        key = (valid, round(float(msg.ref_lat), 7), round(float(msg.ref_lon), 7), round(float(msg.ref_alt), 1)) \
            if valid else (False,)
        if key == self.last_geo_reference:
            return
        self.last_geo_reference = key
        ref = NavSatFix()
        ref.header.stamp = self.get_clock().now().to_msg()
        ref.header.frame_id = 'ekf_origin'
        ref.status.status = NavSatStatus.STATUS_FIX if valid else NavSatStatus.STATUS_NO_FIX
        if valid:
            ref.latitude, ref.longitude = float(msg.ref_lat), float(msg.ref_lon)
            ref.altitude = float(msg.ref_alt) if msg.z_global else math.nan
            self.get_logger().info(f'EKF global reference: lat={ref.latitude:.7f}, lon={ref.longitude:.7f}, '
                                   f'alt={ref.altitude:.1f} m')
        else:
            self.get_logger().warning('EKF has no global reference (no GPS) -- GCS waypoint missions blocked')
        self.geo_reference_pub.publish(ref)

    def publish_flight_active(self) -> None:
        """Publish whether a flight is in progress."""
        self.flight_active_pub.publish(Bool(data=self.flight_active))

    # ---- Outputs to PX4 ------------------------------------------------------------------------

    def publish_offboard_control_mode(self)->None:
        """Tell PX4 which setpoint type is controlled: position or velocity."""
        msg = OffboardControlMode()
        msg.position = self.setpoint_mode == 'position'
        msg.velocity = self.setpoint_mode == 'velocity'
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.thrust_and_torque = False
        msg.direct_actuator = False
        msg.timestamp = self.get_timestamp_us()
        self.offboard_control_mode_pub.publish(msg)

    def publish_trajectory_setpoint(self)->None:
        """Send the current target as the PX4 setpoint: position (NED) or velocity, plus yaw."""
        msg = TrajectorySetpoint()
        if self.setpoint_mode == 'velocity':
            msg.position = [math.nan, math.nan, math.nan]
            msg.velocity = [float(v) for v in self.target_velocity]
        else:
            msg.position = [self.target_x, self.target_y, self.target_z]
        msg.yaw = self.target_yaw
        msg.timestamp = self.get_timestamp_us()
        self.trajectory_setpoint_pub.publish(msg)

    def publish_vehicle_command(self, command: int, param1: float=0.0, param2: float=0.0) -> None:
        """Send a VehicleCommand to the autopilot (system 1, component 1)."""
        msg = VehicleCommand()
        msg.command = command
        msg.param1 = param1
        msg.param2 = param2
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg._from_external = True
        msg.timestamp = self.get_timestamp_us()
        self.vehicle_command_pub.publish(msg)

    def arm(self)->None:
        """Request PX4 to arm."""
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)

    def disarm(self)->None:
        """Request PX4 to disarm."""
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=0.0)

    def land(self)->None:
        """Request PX4 to land in place."""
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)

    def return_to_home(self)->None:
        """Request PX4 to return to launch (RTL)."""
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_RETURN_TO_LAUNCH)

    def now_sec(self) -> float:
        """Current node time in seconds."""
        return self.get_clock().now().nanoseconds * 1e-9

    def get_timestamp_us(self)->int:
        """Current node time in microseconds (PX4 timestamp format)."""
        return int(self.get_clock().now().nanoseconds / 1000)

    # ---- Main loop -----------------------------------------------------------------------------

    def timer_callback(self) -> None:
        """Run at setpoint_rate_hz while a flight is active.

        Each tick publishes OffboardControlMode and the current setpoint; this stream is the OFFBOARD
        heartbeat, and if the node hangs or loses DDS, PX4 leaves OFFBOARD and runs its failsafe. The tick
        also drives the flight steps: request OFFBOARD + ARM after the warmup, abort if PX4 does not arm,
        and send LAND when a landing is requested.
        """
        if not self.flight_active:
            return

        now_s = self.now_sec()
        if self.overridden():
            self.hold_current_position()          # follow the pilot so switching back does not move the drone
        elif (self.setpoint_mode == 'velocity'
              and (self.velocity_time is None or now_s - self.velocity_time > self.velocity_timeout_s)):
            self.hold_current_position()          # velocity setpoints stopped: hold here
        self.publish_offboard_control_mode()
        if not self.land_command_sent:
            self.publish_trajectory_setpoint()

        # Warmup done: request OFFBOARD (custom main mode 6), then ARM.
        if self.offboard_setpoint_counter == self.warmup_ticks:
            self.get_logger().info('Requesting OFFBOARD mode...')
            self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE,
                                         param1=1.0, param2=6.0)
            self.get_logger().info('Requesting ARM ...')
            self.arm()
            self.arm_request_time = now_s

        # PX4 has not armed within arm_timeout_s (pre-arm check, RC, kill switch): abort instead of
        # streaming forever. Checked only when vehicle_control_mode is available, since otherwise the
        # arming state is unknown.
        if (self.have_status and not self.was_armed and self.arm_request_time is not None
                and now_s - self.arm_request_time > self.arm_timeout_s):
            self.get_logger().error(f'PX4 did not arm within {self.arm_timeout_s:.1f} s '
                                    '(pre-arm check? see PX4 messages) -> abort take-off')
            self.end_flight('Take-off aborted')
            return

        # Landing requested before ARM was requested (Land pressed right after Take-off): cancel, do not arm.
        if self.land_requested and self.arm_request_time is None:
            self.end_flight('Take-off cancelled before arming')
            return

        # Send LAND once, and only after OFFBOARD + ARM have been requested.
        if self.land_requested and not self.land_command_sent and self.offboard_setpoint_counter > self.warmup_ticks:
            self.get_logger().info('Sending LAND command to PX4 ...')
            self.land()
            self.land_requested = False
            self.land_command_sent = True

        if self.offboard_setpoint_counter <= self.warmup_ticks:
            self.offboard_setpoint_counter += 1


def main(args=None) -> None:
    rclpy.init(args=args)
    node = OffboardControlNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down offboard node')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
