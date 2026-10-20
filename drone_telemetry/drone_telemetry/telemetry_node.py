#!/usr/bin/env python3
"""Gui trang thai + tan so cua cac thiet bi tren drone xuong dashboard qua radio SiK.

    /diagnostics (lidar, gimbal, camera) ┐
    topic PX4 /fmu/out/... (dem tan so) ├─> DRONE_COMPONENT_STATUS ─┐
                                         ┘                            ├─> /dev/telem_radio
    /fmu/out/vehicle_odometry (PX4, DDS) ──> DRONE_LOCAL_POSE ────────┤   (MAVLink v2,
    /gimbal_state (skydroid) ───────────────> DRONE_GIMBAL_STATE ──────┘    drone_dialect)
                                                   + HEARTBEAT

  * DRONE_COMPONENT_STATUS: moi 1/send_rate_hz s, va ngay khi level / state doi.
    Khong co du lieu > stale_timeout_s -> STALE / NO_DATA. Danh sach thiet bi: config/telemetry.yaml.
  * Trang thai link radio: /diagnostics ("drone_telemetry: radio").
  * Lenh ground (Take-off / Land / Hold / RTL, waypoint): drone_telemetry/commands.py.
    Thread doc serial -> queue -> xu ly tren thread ROS (timer 20 Hz).
  * DRONE_LOCAL_POSE: mau moi nhat, pose.rate_hz (bo mau cu hon pose.max_age_s).
    Bang thong radio 433 ~2 KB/s; 1 goi ~60 B -> 5 Hz ~300 B/s.
  * GLOBAL_POSITION_INT (2 Hz): vi tri ban do cho GCS khi khong co link 915.
"""

import json
import math
import queue

import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from rosidl_runtime_py.utilities import get_message

from drone_telemetry import drone_dialect as mavlink
from drone_telemetry.commands import CommandBridge, MissionReceiver
from drone_telemetry.health import STATE_MAX_LEN, ComponentMonitor
from drone_telemetry.radio_link import RadioLink

def px4_out_topic(msg_cls, name):
    """Ten topic PX4 theo phien ban message: MESSAGE_VERSION > 0 -> hau to _v{N} (quy tac cua PX4)."""
    v = int(getattr(msg_cls, 'MESSAGE_VERSION', 0) or 0)
    return f'/fmu/out/{name}' + (f'_v{v}' if v > 0 else '')


PX4_QOS = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                     history=HistoryPolicy.KEEP_LAST, depth=5)


class TelemetryNode(Node):
    def __init__(self):
        super().__init__('drone_telemetry',
                         automatically_declare_parameters_from_overrides=True)

        port = self._param('port', '/dev/telem_radio')
        baud = self._param('baud', 57600)
        send_rate = self._param('send_rate_hz', 1.0)
        heartbeat_rate = self._param('heartbeat_rate_hz', 1.0)
        stale_timeout = self._param('stale_timeout_s', 3.0)
        diag_topic = self._param('diagnostics_topic', '/diagnostics')

        self._rx_queue = None              # tao trong _setup_commands, truoc khi mo cong
        self._link = RadioLink(port, baud,
                               self._param('system_id', 1),
                               self._param('component_id', mavlink.MAV_COMP_ID_ONBOARD_COMPUTER),
                               self.get_logger(), on_message=self._on_radio_message)
        self._radio_status = None

        # --- thiet bi ---------------------------------------------------------------------
        self._by_diag_name = {}
        self._topic_monitors = []
        self._monitors = []
        self._topic_subs = []
        for name in self._param('components', ['lidar', 'gimbal', 'camera', 'px4']):
            mon = self._make_monitor(name, stale_timeout)
            if mon is not None:
                self._monitors.append(mon)
        self._last_sent = {}   # comp_id -> (level, state) gui cuoi

        self._setup_pose()
        self._setup_global_position()
        self._setup_gimbal()
        self._setup_commands()

        self.create_subscription(DiagnosticArray, diag_topic, self._on_diagnostics, 10)
        self._diag_pub = self.create_publisher(DiagnosticArray, diag_topic, 10)

        self.create_timer(1.0 / send_rate, self._send_all)
        self.create_timer(1.0 / heartbeat_rate, self._send_heartbeat)
        self.create_timer(0.2, self._tick)
        self.create_timer(1.0, self._publish_own_diagnostics)

        self._link.ensure_open()
        self.get_logger().info(
            f"drone_telemetry: {port} @ {baud}, {len(self._monitors)} thiet bi, "
            f"gui {send_rate} Hz")

    # --- cau hinh ------------------------------------------------------------------------
    def _param(self, name, default):
        if self.has_parameter(name):
            value = self.get_parameter(name).value
            if value is not None:
                return value
        return default

    def _make_monitor(self, name, stale_timeout):
        p = lambda key, default: self._param(f'{name}.{key}', default)  # noqa: E731
        comp_id = p('id', 0)
        if not comp_id:
            self.get_logger().error(f"Thiet bi '{name}' thieu {name}.id trong yaml -> bo qua")
            return None
        mon = ComponentMonitor(comp_id, name, stale_timeout,
                               state_key=p('state_key', 'state'),
                               rate_key=p('rate_key', ''),
                               count_key=p('count_key', ''),
                               min_rate_hz=p('min_rate_hz', 0.0))
        source = p('source', 'diagnostics')
        if source == 'diagnostics':
            diag_name = p('diag_name', '')
            if not diag_name:
                self.get_logger().error(f"'{name}': source=diagnostics nhung thieu diag_name")
                return None
            self._by_diag_name[diag_name] = mon
        elif source == 'topic':
            topic, msg_type = p('topic', ''), p('msg_type', '')
            try:
                msg_cls = get_message(msg_type)
                sub = self.create_subscription(
                    msg_cls, topic, lambda _msg, m=mon: m.on_message(self._now()), PX4_QOS)
            except Exception as e:   # thieu package / typesupport
                self.get_logger().error(f"'{name}': khong subscribe duoc {topic} ({msg_type}): {e}")
                return mon   # van gui (luon STALE)
            self._topic_subs.append(sub)
            self._topic_monitors.append(mon)
        else:
            self.get_logger().error(f"'{name}': source '{source}' khong hop le")
            return None
        return mon

    def _now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    # --- pose PX4 -> DRONE_LOCAL_POSE -------------------------------------------------------
    def _setup_pose(self):
        self._pose = None          # (t, VehicleOdometry)
        self._pose_sent = 0
        if not self._param('pose.enabled', True):
            return
        topic = self._param('pose.topic', '/fmu/out/vehicle_odometry')
        msg_type = self._param('pose.msg_type', 'px4_msgs/msg/VehicleOdometry')
        rate = float(self._param('pose.rate_hz', 5.0))
        self._pose_max_age = float(self._param('pose.max_age_s', 0.5))
        try:
            self.create_subscription(get_message(msg_type), topic, self._on_odometry, PX4_QOS)
        except Exception as e:
            self.get_logger().error(f"pose: khong subscribe duoc {topic} ({msg_type}): {e}")
            return
        self.create_timer(1.0 / rate, self._send_pose)
        self.get_logger().info(f"pose: {topic} -> DRONE_LOCAL_POSE {rate} Hz")

    # --- vi tri toan cuc -> GLOBAL_POSITION_INT (du phong khi GCS khong co link 915) ------------
    def _setup_global_position(self):
        """lat / lon / alt: vehicle_global_position; relative_alt, van toc, hdg: vehicle_odometry."""
        self._global = None
        if not self._param('global_position.enabled', True):
            return
        topic = self._param('global_position.topic', '/fmu/out/vehicle_global_position')
        rate = float(self._param('global_position.rate_hz', 2.0))
        self._global_max_age = float(self._param('global_position.max_age_s', 1.0))
        try:
            self.create_subscription(get_message('px4_msgs/msg/VehicleGlobalPosition'), topic,
                                     lambda m: setattr(self, '_global', (self._now(), m)), PX4_QOS)
        except Exception as e:
            self.get_logger().error(f"global_position: khong subscribe duoc {topic}: {e}")
            return
        self.create_timer(1.0 / rate, self._send_global_position)
        self.get_logger().info(f"global_position: {topic} -> GLOBAL_POSITION_INT {rate} Hz")

    def _send_global_position(self):
        if self._global is None or not self._link.is_open:
            return
        t, g = self._global
        if self._now() - t > self._global_max_age or not g.lat_lon_valid:
            return                 # du lieu cu hoac chua co fix
        rel_mm, vx, vy, vz, hdg = 0, 0, 0, 0, 65535
        if self._pose is not None and self._now() - self._pose[0] <= self._pose_max_age:
            o = self._pose[1]
            rel_mm = int(-float(o.position[2]) * 1000)           # NED: z < 0 = len; goc EKF ~ diem cat canh
            vx, vy, vz = (int(float(o.velocity[i]) * 100) for i in range(3))
            w, x, y, z = (float(v) for v in o.q)
            yaw = math.degrees(math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))) % 360.0
            hdg = int(yaw * 100) % 36000
        self._link.send(self._link.mav.global_position_int_encode(
            int(self._now() * 1000) & 0xFFFFFFFF, int(g.lat * 1e7), int(g.lon * 1e7),
            int(float(g.alt) * 1000) if g.alt_valid else 0, rel_mm,
            max(-32768, min(32767, vx)), max(-32768, min(32767, vy)), max(-32768, min(32767, vz)), hdg))

    # --- goc gimbal -> DRONE_GIMBAL_STATE ----------------------------------------------------
    def _setup_gimbal(self):
        self._gimbal = None
        if not self._param('gimbal_state.enabled', True):
            return
        topic = self._param('gimbal_state.topic', '/gimbal_state')
        msg_type = self._param('gimbal_state.msg_type', 'skydroid_msgs/msg/GimbalState')
        rate = float(self._param('gimbal_state.rate_hz', 2.0))
        self._gimbal_max_age = float(self._param('gimbal_state.max_age_s', 1.5))
        try:
            self.create_subscription(get_message(msg_type), topic, self._on_gimbal, 10)
        except Exception as e:
            self.get_logger().error(f"gimbal_state: khong subscribe duoc {topic} ({msg_type}): {e}")
            return
        self.create_timer(1.0 / rate, self._send_gimbal)

    def _on_gimbal(self, msg):
        self._gimbal = (self._now(), msg)

    def _send_gimbal(self):
        if self._gimbal is None or not self._link.is_open:
            return
        t, m = self._gimbal
        if self._now() - t > self._gimbal_max_age:
            return                 # du lieu cu -> khong gui (ground thay STALE)
        self._link.send(self._link.mav.drone_gimbal_state_encode(
            float(m.yaw_deg), float(m.pitch_deg), float(m.roll_deg),
            1 if m.is_connected else 0, int(m.control_mode)))

    def _on_odometry(self, msg):
        self._pose = (self._now(), msg)

    def _send_pose(self):
        if self._pose is None or not self._link.is_open:
            return
        t, m = self._pose
        if self._now() - t > self._pose_max_age:
            return                 # du lieu cu -> khong gui
        # chi ho tro POSE_FRAME_NED (1); frame khac -> bo qua
        if getattr(m, 'pose_frame', 1) not in (0, 1):
            return
        msg = self._link.mav.drone_local_pose_encode(
            int(m.timestamp_sample // 1000) & 0xFFFFFFFF,
            float(m.position[0]), float(m.position[1]), float(m.position[2]),
            [float(v) for v in m.q],
            float(m.velocity[0]), float(m.velocity[1]), float(m.velocity[2]),
            int(m.reset_counter) & 0xFF)
        if self._link.send(msg):
            self._pose_sent += 1

    # --- nhan du lieu --------------------------------------------------------------------
    def _on_diagnostics(self, msg):
        now = self._now()
        for status in msg.status:
            mon = self._by_diag_name.get(status.name)
            if mon is None:
                continue
            # level: rclpy tra ve bytes do dai 1
            level = status.level[0] if isinstance(status.level, (bytes, bytearray)) else int(status.level)
            values = {kv.key: kv.value for kv in status.values}
            mon.update_from_diag(now, level, status.message, values)
            self._send_if_changed(mon, now)

    def _tick(self):
        now = self._now()
        for mon in self._topic_monitors:
            mon.tick(now)
        # nguon topic: STALE / thay doi cung gui ngay
        for mon in self._monitors:
            self._send_if_changed(mon, now)

    def _on_radio_message(self, msg):
        # thread doc serial; RADIO_STATUS do radio SiK tu chen
        t = msg.get_type()
        if t == 'RADIO_STATUS':
            self._radio_status = (self._now(), msg)
        elif t in ('COMMAND_LONG', 'MISSION_COUNT', 'MISSION_ITEM_INT') and self._rx_queue is not None:
            self._rx_queue.put(msg)           # xu ly tren thread ROS

    # --- lenh ground -> PX4 ----------------------------------------------------------------
    def _setup_commands(self):
        self._rx_queue = None
        if not self._param('commands.enabled', True):
            self.get_logger().info('commands: tat (commands.enabled=false)')
            return
        self._sysid = int(self._param('system_id', 1))
        self._compid = int(self._param('component_id', mavlink.MAV_COMP_ID_ONBOARD_COMPUTER))
        self._px4_target = int(self._param('commands.px4_system_id', 1))
        p = lambda k, d: self._param(f'commands.{k}', d)  # noqa: E731
        try:
            self._VehicleCommand = get_message('px4_msgs/msg/VehicleCommand')
            self._cmd_pub = self.create_publisher(self._VehicleCommand, p('command_topic', '/fmu/in/vehicle_command'), 10)
            ack_cls = get_message('px4_msgs/msg/VehicleCommandAck')
            # rong = suy tu MESSAGE_VERSION cua px4_msgs (1.17: vehicle_command_ack; 1.18: ..._v1)
            ack_topic = p('ack_topic', '') or px4_out_topic(ack_cls, 'vehicle_command_ack')
            self.create_subscription(ack_cls, ack_topic, self._on_px4_ack, PX4_QOS)
            self.get_logger().info(f'commands: PX4 ACK <- {ack_topic}')
            # armed lay tu vehicle_control_mode: VehicleStatus lech phien ban (firmware v1, px4_msgs v4)
            self.create_subscription(get_message('px4_msgs/msg/VehicleControlMode'),
                                     p('control_mode_topic', '/fmu/out/vehicle_control_mode'), self._on_control_mode, PX4_QOS)
            self.create_subscription(get_message('px4_msgs/msg/VehicleLandDetected'),
                                     p('land_topic', '/fmu/out/vehicle_land_detected'), self._on_land, PX4_QOS)
            self.create_subscription(get_message('px4_msgs/msg/VehicleGlobalPosition'),
                                     p('global_topic', '/fmu/out/vehicle_global_position'), self._on_global, PX4_QOS)
        except Exception as e:
            self.get_logger().error(f'commands: khong tao duoc pub/sub PX4 ({e}) -> tat lenh')
            return
        # Take-off / Land: 'mission' -> mission_manager -> offboard_control (OFFBOARD); 'px4' -> VehicleCommand
        via = str(p('takeoff_land_via', 'mission'))
        send_mission = None
        if via == 'mission':
            self._gcs_cmd_pub = self.create_publisher(String, p('gcs_command_topic', '/gcs/command'), 10)
            self.create_subscription(String, p('gcs_command_ack_topic', '/gcs/command_ack'),
                                     self._on_mission_ack, 10)
            send_mission = self._send_mission_command
        elif via != 'px4':
            self.get_logger().error(f"commands.takeoff_land_via '{via}' khong hop le (mission | px4) -> px4")
        self._bridge = CommandBridge(self._publish_vehicle_command, self._reply_ack, self._status_text,
                                     send_mission)
        self.get_logger().info(f'commands: Take-off / Land via {via}'
                               + (' (mission_manager -> offboard_control -> PX4 OFFBOARD)' if send_mission else ''))
        self._mission = MissionReceiver(self._link.mav, self._link.send, self._on_mission)
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL, history=HistoryPolicy.KEEP_LAST)
        self._mission_pub = self.create_publisher(String, p('mission_topic', '/gcs/mission'), latched)
        self._gcs_peer = (255, 190)
        self._rx_queue = queue.Queue()
        self.create_timer(0.05, self._process_commands)
        self.get_logger().info('commands: Take-off / Land / Hold / RTL + waypoint -> PX4 (DDS)')

    def _process_commands(self):
        now = self._now()
        while True:
            try:
                m = self._rx_queue.get_nowait()
            except queue.Empty:
                break
            if getattr(m, 'target_system', self._sysid) != self._sysid or \
                    getattr(m, 'target_component', self._compid) not in (self._compid, 0):
                continue
            self._gcs_peer = (m.get_srcSystem(), m.get_srcComponent())
            t = m.get_type()
            if t == 'COMMAND_LONG':
                self._bridge.handle(m.command, [m.param1, m.param2, m.param3, m.param4,
                                                m.param5, m.param6, m.param7], now, m.confirmation)
            elif t == 'MISSION_COUNT':
                self._mission.on_count(m, now)
            elif t == 'MISSION_ITEM_INT':
                self._mission.on_item(m, now)
        self._bridge.tick(now)
        self._mission.tick(now)

    def _publish_vehicle_command(self, cmd, params):
        m = self._VehicleCommand()
        m.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        m.param1, m.param2, m.param3, m.param4 = (float(x) for x in params[:4])
        m.param5, m.param6, m.param7 = float(params[4]), float(params[5]), float(params[6])
        m.command = int(cmd)
        m.target_system, m.target_component = self._px4_target, 1
        m.source_system, m.source_component = self._sysid, self._compid
        m.from_external = True
        self._cmd_pub.publish(m)
        self.get_logger().info(f'commands: -> PX4 cmd {cmd} {[round(x, 2) for x in params]}')

    def _send_mission_command(self, req):
        self._gcs_cmd_pub.publish(String(data=json.dumps(req)))
        self.get_logger().info(f'commands: -> mission_manager {req}')

    def _on_mission_ack(self, msg):
        try:
            a = json.loads(msg.data)
            req_id, result, text = a['id'], str(a['result']), str(a.get('text', ''))
        except (ValueError, KeyError, TypeError) as e:
            self.get_logger().error(f'/gcs/command_ack khong hop le: {msg.data!r} ({e})')
            return
        self._bridge.on_mission_ack(req_id, result, text, self._now())

    def _reply_ack(self, cmd, result):
        self._link.send(self._link.mav.command_ack_encode(cmd, result, 0, 0, *self._gcs_peer))

    def _status_text(self, text, severity):
        self.get_logger().info(f'commands: {text}')
        self._link.send(self._link.mav.statustext_encode(severity, text.encode('ascii', 'replace')[:50]))

    def _on_px4_ack(self, msg):
        self._bridge.on_px4_ack(int(msg.command), int(msg.result), self._now())

    def _on_control_mode(self, msg):
        self._bridge.update_px4(self._now(), armed=bool(msg.flag_armed))

    def _on_land(self, msg):
        self._bridge.update_px4(self._now(), landed=bool(msg.landed))

    def _on_global(self, msg):
        self._bridge.update_px4(self._now(), amsl=float(msg.alt), amsl_valid=bool(msg.alt_valid))

    def _on_mission(self, items):
        self.get_logger().info(f'commands: nhan {len(items)} waypoint tu ground')
        self._mission_pub.publish(String(data=json.dumps(items)))
        self._status_text(f'Mission: {len(items)} waypoints stored on Jetson', 6)

    # --- gui -----------------------------------------------------------------------------
    def _send_component(self, mon, now):
        level, rate, state = mon.snapshot(now)
        state_b = state.encode('ascii', 'replace')[:STATE_MAX_LEN]
        msg = self._link.mav.drone_component_status_encode(mon.comp_id, level, float(rate), state_b)
        if self._link.send(msg):
            self._last_sent[mon.comp_id] = (level, state)

    def _send_if_changed(self, mon, now):
        level, _, state = mon.snapshot(now)
        if self._last_sent.get(mon.comp_id) != (level, state):
            self._send_component(mon, now)

    def _send_all(self):
        if not self._link.ensure_open():
            return
        now = self._now()
        for mon in self._monitors:
            self._send_component(mon, now)
        self._send_radio_status(now)

    def _send_radio_status(self, now):
        """Trang thai radio drone: txbuf (% bo dem phat con trong) + byte/s gui xuong.

        txbuf >= 50 % OK, 30-50 % WARN, < 30 % ERROR (can giam pose.rate_hz).
        """
        last = getattr(self, '_radio_tx', None)
        self._radio_tx = (now, self._link.tx_bytes)
        bps = 0.0 if last is None or now <= last[0] else (self._link.tx_bytes - last[1]) / (now - last[0])
        if self._radio_status is None or now - self._radio_status[0] > 5.0:
            level, state = mavlink.DRONE_COMPONENT_LEVEL_STALE, 'NO_STATUS'
        else:
            txbuf = int(self._radio_status[1].txbuf)
            level = (mavlink.DRONE_COMPONENT_LEVEL_OK if txbuf >= 50 else
                     mavlink.DRONE_COMPONENT_LEVEL_WARN if txbuf >= 30 else mavlink.DRONE_COMPONENT_LEVEL_ERROR)
            state = f'BUF {txbuf}%'
        self._link.send(self._link.mav.drone_component_status_encode(
            mavlink.DRONE_COMPONENT_RADIO, level, float(bps), state.encode()[:STATE_MAX_LEN]))

    def _send_heartbeat(self):
        if not self._link.ensure_open():
            return
        # heartbeat v1: SiK 1.9 chi chen RADIO_STATUS khi thay heartbeat v1
        self._link.send(self._link.mav.heartbeat_encode(
            mavlink.MAV_TYPE_ONBOARD_CONTROLLER, mavlink.MAV_AUTOPILOT_INVALID,
            0, 0, mavlink.MAV_STATE_ACTIVE), force_v1=True)

    # --- tinh trang cua chinh node -------------------------------------------------------
    def _publish_own_diagnostics(self):
        st = DiagnosticStatus(name='drone_telemetry: radio', hardware_id=self._link.port)
        values = {
            'port': self._link.port,
            'connected': str(self._link.is_open).lower(),
            'tx_bytes': str(self._link.tx_bytes),
            'rx_bytes': str(self._link.rx_bytes),
            'tx_errors': str(self._link.tx_errors),
            'pose_sent': str(self._pose_sent),
        }
        if not self._link.is_open:
            st.level, st.message = DiagnosticStatus.ERROR, 'Radio port closed'
        else:
            st.level, st.message = DiagnosticStatus.OK, 'Radio port open'
            if self._radio_status is not None:
                t, rs = self._radio_status
                values.update({'rssi': str(rs.rssi), 'remote_rssi': str(rs.remrssi), 'txbuf': str(rs.txbuf),
                               'noise': str(rs.noise), 'remote_noise': str(rs.remnoise),
                               'rx_errors': str(rs.rxerrors),
                               'radio_status_age_s': f'{self._now() - t:.1f}'})
        st.values = [KeyValue(key=k, value=v) for k, v in values.items()]
        out = DiagnosticArray(status=[st])
        out.header.stamp = self.get_clock().now().to_msg()
        self._diag_pub.publish(out)

    def destroy_node(self):
        self._link.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = TelemetryNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
