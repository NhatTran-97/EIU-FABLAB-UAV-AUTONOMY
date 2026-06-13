import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, TwistStamped
import json, threading, time
import os
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy
from nav_msgs.msg import Path
from sensor_msgs.msg import NavSatFix, BatteryState
from custom_msgs.srv import ModeSignal
from mavros_msgs.msg import State
import math
import sys

# Allow running both via `ros2 run lora_drone_package lora_drone` (installed package) and as a
# plain `python3 lora_drone.py` (dev): make the package importable as lora_drone_package.*
# before importing the sibling modules below.

"""
lora_drone_package/
├── lora_drone.py          # ROS node: wiring + điều phối (gọn lại)
├── protocol.py            # THUẦN: CRC32 framing, clean JSON, wrap/unwrap, finite-check
├── serial_link.py         # SerialLink: open/reopen/close, write có lock, vòng readline
├── mission_assembler.py   # THUẦN: state machine upload chunk (begin/chunk/commit) + parse waypoint
├── telemetry.py           # THUẦN: TelemetryEncoder (đóng gói gói tin nén + phát hiện thay đổi) + ước lượng % pin
└── idempotency.py         # THUẦN: dedupe theo seq + reset session theo sid + cache ACK
"""


_PKG_PARENT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _PKG_PARENT not in sys.path:
    sys.path.insert(0, _PKG_PARENT)

from lora_drone_package.protocol import (
    clean_json_str, wrap_with_crc, unwrap_and_verify_crc,
)
from lora_drone_package.idempotency import IdempotencyCache
from lora_drone_package.telemetry import TelemetryEncoder, estimate_percent_from_voltage
from lora_drone_package.mission_assembler import MissionAssembler, parse_waypoints
from lora_drone_package.serial_link import SerialLink

"""
LORA_DRONE_TELEMETRY_INTERVAL=0.5 python3 lora_drone.py
LORA_DRONE_TELEMETRY_INTERVAL=0.4 python3 lora_drone.py
"""

# Verbose LoRa packet logging is opt-in to avoid flooding ROS logs.
# Enable with: LORA_DRONE_DEBUG=1
DEBUG = os.environ.get("LORA_DRONE_DEBUG", "0") == "1"


# Wire-protocol codec moved to protocol.py:
#   clean_json_str / is_finite_num / wrap_with_crc / unwrap_and_verify_crc


class LoraDrone(Node):
    def __init__(self):
        super().__init__('LoraDrone')

        self.pose = None
        self.lat = None
        self.lon = None
        self.alt = None
        self.last_percentage = None
        self.last_voltage = None
        self._stop_event = threading.Event()
        self.serial_path = os.environ.get('LORA_DRONE_PORT', '/dev/lora_ground')
        self.serial_baudrate = int(os.environ.get('LORA_DRONE_BAUD', '9600'))
        # 0.5s = 2 Hz. Safe on the 19.2kbps E32 link (~500 B/s capacity, ~24% used).
        # Override with LORA_DRONE_TELEMETRY_INTERVAL; do not go below ~0.3 (GPS packets
        # approach the air-rate ceiling). On a 2.4kbps link this must be >=2.0.
        self.telemetry_interval = max(0.2, float(os.environ.get("LORA_DRONE_TELEMETRY_INTERVAL", "0.7")))
        self.vel = None
        self.current_mode = "UNKNOWN"
        self.gps_ok = False

        # Telemetry compaction: slow-changing fields (state, battery) are only (re)sent when
        # they change or on a periodic full refresh, keeping steady-state packets small.
        # Position/speed/gps_ok go every cycle. See telemetry.py.
        self._tlm = TelemetryEncoder(
            full_refresh_sec=float(os.environ.get("LORA_DRONE_TELEMETRY_FULL_REFRESH", "15.0")))

        # Idempotency: a retried command (ground re-sends the SAME seq on ACK loss) is re-ACKed
        # but NOT executed twice — no double mission publish, no double mode trigger. See
        # idempotency.py.
        self._idem = IdempotencyCache(128)
        self._last_gps_at = 0.0         # monotonic time of last fresh GPS fix
        self._mission = MissionAssembler()   # chunked-mission assembly (see mission_assembler.py)
        # When a mission op is in progress we suppress telemetry TX for a short window
        # so the half-duplex LoRa channel is free for the next chunk/ACK exchange.
        self._tlm_suppress_until = 0.0

        # --- RX link-health counters: so a field operator can see WHY an upload fails —
        # truncated lines (length mismatch vs what ground sent), CRC drops, or chunks that
        # never arrive. Logged as a periodic summary + on every mission op. ---
        self._rx_stats = {"ok": 0, "no_json": 0, "decode_fail": 0, "crc_fail": 0}
        self._rx_stats_last_log = 0.0
        self._rx_stats_interval = float(os.environ.get("LORA_DRONE_RXSTATS_INTERVAL", "30.0"))

        # Dedicated debug log FILE (separate from the ROS console) so a field run can be
        # analysed offline next to test.cpp's mission_debug.log. Default $HOME/lora_drone_debug.log;
        # override with LORA_DRONE_LOG_FILE. In the Docker stack point it at a bind-mounted path
        # (e.g. /home/drone_ws/lora_drone_debug.log) so the file is visible on the host.
        self._dbg_path = os.environ.get(
            "LORA_DRONE_LOG_FILE", os.path.expanduser("~/lora_drone_debug.log"))
        try:
            self._dbg_file = open(self._dbg_path, "a", buffering=1)   # line-buffered
            self.get_logger().info(f"📝 LoRa debug log -> {self._dbg_path}")
        except Exception as e:
            self._dbg_file = None
            self.get_logger().warn(f"⚠️ Could not open LoRa debug log {self._dbg_path}: {e}")
        self._dbg("================ lora_drone start ================")

        # Open serial with a few retries (USB-LoRa may enumerate slightly late), then
        # fail loudly instead of leaving a spinning-but-dead node.
        # The link owns the port, the writer lock, reconnect, and the RX loop (see
        # serial_link.py). Its loops/reopen stop when is_alive() goes False — i.e. ROS shut
        # down or we called close(). open_with_retries() raises if the port never opens.
        self._link = SerialLink(
            self.serial_path, self.serial_baudrate, self.get_logger(),
            is_alive=lambda: rclpy.ok() and not self._stop_event.is_set())
        self._link.open_with_retries(5)

        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            depth=10)

        qos_pub = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            depth=10)

        self.subscription = self.create_subscription(
            PoseStamped, '/mavros/local_position/pose', self.pose_callback, qos_profile)
        self.subscription_global_position = self.create_subscription(
            NavSatFix, '/mavros/global_position/global', self.global_position_callback, qos_profile)
        self.subscription_battery = self.create_subscription(
            BatteryState, '/mavros/battery', self.battery_cb, qos_profile)
        self.subscription_speed = self.create_subscription(
            TwistStamped, '/mavros/local_position/velocity_local', self.velocity_cb, qos_profile)
        self.subscription_state=self.create_subscription(State, '/mavros/state', self.state_callback, qos_profile)

        self.mission_pub = self.create_publisher(Path, '/mission_path', qos_pub)

        self.mode_client = self.create_client(ModeSignal, '/mode_signal')

        threading.Thread(target=lambda: self._link.read_loop(self._on_serial_line),
                         daemon=True).start()
        threading.Thread(target=self.send_pose_loop, daemon=True).start()

    def close(self):
        self._stop_event.set()
        self._link.close()

    def _on_serial_line(self, line: str):
        # Called by the link's RX loop for each received line.
        if DEBUG:
            self.get_logger().info(f"[DRONE][RX_LINE] {line}")
        self.handle_command(line)
        self._maybe_log_rx_stats()

    def _send_json(self, obj: dict, log: bool = True, priority: bool = True, wrap: bool = True):
        try:
            if wrap:
                frame = wrap_with_crc(obj)
            else:
                # Telemetry goes out un-wrapped (no CRC) to save ~25 bytes/packet on the
                # bandwidth-starved LoRa link. The ground accepts "plain" packets; a
                # corrupted telemetry frame just fails JSON parse and is skipped. Commands
                # (ground->drone) still REQUIRE CRC — only this display-only path drops it.
                frame = obj
            frame_text = json.dumps(frame, separators=(",", ":"))
            frame_bytes = (frame_text + "\n").encode('utf-8')

            # The link does the locked write (priority) or best-effort skip (telemetry) and
            # reopens the port on a serial error. A False return = not sent this cycle.
            if not self._link.write(frame_bytes, priority=priority):
                return False

            if log:
                crc = frame.get("crc32", "-") if wrap else "-"
                self.get_logger().info(
                    f"📤 TX CRC={crc} payload={json.dumps(obj, separators=(',', ':'))}"
                )
            return True
        except Exception as e:
            self.get_logger().error(f"❌ JSON send error: {e}")
            return False

    def _cache_ack(self, seq, payload):
        # Thin delegate to the idempotency cache (kept so existing call sites don't change).
        self._idem.cache_ack(seq, payload)

    def _send_ack(self, payload, rx_seq=None, sid=None):
        if rx_seq is not None:
            payload["ack_seq"] = rx_seq
        if sid is not None:
            payload["sid"] = sid
        self._send_json(payload)
        self._cache_ack(rx_seq, payload)

    def _publish_mission(self, waypoints):
        path_msg = Path()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = "odom"
        for wp in waypoints:
            pose_stamped = PoseStamped()
            pose_stamped.header = path_msg.header
            pose_stamped.pose.position.x = wp["x"]
            pose_stamped.pose.position.y = wp["y"]
            pose_stamped.pose.position.z = wp["z"]
            pose_stamped.pose.orientation.w = 1.0
            path_msg.poses.append(pose_stamped)
        self.mission_pub.publish(path_msg)

    def _handle_mission_op(self, cmd, rx_seq, sid):
        # Silence outgoing telemetry while we ACK and wait for the next chunk.
        # Keeps the half-duplex LoRa channel free for the command/ACK exchange.
        self._suppress_telemetry(2.0)

        # Validate + assemble in the pure module; it returns the ACK payload and, on a
        # validated commit, the assembled waypoints (see mission_assembler.py).
        result = self._mission.handle_op(
            cmd, sid, time.monotonic(),
            info=self.get_logger().info, warn=self.get_logger().warn, dbg=self._dbg)

        if result.waypoints is None:
            # begin / chunk / any error case: just ACK the prepared payload.
            self._send_ack(result.ack, rx_seq=rx_seq, sid=sid)
            return

        # Validated commit: publish FIRST, then ACK + auto-OFFBOARD. Only discard the mission
        # on a successful publish, so a publish failure leaves it in place for a retry.
        try:
            self._publish_mission(result.waypoints)
            self.get_logger().info(
                f"Published mission {result.mission_id}: {len(result.waypoints)} waypoints "
                f"in {result.total_chunks} chunks"
            )
            self._dbg(f"MISSION {result.mission_id} PUBLISHED {len(result.waypoints)}wps "
                      f"-> /mission_path (OK, full upload)")
            self._mission.discard(result.mission_id)
            self._send_ack(result.ack, rx_seq=rx_seq, sid=sid)
            self.call_mode_service(ModeSignal.Request.OFFBOARD, "OFFBOARD", ack_seq=rx_seq, ack_sid=sid)
        except Exception as e:
            self._send_ack({
                "event": "uploaded",
                "status": False,
                "mission_id": result.mission_id,
                "error": str(e),
            }, rx_seq=rx_seq, sid=sid)
            self.get_logger().error(f"Publish error: {e}")

    def pose_callback(self, msg):
        self.pose = msg.pose

    def global_position_callback(self, msg):
        self.lat = msg.latitude
        self.lon = msg.longitude
        self.alt = msg.altitude
        self.gps_ok = (
            msg.status.status >= 0 and
            math.isfinite(self.lat) and
            math.isfinite(self.lon)
        )
        if self.gps_ok:
            self._last_gps_at = time.monotonic()

    def battery_cb(self, msg: BatteryState):
        v = float(msg.voltage) if (isinstance(msg.voltage, (int, float)) and math.isfinite(msg.voltage)) else None

        p = None
        if isinstance(msg.percentage, (int, float)) and math.isfinite(msg.percentage):
            p = float(msg.percentage)
            if 0.0 <= p <= 1.0:
                p *= 100.0
            elif p < 0:
                p = None
        if p is None and v is not None:
            p = estimate_percent_from_voltage(v, cells=4)
        if p is not None:
            p = max(0.0, min(100.0, p))

        self.last_percentage = p
        self.last_voltage = v

    def velocity_cb(self, msg: TwistStamped):
        try:
            linear = msg.twist.linear
            self.vel = (float(linear.x), float(linear.y), float(linear.z))
        except Exception as e:
            self.get_logger().warn(f"velocity_cb error: {e}")
            self.vel = None
    def state_callback(self, msg:State):
        try:
            mode = msg.mode or "UNKNOWN"
            if mode != self.current_mode:
                self.get_logger().info(f"[STATE] MAVROS mode: {self.current_mode} -> {mode}")
            self.current_mode = mode
        except Exception:
            self.current_mode = "UNKNOWN"
    def call_mode_service(self, mode_const, mode_name, ack_seq=None, ack_sid=None):
        # Non-blocking: never wait_for_service() here — this runs in the serial-read
        # thread, and a 3s block stalls command RX and induces ground retries.
        if not self.mode_client.service_is_ready():
            payload = {
                "event": "mode_push",
                "status": False,
                "mode": mode_name,
                "error": "service not ready",
            }
            if ack_seq is not None:
                payload["ack_seq"] = ack_seq
            if ack_sid is not None:
                payload["sid"] = ack_sid
            self._send_json(payload)
            self._cache_ack(ack_seq, payload)
            self.get_logger().warn("Mode service not ready")
            return

        req = ModeSignal.Request()
        req.mode = mode_const
        future = self.mode_client.call_async(req)

        # Send exactly ONE ACK for this call — whichever of the service response / the
        # timeout fires first wins. Without the timeout, a hung service would leave the seq
        # marked in-flight forever and the ground would never receive an ACK.
        responded = {"done": False}
        resp_lock = threading.Lock()

        def respond_once(payload):
            with resp_lock:
                if responded["done"]:
                    return False
                responded["done"] = True
            if ack_seq is not None:
                payload["ack_seq"] = ack_seq
            if ack_sid is not None:
                payload["sid"] = ack_sid
            self._send_json(payload)
            self._cache_ack(ack_seq, payload)
            return True

        def response_cb(fut):
            try:
                resp = fut.result()
                ok = bool(resp.accepted)
                respond_once({"event": "mode_push", "status": ok, "mode": mode_name, "msg": resp.msg})
                self.get_logger().info(
                    f"✅ Mode {mode_name} -> accepted={ok}, msg={resp.msg}"
                    f"{'' if ack_seq is None else f', ack_seq={ack_seq}'}"
                )
            except Exception as e:
                respond_once({"event": "mode_push", "status": False, "mode": mode_name, "error": str(e)})
                self.get_logger().error(f"Mode service error: {e}")

        def on_timeout():
            if respond_once({"event": "mode_push", "status": False, "mode": mode_name, "error": "service timeout"}):
                self.get_logger().warn(f"⏱️ Mode {mode_name} service timeout")

        future.add_done_callback(response_cb)
        threading.Timer(5.0, on_timeout).start()

    def _suppress_telemetry(self, seconds: float = 2.0):
        """Silence outgoing telemetry for `seconds` to clear the LoRa channel for
        an ACK or the next mission chunk from ground. Call from the command handler."""
        self._tlm_suppress_until = time.monotonic() + seconds

    def send_pose_loop(self):
        while rclpy.ok() and not self._stop_event.is_set():
            try:
                now = time.monotonic()

                # During mission upload suppress telemetry so the half-duplex LoRa
                # channel stays free for command/ACK frames from ground.
                if now < self._tlm_suppress_until:
                    time.sleep(self.telemetry_interval)
                    continue

                # GPS counts as "fresh" only if the topic updated recently. Otherwise we
                # must NOT keep streaming cached lat/lon, or the ground would treat a dead
                # GPS as a valid recent fix and allow OFFBOARD/mission on stale data.
                gps_fresh = (
                    self._last_gps_at > 0.0
                    and (now - self._last_gps_at) < 2.0
                )

                # Build the compact telemetry packet (change-detection lives in the encoder).
                data, token = self._tlm.build(
                    now,
                    gps_ok=self.gps_ok, gps_fresh=gps_fresh,
                    pose=self.pose, lat=self.lat, lon=self.lon, alt=self.alt,
                    vel=self.vel, state=self.current_mode,
                    percentage=self.last_percentage, voltage=self.last_voltage,
                )

                sent = self._send_json(data, log=DEBUG, priority=False, wrap=False)

                # Advance the encoder's "last sent" trackers only when the packet actually went
                # out, so a skipped/failed cycle doesn't suppress the next change.
                if sent:
                    self._tlm.commit(token)
            except Exception as e:
                self.get_logger().error(f"❌ Error sending serial data: {e}")
            time.sleep(self.telemetry_interval)

    def _dbg(self, msg):
        # Append one timestamped line to the dedicated LoRa debug file (best-effort; never
        # let logging crash the RX path). Line-buffered, so it survives a power-off.
        f = self._dbg_file
        if f is None:
            return
        try:
            f.write(f"{time.time():.3f}  {msg}\n")
        except Exception:
            pass

    def _maybe_log_rx_stats(self):
        # Periodic one-line RX-health summary. Watch the bad counters climb in real time:
        # rising no_json/decode = packets arriving truncated (half-duplex collision); rising
        # crc = corrupted bytes. ok climbing with no bad = healthy link.
        now = time.monotonic()
        if now - self._rx_stats_last_log < self._rx_stats_interval:
            return
        self._rx_stats_last_log = now
        s = self._rx_stats
        bad = s["no_json"] + s["decode_fail"] + s["crc_fail"]
        if s["ok"] == 0 and bad == 0:
            return   # nothing received this window — stay quiet
        self.get_logger().info(
            f"📊 RX health (cumulative): ok={s['ok']} | bad={bad} "
            f"(no_json={s['no_json']} decode={s['decode_fail']} crc={s['crc_fail']})"
        )
        self._dbg(f"RX_HEALTH ok={s['ok']} bad={bad} "
                  f"(no_json={s['no_json']} decode={s['decode_fail']} crc={s['crc_fail']})")


    def handle_command(self, line: str):
        if line in ("ON", "OFF"):
            self.get_logger().info(f"ℹ️ Control token ignored: {line}")
            return

        clean = clean_json_str(line)
        if not clean:
            self._rx_stats["no_json"] += 1
            # len= is the truncation tell: if ground sent ~172B but we see len=90, the
            # half-duplex collision cut the packet (or the E32 idle-timeout split it).
            self.get_logger().warn(f"⚠️ No JSON object in line (len={len(line)}B): {line}")
            self._dbg(f"RX_BAD no_json len={len(line)} : {line[:120]}")
            return

        try:
            raw_cmd = json.loads(clean)
        except json.JSONDecodeError:
            self._rx_stats["decode_fail"] += 1
            self.get_logger().warn(f"⚠️ JSON decode failed (len={len(clean)}B): {clean}")
            self._dbg(f"RX_BAD decode len={len(clean)} : {clean[:120]}")
            return

        cmd, crc_status = unwrap_and_verify_crc(raw_cmd)

        if crc_status == "bad_crc":
            self._rx_stats["crc_fail"] += 1
            self.get_logger().warn(f"❌ CRC mismatch -> drop packet (len={len(line)}B): {line}")
            self._dbg(f"RX_BAD crc len={len(line)} : {line[:120]}")
            return

        if cmd is None:
            self.get_logger().warn("⚠️ Invalid packet format")
            return

        # Commands MUST be CRC-wrapped. Reject legacy/plain packets so line noise (or a
        # spoofed frame) can never trigger offboard/land/mission.
        if crc_status != "ok_crc":
            self.get_logger().warn(f"⚠️ Rejecting non-CRC command (status={crc_status})")
            return

        recv_crc = raw_cmd.get("crc32", "")
        self._rx_stats["ok"] += 1
        self.get_logger().info(
            f"✅ RX CRC OK={recv_crc} ({len(line)}B) payload={json.dumps(cmd, separators=(',', ':'))}"
        )
        self._dbg(f"RX_OK len={len(line)} op={cmd.get('op') or cmd.get('cmd') or '?'} "
                  f"seq={cmd.get('seq')}")

        # Ground session id: if it changed, the ground app restarted (its seq counter went
        # back to 0) — clear the dedupe cache so a fresh seq isn't mistaken for a duplicate.
        sid = cmd.get("sid", None)
        if self._idem.reset_session(sid):
            self.get_logger().info(f"🔄 New ground session sid={sid} — dedupe cache cleared")

        rx_seq = cmd.get("seq", None)
        if not isinstance(rx_seq, int):
            rx_seq = None

        # Idempotency guard: a retried command carries the SAME seq. If already handled,
        # re-send the cached ACK (so the ground stops retrying) but do NOT execute again.
        dup_status, cached = self._idem.check(rx_seq)
        if dup_status == "dup_acked":
            self._send_json(cached)
            self.get_logger().info(f"↩️ Duplicate seq={rx_seq}: re-ACK, skip re-exec")
            return
        if dup_status == "dup_inflight":
            self.get_logger().info(f"↩️ Duplicate seq={rx_seq}: in-flight, skip")
            return

        if "op" in cmd or "mission_op" in cmd:
            self._handle_mission_op(cmd, rx_seq, sid)
            return

        items = cmd.get("waypoints")
        if isinstance(items, list):
            try:
                parsed, err = parse_waypoints(items)

                # Accept ONLY a fully-valid, non-empty mission — otherwise reject so the
                # ground never treats a broken/empty upload as success.
                if parsed is not None:
                    self._publish_mission(parsed)
                    total = len(parsed)
                    self.get_logger().info(f"Published {total}/{total} waypoints to /mission_path")
                    self._send_ack({
                        "event": "uploaded",
                        "status": True,
                        "count": total,
                        "total": total,
                        "msg": "mission uploaded",
                    }, rx_seq=rx_seq, sid=sid)
                    # auto-OFFBOARD carries the mission seq so the ground can trace it.
                    self.call_mode_service(ModeSignal.Request.OFFBOARD, "OFFBOARD", ack_seq=rx_seq, ack_sid=sid)
                else:
                    total = len(items)
                    self.get_logger().warn(f"Rejecting mission: {err}")
                    self._send_ack({
                        "event": "uploaded",
                        "status": False,
                        "count": 0,
                        "total": total,
                        "error": err,
                    }, rx_seq=rx_seq, sid=sid)
            except Exception as e:
                self.get_logger().error(f"Publish error: {e}")
                self._send_ack({
                    "event": "uploaded",
                    "status": False,
                    "error": str(e),
                }, rx_seq=rx_seq, sid=sid)
            return

        c = str(cmd.get("cmd", "")).strip().lower()
        if c == "offboard":
            self.call_mode_service(ModeSignal.Request.OFFBOARD, "OFFBOARD", ack_seq=rx_seq, ack_sid=sid)
            return
        if c == "land":
            self.call_mode_service(ModeSignal.Request.LAND, "LAND", ack_seq=rx_seq, ack_sid=sid)
            return

        self.get_logger().info("Ignoring invalid command")


def _safe_rclpy_shutdown():
    # Under ros2 launch, SIGINT may already shut the context down before our
    # finally block runs. Calling rclpy.shutdown() again raises RCLError and
    # turns a clean Ctrl+C into exit code 1.
    try:
        if rclpy.ok():
            rclpy.shutdown()
    except Exception as e:
        if "rcl_shutdown already called" not in str(e):
            print(f"⚠️ rclpy shutdown error: {e}")


def main(args=None):
    rclpy.init(args=args)
    try:
        node = LoraDrone()
    except Exception as e:
        print(f"❌ LoraDrone init failed: {e}")
        _safe_rclpy_shutdown()
        return
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.close()
        except Exception as e:
            print(f"⚠️ serial close error: {e}")
        try:
            node.destroy_node()
        except Exception as e:
            print(f"⚠️ node destroy error: {e}")
        _safe_rclpy_shutdown()


if __name__ == '__main__':
    main()
