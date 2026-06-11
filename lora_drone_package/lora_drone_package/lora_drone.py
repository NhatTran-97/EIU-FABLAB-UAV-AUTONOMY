import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, TwistStamped
import serial, json, threading, time
import os
from collections import deque
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy
from threading import Lock
from nav_msgs.msg import Path
from sensor_msgs.msg import NavSatFix, BatteryState
from custom_msgs.srv import ModeSignal
from mavros_msgs.msg import State
import math
import zlib

"""
LORA_DRONE_TELEMETRY_INTERVAL=0.5 python3 lora_drone.py
LORA_DRONE_TELEMETRY_INTERVAL=0.4 python3 lora_drone.py
"""

# Verbose LoRa packet logging is opt-in to avoid flooding ROS logs.
# Enable with: LORA_DRONE_DEBUG=1
DEBUG = os.environ.get("LORA_DRONE_DEBUG", "0") == "1"


def _clean_json_str(s: str) -> str:
    # Tolerate leading/trailing LoRa line noise: keep only the outermost {...}.
    start = s.find("{")
    end = s.rfind("}")
    if start != -1 and end != -1 and end > start:
        return s[start:end + 1]
    return ""


def _is_finite_num(x):
    try:
        return isinstance(x, (int, float)) and math.isfinite(float(x))
    except Exception:
        return False


def _canonical_json_bytes(obj: dict) -> bytes:
    return json.dumps(obj, separators=(",", ":"), sort_keys=True).encode("utf-8")


def _crc32_hex_from_obj(obj: dict) -> str:
    return f"{zlib.crc32(_canonical_json_bytes(obj)) & 0xFFFFFFFF:08X}"


def _wrap_with_crc(payload: dict) -> dict:
    return {
        "payload": payload,
        "crc32": _crc32_hex_from_obj(payload),
    }


def _unwrap_and_verify_crc(data: dict):
    """
    Return (payload_dict, status)
      status:
        - "ok_crc"     : wrapped packet with valid CRC
        - "plain"      : legacy packet without CRC wrapper
        - "bad_crc"    : wrapped packet but CRC mismatch / malformed
        - "invalid"    : not a dict
    """
    if not isinstance(data, dict):
        return None, "invalid"

    if "payload" in data or "crc32" in data:
        payload = data.get("payload")
        recv_crc = data.get("crc32")
        if not isinstance(payload, dict) or not isinstance(recv_crc, str):
            return None, "bad_crc"

        calc_crc = _crc32_hex_from_obj(payload)
        if recv_crc.upper() != calc_crc:
            return None, "bad_crc"

        return payload, "ok_crc"

    return data, "plain"


class LoraDrone(Node):
    def __init__(self):
        super().__init__('LoraDrone')

        self.pose = None
        self.lat = None
        self.lon = None
        self.alt = None
        self.last_percentage = None
        self.last_voltage = None
        self.serial_lock = Lock()
        self._reopen_lock = Lock()
        self._stop_event = threading.Event()
        self.serial_path = os.environ.get('LORA_DRONE_PORT', '/dev/lora_drone')
        self.serial_baudrate = int(os.environ.get('LORA_DRONE_BAUD', '9600'))
        # 0.5s = 2 Hz. Safe on the 19.2kbps E32 link (~500 B/s capacity, ~24% used).
        # Override with LORA_DRONE_TELEMETRY_INTERVAL; do not go below ~0.3 (GPS packets
        # approach the air-rate ceiling). On a 2.4kbps link this must be >=2.0.
        self.telemetry_interval = max(0.2, float(os.environ.get("LORA_DRONE_TELEMETRY_INTERVAL", "1.0")))
        self.vel = None
        self.current_mode = "UNKNOWN"
        self.gps_ok = False

        # Telemetry compaction: slow-changing fields (state, battery) are only (re)sent
        # when they change or on a periodic full refresh, keeping steady-state packets
        # small so we can run a high rate cheaply. Position/speed/gps_ok go every cycle.
        self._tlm_last_state = None
        self._tlm_last_batt = None
        self._tlm_last_full_at = 0.0
        self._tlm_full_refresh = float(os.environ.get("LORA_DRONE_TELEMETRY_FULL_REFRESH", "15.0"))

        # Idempotency: remember recently handled command seqs (+ their ACK payload) so a
        # retried command (ground re-sends the SAME seq on ACK loss) is re-ACKed but NOT
        # executed twice — no double mission publish, no double mode trigger.
        self._proc_lock = Lock()
        self._processed = {}            # seq -> ACK payload (None while in-flight)
        self._processed_order = deque()
        self._proc_max = 128
        self._cur_sid = None            # ground session id; dedupe cache reset when it changes
        self._last_gps_at = 0.0         # monotonic time of last fresh GPS fix
        self._missions = {}             # mission_id -> chunk assembly state
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
        self.ser = None
        for attempt in range(1, 6):
            try:
                self.ser = self._open_serial_once()
                self.get_logger().info(f"✅ Serial connected at {self.serial_path} (attempt {attempt})")
                break
            except Exception as e:
                self.get_logger().error(f"❌ Open serial failed ({attempt}/5): {e}")
                time.sleep(1.0)
        if self.ser is None:
            self.get_logger().fatal(f"❌ Cannot open {self.serial_path} after retries — aborting node init")
            raise RuntimeError(f"Cannot open serial {self.serial_path}")

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

        threading.Thread(target=self.read_serial, daemon=True).start()
        threading.Thread(target=self.send_pose_loop, daemon=True).start()

    def _open_serial_once(self):
        ser = serial.Serial(
            self.serial_path,
            self.serial_baudrate,
            timeout=1,
            write_timeout=0.5,
        )
        try:
            ser.reset_input_buffer()
            ser.reset_output_buffer()
        except Exception:
            pass
        return ser

    def _reopen_serial(self, reason=""):
        if self._stop_event.is_set() or not rclpy.ok():
            return False
        if not self._reopen_lock.acquire(blocking=False):
            return False
        try:
            self.get_logger().warn(f"🔁 Reopening LoRa serial{': ' + reason if reason else ''}")
            with self.serial_lock:
                try:
                    if self.ser and self.ser.is_open:
                        self.ser.close()
                except Exception:
                    pass

                for attempt in range(1, 4):
                    try:
                        self.ser = self._open_serial_once()
                        self.get_logger().info(
                            f"✅ Serial reconnected at {self.serial_path} (attempt {attempt})"
                        )
                        return True
                    except Exception as e:
                        self.get_logger().error(f"❌ Serial reopen failed ({attempt}/3): {e}")
                        time.sleep(0.5)
            return False
        finally:
            self._reopen_lock.release()

    def close(self):
        self._stop_event.set()
        with self.serial_lock:
            try:
                if self.ser and self.ser.is_open:
                    self.ser.close()
            except Exception as e:
                print(f"⚠️ serial close error: {e}")

    def _send_json(self, obj: dict, log: bool = True, priority: bool = True, wrap: bool = True):
        try:
            if wrap:
                frame = _wrap_with_crc(obj)
            else:
                # Telemetry goes out un-wrapped (no CRC) to save ~25 bytes/packet on the
                # bandwidth-starved LoRa link. The ground accepts "plain" packets; a
                # corrupted telemetry frame just fails JSON parse and is skipped. Commands
                # (ground->drone) still REQUIRE CRC — only this display-only path drops it.
                frame = obj
            frame_text = json.dumps(frame, separators=(",", ":"))
            frame_bytes = (frame_text + "\n").encode('utf-8')

            if priority:
                with self.serial_lock:
                    if not self.ser or not self.ser.is_open:
                        raise serial.SerialException("serial is not open")
                    self.ser.write(frame_bytes)
            else:
                # Telemetry is best-effort. Do not queue behind ACK/control packets on a
                # weak LoRa link; skip this cycle so command responses stay low-latency.
                if not self.serial_lock.acquire(blocking=False):
                    return False
                try:
                    if not self.ser or not self.ser.is_open:
                        raise serial.SerialException("serial is not open")
                    self.ser.write(frame_bytes)
                finally:
                    self.serial_lock.release()

            if log:
                crc = frame.get("crc32", "-") if wrap else "-"
                self.get_logger().info(
                    f"📤 TX CRC={crc} payload={json.dumps(obj, separators=(',', ':'))}"
                )
            return True
        except (serial.SerialTimeoutException, serial.SerialException, OSError) as e:
            self.get_logger().error(f"❌ JSON send serial error: {e}")
            self._reopen_serial(str(e))
            return False
        except Exception as e:
            self.get_logger().error(f"❌ JSON send error: {e}")
            return False

    def _cache_ack(self, seq, payload):
        # Cache the FIRST ACK produced for a seq so a duplicate retry can be re-ACKed
        # without re-executing. Do not overwrite — e.g. the auto-OFFBOARD that follows a
        # mission must not clobber the mission "uploaded" ACK that shares the same seq.
        if seq is None:
            return
        with self._proc_lock:
            if seq in self._processed and self._processed[seq] is None:
                self._processed[seq] = payload

    def _send_ack(self, payload, rx_seq=None, sid=None):
        if rx_seq is not None:
            payload["ack_seq"] = rx_seq
        if sid is not None:
            payload["sid"] = sid
        self._send_json(payload)
        self._cache_ack(rx_seq, payload)

    def _parse_waypoints(self, items):
        parsed = []
        if not isinstance(items, list) or not items:
            return None, "empty waypoint list"

        for wp in items:
            if not isinstance(wp, dict):
                return None, "waypoint must be object"
            if not all(_is_finite_num(wp.get(k)) for k in ("x", "y", "z")):
                return None, "waypoint x/y/z must be finite numbers"
            z = float(wp["z"])
            if z < 0.0:
                return None, "waypoint z must be non-negative"
            parsed.append({
                "x": float(wp["x"]),
                "y": float(wp["y"]),
                "z": z,
            })

        return parsed, ""

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

        op = str(cmd.get("op") if "op" in cmd else cmd.get("mission_op", "")).strip().lower()
        mission_id = cmd.get("mi") if "mi" in cmd else cmd.get("mission_id", None)
        if not isinstance(mission_id, str) or not mission_id:
            error_event = {
                "begin": "mission_begin",
                "chunk": "mission_chunk",
                "commit": "uploaded",
            }.get(op, "uploaded")
            self._send_ack({
                "event": error_event,
                "status": False,
                "error": "invalid mission_id",
            }, rx_seq=rx_seq, sid=sid)
            return

        if op == "begin":
            total_chunks = cmd.get("tc") if "tc" in cmd else cmd.get("total_chunks", None)
            total_count = cmd.get("tn") if "tn" in cmd else cmd.get("total_count", None)
            if not isinstance(total_chunks, int) or total_chunks <= 0:
                self._send_ack({
                    "event": "mission_begin",
                    "status": False,
                    "mission_id": mission_id,
                    "error": "invalid total_chunks",
                }, rx_seq=rx_seq, sid=sid)
                return
            if not isinstance(total_count, int) or total_count <= 0:
                self._send_ack({
                    "event": "mission_begin",
                    "status": False,
                    "mission_id": mission_id,
                    "error": "invalid total_count",
                }, rx_seq=rx_seq, sid=sid)
                return

            self._missions[mission_id] = {
                "sid": sid,
                "coord": str(cmd.get("coord", "local")),
                "total_chunks": total_chunks,
                "total_count": total_count,
                "chunks": {},
                "created_at": time.monotonic(),
            }
            self.get_logger().info(
                f"🧩 mission {mission_id} BEGIN — expecting {total_chunks} chunks, "
                f"{total_count} waypoints"
            )
            self._dbg(f"MISSION {mission_id} BEGIN expect={total_chunks}chunks/{total_count}wps")
            self._send_ack({
                "event": "mission_begin",
                "status": True,
                "mission_id": mission_id,
                "total_chunks": total_chunks,
                "total_count": total_count,
                "msg": "mission begin accepted",
            }, rx_seq=rx_seq, sid=sid)
            return

        if op == "chunk":
            session = self._missions.get(mission_id)
            chunk_index = cmd.get("ci") if "ci" in cmd else cmd.get("chunk_index", None)
            total_chunks = cmd.get("tc") if "tc" in cmd else cmd.get("total_chunks", None)
            if session is None:
                self._send_ack({
                    "event": "mission_chunk",
                    "status": False,
                    "mission_id": mission_id,
                    "error": "mission not started",
                }, rx_seq=rx_seq, sid=sid)
                return
            if session["sid"] != sid:
                self._send_ack({
                    "event": "mission_chunk",
                    "status": False,
                    "mission_id": mission_id,
                    "error": "mission sid mismatch",
                }, rx_seq=rx_seq, sid=sid)
                return
            if total_chunks != session["total_chunks"]:
                self._send_ack({
                    "event": "mission_chunk",
                    "status": False,
                    "mission_id": mission_id,
                    "error": "total_chunks mismatch",
                }, rx_seq=rx_seq, sid=sid)
                return
            if not isinstance(chunk_index, int) or not (0 <= chunk_index < session["total_chunks"]):
                self._send_ack({
                    "event": "mission_chunk",
                    "status": False,
                    "mission_id": mission_id,
                    "error": "invalid chunk_index",
                }, rx_seq=rx_seq, sid=sid)
                return

            parsed, err = self._parse_waypoints(cmd.get("wps") if "wps" in cmd else cmd.get("waypoints"))
            if parsed is None:
                self._send_ack({
                    "event": "mission_chunk",
                    "status": False,
                    "mission_id": mission_id,
                    "chunk_index": chunk_index,
                    "error": err,
                }, rx_seq=rx_seq, sid=sid)
                return

            existing = session["chunks"].get(chunk_index)
            if existing is not None and existing != parsed:
                self._send_ack({
                    "event": "mission_chunk",
                    "status": False,
                    "mission_id": mission_id,
                    "chunk_index": chunk_index,
                    "error": "chunk content changed",
                }, rx_seq=rx_seq, sid=sid)
                return

            session["chunks"][chunk_index] = parsed
            have = sorted(session["chunks"].keys())
            missing = [i for i in range(session["total_chunks"]) if i not in session["chunks"]]
            self.get_logger().info(
                f"🧩 mission {mission_id}: chunk {chunk_index} OK ({len(parsed)} wps) — "
                f"have {have}/{session['total_chunks']}, still missing {missing}"
            )
            self._dbg(f"MISSION {mission_id} CHUNK {chunk_index} OK ({len(parsed)}wps) "
                      f"have={have} missing={missing}")
            self._send_ack({
                "event": "mission_chunk",
                "status": True,
                "mission_id": mission_id,
                "chunk_index": chunk_index,
                "received_chunks": len(session["chunks"]),
                "total_chunks": session["total_chunks"],
                "msg": "chunk accepted",
            }, rx_seq=rx_seq, sid=sid)
            return

        if op == "commit":
            session = self._missions.get(mission_id)
            if session is None:
                self._send_ack({
                    "event": "uploaded",
                    "status": False,
                    "mission_id": mission_id,
                    "error": "mission not started",
                }, rx_seq=rx_seq, sid=sid)
                return
            if session["sid"] != sid:
                self._send_ack({
                    "event": "uploaded",
                    "status": False,
                    "mission_id": mission_id,
                    "error": "mission sid mismatch",
                }, rx_seq=rx_seq, sid=sid)
                return

            missing = [i for i in range(session["total_chunks"]) if i not in session["chunks"]]
            if missing:
                self.get_logger().warn(
                    f"❌ mission {mission_id} COMMIT rejected — missing chunks {missing} "
                    f"(have {sorted(session['chunks'].keys())}/{session['total_chunks']}). "
                    f"Those chunks never arrived intact — check RX truncation logs above."
                )
                self._dbg(f"MISSION {mission_id} COMMIT-FAIL missing={missing} "
                          f"have={sorted(session['chunks'].keys())}")
                self._send_ack({
                    "event": "uploaded",
                    "status": False,
                    "mission_id": mission_id,
                    "error": f"missing chunks: {missing}",
                }, rx_seq=rx_seq, sid=sid)
                return

            waypoints = []
            for i in range(session["total_chunks"]):
                waypoints.extend(session["chunks"][i])

            if len(waypoints) != session["total_count"]:
                self._send_ack({
                    "event": "uploaded",
                    "status": False,
                    "mission_id": mission_id,
                    "count": len(waypoints),
                    "total": session["total_count"],
                    "error": "waypoint count mismatch",
                }, rx_seq=rx_seq, sid=sid)
                return

            try:
                self._publish_mission(waypoints)
                self.get_logger().info(
                    f"Published mission {mission_id}: {len(waypoints)} waypoints "
                    f"in {session['total_chunks']} chunks"
                )
                self._dbg(f"MISSION {mission_id} PUBLISHED {len(waypoints)}wps -> /mission_path "
                          f"(OK, full upload)")
                self._missions.pop(mission_id, None)
                self._send_ack({
                    "event": "uploaded",
                    "status": True,
                    "mission_id": mission_id,
                    "count": len(waypoints),
                    "total": session["total_count"],
                    "msg": "mission uploaded",
                }, rx_seq=rx_seq, sid=sid)
                self.call_mode_service(ModeSignal.Request.OFFBOARD, "OFFBOARD", ack_seq=rx_seq, ack_sid=sid)
            except Exception as e:
                self._send_ack({
                    "event": "uploaded",
                    "status": False,
                    "mission_id": mission_id,
                    "error": str(e),
                }, rx_seq=rx_seq, sid=sid)
                self.get_logger().error(f"Publish error: {e}")
            return

        self._send_ack({
            "event": "uploaded",
            "status": False,
            "mission_id": mission_id,
            "error": f"unknown mission_op: {op}",
        }, rx_seq=rx_seq, sid=sid)

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
            p = self._estimate_percent_from_voltage(v, cells=4)
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
    def _estimate_percent_from_voltage(self, voltage, cells=4, v_full=4.2, v_empty=3.3):
        if not (isinstance(voltage, (int, float)) and math.isfinite(voltage)) or cells <= 0:
            return None
        v_cell = float(voltage) / float(cells)
        if v_cell >= v_full:
            return 100.0
        if v_cell <= v_empty:
            return 0.0
        return (v_cell - v_empty) / (v_full - v_empty) * 100.0

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
        # timeout fires first wins. Without the timeout, a hung service would leave
        # _processed[seq]=None forever and the ground would never receive an ACK.
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

                # Periodically force a full packet so a ground that connects mid-stream
                # still gets state/battery within _tlm_full_refresh seconds.
                force_full = (now - self._tlm_last_full_at) >= self._tlm_full_refresh

                # Compact, un-CRC'd telemetry. Trim precision (cm / ~1 m is plenty for a
                # map display) and omit slow-changing fields unless they changed.
                data = {"hb": 1, "gps_ok": int(self.gps_ok and gps_fresh)}

                if self.pose is not None:
                    data["x"] = round(self.pose.position.x, 2)
                    data["y"] = round(self.pose.position.y, 2)
                    data["z"] = round(self.pose.position.z, 2)

                if gps_fresh and self.lat is not None and self.lon is not None and self.alt is not None:
                    data["lat"] = round(self.lat, 5)
                    data["lon"] = round(self.lon, 5)
                    data["alt"] = round(self.alt, 1)

                if self.vel is not None:
                    vx, vy, vz = self.vel
                    data["speed"] = round(math.sqrt(vx * vx + vy * vy + vz * vz), 1)

                # state: only when it changed (or refresh). Ground keeps the last value.
                state = self.current_mode
                if force_full or state != self._tlm_last_state:
                    data["state"] = state

                # battery: quantize (int %, 0.1 V) then send only on change (or refresh).
                batt = None
                if self.last_percentage is not None and self.last_voltage is not None:
                    batt = (int(round(self.last_percentage)), round(self.last_voltage, 1))
                    if force_full or batt != self._tlm_last_batt:
                        data["battery"] = {"percent": batt[0], "voltage": batt[1]}

                sent = self._send_json(data, log=DEBUG, priority=False, wrap=False)

                # Advance the "last sent" trackers only when the packet actually went out,
                # so a skipped/failed cycle doesn't suppress the next change.
                if sent:
                    self._tlm_last_state = state
                    self._tlm_last_batt = batt
                    if force_full:
                        self._tlm_last_full_at = now
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

    def read_serial(self):
        while rclpy.ok() and not self._stop_event.is_set():
            try:
                # Do NOT hold serial_lock around the blocking readline(): it would
                # stall every writer (telemetry @0.4s + all ACKs) for up to the read
                # timeout (1s). Reads and writes on a serial port are independent;
                # serial_lock only serializes the writers (see _send_json).
                if not self.ser or not self.ser.is_open:
                    self._reopen_serial("serial closed")
                    time.sleep(0.2)
                    continue
                line = self.ser.readline().decode('utf-8', errors='replace').strip()
                if not line:
                    continue
                if DEBUG:
                    self.get_logger().info(f"[DRONE][RX_LINE] {line}")
                self.handle_command(line)
                self._maybe_log_rx_stats()
            except (serial.SerialException, OSError) as e:
                if self._stop_event.is_set() or not rclpy.ok():
                    break
                self.get_logger().error(f"❌ Serial read error: {e}")
                self._reopen_serial(str(e))
                time.sleep(0.2)
            except Exception as e:
                self.get_logger().error(f"❌ Serial read error: {e}")
                time.sleep(0.2)

    def handle_command(self, line: str):
        if line in ("ON", "OFF"):
            self.get_logger().info(f"ℹ️ Control token ignored: {line}")
            return

        clean = _clean_json_str(line)
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

        cmd, crc_status = _unwrap_and_verify_crc(raw_cmd)

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
        if sid is not None and sid != self._cur_sid:
            with self._proc_lock:
                self._processed.clear()
                self._processed_order.clear()
                self._cur_sid = sid
            self.get_logger().info(f"🔄 New ground session sid={sid} — dedupe cache cleared")

        rx_seq = cmd.get("seq", None)
        if not isinstance(rx_seq, int):
            rx_seq = None

        # Idempotency guard: a retried command carries the SAME seq. If already handled,
        # re-send the cached ACK (so the ground stops retrying) but do NOT execute again.
        if rx_seq is not None:
            with self._proc_lock:
                if rx_seq in self._processed:
                    cached = self._processed[rx_seq]
                    if cached is not None:
                        self._send_json(cached)
                        self.get_logger().info(f"↩️ Duplicate seq={rx_seq}: re-ACK, skip re-exec")
                    else:
                        self.get_logger().info(f"↩️ Duplicate seq={rx_seq}: in-flight, skip")
                    return
                if len(self._processed_order) >= self._proc_max:
                    self._processed.pop(self._processed_order.popleft(), None)
                self._processed_order.append(rx_seq)
                self._processed[rx_seq] = None   # in-flight; ACK cached when produced

        if "op" in cmd or "mission_op" in cmd:
            self._handle_mission_op(cmd, rx_seq, sid)
            return

        items = cmd.get("waypoints")
        if isinstance(items, list):
            try:
                parsed, err = self._parse_waypoints(items)

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
