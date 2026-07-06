import os
import serial
import json
import time
import threading
import random

# Wire-protocol codec + id/chunk helpers live in protocol.py now. Re-imported here (so the
# names stay importable from `control` — test/test_logic.py + main.py depend on that).
from protocol import (
    DEBUG,
    log_dbg,
    _is_num,
    _clean_json_str,
    _as_true,
    _canonical_json_bytes,
    _crc32_hex_from_obj,
    _wrap_with_crc,
    _make_session_id,
    _make_mission_id,
    _chunk_list,
    _unwrap_and_verify_crc,
)
from telemetry_parser import parse_frame
from commands import (
    cmd_offboard, cmd_land, mission_begin, mission_chunk, mission_commit,
)


class GroundController:
    def __init__(self, port='/dev/lora_ground', baudrate=9600, gui_bridge=None):
        self.port = port
        self.baudrate = baudrate
        self.ser = None
        self.waypoints = []
        self.received_thread = None
        self.received = False
        self.gui_bridge = gui_bridge

        self._last_hb = 0.0
        self._last_seen = 0.0
        self._link_ok = False
        self._hb_timeout = float(os.environ.get("GROUND_GUI_HB_TIMEOUT", "15.0"))
        self._hb_thread = None

        self._last_ack_mode = None
        self._last_ack_event = None
        self._last_ack_sid = None
        self._last_ack_seq = None
        self._last_ack_status = None
        self._last_ack_msg = ""
        self._last_ack_at = 0.0

        self._tx_seq = 0
        self._seq_lock = threading.Lock()
        self._session_id = _make_session_id()

        # Serialize writes so frames from concurrent commands/retries never interleave on the wire.
        self._tx_lock = threading.Lock()
        # Guard the shared ACK snapshot (written by the reader, read by sender threads).
        self._ack_lock = threading.Lock()
        # Make serial (re)open atomic against rapid CONNECT double-clicks.
        self._conn_lock = threading.Lock()
        # Ensure only one telemetry reader can ever start.
        self._reader_lock = threading.Lock()
        self._reader_running = False
        self._user_requested_stop = False
        self._auto_reconnect = os.environ.get("GROUND_GUI_AUTO_RECONNECT", "1") != "0"
        self._reconnect_lock = threading.Lock()
        self._reconnect_thread = None
        self._reconnect_initial_delay = float(os.environ.get("GROUND_GUI_RECONNECT_INITIAL_DELAY", "1.0"))
        self._reconnect_max_delay = float(os.environ.get("GROUND_GUI_RECONNECT_MAX_DELAY", "8.0"))
        # Bumped on connect()/stop() to cancel in-flight send/retry workers, so a worker
        # from before a disconnect/reconnect never writes into the new serial session.
        self._tx_epoch = 0

        self._last_global_fix_at = 0.0
        self._latest_global_position = None
        # A command is rejected if no position telemetry arrived within this window. Sized to
        # absorb real-world LoRa packet loss: at a 1.5s telemetry interval this tolerates ~2
        # consecutive dropped packets before rejecting. Override via GROUND_GUI_POSITION_TIMEOUT.
        self._global_fix_timeout = float(os.environ.get("GROUND_GUI_POSITION_TIMEOUT", "5.0"))
        self._last_local_fix_at = 0.0   # local-position freshness (indoor/local flight, no GPS)
        self._mission_chunk_size = max(1, int(os.environ.get("GROUND_GUI_MISSION_CHUNK_SIZE", "2")))
        self._send_off_on_stop = os.environ.get("GROUND_GUI_SEND_OFF_ON_STOP", "0") == "1"
        self._mission_tx_lock = threading.Lock()
        self._mission_tx_active = False
        self._last_rx_at = 0.0          # monotonic time of last byte received from drone

    def connect(self):
        with self._conn_lock:
            if self.ser is None or not self.ser.is_open:
                try:
                    self.ser = serial.Serial(self.port, self.baudrate, timeout=0.2, write_timeout=0.5)
                    self.ser.reset_input_buffer()
                    self.ser.reset_output_buffer()
                    time.sleep(0.2)
                    self._tx_epoch += 1   # fresh session: invalidate any stale TX workers
                    print(f"✅ LoRa connected at {self.port} @ {self.baudrate}")
                except Exception as e:
                    print(f"Connected failed: {e}")
                    self.ser = None

    def _send_on(self):
        if self.ser and self.ser.is_open:
            try:
                print("[INFO] Sending ON command to LoRa")
                self.ser.write(b'ON\n')
                self.ser.flush()
                return True
            except Exception as e:
                print(f"[ERROR] Serial port is not open: {e}")
        else:
            print("[ERROR] Serial is not open.")
        return False

    def start(self):
        self._user_requested_stop = False
        self.connect()
        self._send_on()

    def stop(self):
        self._user_requested_stop = True
        self.received = False
        self._reader_running = False
        self._tx_epoch += 1   # cancel any in-flight send/retry workers

        if self.received_thread:
            self.received_thread.join(timeout=1.0)
            self.received_thread = None 
        if self._hb_thread:
            self._hb_thread.join(timeout=1.0)
            self._hb_thread = None 
            
        self._last_hb = 0.0
        self._last_seen = 0.0
        self._last_global_fix_at = 0.0
        self._latest_global_position = None

        if self._link_ok:
            self._link_ok = False
            self._emit_link(False)

        if self.ser and self.ser.is_open:
            if self._send_off_on_stop:
                try:
                    print("[INFO] Sending OFF command to LoRa")
                    self.ser.write(b'OFF\n')
                    self.ser.flush()
                except Exception as e:
                    print(f"Failed to send OFF: {e}")
            else:
                print("[INFO] Skipping OFF command; closing serial only")
            try:
                self.ser.close()
                print("[INFO] Serial connection closed")
            except Exception as e:
                print(f"Error closing serial: {e}")
        else:
            print("[INFO] Serial already closed or not opened.")

    def _close_serial_for_reconnect(self):
        with self._conn_lock:
            self._tx_epoch += 1
            ser = self.ser
            self.ser = None
            if ser and ser.is_open:
                try:
                    ser.close()
                    print("[INFO] Serial closed for reconnect")
                except Exception as e:
                    print(f"Error closing serial for reconnect: {e}")

    def _start_reconnect(self, reason=""):
        if not self._auto_reconnect or self._user_requested_stop or not self.received:
            return
        with self._reconnect_lock:
            if self._reconnect_thread and self._reconnect_thread.is_alive():
                return
            self._reconnect_thread = threading.Thread(
                target=self._reconnect_loop,
                args=(reason,),
                daemon=True,
            )
            self._reconnect_thread.start()

    def _reconnect_loop(self, reason=""):
        delay = max(0.1, self._reconnect_initial_delay)
        max_delay = max(delay, self._reconnect_max_delay)
        print(f"[INFO] Auto reconnect started{': ' + reason if reason else ''}")

        while self.received and not self._user_requested_stop:
            recent_hb = self._last_hb > 0 and (time.monotonic() - self._last_hb) <= self._hb_timeout
            if recent_hb:
                print("[INFO] Auto reconnect stopped: heartbeat restored")
                return

            time.sleep(delay)
            if not self.received or self._user_requested_stop:
                return

            print(f"[INFO] Auto reconnect attempt via {self.port}")
            self._close_serial_for_reconnect()
            self.connect()
            self._send_on()

            # Give the fresh link a full heartbeat window before deciding to reconnect
            # again. Without this, the loop re-closes the connection before the drone
            # has time to respond, causing a rapid open/close/open/close cycle.
            grace_end = time.monotonic() + self._hb_timeout
            while time.monotonic() < grace_end:
                if not self.received or self._user_requested_stop:
                    return
                if self._last_hb > 0 and (time.monotonic() - self._last_hb) <= self._hb_timeout:
                    print("[INFO] Auto reconnect stopped: heartbeat received after reconnect")
                    return
                time.sleep(0.5)

            delay = min(max_delay, delay * 1.5)

    def set_gui_bridge(self, bridge):
        self.gui_bridge = bridge

    def _has_recent_global_position(self) -> bool:
        if self._last_global_fix_at <= 0.0:
            return False
        return (time.monotonic() - self._last_global_fix_at) <= self._global_fix_timeout

    def _has_recent_local_position(self) -> bool:
        if self._last_local_fix_at <= 0.0:
            return False
        return (time.monotonic() - self._last_local_fix_at) <= self._global_fix_timeout

    def _has_recent_position(self) -> bool:
        # Accept EITHER a recent GPS fix OR a recent local-position fix. A drone flying on
        # local position (indoor / no GPS) still streams x/y/z — a valid position reference
        # for OFFBOARD/mission — so we must not require GPS specifically.
        return self._has_recent_global_position() or self._has_recent_local_position()

    def _reject_operation_no_gps(self, op_name: str):
        msg = f"{op_name} rejected: no recent position (GPS or local) at ground side"
        if op_name == "SENDMISSION":
            self._reject_mission(msg)
            return
        print(f"{msg}")
        if self.gui_bridge and hasattr(self.gui_bridge, "mode_push"):
            try:
                self.gui_bridge.mode_push(False, op_name, msg)
            except Exception as e:
                print(f"bridge.mode_push error: {e}")

    def _reject_mission(self, msg: str):
        print(msg)
        if self.gui_bridge and hasattr(self.gui_bridge, "mission_ack"):
            try:
                self.gui_bridge.mission_ack(False, msg)
            except Exception as e:
                print(f"bridge.mission_ack error: {e}")

    def _next_seq(self) -> int:
        with self._seq_lock:
            self._tx_seq += 1
            return self._tx_seq

    def _prepare_frame(self, payload: dict):
        seq = self._next_seq()
        payload_with_seq = dict(payload)
        payload_with_seq["sid"] = self._session_id
        payload_with_seq["seq"] = seq
        frame = _wrap_with_crc(payload_with_seq)
        frame_bytes = (json.dumps(frame, separators=(",", ":")) + "\n").encode("utf-8")
        return seq, payload_with_seq, frame, frame_bytes

    def _send_frame_bytes(self, frame_bytes: bytes):
        if not self.ser or not self.ser.is_open:
            raise RuntimeError("Serial is not connected")
        with self._tx_lock:
            self.ser.write(frame_bytes)
            self.ser.flush()

    def _ack_matches(self, *, expect_event: str, sent_seq: int, since_time: float, expect_mode: str = None) -> bool:
        return self._ack_snapshot_if_matches(
            expect_event=expect_event,
            sent_seq=sent_seq,
            since_time=since_time,
            expect_mode=expect_mode
        ) is not None

    def _ack_snapshot_if_matches(self, *, expect_event: str, sent_seq: int, since_time: float, expect_mode: str = None):
        # Snapshot the shared ACK fields atomically so we never compare a half-updated ACK.
        with self._ack_lock:
            last_event = self._last_ack_event
            last_at = self._last_ack_at
            last_sid = self._last_ack_sid
            last_seq = self._last_ack_seq
            last_mode = self._last_ack_mode
            last_status = self._last_ack_status
            last_msg = self._last_ack_msg

        if last_event != expect_event:
            return None
        if last_at < since_time:
            return None
        if last_sid != self._session_id:
            return None

        # Enforce an exact ack_seq match — no legacy/wildcard ACKs. The drone always
        # echoes ack_seq (including auto-OFFBOARD), so an ACK without one, or with a
        # different seq, is not the reply to this command.
        if last_seq != sent_seq:
            return None

        if expect_mode is not None and last_mode != expect_mode:
            return None

        return {
            "status": bool(last_status),
            "msg": str(last_msg or ""),
            "event": last_event,
            "mode": last_mode,
            "seq": last_seq,
            "sid": last_sid,
        }

    def _wait_for_tx_slot(self, timeout: float = 1.5):
        """Block until the drone has JUST finished a TX cycle.

        The E32 is half-duplex: while drone TX'ing we can't land a clean packet.
        Drone telemetry arrives at _last_rx_at; if we send within 150 ms of that
        timestamp we're in the quiet window before the next drone TX (≈500 ms period,
        leaving ~350 ms slack even for our longest command frame).

        If no telemetry has been seen within `timeout` seconds we give up and send
        anyway (link might be interrupted — better to try than to stall forever).
        """
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            age = time.monotonic() - self._last_rx_at
            if 0.0 < age < 0.15:
                # Drone JUST finished TX'ing; add a tiny guard for E32 mode-switch.
                time.sleep(0.012)
                return
            time.sleep(0.015)
        # Timeout reached — proceed anyway.

    def _send_payload_wait_ack(self, payload: dict, *,
                               expect_event: str,
                               expect_mode: str = None,
                               tries: int = 2,
                               interval: float = 2.0):
        seq, payload_with_seq, frame, frame_bytes = self._prepare_frame(payload)
        epoch = self._tx_epoch

        for i in range(tries + 1):
            if self._tx_epoch != epoch:
                return False, "TX cancelled (link reset)", False
            if not self.ser or not self.ser.is_open:
                return False, "Serial is not connected", False

            # Wait until the drone has JUST finished a telemetry TX cycle so we land
            # in the "quiet window" on the half-duplex LoRa channel. Without this,
            # retries spaced at interval=2.0s are in EXACT phase with the 0.5s drone
            # telemetry period (2.0 = 4 × 0.5) and collide every single time.
            self._wait_for_tx_slot()
            # Small random jitter so retries never re-synchronize with drone cycle.
            time.sleep(random.uniform(0.01, 0.08))

            t0 = time.monotonic()
            try:
                self._send_frame_bytes(frame_bytes)
                # {n}B is the size the DRONE should receive intact — compare against the
                # "len=" in lora_drone's RX logs to spot truncation on the air.
                print(
                    f"[INFO] Sent {expect_event}"
                    f"{'/' + expect_mode if expect_mode else ''}"
                    f" seq={seq} crc={frame['crc32']} {len(frame_bytes)}B (attempt {i+1}/{tries+1})"
                )
            except Exception as e:
                print(f"❌ Send error: {e}")
                time.sleep(interval)
                continue

            while time.monotonic() - t0 < interval:
                if self._tx_epoch != epoch:
                    return False, "TX cancelled (link reset)", False
                ack = self._ack_snapshot_if_matches(
                    expect_event=expect_event,
                    sent_seq=seq,
                    since_time=t0,
                    expect_mode=expect_mode
                )
                if ack is not None:
                    return ack["status"], ack["msg"] or ("ACK OK" if ack["status"] else "ACK failed"), True
                time.sleep(0.05)

            # Inner window elapsed with no matching ACK — make each retry visible so a
            # field operator sees how many attempts a chunk needed (or that it never landed).
            print(
                f"⏳ No ACK for {expect_event} seq={seq} within {interval:.1f}s"
                f" — {'retrying' if i < tries else 'giving up'}"
            )

        return False, f"No ACK for {expect_event} (timeout)", False

    def _send_with_retry(self, payload: dict, *,
                         expect_event: str,
                         expect_mode: str = None,
                         tries: int = 2,
                         interval: float = 1.0,
                         on_fail_msg: str = "No ACK (timeout)"):
        seq, payload_with_seq, frame, frame_bytes = self._prepare_frame(payload)
        epoch = self._tx_epoch

        def worker():
            for i in range(tries + 1):
                if self._tx_epoch != epoch:
                    print(f"⏹️ TX cancelled (link reset) {expect_event} seq={seq}")
                    return
                if not self.ser or not self.ser.is_open:
                    break

                # Wait for the drone to finish its telemetry TX cycle before sending.
                # Without this the drone E32 is busy TX'ing when our packet arrives on
                # the half-duplex channel → it misses the packet head → parse fails.
                self._wait_for_tx_slot()
                time.sleep(random.uniform(0.01, 0.08))

                # Capture the send time *before* writing so an ACK that arrives
                # during the (blocking) write/flush is still matched.
                t0 = time.monotonic()
                try:
                    self._send_frame_bytes(frame_bytes)
                    print(
                        f"[INFO] Sent {expect_event}"
                        f"{'/' + expect_mode if expect_mode else ''}"
                        f" seq={seq} crc={frame['crc32']} {len(frame_bytes)}B (attempt {i+1}/{tries+1})"
                    )
                except Exception as e:
                    print(f"❌ Send error: {e}")
                    time.sleep(interval)
                    continue

                while time.monotonic() - t0 < interval:
                    if self._tx_epoch != epoch:
                        return
                    if self._ack_matches(
                        expect_event=expect_event,
                        sent_seq=seq,
                        since_time=t0,
                        expect_mode=expect_mode
                    ):
                        print(
                            f"✅ ACK received for {expect_event}"
                            f"{'/' + expect_mode if expect_mode else ''}"
                            f" seq={seq}"
                        )
                        return
                    time.sleep(0.05)

            if expect_event == "mode_push" and self.gui_bridge and hasattr(self.gui_bridge, "mode_push"):
                try:
                    self.gui_bridge.mode_push(False, expect_mode or "", on_fail_msg)
                except Exception as e:
                    print(f"⚠️ bridge.mode_push error: {e}")
            elif expect_event == "uploaded" and self.gui_bridge and hasattr(self.gui_bridge, "mission_ack"):
                try:
                    self.gui_bridge.mission_ack(False, on_fail_msg)
                except Exception as e:
                    print(f"⚠️ bridge.mission_ack error: {e}")
            else:
                print(
                    f"⚠️ ACK timeout for {expect_event}"
                    f"{'/' + expect_mode if expect_mode else ''}"
                    f" seq={seq}"
                )

        threading.Thread(target=worker, daemon=True).start()

    def _emit_link(self, ok: bool):
        if self.gui_bridge and hasattr(self.gui_bridge, "update_link"):
            try:
                self.gui_bridge.update_link(bool(ok))
            except Exception as e:
                print(f"⚠️ GUI bridge error (update_link): {e}")

    def _emit_state(self, state):
        if self.gui_bridge is None:
            return
        try:
            self.gui_bridge.update_state(str(state))
        except Exception as e:
            print(f"❌ GUI bridge error (state): {e}")
    def _emit_gps(self, ok:bool):
        if self.gui_bridge and hasattr(self.gui_bridge, "update_gps"):
            try:
                self.gui_bridge.update_gps(bool(ok))
            except Exception as e:
                print(f"GUI bridge error (gps): {e}")

    def _hb_watch(self, timeout=30.0, interval=30, grace=2):
        missed = 0
        while self.received:
            last = max(self._last_hb, getattr(self, "_last_seen", 0.0))
            dt = time.monotonic() - last
            if dt > timeout:
                missed += 1
                if self._link_ok and missed >= grace:
                    self._link_ok = False
                    self._emit_link(False)
                    self._start_reconnect(f"heartbeat timeout {dt:.1f}s")
            else:
                missed = 0
                if not self._link_ok and last > 0:
                    self._link_ok = True
                    self._emit_link(True)
            time.sleep(interval)

    def read_position_from_drone(self):
        with self._reader_lock:
            if self._reader_running or (self.received_thread and self.received_thread.is_alive()):
                print("[INFO] Telemetry reader already running")
                return True
            if not self.ser or not self.ser.is_open:
                print("⚠️ Serial is not connected")
                return False
            self._last_hb = 0.0
            self._last_seen = 0.0
            self._link_ok = False
            self._reader_running = True
            self.received = True

        if not self._hb_thread or not self._hb_thread.is_alive():
            self._hb_thread = threading.Thread(
                target=self._hb_watch,
                kwargs=dict(timeout=self._hb_timeout, interval=0.5, grace=2),
                daemon=True
            )
            self._hb_thread.start()

        def _read_loop():
            print("📡 Starting to receive telemetry from drone...")
            buffer = ""
            while self.received:
                try:
                    # Drain whatever is buffered for low latency; fall back to a
                    # short blocking read(1) (timeout=0.2s) when the line is idle.
                    ser = self.ser
                    if not ser or not ser.is_open:
                        self._start_reconnect("serial closed")
                        time.sleep(0.2)
                        continue

                    try:
                        waiting = ser.in_waiting
                    except (serial.SerialException, OSError, TypeError):
                        # TOCTOU: port was open at the is_open check above but the
                        # reconnect thread closed it before in_waiting could run.
                        # Fall through to the next iteration — self.ser will be fresh.
                        time.sleep(0.1)
                        continue
                    chunk = ser.read(waiting if isinstance(waiting, int) and waiting > 0 else 1)
                    if not chunk:
                        continue

                    log_dbg(f"[GROUND][RX_BYTES] {chunk!r}")
                    buffer += chunk.decode('utf-8', errors='replace')

                    if "\r" in buffer:
                        buffer = buffer.replace("\r\n", "\n").replace("\r", "\n")

                    while True:
                        nl = buffer.find("\n")
                        if nl == -1:
                            break

                        line = buffer[:nl]
                        buffer = buffer[nl + 1:]

                        line = line.strip()
                        if not line:
                            continue

                        clean_line = _clean_json_str(line)
                        if not clean_line:
                            log_dbg(f"Rejected: {line}")
                            continue

                        log_dbg(f"[RAW] {clean_line}")
                        try:
                            raw_data = json.loads(clean_line)
                        except json.JSONDecodeError:
                            log_dbg(f"JSON decode failed: {clean_line}")
                            continue

                        data, crc_status = _unwrap_and_verify_crc(raw_data)
                        log_dbg(f"[CRC_STATUS] {crc_status}")
                        log_dbg(f"[DATA] {data}")
                        if crc_status == "bad_crc":
                            log_dbg("CRC mismatch: drop packet")
                            continue
                        if data is None:
                            log_dbg("Invalid packet format")
                            continue

                        now = time.monotonic()
                        self._last_seen = now
                        self._last_rx_at = now      # track every packet for TX slot logic

                        # Pure parse (telemetry_parser.py); the side effects (ACK state under
                        # lock, GUI bridge calls, freshness timestamps, emits) stay here.
                        pf = parse_frame(data)

                        if pf.ack is not None:
                            a = pf.ack
                            with self._ack_lock:
                                self._last_ack_event = a.event
                                self._last_ack_mode = a.mode
                                self._last_ack_sid = a.sid
                                self._last_ack_seq = a.seq
                                self._last_ack_status = a.status
                                self._last_ack_msg = a.msg
                                self._last_ack_at = now

                            if a.event == "mode_push":
                                if self.gui_bridge:
                                    try:
                                        if hasattr(self.gui_bridge, "mode_push"):
                                            self.gui_bridge.mode_push(a.status, a.mode, a.msg)
                                        elif hasattr(self.gui_bridge, "modePush"):
                                            self.gui_bridge.modePush(a.status, a.mode, a.msg)
                                    except Exception as e:
                                        print(f"GUI bridge error (mode_push): {e}")
                            elif a.event == "uploaded":
                                print(
                                    f"📥 Upload ACK status={a.status}"
                                    f"{'' if a.seq is None else f' ack_seq={a.seq}'}"
                                )
                                if self.gui_bridge and hasattr(self.gui_bridge, "mission_ack"):
                                    try:
                                        self.gui_bridge.mission_ack(a.status, a.msg)
                                    except Exception as e:
                                        print(f"GUI bridge error (mission_ack): {e}")
                            else:  # mission_begin / mission_chunk
                                log_dbg(
                                    f"📥 {a.event} ACK status={a.status}"
                                    f"{'' if a.seq is None else f' ack_seq={a.seq}'}"
                                )

                        if pf.hb:
                            self._last_hb = now
                            if not self._link_ok:
                                self._link_ok = True
                                self._emit_link(True)
                        if pf.state is not None:
                            log_dbg(f"[GROUND][STATE_RX] {pf.state}")
                            self._emit_state(pf.state)
                        if pf.gps_ok is not None:
                            self._emit_gps(pf.gps_ok)

                        if pf.local is not None:
                            x, y, z = pf.local
                            self._last_local_fix_at = now   # local position is fresh
                            log_dbg(f"📥 Local position: x={x}, y={y}, z={z}")
                            if self.gui_bridge and hasattr(self.gui_bridge, "update_position"):
                                try:
                                    self.gui_bridge.update_position(x, y, z)
                                except Exception as e:
                                    print(f"GUI bridge error (pos): {e}")

                        if pf.global_pos is not None:
                            lat, lon, alt = pf.global_pos
                            self._last_global_fix_at = now
                            self._latest_global_position = (lat, lon, alt)
                            log_dbg(f"📥 Global position: lat={lat}, lon={lon}, alt={alt}")
                            if self.gui_bridge and hasattr(self.gui_bridge, "update_global_position"):
                                try:
                                    self.gui_bridge.update_global_position(lat, lon, alt)
                                except Exception as e:
                                    print(f"GUI bridge error (gps): {e}")

                        if pf.battery is not None and self.gui_bridge and hasattr(self.gui_bridge, "update_battery"):
                            try:
                                percent, voltage = pf.battery
                                p = float(percent) if percent is not None else -1.0
                                # Never emit NaN over QWebChannel (Qt serializes NaN -> null,
                                # which trips "Expected value of type number, found null"); use -1 sentinel.
                                v = float(voltage) if voltage is not None else -1.0
                                self.gui_bridge.update_battery(p, v)
                            except Exception as e:
                                print(f"GUI bridge error (battery): {e}")

                        if pf.speed is not None and self.gui_bridge and hasattr(self.gui_bridge, "update_speed"):
                            try:
                                self.gui_bridge.update_speed(pf.speed)
                            except Exception as e:
                                print(f"GUI bridge error (speed): {e}")

                        if pf.heading is not None and self.gui_bridge and hasattr(self.gui_bridge, "update_heading"):
                            try:
                                self.gui_bridge.update_heading(float(pf.heading))
                            except Exception as e:
                                print(f"GUI bridge error (heading): {e}")

                except Exception as e:
                    print(f"Serial read error: {e}")
                    self._start_reconnect(f"serial read error: {e}")
                    time.sleep(0.5)

        self.received_thread = threading.Thread(target=_read_loop, daemon=True)
        self.received_thread.start()
        return True

    def update_waypoints(self, new_waypoints):
        self.waypoints = []
        for i, wp in enumerate(new_waypoints):
            try:
                x = float(wp.get("x"))
                y = float(wp.get("y"))
                z = float(wp.get("z", 3.5))
                if not (_is_num(x) and _is_num(y) and _is_num(z)) or z < 0.0:
                    raise ValueError("waypoint must have finite x/y/z and non-negative z")
                parsed = {
                    "x": x,
                    "y": y,
                    "z": z
                }
                self.waypoints.append(parsed)
                print(f"   -> WP{i+1}: x={parsed['x']:.3f}, y={parsed['y']:.3f}, z={parsed['z']:.3f}")
            except Exception as e:
                print(f"Error processing waypoint {i+1}: {e}")
        print(f"✅ Updated {len(self.waypoints)} waypoint")

    def remove_waypoint_by_index(self, index: int):
        if not self.waypoints:
            return print("Waypoint list is empty")
        if index < 1 or index > len(self.waypoints):
            return print(f"❌ Invalid waypoint index : {index}")
        del self.waypoints[index - 1]
        print("✅ Waypoint removed")

    def send_waypoints_to_drone(self):
        if not self.ser or not self.ser.is_open:
            self._reject_mission("SENDMISSION rejected: serial is not connected")
            return
        if not self.waypoints:
            self._reject_mission("SENDMISSION rejected: there is no waypoint to send")
            return
        if not self._has_recent_position():
            self._reject_operation_no_gps("SENDMISSION")
            return

        with self._mission_tx_lock:
            if self._mission_tx_active:
                self._reject_mission("SENDMISSION rejected: mission upload already in progress")
                return
            self._mission_tx_active = True

        mission_waypoints = [dict(wp) for wp in self.waypoints]
        mission_id = _make_mission_id()
        chunks = _chunk_list(mission_waypoints, self._mission_chunk_size)

        def worker():
            try:
                total_chunks = len(chunks)
                total_count = len(mission_waypoints)
                print(
                    f"📤 Mission {mission_id}: sending {total_count} waypoints in "
                    f"{total_chunks} chunks (chunk_size={self._mission_chunk_size})"
                )

                ok, msg, _ = self._send_payload_wait_ack(
                    mission_begin(mission_id, total_chunks, total_count),
                    expect_event="mission_begin", tries=2, interval=2.0,
                )
                if not ok:
                    self._reject_mission(f"SENDMISSION begin failed: {msg}")
                    return

                for chunk_index, chunk in enumerate(chunks):
                    ok, msg, _ = self._send_payload_wait_ack(
                        mission_chunk(mission_id, chunk_index, total_chunks, chunk),
                        expect_event="mission_chunk",
                        tries=2,
                        interval=2.0,
                    )
                    if not ok:
                        self._reject_mission(f"SENDMISSION chunk {chunk_index + 1}/{total_chunks} failed: {msg}")
                        return

                ok, msg, acked = self._send_payload_wait_ack(
                    mission_commit(mission_id),
                    expect_event="uploaded",
                    tries=2,
                    interval=2.0,
                )
                if not ok and not acked:
                    self._reject_mission(f"SENDMISSION commit failed: {msg}")
                else:
                    print(f"✅ Mission {mission_id} upload complete ({total_count} waypoints)")
            finally:
                with self._mission_tx_lock:
                    self._mission_tx_active = False

        threading.Thread(target=worker, daemon=True).start()

    def offboard_req(self):
        if self.ser and self.ser.is_open:
            # Gate on a recent position of ANY kind (GPS or local) — accepts indoor/local
            # flight, unlike the old GPS-only check.
            if not self._has_recent_position():
                self._reject_operation_no_gps("OFFBOARD")
                return

            payload = cmd_offboard()
            self._send_with_retry(payload, expect_event="mode_push",
                expect_mode="OFFBOARD", tries=2, interval=2.0, on_fail_msg="No ACK (timeout)")
        else:
            print("Serial port is not open")

    def land_req(self):
        if self.ser and self.ser.is_open:
            payload = cmd_land()
            self._send_with_retry(payload,
                expect_event="mode_push", expect_mode="LAND", tries=2,
                interval=2.0, on_fail_msg="No ACK (timeout)")
        else:
            print("Serial is not open")


def main():
    controller = GroundController(port='/dev/lora_ground', baudrate=9600)
    controller.start()
    controller.read_position_from_drone()

    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("Terminated by user (Ctrl+C)")
        controller.stop()


if __name__ == '__main__':
    main()
