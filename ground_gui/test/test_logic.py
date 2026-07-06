"""
Logic tests for the Ground_GUI serial protocol layer (control.py).

Runs with the standard library only:
    python -m unittest test_logic -v
    # or
    python test_logic.py

Covers the pure helpers (CRC framing, JSON cleanup, coercions) and the
GroundController logic that does not require an open serial port
(sequence numbers, frame preparation, ACK matching, waypoint handling,
recent-GPS gating). No hardware / no Qt needed.
"""

import io
import json
import time
import threading
import unittest
import contextlib

import control
from control import (
    _is_num,
    _clean_json_str,
    _as_true,
    _canonical_json_bytes,
    _crc32_hex_from_obj,
    _wrap_with_crc,
    _unwrap_and_verify_crc,
    _chunk_list,
    GroundController,
)


def make_controller():
    # __init__ does not open the serial port (connect() does), so this is safe.
    return GroundController(port="/dev/null-test", baudrate=9600, gui_bridge=None)


class DummyBridge:
    def __init__(self):
        self.mission_acks = []
        self.mode_pushes = []

    def mission_ack(self, ok, msg):
        self.mission_acks.append((bool(ok), str(msg)))

    def mode_push(self, ok, mode, msg):
        self.mode_pushes.append((bool(ok), str(mode), str(msg)))


class DummySerial:
    is_open = True


class CloseOnlySerial:
    def __init__(self):
        self.is_open = True
        self.writes = []
        self.closed = False

    def write(self, data):
        self.writes.append(data)
        return len(data)

    def flush(self):
        pass

    def close(self):
        self.is_open = False
        self.closed = True


@contextlib.contextmanager
def quiet():
    """Silence the chatty print()s inside some controller methods during tests."""
    with contextlib.redirect_stdout(io.StringIO()):
        yield


# --------------------------------------------------------------------------- #
# Pure helpers
# --------------------------------------------------------------------------- #
class TestIsNum(unittest.TestCase):
    def test_finite(self):
        for v in (0, -2, 3.14, 1e9):
            self.assertTrue(_is_num(v), v)

    def test_non_finite(self):
        for v in (float("nan"), float("inf"), float("-inf")):
            self.assertFalse(_is_num(v), v)

    def test_non_numbers(self):
        for v in (None, "3", [1], {}, "nan"):
            self.assertFalse(_is_num(v), v)


class TestCleanJsonStr(unittest.TestCase):
    def test_extracts_object(self):
        self.assertEqual(_clean_json_str('noise {"a":1} tail'), '{"a":1}')

    def test_nested(self):
        self.assertEqual(_clean_json_str('x {"a":{"b":2}} y'), '{"a":{"b":2}}')

    def test_no_object(self):
        self.assertEqual(_clean_json_str("no braces here"), "")

    def test_empty(self):
        self.assertEqual(_clean_json_str(""), "")


class TestAsTrue(unittest.TestCase):
    def test_bool(self):
        self.assertTrue(_as_true(True))
        self.assertFalse(_as_true(False))

    def test_int(self):
        self.assertTrue(_as_true(1))
        self.assertFalse(_as_true(0))
        self.assertFalse(_as_true(2))  # only exactly 1 is true

    def test_str(self):
        for s in ("1", "true", "T", "yes", "y", "TRUE", " true "):
            self.assertTrue(_as_true(s), s)
        for s in ("0", "false", "no", "", "2"):
            self.assertFalse(_as_true(s), s)

    def test_other(self):
        self.assertFalse(_as_true(None))


# --------------------------------------------------------------------------- #
# CRC framing  (the heart of ground <-> drone compatibility)
# --------------------------------------------------------------------------- #
class TestCrcFraming(unittest.TestCase):
    def test_canonical_is_order_independent(self):
        self.assertEqual(
            _canonical_json_bytes({"b": 2, "a": 1}),
            _canonical_json_bytes({"a": 1, "b": 2}),
        )
        self.assertEqual(_canonical_json_bytes({"a": 1, "b": 2}), b'{"a":1,"b":2}')

    def test_crc_hex_format(self):
        h = _crc32_hex_from_obj({"a": 1})
        self.assertEqual(len(h), 8)
        self.assertEqual(h, h.upper())
        int(h, 16)  # must be valid hex

    def test_wrap_unwrap_roundtrip(self):
        payload = {"cmd": "offboard", "seq": 5}
        out, status = _unwrap_and_verify_crc(_wrap_with_crc(payload))
        self.assertEqual(status, "ok_crc")
        self.assertEqual(out, payload)

    def test_bad_crc(self):
        frame = _wrap_with_crc({"cmd": "land", "seq": 1})
        frame["crc32"] = "DEADBEEF"
        out, status = _unwrap_and_verify_crc(frame)
        self.assertEqual(status, "bad_crc")
        self.assertIsNone(out)

    def test_tampered_payload_detected(self):
        frame = _wrap_with_crc({"x": 1})
        frame["payload"]["x"] = 999  # tamper after CRC computed
        _, status = _unwrap_and_verify_crc(frame)
        self.assertEqual(status, "bad_crc")

    def test_plain_packet(self):
        out, status = _unwrap_and_verify_crc({"hb": 1})
        self.assertEqual(status, "plain")
        self.assertEqual(out, {"hb": 1})

    def test_invalid_non_dict(self):
        out, status = _unwrap_and_verify_crc([1, 2, 3])
        self.assertEqual(status, "invalid")
        self.assertIsNone(out)

    def test_crc_case_insensitive(self):
        payload = {"a": 1}
        frame = {"payload": payload, "crc32": _crc32_hex_from_obj(payload).lower()}
        _, status = _unwrap_and_verify_crc(frame)
        self.assertEqual(status, "ok_crc")

    def test_payload_not_dict_is_bad_crc(self):
        _, status = _unwrap_and_verify_crc({"payload": "x", "crc32": "AABBCCDD"})
        self.assertEqual(status, "bad_crc")


# --------------------------------------------------------------------------- #
# GroundController: sequence + frame preparation
# --------------------------------------------------------------------------- #
class TestSeqAndFrame(unittest.TestCase):
    def test_seq_increments_from_one(self):
        c = make_controller()
        self.assertEqual([c._next_seq() for _ in range(3)], [1, 2, 3])

    def test_prepare_frame(self):
        c = make_controller()
        seq, payload_with_seq, frame, frame_bytes = c._prepare_frame({"cmd": "offboard"})
        self.assertEqual(seq, 1)
        self.assertEqual(payload_with_seq, {
            "cmd": "offboard",
            "sid": c._session_id,
            "seq": 1,
        })
        self.assertTrue(frame_bytes.endswith(b"\n"))
        # The wire bytes decode back to the frame, and the CRC verifies.
        decoded = json.loads(frame_bytes.decode("utf-8").strip())
        self.assertEqual(decoded, frame)
        out, status = _unwrap_and_verify_crc(frame)
        self.assertEqual(status, "ok_crc")
        self.assertEqual(out, payload_with_seq)

    def test_session_id_is_stable_per_controller(self):
        c = make_controller()
        _, first_payload, _, _ = c._prepare_frame({"cmd": "offboard"})
        _, second_payload, _, _ = c._prepare_frame({"cmd": "land"})
        self.assertEqual(first_payload["sid"], c._session_id)
        self.assertEqual(second_payload["sid"], c._session_id)
        self.assertEqual(first_payload["sid"], second_payload["sid"])
        self.assertNotEqual(first_payload["seq"], second_payload["seq"])

    def test_chunk_list(self):
        self.assertEqual(_chunk_list([1, 2, 3, 4, 5], 2), [[1, 2], [3, 4], [5]])
        self.assertEqual(_chunk_list([1, 2], 10), [[1, 2]])
        self.assertEqual(_chunk_list([1, 2], 0), [[1], [2]])


# --------------------------------------------------------------------------- #
# GroundController: ACK matching
# --------------------------------------------------------------------------- #
class TestAckMatches(unittest.TestCase):
    def setUp(self):
        self.c = make_controller()

    def _set_ack(self, event, mode, seq, at, sid="CURRENT"):
        self.c._last_ack_event = event
        self.c._last_ack_mode = mode
        self.c._last_ack_sid = self.c._session_id if sid == "CURRENT" else sid
        self.c._last_ack_seq = seq
        self.c._last_ack_at = at

    def test_match(self):
        t = time.monotonic()
        self._set_ack("mode_push", "OFFBOARD", 7, t + 0.1)
        self.assertTrue(self.c._ack_matches(
            expect_event="mode_push", sent_seq=7, since_time=t, expect_mode="OFFBOARD"))

    def test_wrong_event(self):
        t = time.monotonic()
        self._set_ack("uploaded", None, 7, t + 0.1)
        self.assertFalse(self.c._ack_matches(
            expect_event="mode_push", sent_seq=7, since_time=t, expect_mode="OFFBOARD"))

    def test_stale_ack_rejected(self):
        t = time.monotonic()
        self._set_ack("mode_push", "OFFBOARD", 7, t - 1.0)  # arrived before we sent
        self.assertFalse(self.c._ack_matches(
            expect_event="mode_push", sent_seq=7, since_time=t, expect_mode="OFFBOARD"))

    def test_wrong_seq_rejected(self):
        t = time.monotonic()
        self._set_ack("mode_push", "OFFBOARD", 99, t + 0.1)
        self.assertFalse(self.c._ack_matches(
            expect_event="mode_push", sent_seq=7, since_time=t, expect_mode="OFFBOARD"))

    def test_missing_sid_rejected(self):
        t = time.monotonic()
        self._set_ack("mode_push", "OFFBOARD", 7, t + 0.1, sid=None)
        self.assertFalse(self.c._ack_matches(
            expect_event="mode_push", sent_seq=7, since_time=t, expect_mode="OFFBOARD"))

    def test_wrong_sid_rejected(self):
        t = time.monotonic()
        self._set_ack("mode_push", "OFFBOARD", 7, t + 0.1, sid="old-session")
        self.assertFalse(self.c._ack_matches(
            expect_event="mode_push", sent_seq=7, since_time=t, expect_mode="OFFBOARD"))

    def test_null_seq_rejected(self):
        # ACK without ack_seq is no longer wildcard-accepted (enforce ack_seq).
        t = time.monotonic()
        self._set_ack("mode_push", "OFFBOARD", None, t + 0.1)
        self.assertFalse(self.c._ack_matches(
            expect_event="mode_push", sent_seq=7, since_time=t, expect_mode="OFFBOARD"))

    def test_wrong_mode_rejected(self):
        t = time.monotonic()
        self._set_ack("mode_push", "LAND", 7, t + 0.1)
        self.assertFalse(self.c._ack_matches(
            expect_event="mode_push", sent_seq=7, since_time=t, expect_mode="OFFBOARD"))

    def test_uploaded_without_mode(self):
        t = time.monotonic()
        self._set_ack("uploaded", None, 3, t + 0.1)
        self.assertTrue(self.c._ack_matches(
            expect_event="uploaded", sent_seq=3, since_time=t))


# --------------------------------------------------------------------------- #
# GroundController: waypoints
# --------------------------------------------------------------------------- #
class TestWaypoints(unittest.TestCase):
    def test_update_applies_default_z(self):
        c = make_controller()
        with quiet():
            c.update_waypoints([{"x": 1, "y": 2}, {"x": 3, "y": 4, "z": 7.0}])
        self.assertEqual(c.waypoints, [
            {"x": 1.0, "y": 2.0, "z": 3.5},
            {"x": 3.0, "y": 4.0, "z": 7.0},
        ])

    def test_update_skips_unparseable(self):
        c = make_controller()
        with quiet():
            c.update_waypoints([{"x": 1, "y": 2}, {"x": "bad", "y": 2}])
        self.assertEqual(len(c.waypoints), 1)

    def test_update_skips_non_finite_or_negative_altitude(self):
        c = make_controller()
        with quiet():
            c.update_waypoints([
                {"x": 1, "y": 2},
                {"x": float("nan"), "y": 2},
                {"x": 3, "y": 4, "z": -1},
            ])
        self.assertEqual(c.waypoints, [{"x": 1.0, "y": 2.0, "z": 3.5}])

    def test_remove_by_index_one_based(self):
        c = make_controller()
        with quiet():
            c.update_waypoints([{"x": 1, "y": 1}, {"x": 2, "y": 2}, {"x": 3, "y": 3}])
            c.remove_waypoint_by_index(2)
        self.assertEqual([wp["x"] for wp in c.waypoints], [1.0, 3.0])

    def test_remove_out_of_range_noop(self):
        c = make_controller()
        with quiet():
            c.update_waypoints([{"x": 1, "y": 1}])
            c.remove_waypoint_by_index(5)
            c.remove_waypoint_by_index(0)
        self.assertEqual(len(c.waypoints), 1)


# --------------------------------------------------------------------------- #
# GroundController: recent-GPS gating
# --------------------------------------------------------------------------- #
class TestRecentGlobalFix(unittest.TestCase):
    def test_none_initially(self):
        self.assertFalse(make_controller()._has_recent_global_position())

    def test_recent_is_true(self):
        c = make_controller()
        c._last_global_fix_at = time.monotonic()
        self.assertTrue(c._has_recent_global_position())

    def test_stale_is_false(self):
        c = make_controller()
        c._last_global_fix_at = time.monotonic() - (c._global_fix_timeout + 1.0)
        self.assertFalse(c._has_recent_global_position())


class TestRecentPosition(unittest.TestCase):
    """_has_recent_position() accepts a recent LOCAL fix or a recent GLOBAL fix."""

    def test_none_initially(self):
        c = make_controller()
        self.assertFalse(c._has_recent_position())

    def test_recent_local_only_is_accepted(self):
        c = make_controller()
        c._last_local_fix_at = time.monotonic()       # local fresh, no GPS
        self.assertTrue(c._has_recent_local_position())
        self.assertTrue(c._has_recent_position())
        self.assertFalse(c._has_recent_global_position())

    def test_recent_global_only_is_accepted(self):
        c = make_controller()
        c._last_global_fix_at = time.monotonic()      # GPS fresh, no local
        self.assertTrue(c._has_recent_position())

    def test_stale_local_is_rejected(self):
        c = make_controller()
        c._last_local_fix_at = time.monotonic() - (c._global_fix_timeout + 1.0)
        self.assertFalse(c._has_recent_position())


# --------------------------------------------------------------------------- #
# GroundController: local mission rejects must notify the UI via missionAck
# --------------------------------------------------------------------------- #
class TestMissionRejects(unittest.TestCase):
    def test_no_serial_emits_mission_ack(self):
        bridge = DummyBridge()
        c = GroundController(port="/dev/null-test", baudrate=9600, gui_bridge=bridge)
        c.waypoints = [{"x": 1.0, "y": 2.0, "z": 3.5}]
        with quiet():
            c.send_waypoints_to_drone()
        self.assertEqual(len(bridge.mission_acks), 1)
        self.assertFalse(bridge.mission_acks[0][0])
        self.assertIn("serial is not connected", bridge.mission_acks[0][1])

    def test_no_waypoint_emits_mission_ack(self):
        bridge = DummyBridge()
        c = GroundController(port="/dev/null-test", baudrate=9600, gui_bridge=bridge)
        c.ser = DummySerial()
        with quiet():
            c.send_waypoints_to_drone()
        self.assertEqual(len(bridge.mission_acks), 1)
        self.assertFalse(bridge.mission_acks[0][0])
        self.assertIn("no waypoint", bridge.mission_acks[0][1])

    def test_no_position_for_mission_emits_mission_ack_not_mode_push(self):
        bridge = DummyBridge()
        c = GroundController(port="/dev/null-test", baudrate=9600, gui_bridge=bridge)
        c.ser = DummySerial()
        c.waypoints = [{"x": 1.0, "y": 2.0, "z": 3.5}]
        with quiet():
            c.send_waypoints_to_drone()
        self.assertEqual(len(bridge.mission_acks), 1)
        self.assertFalse(bridge.mission_acks[0][0])
        self.assertIn("no recent position", bridge.mission_acks[0][1])
        self.assertEqual(bridge.mode_pushes, [])


# --------------------------------------------------------------------------- #
# GroundController: lifecycle (TX epoch cancels in-flight retries)
# --------------------------------------------------------------------------- #
class TestLifecycle(unittest.TestCase):
    def test_stop_bumps_tx_epoch(self):
        c = make_controller()
        e0 = c._tx_epoch
        with quiet():
            c.stop()  # safe with no serial/threads open
        self.assertEqual(c._tx_epoch, e0 + 1)

    def test_stop_clears_link_timestamps(self):
        c = make_controller()
        c._last_hb = 123.0
        c._last_seen = 456.0
        with quiet():
            c.stop()
        self.assertEqual(c._last_hb, 0.0)
        self.assertEqual(c._last_seen, 0.0)

    def test_stop_skips_off_token_by_default(self):
        c = make_controller()
        c.ser = CloseOnlySerial()
        c._send_off_on_stop = False
        with quiet():
            c.stop()
        self.assertEqual(c.ser.writes, [])
        self.assertTrue(c.ser.closed)

    def test_hb_watch_starts_reconnect_on_link_loss(self):
        c = make_controller()
        c.received = True
        c._link_ok = True
        c._last_hb = time.monotonic() - 1.0
        calls = []

        def fake_reconnect(reason=""):
            calls.append(reason)
            c.received = False

        c._start_reconnect = fake_reconnect
        t = threading.Thread(
            target=c._hb_watch,
            kwargs=dict(timeout=0.01, interval=0.01, grace=1),
            daemon=True,
        )
        with quiet():
            t.start()
            t.join(timeout=0.5)
        self.assertFalse(c._link_ok)
        self.assertTrue(calls)

    def test_reconnect_loop_reopens_serial_and_sends_on(self):
        c = make_controller()
        c.received = True
        c._last_hb = 0.0
        c._reconnect_initial_delay = 0.01
        c._reconnect_max_delay = 0.01
        calls = []

        c._close_serial_for_reconnect = lambda: calls.append("close")
        c.connect = lambda: calls.append("connect")

        def fake_send_on():
            calls.append("on")
            c.received = False
            return True

        c._send_on = fake_send_on
        with quiet():
            c._reconnect_loop("test")
        self.assertEqual(calls, ["close", "connect", "on"])

    def test_worker_sees_epoch_change(self):
        # A retry worker captures the epoch at send time; once stop() bumps it, the
        # captured value differs -> the worker aborts instead of writing to a dead link.
        c = make_controller()
        epoch = c._tx_epoch
        with quiet():
            c.stop()
        self.assertNotEqual(c._tx_epoch, epoch)


if __name__ == "__main__":
    unittest.main(verbosity=2)
