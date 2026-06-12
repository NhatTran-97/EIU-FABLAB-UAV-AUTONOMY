"""Chunked-mission assembly — pure, NO ROS / serial dependency (unit-testable).

A mission is uploaded over LoRa in three op kinds: `begin` (announce id + chunk/waypoint
counts), `chunk` (a slice of waypoints), `commit` (assemble + finalize). This module owns the
per-mission assembly state and validates each op. It does NOT publish or send ACKs — it returns
the ACK payload (and, on a validated commit, the assembled waypoints) for the node to act on.

The node keeps the ROS publish + ACK send + auto-OFFBOARD orchestration so the exact ordering
and the "keep the mission for retry if publish fails" behaviour are preserved.
"""

from collections import namedtuple

try:  # works both installed (ros2 run) and standalone (python3 from the package dir)
    from lora_drone_package.protocol import is_finite_num
except ImportError:  # pragma: no cover
    from protocol import is_finite_num


# ack         : dict payload to ACK back (node adds ack_seq/sid + sends).
# waypoints   : list[dict x,y,z] ONLY on a validated commit (node publishes); else None.
# mission_id  : id to discard() after a successful publish (commit case only).
# total_chunks: for the node's "Published ... in N chunks" log (commit case only).
MissionResult = namedtuple("MissionResult", ["ack", "waypoints", "mission_id", "total_chunks"])


def parse_waypoints(items):
    """Return (parsed, err). parsed is a list of {x,y,z} floats, or None on the first invalid
    waypoint (with err describing why). An empty/non-list input is rejected."""
    parsed = []
    if not isinstance(items, list) or not items:
        return None, "empty waypoint list"

    for wp in items:
        if not isinstance(wp, dict):
            return None, "waypoint must be object"
        if not all(is_finite_num(wp.get(k)) for k in ("x", "y", "z")):
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


class MissionAssembler:
    def __init__(self):
        self._missions = {}   # mission_id -> assembly state

    def discard(self, mission_id):
        """Drop a mission's state (call after a successful publish)."""
        self._missions.pop(mission_id, None)

    def handle_op(self, cmd, sid, now, *, info=None, warn=None, dbg=None):
        """Process one begin/chunk/commit op. Returns a MissionResult. Logging is emitted via
        the optional info/warn/dbg callbacks (default no-op) so the node keeps identical logs."""
        info = info or (lambda m: None)
        warn = warn or (lambda m: None)
        dbg = dbg or (lambda m: None)

        op = str(cmd.get("op") if "op" in cmd else cmd.get("mission_op", "")).strip().lower()
        mission_id = cmd.get("mi") if "mi" in cmd else cmd.get("mission_id", None)
        if not isinstance(mission_id, str) or not mission_id:
            error_event = {
                "begin": "mission_begin",
                "chunk": "mission_chunk",
                "commit": "uploaded",
            }.get(op, "uploaded")
            return MissionResult({
                "event": error_event,
                "status": False,
                "error": "invalid mission_id",
            }, None, None, None)

        if op == "begin":
            total_chunks = cmd.get("tc") if "tc" in cmd else cmd.get("total_chunks", None)
            total_count = cmd.get("tn") if "tn" in cmd else cmd.get("total_count", None)
            if not isinstance(total_chunks, int) or total_chunks <= 0:
                return MissionResult({
                    "event": "mission_begin",
                    "status": False,
                    "mission_id": mission_id,
                    "error": "invalid total_chunks",
                }, None, None, None)
            if not isinstance(total_count, int) or total_count <= 0:
                return MissionResult({
                    "event": "mission_begin",
                    "status": False,
                    "mission_id": mission_id,
                    "error": "invalid total_count",
                }, None, None, None)

            self._missions[mission_id] = {
                "sid": sid,
                "coord": str(cmd.get("coord", "local")),
                "total_chunks": total_chunks,
                "total_count": total_count,
                "chunks": {},
                "created_at": now,
            }
            info(f"🧩 mission {mission_id} BEGIN — expecting {total_chunks} chunks, "
                 f"{total_count} waypoints")
            dbg(f"MISSION {mission_id} BEGIN expect={total_chunks}chunks/{total_count}wps")
            return MissionResult({
                "event": "mission_begin",
                "status": True,
                "mission_id": mission_id,
                "total_chunks": total_chunks,
                "total_count": total_count,
                "msg": "mission begin accepted",
            }, None, None, None)

        if op == "chunk":
            session = self._missions.get(mission_id)
            chunk_index = cmd.get("ci") if "ci" in cmd else cmd.get("chunk_index", None)
            total_chunks = cmd.get("tc") if "tc" in cmd else cmd.get("total_chunks", None)
            if session is None:
                return MissionResult({
                    "event": "mission_chunk",
                    "status": False,
                    "mission_id": mission_id,
                    "error": "mission not started",
                }, None, None, None)
            if session["sid"] != sid:
                return MissionResult({
                    "event": "mission_chunk",
                    "status": False,
                    "mission_id": mission_id,
                    "error": "mission sid mismatch",
                }, None, None, None)
            if total_chunks != session["total_chunks"]:
                return MissionResult({
                    "event": "mission_chunk",
                    "status": False,
                    "mission_id": mission_id,
                    "error": "total_chunks mismatch",
                }, None, None, None)
            if not isinstance(chunk_index, int) or not (0 <= chunk_index < session["total_chunks"]):
                return MissionResult({
                    "event": "mission_chunk",
                    "status": False,
                    "mission_id": mission_id,
                    "error": "invalid chunk_index",
                }, None, None, None)

            parsed, err = parse_waypoints(cmd.get("wps") if "wps" in cmd else cmd.get("waypoints"))
            if parsed is None:
                return MissionResult({
                    "event": "mission_chunk",
                    "status": False,
                    "mission_id": mission_id,
                    "chunk_index": chunk_index,
                    "error": err,
                }, None, None, None)

            existing = session["chunks"].get(chunk_index)
            if existing is not None and existing != parsed:
                return MissionResult({
                    "event": "mission_chunk",
                    "status": False,
                    "mission_id": mission_id,
                    "chunk_index": chunk_index,
                    "error": "chunk content changed",
                }, None, None, None)

            session["chunks"][chunk_index] = parsed
            have = sorted(session["chunks"].keys())
            missing = [i for i in range(session["total_chunks"]) if i not in session["chunks"]]
            info(f"🧩 mission {mission_id}: chunk {chunk_index} OK ({len(parsed)} wps) — "
                 f"have {have}/{session['total_chunks']}, still missing {missing}")
            dbg(f"MISSION {mission_id} CHUNK {chunk_index} OK ({len(parsed)}wps) "
                f"have={have} missing={missing}")
            return MissionResult({
                "event": "mission_chunk",
                "status": True,
                "mission_id": mission_id,
                "chunk_index": chunk_index,
                "received_chunks": len(session["chunks"]),
                "total_chunks": session["total_chunks"],
                "msg": "chunk accepted",
            }, None, None, None)

        if op == "commit":
            session = self._missions.get(mission_id)
            if session is None:
                return MissionResult({
                    "event": "uploaded",
                    "status": False,
                    "mission_id": mission_id,
                    "error": "mission not started",
                }, None, None, None)
            if session["sid"] != sid:
                return MissionResult({
                    "event": "uploaded",
                    "status": False,
                    "mission_id": mission_id,
                    "error": "mission sid mismatch",
                }, None, None, None)

            missing = [i for i in range(session["total_chunks"]) if i not in session["chunks"]]
            if missing:
                warn(f"❌ mission {mission_id} COMMIT rejected — missing chunks {missing} "
                     f"(have {sorted(session['chunks'].keys())}/{session['total_chunks']}). "
                     f"Those chunks never arrived intact — check RX truncation logs above.")
                dbg(f"MISSION {mission_id} COMMIT-FAIL missing={missing} "
                    f"have={sorted(session['chunks'].keys())}")
                return MissionResult({
                    "event": "uploaded",
                    "status": False,
                    "mission_id": mission_id,
                    "error": f"missing chunks: {missing}",
                }, None, None, None)

            waypoints = []
            for i in range(session["total_chunks"]):
                waypoints.extend(session["chunks"][i])

            if len(waypoints) != session["total_count"]:
                return MissionResult({
                    "event": "uploaded",
                    "status": False,
                    "mission_id": mission_id,
                    "count": len(waypoints),
                    "total": session["total_count"],
                    "error": "waypoint count mismatch",
                }, None, None, None)

            # Validated commit. Return the success ACK + waypoints WITHOUT discarding the
            # mission — the node publishes first and only discard()s on success, so a publish
            # failure leaves the mission in place for a retry (same as before).
            return MissionResult({
                "event": "uploaded",
                "status": True,
                "mission_id": mission_id,
                "count": len(waypoints),
                "total": session["total_count"],
                "msg": "mission uploaded",
            }, waypoints, mission_id, session["total_chunks"])

        return MissionResult({
            "event": "uploaded",
            "status": False,
            "mission_id": mission_id,
            "error": f"unknown mission_op: {op}",
        }, None, None, None)
