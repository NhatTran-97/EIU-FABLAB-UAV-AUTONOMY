"""LoRa command builders (ground side) — pure, NO serial / Qt.

One named builder per ground→drone command. Centralises the wire command schema (so the ground
and the drone agree) and makes adding a command a one-function change here — the controller then
sends the returned payload via its reliable-TX path. Mission ops use short keys (op/mi/tc/tn/
ci/wps) to fit the LoRa MTU.
"""


def cmd_offboard() -> dict:
    return {"cmd": "offboard"}


def cmd_land() -> dict:
    return {"cmd": "land"}


def mission_begin(mission_id, total_chunks, total_count) -> dict:
    return {"op": "begin", "mi": mission_id, "tc": total_chunks, "tn": total_count}


def mission_chunk(mission_id, chunk_index, total_chunks, waypoints) -> dict:
    return {"op": "chunk", "mi": mission_id, "ci": chunk_index, "tc": total_chunks, "wps": waypoints}


def mission_commit(mission_id) -> dict:
    return {"op": "commit", "mi": mission_id}
