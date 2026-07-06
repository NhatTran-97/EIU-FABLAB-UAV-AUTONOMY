"""LoRa wire protocol (ground side) — pure codec + small id/chunk helpers, NO serial / Qt.

Mirror of the drone-side codec: one JSON object per line, optionally CRC-wrapped. Commands the
ground SENDS are CRC-wrapped; telemetry it RECEIVES may be plain. Keeping this pure makes it
unit-testable (see test/test_logic.py, which imports these names via control.py).
"""

import json
import math
import os
import random
import time
import zlib


# Verbose per-packet logging is opt-in so the serial reader thread stays lean.
# Enable with:  GROUND_GUI_DEBUG=1
DEBUG = os.environ.get("GROUND_GUI_DEBUG", "0") == "1"


def log_dbg(*args, **kwargs):
    if DEBUG:
        print(*args, **kwargs)


def _is_num(x):
    try:
        return isinstance(x, (int, float)) and math.isfinite(float(x))
    except Exception:
        return False


def _clean_json_str(s: str) -> str:
    start = s.find("{")
    end = s.rfind("}")
    if start != -1 and end != -1 and end > start:
        return s[start:end + 1]
    return ""


def _as_true(v):
    if isinstance(v, bool):
        return v
    if isinstance(v, (int, float)):
        return int(v) == 1
    if isinstance(v, str):
        return v.strip().lower() in ("1", "true", "t", "yes", "y")
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


def _make_session_id() -> str:
    # Distinguish a fresh ground process whose seq counter starts from 1 again.
    return f"{int(time.time() * 1000):x}-{random.getrandbits(32):08x}"


def _make_mission_id() -> str:
    return f"{random.getrandbits(32):08x}"


def _chunk_list(items, chunk_size: int):
    size = max(1, int(chunk_size))
    return [items[i:i + size] for i in range(0, len(items), size)]


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
