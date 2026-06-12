"""LoRa wire protocol — pure codec, NO ROS / serial dependency (unit-testable, reusable on
the ground side too).

Frame format on the wire is one JSON object per line. Two kinds:
  * CRC-wrapped : {"payload": {...}, "crc32": "AABBCCDD"}  — REQUIRED for ground->drone commands.
  * plain       : {...}                                     — used for display-only telemetry.

The CRC is computed over the canonical (sorted-keys, no-space) encoding of the payload so both
ends compute the same value regardless of key order or whitespace.
"""

import json
import math
import zlib


def clean_json_str(s: str) -> str:
    """Tolerate leading/trailing LoRa line noise: keep only the outermost {...}."""
    start = s.find("{")
    end = s.rfind("}")
    if start != -1 and end != -1 and end > start:
        return s[start:end + 1]
    return ""


def is_finite_num(x) -> bool:
    try:
        return isinstance(x, (int, float)) and math.isfinite(float(x))
    except Exception:
        return False


def canonical_json_bytes(obj: dict) -> bytes:
    """Deterministic encoding (sorted keys, no spaces) so both ends agree on the CRC input."""
    return json.dumps(obj, separators=(",", ":"), sort_keys=True).encode("utf-8")


def crc32_hex_from_obj(obj: dict) -> str:
    return f"{zlib.crc32(canonical_json_bytes(obj)) & 0xFFFFFFFF:08X}"


def wrap_with_crc(payload: dict) -> dict:
    """Wrap a payload in the CRC envelope (used for commands and ACKs)."""
    return {
        "payload": payload,
        "crc32": crc32_hex_from_obj(payload),
    }


def unwrap_and_verify_crc(data: dict):
    """
    Return (payload_dict, status)
      status:
        - "ok_crc"  : wrapped packet with valid CRC
        - "plain"   : legacy packet without CRC wrapper
        - "bad_crc" : wrapped packet but CRC mismatch / malformed
        - "invalid" : not a dict
    """
    if not isinstance(data, dict):
        return None, "invalid"

    if "payload" in data or "crc32" in data:
        payload = data.get("payload")
        recv_crc = data.get("crc32")
        if not isinstance(payload, dict) or not isinstance(recv_crc, str):
            return None, "bad_crc"

        calc_crc = crc32_hex_from_obj(payload)
        if recv_crc.upper() != calc_crc:
            return None, "bad_crc"

        return payload, "ok_crc"

    return data, "plain"
