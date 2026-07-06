"""Telemetry / ACK frame parsing (ground side) — pure, NO serial / Qt (unit-testable).

Given one decoded, CRC-verified `data` dict (a telemetry or ACK frame from the drone), extract
the structured fields the controller acts on. The controller keeps the side effects (ACK-state
under lock, GUI bridge calls, freshness timestamps, link/state/gps emits).

Extension point: to surface a NEW telemetry field, add its extraction here (one place) and have
the controller act on `ParsedFrame.<field>`.
"""

from dataclasses import dataclass, field
from typing import Optional, Tuple

from protocol import _is_num, _as_true


def _int_or_none(v):
    return int(v) if isinstance(v, int) else None


@dataclass
class AckInfo:
    event: str                       # "mode_push" | "uploaded" | "mission_begin" | "mission_chunk"
    status: bool
    msg: str
    seq: Optional[int]
    sid: object
    mode: Optional[str] = None       # only for mode_push


@dataclass
class ParsedFrame:
    ack: Optional[AckInfo] = None
    hb: bool = False
    state: Optional[str] = None                     # present iff "state" in frame
    gps_ok: Optional[bool] = None                   # present iff "gps_ok" in frame
    local: Optional[Tuple[float, float, float]] = None     # (x, y, z)
    global_pos: Optional[Tuple[float, float, float]] = None  # (lat, lon, alt)
    battery: Optional[Tuple[Optional[float], Optional[float]]] = None  # (percent, voltage)
    speed: Optional[float] = None
    heading: Optional[float] = None    # yaw in degrees, ENU frame (0°=East, CCW+)


def _parse_ack(data: dict) -> Optional[AckInfo]:
    ev = data.get("event")
    seq = _int_or_none(data.get("ack_seq", None))
    sid = data.get("sid", None)
    status = bool(data.get("status", False))
    if ev == "mode_push":
        # Drone uses "error" on the failure path; fall back so the GUI shows a reason.
        msg = str(data.get("msg", "")) or str(data.get("error", ""))
        return AckInfo("mode_push", status, msg, seq, sid, mode=str(data.get("mode", "")).upper())
    if ev == "uploaded":
        msg = (str(data.get("msg", "")) or str(data.get("error", ""))
               or ("Mission uploaded" if status else "Upload rejected"))
        return AckInfo("uploaded", status, msg, seq, sid)
    if ev in ("mission_begin", "mission_chunk"):
        msg = (str(data.get("msg", "")) or str(data.get("error", ""))
               or ("ACK OK" if status else "ACK failed"))
        return AckInfo(str(ev), status, msg, seq, sid)
    return None


def _parse_battery(data: dict):
    """Return (percent, voltage) — either may be None. Mirrors the legacy fallback order."""
    percent = None
    voltage = None
    if "battery" in data and isinstance(data["battery"], dict):
        b = data["battery"]
        if "percent" in b and _is_num(b["percent"]):
            pv = float(b["percent"])
            percent = pv * 100.0 if pv <= 1.0 else pv
        if "voltage" in b and _is_num(b["voltage"]):
            voltage = float(b["voltage"])
    if percent is None and "percent" in data and _is_num(data["percent"]):
        pv = float(data["percent"])
        percent = pv * 100.0 if pv <= 1.0 else pv
    if percent is None and "battery" in data and _is_num(data["battery"]):
        pv = float(data["battery"])
        percent = pv * 100.0 if pv <= 1.0 else pv
    if voltage is None and "voltage" in data and _is_num(data["voltage"]):
        voltage = float(data["voltage"])
    if voltage is None and "volt" in data and _is_num(data["volt"]):
        voltage = float(data["volt"])
    return percent, voltage


def parse_frame(data: dict) -> ParsedFrame:
    """Extract every field the controller reacts to from one telemetry/ACK frame."""
    pf = ParsedFrame()
    pf.ack = _parse_ack(data)
    pf.hb = _as_true(data.get("hb", 0))

    if "state" in data:
        pf.state = str(data["state"])
    if "gps_ok" in data:
        pf.gps_ok = _as_true(data["gps_ok"])

    if all(k in data for k in ("x", "y", "z")) and _is_num(data["x"]) and _is_num(data["y"]) and _is_num(data["z"]):
        pf.local = (float(data["x"]), float(data["y"]), float(data["z"]))

    if all(k in data for k in ("lat", "lon", "alt")) and _is_num(data["lat"]) and _is_num(data["lon"]) and _is_num(data["alt"]):
        pf.global_pos = (float(data["lat"]), float(data["lon"]), float(data["alt"]))

    percent, voltage = _parse_battery(data)
    if percent is not None or voltage is not None:
        pf.battery = (percent, voltage)

    if "speed" in data and _is_num(data["speed"]):
        pf.speed = float(data["speed"])
    elif "vel" in data and _is_num(data["vel"]):
        pf.speed = float(data["vel"])

    if "hdg" in data and _is_num(data["hdg"]):
        pf.heading = float(data["hdg"])

    return pf
