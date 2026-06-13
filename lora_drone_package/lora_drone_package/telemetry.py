"""Telemetry packet building — pure, NO ROS dependency (unit-testable).

Builds the compact, un-CRC'd telemetry dict sent to the ground at the telemetry rate. To keep
steady-state packets small on the bandwidth-starved LoRa link, slow-changing fields (state,
battery) are only included when they CHANGE or on a periodic full refresh; position / speed /
gps_ok go every cycle.

The encoder owns the change-detection trackers. The node calls `build()` each cycle, sends the
returned dict, and calls `commit()` ONLY if the send actually succeeded — so a skipped/failed
cycle does not suppress the next change.
"""

import math


def estimate_percent_from_voltage(voltage, cells=4, v_full=4.2, v_empty=3.3):
    """Linear LiPo SoC estimate from pack voltage. Returns 0..100 or None if invalid."""
    if not (isinstance(voltage, (int, float)) and math.isfinite(voltage)) or cells <= 0:
        return None
    v_cell = float(voltage) / float(cells)
    if v_cell >= v_full:
        return 100.0
    if v_cell <= v_empty:
        return 0.0
    return (v_cell - v_empty) / (v_full - v_empty) * 100.0


class TelemetryEncoder:
    def __init__(self, full_refresh_sec=15.0):
        self._last_state = None
        self._last_batt = None
        self._last_yaw = 0.0
        self._last_gps_ok = None
        self._last_full_at = 0.0
        self._full_refresh = full_refresh_sec

    def build(self, now, *, gps_ok, gps_fresh, pose, lat, lon, alt, vel, state,
              percentage, voltage):
        """Return (data, token). `data` is the compact packet to send; pass `token` to
        commit() iff the packet was actually sent.

        Args mirror the node's cached state: `pose` is a ROS Pose (read .position.x/y/z,
        duck-typed — no import) or None; `vel` is an (x,y,z) tuple or None; `state` is the
        flight-mode string; `percentage`/`voltage` are the latest battery readings or None.
        """
        # Periodically force a full packet so a ground that connects mid-stream still gets
        # state/battery within _full_refresh seconds.
        force_full = (now - self._last_full_at) >= self._full_refresh

        data = {"hb": 1}

        # gps_ok: only send on change or full-refresh
        gps_ok_val = int(bool(gps_ok) and bool(gps_fresh))
        if force_full or gps_ok_val != self._last_gps_ok:
            data["gps_ok"] = gps_ok_val

        if pose is not None:
            # 1 decimal = 10 cm resolution — enough for GCS display, saves 1-2 bytes/field
            data["x"] = round(pose.position.x, 1)
            data["y"] = round(pose.position.y, 1)
            data["z"] = round(pose.position.z, 1)
            # Yaw from quaternion (ENU frame: 0°=East, CCW positive).
            # Integer degrees saves 2 bytes; ±1° resolution is fine for map display.
            qx, qy, qz, qw = (pose.orientation.x, pose.orientation.y,
                               pose.orientation.z, pose.orientation.w)
            yaw_deg = int(round(math.degrees(
                math.atan2(2*(qw*qz + qx*qy), 1 - 2*(qy*qy + qz*qz))
            )))
            if force_full or abs(yaw_deg - self._last_yaw) >= 1:
                data["hdg"] = yaw_deg

        if gps_fresh and lat is not None and lon is not None and alt is not None:
            data["lat"] = round(lat, 5)
            data["lon"] = round(lon, 5)
            data["alt"] = round(alt, 1)

        if vel is not None:
            vx, vy, vz = vel
            data["speed"] = round(math.sqrt(vx * vx + vy * vy + vz * vz), 1)

        # state: only when it changed (or refresh). Ground keeps the last value.
        if force_full or state != self._last_state:
            data["state"] = state

        # battery: quantize (int %, 0.1 V) then send only on change (or refresh).
        batt = None
        if percentage is not None and voltage is not None:
            batt = (int(round(percentage)), round(voltage, 1))
            if force_full or batt != self._last_batt:
                data["battery"] = {"percent": batt[0], "voltage": batt[1]}

        return data, (state, batt, data.get("hdg"), data.get("gps_ok"), force_full, now)

    def commit(self, token):
        """Advance the change-detection trackers — call ONLY after a successful send."""
        state, batt, hdg, gps_ok_sent, force_full, now = token
        self._last_state = state
        self._last_batt = batt
        if hdg is not None:
            self._last_yaw = hdg
        if gps_ok_sent is not None:
            self._last_gps_ok = gps_ok_sent
        if force_full:
            self._last_full_at = now
