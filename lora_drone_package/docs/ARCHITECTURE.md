# lora_drone_package — Architecture

The drone-side bridge between the LoRa radio (E32-433T20D) and ROS2/MAVROS/PX4. It receives
commands (OFFBOARD / LAND / mission upload) from the Ground GUI over LoRa and republishes them
to ROS, and streams compact telemetry back to the ground.

The main bridge (`lora_drone`) is split into focused modules — the node orchestrates; the
policies (wire codec, dedupe, telemetry, mission assembly, serial I/O) live in their own files,
most of them pure (no ROS) and unit-testable.

---

## Table of contents

1. [File map](#1-file-map)
2. [Pipeline & data flow](#2-pipeline--data-flow)
3. [Module API reference](#3-module-api-reference)
4. [Wire protocol](#4-wire-protocol)
5. [Threading & concurrency](#5-threading--concurrency)
6. [Extending](#6-extending)

---

## 1. File map

Everything lives in `lora_drone_package/lora_drone_package/`. Two ROS nodes ship from this
package (`lora_drone`, `lora_gimbal_bridge`); only `lora_drone` is started by `bringup.launch.py`.

| File | Kind | Function |
|---|---|---|
| `lora_drone.py` | **node** (`LoraDrone`) | The bridge. ROS subs/pub/service wiring, sensor callbacks that cache state, command routing (`handle_command`), the telemetry TX loop (`send_pose_loop`), and the mode service call. Owns one instance each of the modules below. |
| `protocol.py` | module — **pure** | LoRa wire codec: CRC32 framing, JSON line cleaning, finite-number check. |
| `idempotency.py` | module — **pure** | `IdempotencyCache`: command dedupe by `seq`, session reset by `sid`, ACK caching. |
| `telemetry.py` | module — **pure** | `TelemetryEncoder` (compact packet build + change-detection) + `estimate_percent_from_voltage`. |
| `mission_assembler.py` | module — **pure** | `MissionAssembler` + `parse_waypoints`: chunked mission upload state machine (begin/chunk/commit). |
| `serial_link.py` | module — infra | `SerialLink`: owns the serial port, the writer lock, reconnect, and the RX read loop. ROS-agnostic. |
| `lora_gimbal_bridge_node.py` | **node** (separate) | Independent gimbal bridge: LoRa ↔ `/gimbal_command` / `/gimbal_state` over a binary protocol. Not part of the `lora_drone` refactor. |
| `e32_config.py` | tool (CLI) | Standalone utility to configure the E32 module registers. Not a ROS node. |
| `test_gimbal_rx.py` | tool (CLI) | Standalone debug monitor for gimbal LoRa packets. |
| `__init__.py` | — | Package marker. |

"pure" = no ROS / serial import → importable and unit-testable on its own. `CMakeLists` is not
used (Python package); `find_packages()` in `setup.py` auto-includes every `.py` here, so new
modules are importable as `lora_drone_package.<module>` with no extra config.

---

## 2. Pipeline & data flow

```
Ground GUI ──LoRa──► [ E32 radio ] ──serial──► lora_drone ──┬─ /mission_path (Path)      ─► offboard_control
   ▲                                                         ├─ /mode_signal  (srv call)  ─►
   │  telemetry (compact JSON)                               └─ telemetry built from /mavros/* topics
   └─────────────────────────────────────────────────────────
```

**RX (ground → drone), per received line:** `SerialLink.read_loop` reads a line → node
`_on_serial_line` → `handle_command`:
1. `protocol.clean_json_str` + `json.loads` + `protocol.unwrap_and_verify_crc` — reject
   non-CRC / bad-CRC packets (RX-health counters bumped on each failure kind).
2. `IdempotencyCache.reset_session(sid)` — clear dedupe cache if the ground restarted.
3. `IdempotencyCache.check(seq)` — a retried command is re-ACKed (cached payload), not re-run.
4. Route: mission op → `MissionAssembler`; a `waypoints` array → direct publish; `cmd` →
   `call_mode_service(OFFBOARD/LAND)`.

**Mission upload (chunked):** `begin` → `chunk`×N → `commit`. `MissionAssembler.handle_op`
validates and assembles; on a valid commit the node publishes `/mission_path`, ACKs success,
then auto-triggers OFFBOARD. Telemetry is suppressed (`_suppress_telemetry`) during the exchange
so the half-duplex link stays free for ACKs/chunks.

**TX (drone → ground):** `send_pose_loop` runs at `telemetry_interval`; `TelemetryEncoder.build`
produces a compact packet (position/speed every cycle; state/battery only on change or refresh);
`SerialLink.write(..., priority=False)` sends it best-effort so it never queues behind commands.

---

## 3. Module API reference

### 3.1 `protocol.py` — pure

| Function | What it does |
|---|---|
| `clean_json_str(s) -> str` | Keep only the outermost `{...}` (tolerate LoRa line noise). |
| `is_finite_num(x) -> bool` | True if `x` is a finite int/float. |
| `wrap_with_crc(payload) -> dict` | `{"payload": payload, "crc32": "AABBCCDD"}` (commands/ACKs). |
| `unwrap_and_verify_crc(data) -> (payload, status)` | `status ∈ {ok_crc, plain, bad_crc, invalid}`. |
| `crc32_hex_from_obj` / `canonical_json_bytes` | Internal: CRC over sorted-keys / no-space JSON so both ends agree. |

### 3.2 `idempotency.py` — pure

`IdempotencyCache(max_entries=128)`:

| Member | What it does |
|---|---|
| `reset_session(sid) -> bool` | Clears the cache when `sid` changes (ground restarted). Returns True on reset. |
| `check(seq) -> (status, payload)` | `new` (execute) / `dup_acked` (re-send payload) / `dup_inflight` (skip) / `untracked` (no seq). Registers a new seq as in-flight. |
| `cache_ack(seq, payload)` | Store the FIRST ACK for a seq (never overwrite — auto-OFFBOARD must not clobber the mission ACK). |

### 3.3 `telemetry.py` — pure

| Member | What it does |
|---|---|
| `estimate_percent_from_voltage(v, cells=4, ...)` | Linear LiPo SoC estimate (0..100) or None. |
| `TelemetryEncoder(full_refresh_sec=15.0)` | Owns change-detection trackers. |
| `.build(now, *, gps_ok, gps_fresh, pose, lat, lon, alt, vel, state, percentage, voltage) -> (data, token)` | Build the compact packet. Position/speed/gps every cycle; state/battery only when changed or on the periodic full refresh. |
| `.commit(token)` | Advance the trackers — call **only** after a successful send. |

### 3.4 `mission_assembler.py` — pure

| Member | What it does |
|---|---|
| `parse_waypoints(items) -> (parsed, err)` | Validate a waypoint list → `[{x,y,z}]` floats, or `(None, reason)`. |
| `MissionAssembler()` | Owns per-mission assembly state. |
| `.handle_op(cmd, sid, now, *, info, warn, dbg) -> MissionResult` | Process one begin/chunk/commit op. Logs via injected callbacks. |
| `.discard(mission_id)` | Drop a mission (call after a successful publish). |
| `MissionResult(ack, waypoints, mission_id, total_chunks)` | `ack` always set; `waypoints` set **only** on a validated commit → node publishes; node `discard()`s only on publish success (so a failed publish keeps the mission for retry). |

### 3.5 `serial_link.py` — infrastructure (ROS-agnostic)

`SerialLink(path, baud, logger, is_alive)` — `logger` has `.info/.warn/.error/.fatal`;
`is_alive()` returns False to stop the loops (the node passes `rclpy.ok() and not stopped`).

| Member | What it does |
|---|---|
| `open_with_retries(attempts=5)` | Initial open with retries; raises `RuntimeError` if the port never opens. |
| `write(frame_bytes, priority=True) -> bool` | `priority=True` blocks for the writer lock (commands/ACKs); `priority=False` is best-effort (skips if the lock is held — telemetry). Reopens on a serial error. |
| `read_loop(on_line)` | Blocking RX loop: `readline()` → `on_line(str)`; reconnects on closed port / serial error. Does **not** hold the lock during `readline()`. |
| `reopen(reason) / close()` | Reconnect (≤3 attempts) / close the port. |

---

## 4. Wire protocol

One JSON object per line. Two frame kinds:

- **CRC-wrapped** `{"payload": {...}, "crc32": "AABBCCDD"}` — **required** for ground→drone
  commands. The drone rejects plain/bad-CRC commands so line noise can never trigger
  offboard/land/mission.
- **plain** `{...}` — used only for drone→ground telemetry (saves ~25 B/packet; a corrupted
  telemetry frame just fails JSON parse and is skipped).

Common command fields: `sid` (ground session id), `seq` (per-command sequence for dedupe/ACK).
Mission ops use short keys (`op`, `mi`, `tc`, `tn`, `ci`, `wps`) to fit the LoRa MTU; legacy long
keys (`mission_op`, `mission_id`, …) are still accepted.

---

## 5. Threading & concurrency

| Thread | Role |
|---|---|
| ROS executor (`rclpy.spin`) | Sensor subscription callbacks (cache pose/gps/battery/vel/state) + mode-service done-callbacks. |
| RX thread | `SerialLink.read_loop(_on_serial_line)` — blocking readline → `handle_command`. |
| TX thread | `send_pose_loop` — telemetry at `telemetry_interval`. |
| Timer threads | One-shot `threading.Timer` for the mode-service ACK timeout. |

**Locks:** `SerialLink`'s internal writer lock serialises ALL writers (telemetry + every ACK);
reads are independent so the blocking `readline()` does not hold it. `IdempotencyCache` has its
own lock. The node's `_stop_event` + `rclpy.ok()` (via the injected `is_alive`) stop the loops on
shutdown; `close()` sets the event and closes the port.

---

## 6. Extending

| To… | Do this |
|---|---|
| **Send a new telemetry field** | Add it in `TelemetryEncoder.build()` (one place). Pass the value from `send_pose_loop`. |
| **Add a new command** | Add a branch in `handle_command` (after the dedupe guard). |
| **Change framing / CRC** | `protocol.py` only — both wrap (TX) and unwrap (RX) live there. |
| **Change the mission upload protocol** | `mission_assembler.py` only; the node keeps publish/ACK/OFFBOARD orchestration. |
| **Reuse the serial link** | `SerialLink` is ROS-agnostic — the gimbal bridge (or any node) can use it by passing its own logger + `is_alive`. |
| **Unit-test logic without hardware** | Import `protocol` / `idempotency` / `telemetry` / `mission_assembler` directly (no ROS, no serial). |

---

*Last updated: 2026-06-12*
