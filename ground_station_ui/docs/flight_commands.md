# Flight commands from the Ground Station to PX4 (direct or via the Jetson)

How the **Take-off / Land / Hold / RTL / Send WP / Start mission** buttons of the EIU Drone GCS reach PX4,
the safety rules, the acknowledgement protocol and how to test them.

> **Status:** verified with automated tests and an end-to-end run app ↔ Jetson with simulated PX4.
> **Not yet flown on the real drone** — for the first time, follow [Testing](#6-testing).

---

## 1. Overview

```
┌──────────── Laptop ────────────┐      ┌─────────────── Jetson (container) ───────────────┐      ┌── Pixhawk ──┐
│ EIU Drone GCS                  │      │ drone_telemetry                                    │      │ PX4 1.17    │
│  FlightCommands (QML)          │ 433  │  RadioLink ─► queue ─► CommandBridge ──────────────┼─DDS─►│ commander   │
│  CommandSender / Uploader      │─────►│                        MissionReceiver ─► /gcs/mission     │ navigator   │
│                                │◄─────│  COMMAND_ACK / MISSION_* / STATUSTEXT              │◄─DDS─│             │
└────────────────────────────────┘ MHz  └────────────────────────────────────────────────────┘      └─────────────┘
```

| Segment | Channel | Format |
|---|---|---|
| App ↔ Jetson | SiK radio 433 MHz (`/dev/telem_radio` on the Jetson) | MAVLink v2, `drone_dialect` |
| Jetson ↔ PX4 | uXRCE-DDS over `/dev/ttyTHS0` (MicroXRCEAgent) | ROS 2 topics `/fmu/in/*`, `/fmu/out/*` (`px4_msgs`) |
| App ↔ PX4 (direct) | SiK radio 915 MHz | Standard MAVLink — status + flight commands (**px4** route) |

**Two command routes, each with its own controls** — `core/commands/flight.py`:

| Control | Route | Commands | Acknowledged by |
|---|---|---|---|
| **PX4 box** in the top bar (click → popup) | App → radio 915 → PX4 (direct MAVLink) | Take-off = `COMPONENT_ARM_DISARM` (if disarmed) → `NAV_TAKEOFF`; Land = `NAV_LAND` | PX4 |
| **Buttons under the Map** | App → radio 433 → Jetson → DDS → PX4 (sections below) | Take-off / Land / Hold / RTL / Send WP / Start mission | Jetson |

On the Jetson, Take-off / Land go through `mission_manager` → `offboard_control` (OFFBOARD) by default
(`commands.takeoff_land_via: mission` in `telemetry.yaml`); with `px4` they are sent as PX4 auto
Takeoff / Land commands, as described in section 2. Hold / RTL always use `VehicleCommand`.

**Cross-route lock.** The route that sends a flight command first **owns control** until the drone is
**landed and disarmed**; the other route is locked (*"flight controlled from …"*). Ownership is released
early if the command is rejected / times out, or is accepted but the drone does not arm within 3 s.
Exception: if the owning route **loses its link**, the other route is unlocked (the drone is never left
without control). The RC transmitter always has priority over both.

**Per-button conditions** (`FlightCommander.blocker`, reason shown on the button / status line):

| Condition | PX4 box (915) | Buttons under the Map (Jetson) |
|---|---|---|
| The route's link is lost | locked | locked |
| Jetson has no PX4 data over DDS (*PX4 (DDS)* card ≠ ok) | — | Take-off / Land / Hold / RTL locked |
| No 3D GPS fix | — | Take-off and Send WP locked in the **Outdoor** environment; RTL always needs GPS. Land / Hold are never locked by GPS |
| Cross-route lock | locked while the Jetson owns control | locked while PX4 (915) owns control (Send WP excepted) |

Direct route: the app converts the take-off altitude to AMSL (`GLOBAL_POSITION_INT.alt` + entered
altitude, needs a 3D GPS fix); otherwise `NaN` is sent and PX4 uses `MIS_TAKEOFF_ALT`.

Autonomy (Offboard, obstacle avoidance, waypoint flight) runs on the Jetson. Land / Hold from the app take
PX4 out of Offboard; the Offboard node on the Jetson must detect this and not take control back.

**Source code**

| Part | File |
|---|---|
| Buttons, confirm dialog, pre-flight check | `ground_station_ui/qml/components/FlightCommands.qml` |
| Send command + wait for ACK + retry | `ground_station_ui/ground_station_ui/core/commands/sender.py` |
| Waypoint upload (mission protocol) | `ground_station_ui/ground_station_ui/core/mission/uploader.py` |
| Command reception, safety rules, forwarding to PX4 | `drone_telemetry/drone_telemetry/commands.py` (`CommandBridge`) |
| Waypoint reception | `drone_telemetry/drone_telemetry/commands.py` (`MissionReceiver`) |
| ROS wiring (topics, radio, timer) | `drone_telemetry/drone_telemetry/telemetry_node.py` (`_setup_commands`) |
| Take-off / Land / mission flight (OFFBOARD) | `offboard_control/px4_offboard/mission_manager.py`, `offboard_control.py` |

---

## 2. Commands

| Button | App sends (`COMMAND_LONG` to sysid 1 / compid 191) | Jetson sends to PX4 (`VehicleCommand`, px4 route) | PX4 action |
|---|---|---|---|
| **Take-off** | `MAV_CMD_NAV_TAKEOFF` (22), `param7` = altitude **above the take-off point** (m) | `COMPONENT_ARM_DISARM` (400, p1 = 1) if disarmed → `NAV_TAKEOFF` (22) | Climbs, then **holds** — does not land by itself |
| **Land** | `MAV_CMD_NAV_LAND` (21) | `NAV_LAND` (21), lat/lon NaN = in place | Lands in place, disarms on touchdown |
| **Hold** | `MAV_CMD_NAV_LOITER_UNLIM` (17) | `DO_SET_MODE` (176): p1 = 1 (custom), p2 = 4 (AUTO), p3 = 3 (LOITER) | Stops and holds position |
| **RTL** | `MAV_CMD_NAV_RETURN_TO_LAUNCH` (20) | `NAV_RETURN_TO_LAUNCH` (20) | Climbs to `RTL_RETURN_ALT`, flies Home, lands |
| **Send WP** | Mission protocol (section 5) | — | — |
| **Start mission** | `MAV_CMD_MISSION_START` (300), `param3` bit 0 = obstacle avoidance | — (`mission_manager` flies the waypoints and their tasks in OFFBOARD) | Follows the setpoints |
| **Continue** | `MAV_CMD_DO_PAUSE_CONTINUE` (193), `param1 = 1` | — (`mission_manager` ends a manual task or resumes after a pause; `offboard_control` requests OFFBOARD again if needed) | Back in OFFBOARD |

**Take-off altitude (px4 route).** PX4 interprets `NAV_TAKEOFF` `param7` as **AMSL** altitude. The Jetson
converts:

* With a global altitude (`vehicle_global_position.alt_valid`): `AMSL = current alt + altitude from the app`.
* Without one (indoors, no GPS): `NaN` is sent, PX4 uses `MIS_TAKEOFF_ALT` (1.5 m in
  `config/px4/indoor.params`), and the Jetson reports *"Take-off: no global altitude, PX4 uses
  MIS_TAKEOFF_ALT"* to the app.

---

## 3. Safety rules

Enforced on the **Jetson** (`CommandBridge.handle`) — they hold even if the app misbehaves or the command
comes from elsewhere:

| Condition | Result |
|---|---|
| No PX4 data for 2 s (DDS agent down, no `vehicle_land_detected`) | `TEMPORARILY_REJECTED` — *"no PX4 data (DDS agent?)"* |
| Take-off while **in the air** (`landed = false`) | `DENIED` — *"already flying"* |
| Land / Hold / RTL while **on the ground** | `DENIED` — *"vehicle is on the ground"* |
| A new command while the previous one is running | The new command **replaces** the old one (e.g. Land during take-off), reported as *"… superseded by …"* |
| PX4 rejects (pre-arm check, no position, RTL without GPS…) | The PX4 result is returned (`DENIED` / `FAILED`…) with the reason |

Additional checks in the **app** (`FlightCommands.qml`) to prevent operator mistakes:

* The button bar follows the flight state (PX4 `EXTENDED_SYS_STATE`):
  on the ground → **Take-off · Send WP**; in the air → **Send WP · Start mission · Hold · RTL · Land**.
* Take-off / Land / RTL require **holding the button for 1 s** in the confirm dialog. Hold acts immediately
  (emergency stop).
* Take-off runs a **pre-flight check** (`core/alerts/health.py`): a critical problem (PX4 lost, sensor
  failure, battery ≤ 15 %, no GPS fix outdoors) locks the button. Land is never locked.
* RTL is locked when the flight environment is **Indoor** (RTL needs GPS).
* State rules (`check_state`, applied to **both routes** once PX4 reports `EXTENDED_SYS_STATE`): Take-off in
  the air → blocked *"already flying"*; Land / Hold / RTL on the ground → blocked *"vehicle is on the
  ground"*. Unknown state → left to PX4 / the Jetson.
* Cross-route lock and per-button conditions: see the tables in section 1.
* The direct route has no Jetson rules — PX4 rejects commands itself when conditions are not met (pre-arm
  check, no position...); the reason appears under *Diagnostics → PX4 messages*.

The RC transmitter **always has priority**: the mode switch (Stabilized / Position) or Kill takes effect
regardless of commands from the app.

---

## 4. Acknowledgement protocol (ACK)

```
App                         Jetson                                PX4
 │ COMMAND_LONG (conf=0) ──►│                                      │
 │◄── ACK IN_PROGRESS ──────│ VehicleCommand ARM ─────────────────►│
 │                          │◄────────────── vehicle_command_ack ──│ ACCEPTED
 │◄── ACK IN_PROGRESS (0.5s)│ VehicleCommand NAV_TAKEOFF ─────────►│
 │                          │◄────────────── vehicle_command_ack ──│ ACCEPTED
 │◄── ACK ACCEPTED ─────────│ STATUSTEXT "Take-off: accepted by PX4"
```

| Side | Timeout | On timeout |
|---|---|---|
| App waits for an ACK (Jetson or PX4, per step) | 1.5 s of silence (`commands.ack_timeout_s` in `gcs.yaml`) | Resends (`confirmation` +1) — **also while IN_PROGRESS** (polls the state). Reports *"Jetson not responding"* only after **3 consecutive** silent sends (`retries`); any ACK resets the count |
| Jetson keeps the app informed | Resends `IN_PROGRESS` every 0.5 s while waiting for PX4 | — |
| Jetson waits for the PX4 ACK of each step | 3 s | Resends once, then returns `FAILED` *"no ACK from PX4"* |

* When the app resends the same command (no ACK seen, `confirmation > 0`) the Jetson **does not execute it
  again**: a running command returns `IN_PROGRESS`; a command that **just finished** (≤ 10 s,
  `RESULT_HOLD_S`) returns **the previous result**. Without this, a lost result ACK would make the resend
  look like a new command (a Take-off already in the air reported as *"already flying"*, or a second ARM +
  take-off if the land detector had not updated yet).
* Send WP: if the final `MISSION_ACK` is lost, the app resends the last item and the Jetson **resends the
  ACK** (≤ 10 s) without restarting the upload.
* The status line shows the time from press to result, e.g. *"✓ Take-off 5 m: accepted by Jetson (0.8 s)"*.
* Rejection reasons / results are also sent as `STATUSTEXT` (component 191) and appear under
  **Diagnostics → PX4 messages**; the command status is shown right below the button bar on the Map.

Result values (MAVLink `MAV_RESULT` = PX4 `VEHICLE_CMD_RESULT`):
`0 ACCEPTED · 1 TEMPORARILY_REJECTED · 2 DENIED · 3 UNSUPPORTED · 4 FAILED · 5 IN_PROGRESS`.

---

## 5. Waypoint upload (Send WP)

Standard MAVLink mission protocol; the Jetson requests each item:

```
App ── MISSION_COUNT(n) ─────────────► Jetson
    ◄─ MISSION_REQUEST_INT(0) ────────┘
    ── MISSION_ITEM_INT(0) ──────────►
    …                                   (lost packet → the Jetson requests again after 1.5 s, up to 5 times)
    ◄─ MISSION_ACK(ACCEPTED) ─────────  + STATUSTEXT "Mission: n waypoints stored on Jetson"
```

* Each item: `MAV_FRAME_GLOBAL_RELATIVE_ALT_INT`, `MAV_CMD_NAV_WAYPOINT`, lat/lon × 1e7, altitude (m).
* The Jetson publishes the list on `/gcs/mission` (`std_msgs/String`, JSON, transient-local QoS — late
  subscribers still get the latest list):
  ```json
  [{"seq": 0, "lat": 11.0529, "lon": 106.6660, "alt": 20.0, "command": 16, "frame": 6}, ...]
  ```
* **Tasks:** a waypoint with a pick task is sent with `command = MAV_CMD_USER_3` (31012) instead of
  `NAV_WAYPOINT`, with `param1` = 0 automatic / 1 manual hook, `param2` = search height and `param3` = hook height
  (m above ground). The Jetson keeps the command and params in the JSON list.
* **Flying the waypoints:** after Take-off (HOVER), the *Start mission* button sends `MAV_CMD_MISSION_START`;
  `mission_manager` converts lat/lon to local NED using the EKF origin (`/mission/geo_reference` from
  `offboard_control`), flies the points and their tasks in order and hovers at the last one. Progress comes back
  as `MISSION_CURRENT`, and requests to the operator (e.g. take control for a manual hook) as `STATUSTEXT`; the
  Map shows both in the mission banner with a Continue button. The mission is not uploaded to PX4 itself
  (uXRCE-DDS cannot upload missions). Full description: `offboard_control/docs/MISSION_PIPELINE.md`.

---

## 6. Testing

### 🧪 Test button (lab — checks that commands reach the Jetson)

The **🧪 Test** button next to *Land* is always enabled and toggles **test mode** (press again to turn it
off; not persisted — restarting the app turns it off). While on (*🧪 Test ON*):

* **Take-off / Land / Send WP** (the normal buttons) are **unlocked** and sent straight to the Jetson,
  **skipping the GCS-side checks** (DDS, GPS, cross-route lock, flight state). The 1 s hold is still
  required; the confirm dialog shows *🧪 TEST*.
* A panel above shows **📡 Ping Jetson** (`MAV_CMD_USER_1` = 31010 → the Jetson replies `ACCEPTED` +
  STATUSTEXT *"Link test OK: Jetson got ping #n"*, **without touching PX4**) and the 3 latest messages from
  the Jetson / PX4.
* The Jetson **still applies its own safety rules**: without PX4 it replies *"no PX4 data (DDS agent?)"*
  (which still proves the command arrived and the reply came back). **With PX4 over DDS and on the ground,
  Take-off WILL ARM — remove the propellers.**

Result: the status line under the buttons (*"✓ Link test: accepted by Jetson (0.4 s)"*, or *"Jetson not
responding"* = the command did not arrive / the ACK did not come back) plus the 3 latest messages in the
panel. On the Jetson: log lines `commands: Link test OK…` / `commands: -> PX4 cmd …`. An old Jetson build
replies *UNSUPPORTED* to the ping and the app still reports *"reached Jetson"*. For real flights the normal
buttons keep all their conditions.

### Test flight

**First time — REMOVE THE PROPELLERS**, drone on the floor, one person holding the RC with a hand on the
**Kill** switch.

1. Build and run on the Jetson:
   ```bash
   cd /home/drone_ws && colcon build --packages-select drone_telemetry && source install/setup.bash
   ros2 launch drone_bringup bringup.launch.py           # terminal 1 (includes MicroXRCEAgent)
   ros2 launch drone_telemetry telemetry.launch.py       # terminal 2
   ```
   The log must contain `commands: Take-off / Land / Hold / RTL + waypoint -> PX4 (DDS)`.
2. **Rule check:** press Land with the drone at rest → the app reports a rejection and PX4 messages show
   *"Land: vehicle is on the ground"*; the Jetson log has **no** `-> PX4 cmd` line.
3. **Take-off:** press Take-off and hold 1 s → motors arm and spool up; Jetson log `-> PX4 cmd 400 …` then
   `-> PX4 cmd 22 …`; the app reports *accepted*. Kill / disarm with the RC.
4. **Send WP:** add 2–3 points on the Map → Send WP → the app shows *"Uploaded n waypoints"*; on the Jetson:
   `ros2 topic echo /gcs/mission`.
5. Only then fit the propellers and fly low, ready to switch back to Stabilized.

**Automated tests**

```bash
cd ~/drone_ws/src/ground_station_ui && python3 -m pytest test/        # app
cd ~/drone_ws/src/drone_telemetry   && python3 -m pytest test/         # Jetson (test_commands.py)
```

---

## 7. Configuration

**Jetson — `drone_telemetry/config/telemetry.yaml`**

```yaml
commands:
  enabled: true                                   # false: ignore all ground commands
  takeoff_land_via: mission                       # mission | px4
  px4_system_id: 1
  command_topic: /fmu/in/vehicle_command
  ack_topic: ""                                   # empty: derived from px4_msgs MESSAGE_VERSION
  control_mode_topic: /fmu/out/vehicle_control_mode   # armed state
  land_topic: /fmu/out/vehicle_land_detected
  global_topic: /fmu/out/vehicle_global_position
  mission_topic: /gcs/mission
```

**App — `ground_station_ui/config/gcs.yaml`**

```yaml
commands:
  link: jetson
  target_system: 1
  target_component: 191
  ack_timeout_s: 1.5
  retries: 3
  takeoff_altitude_indoor: 1.5
  takeoff_altitude_outdoor: 5
```

---

## 8. PX4 message versions (important when updating firmware)

The drone runs **PX4 1.17**; `px4_msgs` in the Jetson workspace is newer (~1.18). DDS only matches topics
whose **type name and layout** are identical:

| Topic | Firmware | Workspace `px4_msgs` | Usable? |
|---|---|---|---|
| `vehicle_command` | v0 | v0 | ✓ |
| `vehicle_command_ack` | v0 | v1 (same fields) | ✓ |
| `vehicle_land_detected` | v0 | v0 | ✓ |
| `vehicle_global_position` | v0 | v0 | ✓ |
| `vehicle_control_mode` | v0 | v0 | ✓ (uses `flag_armed`) |
| `vehicle_status` (`_v1`) | v1 | **v4** | ✗ — not used |

When upgrading the firmware, compare `MESSAGE_VERSION` in `px4_msgs/msg/*.msg` with the topic suffixes
(`ros2 topic list | grep fmu`), or run PX4's `translation_node`.

---

## 9. Troubleshooting

| Symptom | Common cause | Check |
|---|---|---|
| App: *"Jetson not responding"* | `drone_telemetry` not running / not rebuilt / 433 radio unplugged | Jetson log has `commands: …` lines; Jetson link in the Connections tab |
| *"no PX4 data (DDS agent?)"* | MicroXRCEAgent not running or `/dev/ttyTHS0` lost | `ros2 topic hz /fmu/out/vehicle_land_detected` |
| PX4 rejects Take-off | Pre-arm check (battery, sensors, no position, RC mode) | Diagnostics tab → PX4 messages; QGC |
| Take-off *"no ACK from PX4"* | Command not reaching PX4 (wrong topic / message version mismatch) | `ros2 topic echo /fmu/out/vehicle_command_ack` |
| RTL rejected | No GPS / Home | Use only outdoors with a 3D GPS fix |
