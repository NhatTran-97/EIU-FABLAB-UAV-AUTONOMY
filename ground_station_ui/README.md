# ground_station_ui — Drone GCS

In-flight operation app for the drone (PySide6 + QML). It brings together what previously required QGC and
the LoRa GUI open side by side. **Firmware upload, PX4 configuration and calibration still use
QGroundControl** (close this app first: each serial port can be opened by only one app).

| Link | Transport | Content |
|---|---|---|
| `px4` | SiK radio 915 MHz | Standard PX4 MAVLink: heartbeat, mode, GPS, battery, sensors, altitude, STATUSTEXT |
| `jetson` | SiK radio 433 MHz (NETID 77) | `drone_telemetry`: status and rate of lidar / gimbal / camera / PX4-DDS, pose, commands |
| `sitl` | UDP 14550 | PX4 SITL on the laptop, for testing the UI without a drone |

Links and map layers are configured in [config/gcs.yaml](config/gcs.yaml); no code changes needed.

## Offline map and waypoints

Same approach as QGC: an online map plus a **prefetched cache**. QtLocation loads tiles from a local tile
server inside the app; the tile server serves tiles from `.mbtiles` files (SQLite) in the package's
`data/maps/`, and downloads missing tiles from the online source (when online) and stores them. Offline,
only the cache is used.

- **Layers** (chosen on the **Settings** tab, remembered): *Hybrid* (default: Google imagery, the same source
  as QGC and the sharpest at the flying field, plus Esri labels) and *Azure* (Maxar/Bing imagery, needs a
  key, see below). The ⚑ button shows / hides the waypoint list. Composite layers (Hybrid = Google imagery +
  Esri road/place labels; Azure) are merged by the tile server from several sources, each with its own cache.
- **Azure Maps**: key in `config/keys.yaml` (`AZURE_MAPS_KEY: "..."`, git-ignored) or the `AZURE_MAPS_KEY`
  environment variable. Without a key the layer is cache-only; with an empty cache it is hidden.
- **Zoom beyond 19**: QtLocation upscales z19 tiles; detail is limited by the source imagery.
- **Before going to the field** (while online): Map tab → pan to the flying area → ⬇ button → choose zoom →
  *Download*. Do it for each layer (Hybrid downloads all three sources). A 2×2 km area up to zoom 19 is a
  few tens of MB per layer.
- **Own drone orthophoto** (sharpest, and licence-free): WebODM → `orthophoto.tif` → `gdal2tiles.py --xyz`
  / `rio mbtiles` → add a layer with `file: ortho.mbtiles` (no `url`) in `gcs.yaml`.
- **Go to a position**: type `11.052919, 106.666057` (copied from Google Maps works) in the top-left box →
  Enter: the map flies there and drops a red pin; *＋ Waypoint* adds that point. The last viewed area is
  remembered.
- **Map view**: right-drag rotates (horizontal) and tilts (vertical) the map; ⟲N resets it. On a tilted map
  the drone is drawn as the 3D model, raised above its ground shadow by its altitude. The marker scales
  with the map (real size × exaggeration, clamped to a minimum on-screen size).
- **Flown path** (orange): recorded while flying, one segment per flight, drawn on the map and saved as GPX to
  `data/tracks/` when the flight ends. Positions come from PX4 (915 / SITL) or, without that link, from the
  Jetson; without the PX4 heartbeat, "flying" means more than 0.5 m above Home. The legend at the bottom left has
  a *Flown path* checkbox (remembered), the length flown, ● REC while recording and *Clear*; the 〰 button does
  the same. The planned path (Home → waypoints) is drawn in violet.
- **Flight environment** (Settings tab: Auto / Indoor / Outdoor). *Auto* follows the loaded PX4 parameter
  set (`EKF2_GPS_CTRL = 0` or `EKF2_HGT_REF = 2` → indoor; GPS on + baro/GPS height → outdoor), or the GPS
  fix until the parameters are read. It selects the altitude source (indoor = rangefinder via the EKF,
  outdoor = above Home), the default take-off altitude and the GPS warning colour. The app reads
  `EKF2_GPS_CTRL` / `EKF2_HGT_REF` from PX4 and **warns if the loaded parameter set does not match**
  (e.g. flying outdoors without loading `outdoor.params`).
- **Instruments** (top right): attitude (roll / pitch), compass, altitude, vertical and ground speed, from
  PX4 `ATTITUDE`, `VFR_HUD` and `LOCAL_POSITION_NED`. Without the PX4 link, attitude and heading fall back
  to the Jetson data.
- **Waypoints**: *＋ Add point*, then click on the map; drag a point to move it; right-click to delete;
  edit altitudes in the list on the right.
- **Tasks at waypoints**: each waypoint in the list has a *Task*: *Fly through* (default), *Pick (auto hook)* or
  *Pick (manual hook)*, with the search and hook heights above ground. A pick makes the drone descend, find the
  ArUco marker, align over it, hook the load (automatically, or the pilot does it and then presses Continue) and
  climb back. Waypoints with a task show an **A** or **M** badge on the map. Details:
  `offboard_control/docs/MISSION_PIPELINE.md`.
- **Prohibited / restricted airspace** (⛔ button: show / hide, long-press = reload files): copy `.geojson`
  or OpenAir (`.txt`) files into `data/airspace/`. Colours follow cambay.mod.gov.vn: red (prohibited),
  orange (restricted), yellow (danger), blue (airport CTR); checkboxes in the legend show / hide each kind;
  waypoints / pins inside a zone show ⚠. GeoJSON: Polygon / MultiPolygon, or Point + `radius_m`;
  `properties: name, kind, lower, upper`. The official Ministry of Defence source is
  http://cambay.mod.gov.vn (no public API) → redraw the zones around the flying area with geojson.io. The
  app only **warns**; it does not replace a flight permit.
- Terms of use: Esri / Google / Bing restrict bulk downloading — choose the sources in `gcs.yaml` accordingly.

## 3D page (replaces RViz)

Ground grid, `map` axes (PX4 EKF origin), the drone model with its `base_link` axes, the flight trail (blue)
and the planned path of the last Send WP (violet, with a sphere and a drop line per waypoint, green for pick
tasks; *Planned path* checkbox). The grid grows with the area flown and the planned path (40, 60, 100, 150 m…).
The planned path is placed by anchoring the GPS waypoints on the drone's current GPS position and local pose,
since the EKF origin itself is not received; it needs a GPS position and the 3D pose.
left drag rotates, wheel zooms, right drag pans; *Follow drone* / *Clear trail*. Source: the Jetson
subscribes to `/fmu/out/vehicle_odometry` (PX4 over uXRCE-DDS) → `DRONE_LOCAL_POSE` (id 45001, ~57 B, 5 Hz
≈ 300 B/s) → radio 433 → app. Without the Jetson, PX4 `LOCAL_POSITION_NED` over the 915 link (or SITL) is
used instead.

Conventions: data is transmitted in **NED** (PX4 as is); the app converts **once** to **ENU / FLU** like ROS
(`core/geometry/frames.py`, same as `px4_ros_com`) so lidar, ArUco and TF data from the Jetson can be used
directly. Qt Quick 3D is Y-up: scene = (E, U, −N) × 100 (1 m = 100 units), only in `qt/geometry3d.py` and
`View3DPage.qml`.

The drone model is built in Blender from parameters and real part models: see
[tools/drone_model/README.md](tools/drone_model/README.md).

Planned: ArUco markers (`Axes3D`).

## EIU branding

`images/logo.png` (original logo) · `images/logo_dark.png` (dark-background version: navy → white, yellow
stripes kept, generated from the original) · `images/eiu.png` (campus photo). Small logo at the top of the
menu; ~7 s splash at startup (campus photo + logo, click to skip, `holdMs` in
`qml/components/Splash.qml`); Settings → About. No large images or logos on the operational pages (Map, 3D,
Telemetry, Diagnostics).

**App icon:** `images/icons/drone-gcs-*.png` (generated from `images/logo_desktop.png`, background outside
the frame made transparent). The window uses this icon; for an application-menu / dock launcher run
`scripts/install_desktop.sh` (inside `drone_env`; remove with `--uninstall`).

## Alerts and safety

- **Top bar:** large flight mode text + **ARMED** (red) / DISARMED badge; the battery shows ⚠ (≤ 25 %) /
  ⛔ (≤ 15 %) icons, not just a colour change.
- **Alert Center** (⚠ n button on the right): current problem list, from the same source as **System
  Health** at the top of the Diagnostics tab and the **pre-flight check** in the Take-off confirm dialog (a
  critical problem locks the take-off button). Rules: `core/alerts/health.py`.
- **Voice** (English, enabled / disabled in Settings): mode changes, arm / disarm, link loss, battery
  30/20/15/10 %.

## Take-off / Land / Send waypoints

**Flight commands have two routes:** click the **PX4 box** in the top bar → Take-off / Land **directly to
PX4 over radio 915** (ARM → `NAV_TAKEOFF`, `NAV_LAND`, ACK from PX4); the **buttons under the Map** go **via
the Jetson (433)** as in the table below. **Cross-route lock:** the route that took off / landed owns control
until the drone has landed and disarmed; the other route is locked. Jetson buttons are also locked when the
Jetson is lost, the Jetson has no PX4 data (DDS), or there is no GPS fix outdoors.

How it works, safety rules, the ACK protocol and how to test: [docs/flight_commands.md](docs/flight_commands.md).

The button bar at the bottom centre of the Map tab (Jetson route) is locked by the conditions above, with
the reason shown under the buttons. Take-off / Land must be **held for 1 second** to confirm; Send WP warns
if a point lies inside a prohibited / restricted zone.

| Button | Jetson route: MAVLink to sysid 1 / compid 191 (link `jetson`) | Jetson reply |
|---|---|---|
| Take-off | `COMMAND_LONG` `MAV_CMD_NAV_TAKEOFF` (22), `param7` = altitude in m | `COMMAND_ACK` |
| Land | `COMMAND_LONG` `MAV_CMD_NAV_LAND` (21) | `COMMAND_ACK` |
| Hold | `COMMAND_LONG` `MAV_CMD_NAV_LOITER_UNLIM` (17) → Jetson: `DO_SET_MODE` AUTO.LOITER | `COMMAND_ACK` |
| RTL | `COMMAND_LONG` `MAV_CMD_NAV_RETURN_TO_LAUNCH` (20), outdoors only (needs GPS) | `COMMAND_ACK` |
| Send WP | `MISSION_COUNT` → (Jetson requests) `MISSION_REQUEST_INT` → `MISSION_ITEM_INT` (`GLOBAL_RELATIVE_ALT_INT`, `NAV_WAYPOINT`) | `MISSION_ACK` |
| Start mission | `COMMAND_LONG` `MAV_CMD_MISSION_START` (300), `param3` bit 0 = obstacle avoidance, `param4` = mission speed (m/s), `param5` / `param6` = arrival tolerance xy / z (m) | `COMMAND_ACK` |
| Continue | `COMMAND_LONG` `MAV_CMD_DO_PAUSE_CONTINUE` (193), `param1 = 1` | `COMMAND_ACK` |
| Live settings | `COMMAND_LONG` `MAV_CMD_USER_2` (31011), `param1` = speed, `param2` / `param3` = tolerance xy / z; sent 0.6 s after the last change of speed or tolerance while a mission is active or paused, through its own sender | `COMMAND_ACK` |

Without a reply the command is resent (`commands.ack_timeout_s`, `retries` in `gcs.yaml`); after the last
retry the app reports "Jetson not responding". On the Jetson, `drone_telemetry/commands.py` forwards
Take-off / Land to `mission_manager`, which flies them in OFFBOARD through `offboard_control`; Hold / RTL go
to PX4 as `/fmu/in/vehicle_command`. The Jetson replies with `COMMAND_ACK` (IN_PROGRESS while waiting, then
the result) and a `STATUSTEXT` with the reason. Jetson safety rules: Take-off only on the ground; Land /
Hold / RTL only in the air; no PX4 data → rejected. Waypoints are stored on the Jetson (`/gcs/mission`,
JSON) and flown by `mission_manager` on *Start mission*, from HOVER.
The button bar follows the flight state: on the ground → Take-off / Send WP; in the air → Send WP / Start
mission / Hold / RTL / Land.

**Mission speed:** set in the waypoint list (*Mission speed*, 0.5–10 m/s, default `mission.default_speed` in
`gcs.yaml`). It is sent with Start mission and applies to every leg of the mission; the Jetson limits it to its
`max_cruise_speed` (5 m/s), and PX4 `MPC_XY_VEL_MAX` remains the hard limit.

**Arrival tolerance:** also in the waypoint list (*XY* / *Z*, default `mission.xy_tolerance` 0.75 m and
`mission.z_tolerance` 0.5 m in `gcs.yaml`): the distance at which a waypoint counts as reached. Sent with Start
mission for the whole mission; the Jetson accepts 0.2–5 m (XY) and 0.1–3 m (Z).

**Start mission options:** the confirm dialog shows the speed and tolerances, how many pick tasks the mission contains, and has an
*Obstacle avoidance* checkbox. The Jetson refuses the mission with a reason if an option or a task needs a node
that is not running (avoidance, ArUco localizer, payload) or that is disabled on the drone.

**Mission banner and Continue:** while a mission runs, a banner above the buttons shows the current waypoint
(`MISSION_CURRENT` from the Jetson). When the mission waits for the operator, the banner turns orange, shows the
Jetson's message and a **▶ Continue** button:

- *manual pick*: the drone holds over the marker and asks the pilot to take control; after hooking, the pilot
  flips the RC switch back to Offboard, or the operator presses Continue;
- *pilot override*: the pilot moved the sticks during the mission; Continue (or the RC switch to Offboard)
  resumes from where the drone is.

## Data inside the package

`data/maps/*.mbtiles` (offline map cache), `data/airspace/` (airspace), `data/tracks/` (GPX flight tracks),
`config/keys.yaml` (API keys). Copying the whole `ground_station_ui` directory to another machine is enough
to run it with the downloaded maps. `*.mbtiles`, `data/tracks/` and `keys.yaml` are git-ignored (large /
private), so they are not included when shared through git.

## Running

```bash
pip install -r requirements.txt     # PySide6, pyserial, pyyaml, Pillow (PySide6 is not in rosdep)

# Build as a ROS 2 package
cd ~/drone_ws && colcon build --packages-select ground_station_ui
source install/setup.bash
ros2 run ground_station_ui gcs

# Or run directly from source (edit QML/Python and rerun immediately)
python3 ~/drone_ws/src/ground_station_ui/ground_station_ui/main.py
```

First run: **Links** tab → choose the port → Connect. The port is remembered and reconnected next time.
Quit: close the window or press `Ctrl+C` in the terminal.

Testing with SITL: `cd ~/px4/PX4-Autopilot && make px4_sitl gz_x500` → Links tab → `PX4 SITL` → Connect.

## Architecture

```
┌─ QML ── one page per feature (qml/pages), menu generated from qt/features.py ───────┐
├─ qt/ ── adapters: FactGroup -> QML (values / items), updated at most at ui_rate_hz ──┤
├─ core/ (pure Python, tested with pytest, no Qt) ─────────────────────────────────────┤
│  vehicle/  Vehicle = FactGroups: status, gps, home, battery, sensors, flight,          │
│            attitude, localPose, track, messages, companion. Each subscribes itself.    │
│  mission/  Mission: waypoint list being edited; uploader (MAVLink mission protocol)   │
│  maps/     MBTiles cache, TileServer (localhost), Prefetcher (area download)           │
│  router.py MessageRouter: subscribe('GPS_RAW_INT', handler) — no long if/elif chains   │
│  links/    Link (base class) -> SerialLink, UdpLink; LinkManager built from gcs.yaml   │
└─────────────────────────────────────────────────────────────────────────────────────┘
```

**Data flow:** each link thread reads and parses → sends **batches** of messages to the main thread →
`MessageRouter.dispatch` → FactGroups update `values` / `items` and set `dirty` → `UiTicker` (5 Hz) calls
`tick()` (heartbeat age, STALE...) and then the adapters' `flush()` → QML repaints. The whole domain runs on
the main thread, so no locks are needed; threads exist only in the link layer. Flight paths (map track, 3D
trail) are passed to QML incrementally (`PathLog` / `PathAdapter`), so the UI cost does not grow with
flight time.

**Extending:**

| To add | Do |
|---|---|
| A new link (serial / UDP) | add it to `config/gcs.yaml` |
| A new link type (TCP, .tlog replay...) | subclass `core/links/base.py:Link` (4 I/O methods) + register it in `LINK_TYPES` |
| New PX4 data | one `FactGroup` in `core/vehicle/groups.py`, added to `Vehicle.groups`, plus a property in `qt/adapters.py:VehicleAdapter` |
| A new feature / page | one entry in `qt/features.py` + one `qml/pages/*.qml` file |
| Several drones | **not supported** — needs a VehicleRegistry / routing by (link, sysid); currently one `Vehicle` with explicit identity (see below) |

**Data source identity** (`core/vehicle/vehicle.py`, `vehicle:` section of `gcs.yaml`): the autopilot is
accepted only on `autopilot_links` (order = priority — real radio before SITL; switches only after the
current link is silent for > 5 s); PX4 messages are filtered by (link, sysid, compid); Jetson data
(`DRONE_*`) only from `companion_link` with compid 191 and the same sysid. The active source is shown on the
Flight Controller card (Diagnostics). `gcs.yaml` is validated at startup (`core/config.py`); errors are
reported precisely and the app does not start.

**Performance:** parsing happens on the link threads and messages are handed over in batches; the UI
updates only on change and at most at `ui_rate_hz`; QML Repeaters use a **count** as the model so delegates
are not recreated on every update (ComboBox selections are kept, less CPU).

## MAVLink dialect

`ground_station_ui/core/mavlink/drone_dialect.py` is a copy of
`drone_telemetry/drone_telemetry/drone_dialect.py` (generated from `drone_telemetry/mavlink/drone_telemetry.xml`).
Edit the XML → run `scripts/gen_dialect.sh` in `drone_telemetry` → copy the `.py` file here. Both ends must
use **the same** file.

## Safety

Each link sends the GCS HEARTBEAT on its own thread. If the app is closed or hangs in flight, PX4 loses the
GCS heartbeat and reacts according to `NAV_DLL_ACT` — check this parameter in QGC before flying.

## Tests

```bash
cd ~/drone_ws/src/ground_station_ui && python3 -m pytest test/
# or through ROS: colcon test --packages-select ground_station_ui (runs all of test/)
```

## Roadmap

1. ✅ Links, status bar, Diagnostics, flight values; layered architecture
2. ✅ Offline map (MBTiles cache, area prefetch), drone position, Home, waypoint editing on the map
3. ✅ Take-off / Land / Send waypoints to the Jetson (ACK + retries); missions flown by `mission_manager`
4. ✅ 3D page with the real drone model; map track with GPX export
5. HUD, RTSP video
6. `.tlog` recording / replay (replay link) for debugging without a drone
