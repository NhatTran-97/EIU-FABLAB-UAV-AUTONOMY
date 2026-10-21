"""Compile the uploaded mission (GCS waypoints + options) into a plan of steps.

Mission items come from /gcs/mission (JSON, one dict per item):
    {seq, lat, lon, alt (m above Home), command, frame, params: [p1, p2, p3, p4]}

Supported item commands:
    16    NAV_WAYPOINT     Navigate
    19    NAV_LOITER_TIME  Navigate, Hold(p1 s)
    31012 PICK (MAV_CMD_USER_3, project convention)
          p1 = 0 automatic hook, 1 manual hook; p2 = search height AGL (m); p3 = hook height AGL (m)
          Navigate, Descend(p2), Search, Align(p3), Hook (PayloadHook or ManualHandover), Ascend(waypoint z)
          Search / Align / automatic Hook fall back to ManualHandover when they fail.

Any other command, frame or out-of-range value rejects the whole mission.
"""

import math
from dataclasses import dataclass

from mission_pipeline.executor import PlanEntry
from mission_pipeline.steps import (Align, Ascend, Descend, HoldTime, ManualHandover, Navigate, PayloadHook,
                                    Search)

EARTH_RADIUS_M = 6371000.0     # same value as PX4 CONSTANTS_RADIUS_OF_EARTH

CMD_WAYPOINT = 16
CMD_LOITER_TIME = 19
CMD_PICK = 31012               # MAV_CMD_USER_3
FRAME_GLOBAL_RELATIVE_ALT = 6  # MAV_FRAME_GLOBAL_RELATIVE_ALT(_INT)
PICK_AUTO, PICK_MANUAL = 0, 1


@dataclass
class MissionOptions:
    """Options sent with Start mission; they apply to the whole mission. 0 means the node default."""
    avoidance: bool = False
    speed: float = 0.0               # m/s between waypoints
    xy_tolerance: float = 0.0        # m, horizontal distance at which a waypoint counts as reached
    z_tolerance: float = 0.0         # m, vertical distance at which a waypoint / height counts as reached

    @staticmethod
    def from_dict(d):
        d = d if isinstance(d, dict) else {}

        def positive(key):
            try:
                v = float(d.get(key) or 0.0)
            except (TypeError, ValueError):
                return 0.0
            return v if math.isfinite(v) and v > 0 else 0.0

        return MissionOptions(avoidance=bool(d.get('avoidance', False)), speed=positive('speed'),
                              xy_tolerance=positive('xy_tolerance'), z_tolerance=positive('z_tolerance'))


@dataclass
class CompileConfig:
    max_altitude: float              # m above Home, also the take-off limit
    max_distance: float              # m horizontally from the drone
    default_search_agl: float = 3.0  # m, used when p2 is missing
    default_hook_agl: float = 1.0    # m, used when p3 is missing
    min_agl: float = 0.5             # m, lowest allowed search / hook height


def global_to_local_ned(lat, lon, ref_lat, ref_lon):
    """Convert lat/lon (deg) to (x north, y east) in metres from the EKF origin.

    Same azimuthal equidistant projection as PX4 (MapProjection::project), so the result matches the
    position PX4 controls on.
    """
    lat_r, lon_r = math.radians(lat), math.radians(lon)
    rlat, rlon = math.radians(ref_lat), math.radians(ref_lon)
    cos_d_lon = math.cos(lon_r - rlon)
    arg = math.sin(rlat) * math.sin(lat_r) + math.cos(rlat) * math.cos(lat_r) * cos_d_lon
    c = math.acos(max(-1.0, min(1.0, arg)))
    k = 1.0 if abs(c) < 1e-12 else c / math.sin(c)
    x = k * (math.cos(rlat) * math.sin(lat_r) - math.sin(rlat) * math.cos(lat_r) * cos_d_lon) * EARTH_RADIUS_M
    y = k * math.cos(lat_r) * math.sin(lon_r - rlon) * EARTH_RADIUS_M
    return x, y


def _param(params, i, default):
    try:
        v = float(params[i])
    except (IndexError, TypeError, ValueError):
        return default
    return v if math.isfinite(v) and v > 0 else default


def compile_mission(items, geo_reference, ground_z, position, options, cfg, yaw=math.nan):
    """Return (plan, needs, '') or (None, set(), reason).

    plan  : list of PlanEntry
    needs : capabilities required by the plan: 'avoidance', 'rangefinder', 'marker', 'payload'
    """
    if not items:
        return None, set(), 'no waypoints uploaded (Send WP first)'
    if geo_reference is None:
        return None, set(), 'no GPS global reference (EKF)'
    if ground_z is None or position is None:
        return None, set(), 'unknown ground altitude (take off from GCS first)'
    ref_lat, ref_lon = geo_reference
    needs = {'avoidance'} if options.avoidance else set()
    plan = []
    for i, wp in enumerate(items):
        n = i + 1
        try:
            lat, lon, alt = float(wp['lat']), float(wp['lon']), float(wp['alt'])
            command = int(wp.get('command', CMD_WAYPOINT))
            frame = int(wp.get('frame', FRAME_GLOBAL_RELATIVE_ALT))
        except (KeyError, TypeError, ValueError):
            return None, set(), f'waypoint #{n} invalid'
        if frame != FRAME_GLOBAL_RELATIVE_ALT:
            return None, set(), f'waypoint #{n}: altitude frame {frame} not supported'
        if not 0.0 < alt <= cfg.max_altitude:
            return None, set(), f'waypoint #{n}: altitude {alt:.1f} m outside (0, {cfg.max_altitude:.0f}] m'
        x, y = global_to_local_ned(lat, lon, ref_lat, ref_lon)
        dist = math.hypot(x - position[0], y - position[1])
        if dist > cfg.max_distance:
            return None, set(), f'waypoint #{n} is {dist:.0f} m away (max {cfg.max_distance:.0f} m)'
        point = (x, y, ground_z - alt)
        params = wp.get('params') or []

        plan.append(PlanEntry(n, Navigate(point, yaw)))
        if command == CMD_WAYPOINT:
            continue
        if command == CMD_LOITER_TIME:
            plan.append(PlanEntry(n, HoldTime(_param(params, 0, 0.0))))
            continue
        if command != CMD_PICK:
            return None, set(), f'waypoint #{n}: command {command} not supported'

        mode = int(_param(params, 0, 0.0)) if params else PICK_AUTO
        search = _param(params, 1, cfg.default_search_agl)
        hook = _param(params, 2, cfg.default_hook_agl)
        if min(search, hook) < cfg.min_agl or hook > search or search > alt:
            return None, set(), (f'waypoint #{n}: pick heights invalid (hook {hook:.1f} <= search {search:.1f} '
                                 f'<= waypoint {alt:.1f} m, min {cfg.min_agl:.1f} m)')
        label = f'WP{n} pick'
        manual = ManualHandover(label)
        plan.append(PlanEntry(n, Descend(search), group=n))
        plan.append(PlanEntry(n, Search(), group=n, fallback=ManualHandover(label), skippable=True))
        plan.append(PlanEntry(n, Align(hook), group=n, fallback=ManualHandover(label), skippable=True))
        if mode == PICK_MANUAL:
            plan.append(PlanEntry(n, manual, group=n, skippable=True))
        else:
            plan.append(PlanEntry(n, PayloadHook(), group=n, fallback=ManualHandover(label), skippable=True))
            needs.add('payload')
        plan.append(PlanEntry(n, Ascend(point[2]), group=n))
        needs |= {'rangefinder', 'marker'}
    return plan, needs, ''
