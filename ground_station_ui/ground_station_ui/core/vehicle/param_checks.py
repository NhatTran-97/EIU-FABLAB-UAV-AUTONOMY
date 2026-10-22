"""PX4 safety parameters: what RTL, the geofence and landing will actually do, checked against each other,
against the mission and against the project parameter file (config/px4/<indoor|outdoor>.params).

Pure Python. check(...) returns one row per parameter for the Telemetry page and the list of problems for the
alerts. No threshold is invented here: every rule compares two values the operator set (a PX4 parameter with
another one, with the mission, or with the parameter file).
"""

import math

# name, label, unit, what it controls
SAFETY_PARAMS = (
    ('RTL_RETURN_ALT', 'RTL climb altitude', 'm', 'height above Home flown back at'),
    ('RTL_DESCEND_ALT', 'RTL descend altitude', 'm', 'height above Home before landing'),
    ('RTL_LAND_DELAY', 'RTL land delay', 's', '-1 = hover above Home, never land'),
    ('RTL_CONE_ANG', 'RTL cone angle', 'deg', 'lower climb near Home; 0 = always climb fully'),
    ('GF_MAX_VER_DIST', 'Geofence height', 'm', 'ceiling above Home'),
    ('GF_MAX_HOR_DIST', 'Geofence radius', 'm', 'distance from Home'),
    ('GF_ACTION', 'Geofence action', '', '0 none, 1 warn, 2 hold, 3 RTL, 4 terminate, 5 land'),
    ('COM_DISARM_LAND', 'Disarm after landing', 's', '<= 0: never disarms by itself'),
    ('MPC_XY_VEL_MAX', 'Max horizontal speed', 'm/s', 'hard limit, also in OFFBOARD'),
    ('MPC_XY_P', 'Position P gain', '', 'must equal px4_xy_p on the Jetson'),
)
NAMES = tuple(p[0] for p in SAFETY_PARAMS)


def load_params_file(path):
    """QGC parameter file (vehicle, component, name, value, type per line) -> {name: value}."""
    out = {}
    try:
        with open(path) as f:
            for line in f:
                parts = line.split()
                if len(parts) >= 4 and not line.startswith('#'):
                    try:
                        out[parts[2]] = float(parts[3])
                    except ValueError:
                        pass
    except OSError:
        pass
    return out


def _geo_dist(lat1, lon1, lat2, lon2):
    k = math.pi / 180 * 6371000.0
    return math.hypot((lat2 - lat1) * k, (lon2 - lon1) * k * math.cos(math.radians((lat1 + lat2) / 2)))


def check(values, expected=None, mission=None, home=None, speed=None):
    """values: {name: value or None} read from PX4; expected: {name: value} from the parameter file;
    mission: [{lat, lon, alt}] waypoints (alt above Home); home: (lat, lon) or None; speed: mission speed.

    Returns (rows, problems):
        rows     [{name, label, unit, value, expected, level: ok | warn | unknown, note}]
        problems [text] for the alerts
    """
    expected = expected or {}
    mission = mission or []
    v = {n: values.get(n) for n in NAMES}
    notes = {n: [] for n in NAMES}

    def warn(name, text):
        notes[name].append(text)

    if v['RTL_LAND_DELAY'] is not None and v['RTL_LAND_DELAY'] < 0:
        warn('RTL_LAND_DELAY', 'RTL hovers above Home and never lands')
    if v['COM_DISARM_LAND'] is not None and v['COM_DISARM_LAND'] <= 0:
        warn('COM_DISARM_LAND', 'the drone stays armed after landing')
    if None not in (v['RTL_RETURN_ALT'], v['GF_MAX_VER_DIST']) and v['GF_MAX_VER_DIST'] > 0 \
            and v['RTL_RETURN_ALT'] >= v['GF_MAX_VER_DIST']:
        warn('RTL_RETURN_ALT', f"RTL climbs to the geofence ceiling ({v['GF_MAX_VER_DIST']:g} m)")
    if mission and v['GF_MAX_VER_DIST'] is not None and v['GF_MAX_VER_DIST'] > 0:
        top = max(float(w['alt']) for w in mission)
        if top >= v['GF_MAX_VER_DIST']:
            warn('GF_MAX_VER_DIST', f'waypoint at {top:g} m reaches the geofence ceiling')
    if mission and home and v['GF_MAX_HOR_DIST'] is not None and v['GF_MAX_HOR_DIST'] > 0:
        far = max(_geo_dist(home[0], home[1], w['lat'], w['lon']) for w in mission)
        if far >= v['GF_MAX_HOR_DIST']:
            warn('GF_MAX_HOR_DIST', f'waypoint {far:.0f} m from Home is outside the geofence')
    if speed and v['MPC_XY_VEL_MAX'] is not None and speed > v['MPC_XY_VEL_MAX']:
        warn('MPC_XY_VEL_MAX', f'mission speed {speed:g} m/s is above this limit')

    rows, problems = [], []
    for name, label, unit, what in SAFETY_PARAMS:
        value, exp = v[name], expected.get(name)
        if value is not None and exp is not None and not math.isclose(value, exp, rel_tol=1e-3, abs_tol=1e-3):
            warn(name, f'parameter file has {exp:g}')
        level = 'unknown' if value is None else ('warn' if notes[name] else 'ok')
        rows.append({'name': name, 'label': label, 'unit': unit, 'value': value, 'expected': exp,
                     'level': level, 'note': '; '.join(notes[name]) or what})
        problems += [f'{label}: {t}' for t in notes[name]]
    return rows, problems
