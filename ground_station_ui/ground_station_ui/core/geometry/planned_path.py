"""Planned path (uploaded waypoints) in the local ENU frame of the 3D view.

The 3D view uses the PX4 EKF local frame, whose GPS origin the ground station does not receive. The path is
therefore anchored on the drone: at the same moment the drone has a GPS position (lat, lon, height above Home)
and a local position (e, n, u), so for any waypoint

    e_wp = e_drone + east offset (drone -> waypoint)      n_wp = n_drone + north offset
    u_wp = u_home + alt_wp,   u_home = u_drone - drone height above Home

Offsets use an equirectangular approximation, accurate to centimetres over a few hundred metres.
"""

import math

EARTH_RADIUS_M = 6371000.0


def offset_m(lat0, lon0, lat, lon):
    """(east, north) in metres from (lat0, lon0) to (lat, lon)."""
    north = math.radians(lat - lat0) * EARTH_RADIUS_M
    east = math.radians(lon - lon0) * EARTH_RADIUS_M * math.cos(math.radians((lat + lat0) / 2))
    return east, north


def planned_path_enu(waypoints, home, anchor):
    """Waypoints [{lat, lon, alt (above Home), task}] -> (points, markers) in local ENU metres.

    home   : (lat, lon) or None; when given, the path starts at Home on the ground, as on the map.
    anchor : (lat, lon, height_above_home, e, n, u) of the drone at one instant.
    points : [[e, n, u], ...] polyline (Home first, then the waypoints)
    markers: [{seq, e, n, u, task}] one per waypoint
    """
    lat0, lon0, rel, e0, n0, u0 = anchor
    u_home = u0 - rel
    points, markers = [], []
    if home is not None:
        de, dn = offset_m(lat0, lon0, home[0], home[1])
        points.append([round(e0 + de, 2), round(n0 + dn, 2), round(u_home, 2)])
    for i, wp in enumerate(waypoints):
        de, dn = offset_m(lat0, lon0, wp['lat'], wp['lon'])
        p = [round(e0 + de, 2), round(n0 + dn, 2), round(u_home + float(wp['alt']), 2)]
        points.append(p)
        markers.append({'seq': i + 1, 'e': p[0], 'n': p[1], 'u': p[2], 'task': wp.get('task', 'none')})
    return points, markers
