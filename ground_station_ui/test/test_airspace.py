import json
import math

from ground_station_ui.core.maps.airspace import (bearing_distance, circle, load_dirs, parse_coord,
                                                   parse_geojson, parse_openair, zones_at)

SQUARE = {'type': 'FeatureCollection', 'features': [{
    'type': 'Feature',
    'properties': {'name': 'Khu A', 'kind': 'prohibited', 'lower': 'SFC', 'upper': '500 m'},
    'geometry': {'type': 'Polygon', 'coordinates': [[[106.0, 11.0], [106.1, 11.0], [106.1, 11.1],
                                                     [106.0, 11.1], [106.0, 11.0]]]},
}, {
    'type': 'Feature',
    'properties': {'name': 'Tron B', 'kind': 'restricted', 'radius_m': 1000},
    'geometry': {'type': 'Point', 'coordinates': [106.5, 11.5]},
}, {
    'type': 'Feature',
    'properties': {'name': 'OpenAIP CTR', 'type': 4},
    'geometry': {'type': 'MultiPolygon', 'coordinates': [[[[0, 0], [1, 0], [1, 1], [0, 0]]]]},
}]}


def test_geojson_polygon_circle_and_openaip_types():
    zones = parse_geojson(json.dumps(SQUARE), source='t.geojson')
    assert [(z.name, z.kind) for z in zones] == [('Khu A', 'prohibited'), ('Tron B', 'restricted'),
                                                 ('OpenAIP CTR', 'ctr')]
    a, b = zones[0], zones[1]
    assert a.contains(11.05, 106.05) and not a.contains(11.15, 106.05)
    assert a.upper == '500 m'
    # vung tron ban kinh 1 km: 900 m trong, 1100 m ngoai
    _, d_in = bearing_distance(11.5, 106.5, 11.5 + 900 / 111195, 106.5)
    assert d_in < 1000 and b.contains(11.5 + 900 / 111195, 106.5)
    assert not b.contains(11.5 + 1100 / 111195, 106.5)


def test_openair_points_circle_and_arc():
    text = """
* vi du OpenAir
AC P
AN VVP1 Test
AL SFC
AH 2000FT
DP 11:00:00 N 106:00:00 E
DP 11:06:00 N 106:00:00 E
DP 11:06:00 N 106:06:00 E
DP 11:00:00 N 106:06:00 E

AC R
AN Tron 2NM
V X=11:30:00 N 106:30:00 E
DC 2

AC Q
AN Cung
V X=12:00:00 N 107:00:00 E
V D=+
DP 12:00:00 N 107:00:00 E
DB 12:03:00 N 107:00:00 E, 12:00:00 N 107:03:04 E
"""
    zones = parse_openair(text, source='t.txt')
    assert [(z.name, z.kind) for z in zones] == [('VVP1 Test', 'prohibited'), ('Tron 2NM', 'restricted'),
                                                 ('Cung', 'danger')]
    sq, circ, sector = zones
    assert sq.lower == 'SFC' and sq.upper == '2000FT'
    assert sq.contains(11.05, 106.05) and not sq.contains(11.2, 106.05)
    assert circ.contains(11.5, 106.5) and not circ.contains(11.5 + 2.2 * 1852 / 111195, 106.5)
    assert sector.contains(12.02, 107.02)       # goc phan tu Dong-Bac
    assert not sector.contains(11.98, 107.02)   # phia Nam tam: ngoai cung


def test_parse_coord_formats():
    assert parse_coord('21:01:30 N 105:51:00 E') == (21 + 1.5 / 60, 105 + 51 / 60)
    assert parse_coord('10:45.5S 106:40.25W') == (-(10 + 45.5 / 60), -(106 + 40.25 / 60))


def test_circle_radius():
    for lat, lon in circle(11.0, 106.0, 2000):
        assert math.isclose(bearing_distance(11.0, 106.0, lat, lon)[1], 2000, rel_tol=1e-6)


def test_load_dirs_skips_bad_files(tmp_path):
    (tmp_path / 'a.geojson').write_text(json.dumps(SQUARE))
    (tmp_path / 'bad.geojson').write_text('{khong phai json')
    (tmp_path / 'note.md').write_text('bo qua')
    zones = load_dirs([tmp_path, tmp_path / 'khong-co'])
    assert len(zones) == 3
    assert [z.name for z in zones_at(zones, 11.05, 106.05)] == ['Khu A']
