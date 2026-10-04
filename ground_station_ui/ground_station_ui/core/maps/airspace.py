"""Vung cam / han che bay: doc tu file, ve len ban do, canh bao waypoint nam trong vung.

Nguon du lieu (dat file vao thu muc trong gcs.yaml -> airspace.dirs):
  *.geojson   Polygon / MultiPolygon, hoac Point + properties.radius_m (vung tron).
              properties: name, kind (prohibited|restricted|danger|ctr|military|other),
              lower / upper (chuoi tu do, vd "SFC", "500 m"). Ve tay: geojson.io
              Ban xuat GeoJSON cua OpenAIP (type = so) cung doc duoc.
  *.txt *.openair
              dinh dang OpenAir (AC/AN/AL/AH/DP/V/DC/DA/DB) -- dinh dang chuan cua
              du lieu vung tro hang khong (AIP), OpenAIP xuat duoc.

Moi vung chi la polygon (lat, lon); vung tron / cung tron duoc chia thanh nhieu diem.
"""

import json
import logging
import math
import re
from pathlib import Path

log = logging.getLogger(__name__)

EARTH_R = 6371008.8
NM = 1852.0
ARC_STEP_DEG = 5.0

KINDS = ('prohibited', 'restricted', 'danger', 'military', 'ctr', 'other')

# OpenAir AC -> kind
_OPENAIR_CLASS = {
    'P': 'prohibited', 'R': 'restricted', 'Q': 'danger', 'D': 'ctr', 'CTR': 'ctr',
    'C': 'ctr', 'B': 'ctr', 'A': 'ctr',
}
# OpenAIP GeoJSON: properties.type la so
_OPENAIP_TYPE = {1: 'restricted', 2: 'danger', 3: 'prohibited', 4: 'ctr'}
_KIND_ALIASES = {
    'p': 'prohibited', 'prohibited': 'prohibited', 'cam': 'prohibited', 'no-fly': 'prohibited',
    'r': 'restricted', 'restricted': 'restricted', 'han che': 'restricted',
    'q': 'danger', 'd': 'danger', 'danger': 'danger', 'nguy hiem': 'danger',
    'military': 'military', 'quan su': 'military', 'ctr': 'ctr', 'airport': 'ctr',
}


class Zone:
    __slots__ = ('name', 'kind', 'lower', 'upper', 'polygon', 'source', '_bbox')

    def __init__(self, name, kind, polygon, lower='', upper='', source=''):
        self.name = name or ''          # o luoi tu trang BQP thuong khong co ten
        self.kind = kind if kind in KINDS else 'other'
        self.polygon = polygon                       # [(lat, lon), ...] khong lap diem dau
        self.lower, self.upper, self.source = str(lower or ''), str(upper or ''), source
        lats = [p[0] for p in polygon]
        lons = [p[1] for p in polygon]
        self._bbox = (min(lats), min(lons), max(lats), max(lons))

    def contains(self, lat, lon):
        s, w, n, e = self._bbox
        if not (s <= lat <= n and w <= lon <= e):
            return False
        inside = False
        pts = self.polygon
        j = len(pts) - 1
        for i in range(len(pts)):
            yi, xi = pts[i]
            yj, xj = pts[j]
            if (yi > lat) != (yj > lat) and lon < (xj - xi) * (lat - yi) / (yj - yi) + xi:
                inside = not inside
            j = i
        return inside

    def to_dict(self):
        return {'name': self.name, 'kind': self.kind, 'lower': self.lower, 'upper': self.upper,
                'source': self.source, 'path': [{'lat': a, 'lon': b} for a, b in self.polygon]}


# --- hinh hoc -----------------------------------------------------------------------------
def destination(lat, lon, bearing_deg, dist_m):
    """Diem cach (lat, lon) dist_m theo huong bearing (do, tinh tu Bac, theo chieu kim dong ho)."""
    d = dist_m / EARTH_R
    b = math.radians(bearing_deg)
    p1, l1 = math.radians(lat), math.radians(lon)
    p2 = math.asin(math.sin(p1) * math.cos(d) + math.cos(p1) * math.sin(d) * math.cos(b))
    l2 = l1 + math.atan2(math.sin(b) * math.sin(d) * math.cos(p1), math.cos(d) - math.sin(p1) * math.sin(p2))
    return math.degrees(p2), (math.degrees(l2) + 540.0) % 360.0 - 180.0


def bearing_distance(lat1, lon1, lat2, lon2):
    p1, p2 = math.radians(lat1), math.radians(lat2)
    dl = math.radians(lon2 - lon1)
    y = math.sin(dl) * math.cos(p2)
    x = math.cos(p1) * math.sin(p2) - math.sin(p1) * math.cos(p2) * math.cos(dl)
    a = math.sin((p2 - p1) / 2) ** 2 + math.cos(p1) * math.cos(p2) * math.sin(dl / 2) ** 2
    return (math.degrees(math.atan2(y, x)) + 360.0) % 360.0, 2 * EARTH_R * math.asin(math.sqrt(a))


def arc(lat, lon, radius_m, start_deg, end_deg, clockwise=True):
    """Cac diem tren cung tron tu start den end (bearing, do)."""
    if clockwise:
        sweep = (end_deg - start_deg) % 360.0 or 360.0
    else:
        sweep = -((start_deg - end_deg) % 360.0 or 360.0)
    n = max(2, int(abs(sweep) / ARC_STEP_DEG) + 1)
    return [destination(lat, lon, start_deg + sweep * i / n, radius_m) for i in range(n + 1)]


def circle(lat, lon, radius_m):
    return arc(lat, lon, radius_m, 0.0, 360.0)[:-1]


# --- GeoJSON ------------------------------------------------------------------------------
def _ring(coords):
    pts = [(float(c[1]), float(c[0])) for c in coords]
    if len(pts) > 1 and pts[0] == pts[-1]:
        pts.pop()
    return pts


def _kind_from_props(p):
    k = p.get('kind', p.get('type', p.get('class', '')))
    if isinstance(k, (int, float)):
        return _OPENAIP_TYPE.get(int(k), 'other')
    return _KIND_ALIASES.get(str(k).strip().lower(), str(k).strip().lower())


def _limit(v):
    if isinstance(v, dict):          # OpenAIP: {"value": 1500, "unit": 1, "referenceDatum": 0}
        return f"{v.get('value', '')}"
    return v or ''


def parse_geojson(text, source=''):
    data = json.loads(text)
    feats = data.get('features', [data] if data.get('type') == 'Feature' else [])
    zones = []
    for f in feats:
        g, p = f.get('geometry') or {}, f.get('properties') or {}
        name = p.get('name', p.get('ten', ''))
        kind = _kind_from_props(p)
        lower = _limit(p.get('lower', p.get('lowerLimit', '')))
        upper = _limit(p.get('upper', p.get('upperLimit', '')))
        t = g.get('type')
        rings = []
        if t == 'Polygon':
            rings = [g['coordinates'][0]]
        elif t == 'MultiPolygon':
            rings = [poly[0] for poly in g['coordinates']]
        elif t == 'Point' and p.get('radius_m'):
            lon, lat = g['coordinates'][:2]
            zones.append(Zone(name, kind, circle(float(lat), float(lon), float(p['radius_m'])),
                              lower, upper, source))
            continue
        for r in rings:
            pts = _ring(r)
            if len(pts) >= 3:
                zones.append(Zone(name, kind, pts, lower, upper, source))
    return zones


# --- OpenAir ------------------------------------------------------------------------------
_COORD = re.compile(
    r'(\d+(?:\.\d+)?)(?::(\d+(?:\.\d+)?))?(?::(\d+(?:\.\d+)?))?\s*([NS])\s*[, ]?\s*'
    r'(\d+(?:\.\d+)?)(?::(\d+(?:\.\d+)?))?(?::(\d+(?:\.\d+)?))?\s*([EW])', re.I)


def _dms(d, m, s):
    return float(d) + float(m or 0) / 60.0 + float(s or 0) / 3600.0


def parse_coord(text):
    m = _COORD.search(text)
    if not m:
        raise ValueError(f'toa do OpenAir khong hop le: {text!r}')
    lat = _dms(*m.group(1, 2, 3)) * (-1 if m.group(4).upper() == 'S' else 1)
    lon = _dms(*m.group(5, 6, 7)) * (-1 if m.group(8).upper() == 'W' else 1)
    return lat, lon


def parse_openair(text, source=''):
    zones = []
    cur = None
    center, clockwise = None, True

    def flush():
        if cur and len(cur['pts']) >= 3:
            zones.append(Zone(cur['name'], cur['kind'], cur['pts'], cur['lower'], cur['upper'], source))

    for lineno, raw in enumerate(text.splitlines(), 1):
        line = raw.split('*', 1)[0].strip() if not raw.lstrip().startswith('*') else ''
        if not line:
            continue
        cmd, _, arg = line.partition(' ')
        cmd, arg = cmd.upper(), arg.strip()
        try:
            if cmd == 'AC':
                flush()
                cur = {'name': '', 'kind': _OPENAIR_CLASS.get(arg.upper(), 'other'),
                       'lower': '', 'upper': '', 'pts': []}
                center, clockwise = None, True
            elif cur is None:
                continue
            elif cmd == 'AN':
                cur['name'] = arg
            elif cmd == 'AL':
                cur['lower'] = arg
            elif cmd == 'AH':
                cur['upper'] = arg
            elif cmd == 'DP':
                cur['pts'].append(parse_coord(arg))
            elif cmd == 'V':
                key, _, val = arg.partition('=')
                key = key.strip().upper()
                if key == 'X':
                    center = parse_coord(val)
                elif key == 'D':
                    clockwise = val.strip() != '-'
            elif cmd == 'DC' and center:
                cur['pts'].extend(circle(*center, float(arg) * NM))
            elif cmd == 'DA' and center:
                r, a1, a2 = (float(v) for v in arg.split(','))
                cur['pts'].extend(arc(*center, r * NM, a1, a2, clockwise))
            elif cmd == 'DB' and center:
                p1_txt, p2_txt = arg.split(',', 1) if arg.count(',') == 1 else _split_db(arg)
                p1, p2 = parse_coord(p1_txt), parse_coord(p2_txt)
                b1, r = bearing_distance(*center, *p1)
                b2, _ = bearing_distance(*center, *p2)
                cur['pts'].extend(arc(*center, r, b1, b2, clockwise))
        except (ValueError, IndexError) as e:
            log.warning('%s:%d bo qua dong %r (%s)', source, lineno, raw.strip(), e)
    flush()
    return zones


def _split_db(arg):
    """'DB 21:00:00 N 105:00:00 E, 21:10:00 N 105:10:00 E' -- tach theo toa do thu nhat."""
    m = _COORD.search(arg)
    return arg[:m.end()], arg[m.end():].lstrip(' ,')


# --- nap ca thu muc -----------------------------------------------------------------------
PARSERS = {'.geojson': parse_geojson, '.json': parse_geojson, '.txt': parse_openair, '.openair': parse_openair}


def load_dirs(dirs):
    zones = []
    for d in dirs:
        root = Path(d).expanduser()
        if not root.is_dir():
            continue
        for f in sorted(root.iterdir()):
            parser = PARSERS.get(f.suffix.lower())
            if parser is None:
                continue
            try:
                got = parser(f.read_text(encoding='utf-8', errors='replace'), source=f.name)
            except Exception as e:
                log.warning('Khong doc duoc file vung bay %s: %s', f, e)
                continue
            log.info('Vung bay: %d vung tu %s', len(got), f.name)
            zones.extend(got)
    return zones


def zones_at(zones, lat, lon):
    return [z for z in zones if z.contains(lat, lon)]
