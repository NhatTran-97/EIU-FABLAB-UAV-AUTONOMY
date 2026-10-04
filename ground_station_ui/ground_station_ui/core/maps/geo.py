"""Doc toa do nguoi dung nhap (copy tu Google Maps, QGC...)."""

import re

_NUM = r'[-+]?\d+(?:\.\d+)?'
_DECIMAL = re.compile(rf'^\s*({_NUM})\s*°?\s*([NSns])?\s*[,;\s]\s*({_NUM})\s*°?\s*([EWew])?\s*$')


def parse_latlon(text):
    """'11.052919, 106.666057' / '11.052919 106.666057' / '11.05N, 106.66E' -> (lat, lon).

    Tra ve None neu khong hop le (sai dinh dang hoac ngoai [-90,90] x [-180,180]).
    """
    m = _DECIMAL.match(text or '')
    if not m:
        return None
    lat, ns, lon, ew = float(m.group(1)), m.group(2), float(m.group(3)), m.group(4)
    if ns and ns.upper() == 'S':
        lat = -abs(lat)
    if ew and ew.upper() == 'W':
        lon = -abs(lon)
    if not (-90.0 <= lat <= 90.0 and -180.0 <= lon <= 180.0):
        return None
    return lat, lon
