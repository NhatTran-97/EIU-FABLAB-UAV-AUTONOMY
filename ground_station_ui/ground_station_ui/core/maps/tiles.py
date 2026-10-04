"""Tinh toan o ban do Web Mercator (XYZ, giong Google / OSM / Esri)."""

import math


def lonlat_to_tile(lon, lat, z):
    lat = max(min(lat, 85.0511), -85.0511)
    n = 1 << z
    x = int((lon + 180.0) / 360.0 * n)
    y = int((1.0 - math.asinh(math.tan(math.radians(lat))) / math.pi) / 2.0 * n)
    return min(max(x, 0), n - 1), min(max(y, 0), n - 1)


def tiles_in_bbox(west, south, east, north, z):
    """Moi o (x, y) phu vung bbox o muc zoom z."""
    if not valid_bbox(west, south, east, north):
        return
    x0, y0 = lonlat_to_tile(west, north, z)
    x1, y1 = lonlat_to_tile(east, south, z)
    for x in range(x0, x1 + 1):
        for y in range(y0, y1 + 1):
            yield x, y


def valid_bbox(west, south, east, north):
    vals = (west, south, east, north)
    return all(math.isfinite(v) for v in vals) and west < east and south < north


def count_tiles(west, south, east, north, zmin, zmax, limit=None):
    """So o trong vung. `limit`: dung dem khi vuot (vung qua lon, vd dang xem ca the gioi)."""
    if not valid_bbox(west, south, east, north):
        return 0      # vd ban do chua co kich thuoc -> vung khong hop le
    total = 0
    for z in range(zmin, zmax + 1):
        x0, y0 = lonlat_to_tile(west, north, z)
        x1, y1 = lonlat_to_tile(east, south, z)
        total += (x1 - x0 + 1) * (y1 - y0 + 1)
        if limit is not None and total > limit:
            return limit + 1
    return total
