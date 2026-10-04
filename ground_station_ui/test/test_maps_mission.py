"""Test ban do offline (MBTiles, tile server, tai truoc) va soan mission -- khong can mang that."""

import threading
import time
import urllib.error
import urllib.request
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import pytest

from ground_station_ui.core.maps.mbtiles import MBTiles
from ground_station_ui.core.maps.prefetch import Prefetcher
from ground_station_ui.core.maps.tile_server import TileServer
from ground_station_ui.core.maps.tiles import count_tiles, lonlat_to_tile, tiles_in_bbox
from ground_station_ui.core.mission.mission import Mission


@pytest.fixture
def upstream():
    """Nguon online gia: tra ve bytes 'z/x/y', dem so request."""
    hits = []

    class H(BaseHTTPRequestHandler):
        def do_GET(self):
            hits.append(self.path)
            body = self.path.encode()
            self.send_response(200)
            self.send_header('Content-Length', str(len(body)))
            self.end_headers()
            self.wfile.write(body)

        def log_message(self, *a):
            pass

    srv = ThreadingHTTPServer(('127.0.0.1', 0), H)
    threading.Thread(target=srv.serve_forever, daemon=True).start()
    yield f'http://127.0.0.1:{srv.server_address[1]}', hits
    srv.shutdown()


def get(url):
    with urllib.request.urlopen(url, timeout=3) as r:
        return r.read()


def test_tile_math():
    # Ho Chi Minh, zoom 16 -- doi chieu cong thuc doc lap: y = (1 - ln(tan(pi/4 + lat/2)) / pi) / 2 * 2^z
    assert lonlat_to_tile(106.6297, 10.8231, 16) == (52179, 30785)
    # Dem nhanh phai khop liet ke tung o, va vung phai phu ca 2 goc bbox
    bbox = (106.62, 10.82, 106.63, 10.83)
    cells = set(tiles_in_bbox(*bbox, 16))
    assert count_tiles(*bbox, 16, 16) == len(cells)
    assert lonlat_to_tile(106.62, 10.83, 16) in cells and lonlat_to_tile(106.63, 10.82, 16) in cells


def test_mbtiles_roundtrip_uses_tms_rows(tmp_path):
    db = MBTiles(tmp_path / 'a.mbtiles', name='a')
    db.put(16, 52179, 30785, b'img')
    assert db.get(16, 52179, 30785) == b'img'
    row = db._db.execute('SELECT tile_row FROM tiles').fetchone()[0]
    assert row == (1 << 16) - 1 - 30785          # MBTiles luu theo TMS
    assert db.stats()[0] == 1


def test_server_downloads_once_then_serves_from_cache(tmp_path, upstream):
    url, hits = upstream
    server = TileServer([{'id': 'sat', 'url': url + '/{z}/{x}/{y}'}], tmp_path)
    port = server.start()
    try:
        tile = f'http://127.0.0.1:{port}/sat/16/52179/30785.png'
        assert get(tile) == b'/16/52179/30785'
        assert get(tile) == b'/16/52179/30785'
        assert len(hits) == 1                        # lan 2 lay tu cache .mbtiles
        assert server.stats['downloaded'] == 1 and server.stats['cache_hits'] == 1
    finally:
        server.stop()


def test_offline_serves_cache_and_404_for_missing(tmp_path):
    # Nguon online khong ton tai = mat mang
    server = TileServer([{'id': 'sat', 'url': 'http://127.0.0.1:9/{z}/{x}/{y}'}], tmp_path)
    server.layers['sat'].cache.put(15, 1, 2, b'cached')
    port = server.start()
    try:
        assert get(f'http://127.0.0.1:{port}/sat/15/1/2.png') == b'cached'
        with pytest.raises(urllib.error.HTTPError) as e:
            get(f'http://127.0.0.1:{port}/sat/15/9/9.png')
        assert e.value.code == 404
        assert not server.online                     # tam ngung tai online, khong cho timeout
    finally:
        server.stop()


def test_prefetch_area(tmp_path, upstream):
    url, hits = upstream
    server = TileServer([{'id': 'sat', 'url': url + '/{z}/{x}/{y}'}], tmp_path)
    pre = Prefetcher(server)
    bbox = (106.62, 10.82, 106.63, 10.83)
    assert pre.start('sat', bbox, 15, 16, max_tiles=100) == ''
    deadline = time.monotonic() + 5
    while pre.progress()['running'] and time.monotonic() < deadline:
        time.sleep(0.05)
    p = pre.progress()
    assert p['done'] == p['total'] == count_tiles(*bbox, 15, 16)
    assert server.layers['sat'].cache.stats()[0] == p['total']
    assert 'too large' in pre.start('sat', bbox, 10, 19, max_tiles=10)
    server.stop()


def test_mission_edit():
    m = Mission(default_altitude=25)
    m.add(10.0, 106.0)
    m.add(10.1, 106.1)
    m.insert(1, 10.05, 106.05)
    assert [wp['seq'] for wp in m.items] == [1, 2, 3]
    assert m.items[1]['alt'] == 25                   # chen giua: lay do cao diem truoc
    m.move(2, 10.2, 106.2)
    m.set_altitude(0, 40)
    m.remove(1)
    assert [(wp['seq'], wp['lat'], wp['alt']) for wp in m.items] == [(1, 10.0, 40.0), (2, 10.2, 25.0)]
    assert m.values['count'] == 2
    m.clear()
    assert m.values['count'] == 0 and m.items == []


def test_hybrid_composes_base_and_overlays(tmp_path):
    """Hybrid = nen (JPEG) + overlay trong suot (PNG); moi nguon cache rieng, offline van ghep duoc."""
    import io
    from PIL import Image

    def img(fmt, color):
        buf = io.BytesIO()
        Image.new('RGBA' if fmt == 'PNG' else 'RGB', (256, 256), color).save(buf, fmt)
        return buf.getvalue()

    base, label = img('JPEG', (0, 0, 255)), img('PNG', (255, 0, 0, 0))
    label_img = Image.open(io.BytesIO(label)).convert('RGBA')
    for dx in range(8):                                      # 1 net "chu" do 8x8, con lai trong suot
        for dy in range(8):
            label_img.putpixel((8 + dx, 8 + dy), (255, 0, 0, 255))
    buf = io.BytesIO(); label_img.save(buf, 'PNG'); label = buf.getvalue()

    class H(BaseHTTPRequestHandler):
        def do_GET(self):
            body = base if self.path.startswith('/base') else label
            self.send_response(200); self.send_header('Content-Length', str(len(body))); self.end_headers()
            self.wfile.write(body)

        def log_message(self, *a):
            pass

    up = ThreadingHTTPServer(('127.0.0.1', 0), H)
    threading.Thread(target=up.serve_forever, daemon=True).start()
    url = f'http://127.0.0.1:{up.server_address[1]}'
    layers = [{'id': 'hybrid', 'compose': ['sat', 'labels']},
              {'id': 'sat', 'url': url + '/base/{z}/{x}/{y}'},
              {'id': 'labels', 'hidden': True, 'url': url + '/lbl/{z}/{x}/{y}'}]
    server = TileServer(layers, tmp_path)
    try:
        out = Image.open(io.BytesIO(server.get_tile('hybrid', 17, 1, 2))).convert('RGB')
        r, g, b = out.getpixel((11, 11))
        assert r > 200 and b < 60                            # net nhan de len anh nen
        r, g, b = out.getpixel((100, 100))
        assert b > 200 and r < 60                            # cho trong suot -> thay anh nen
        assert server.layers['sat'].cache.has(17, 1, 2) and server.layers['labels'].cache.has(17, 1, 2)
        assert [s.id for s in server.sources('hybrid')] == ['sat', 'labels']
        up.shutdown()                                        # mat mang: van ghep tu cache
        assert server.get_tile('hybrid', 17, 1, 2) is not None
        pre = Prefetcher(server)
        assert pre.start('hybrid', (106.62, 10.82, 106.63, 10.83), 15, 15, 100) == ''
        while pre.progress()['running']:
            time.sleep(0.05)
        assert pre.progress()['total'] == 2 * count_tiles(106.62, 10.82, 106.63, 10.83, 15, 15)
    finally:
        server.stop()


def test_parse_latlon():
    from ground_station_ui.core.maps.geo import parse_latlon
    assert parse_latlon('11.052919, 106.666057') == (11.052919, 106.666057)
    assert parse_latlon('  11.052919 106.666057 ') == (11.052919, 106.666057)
    assert parse_latlon('11.052919;106.666057') == (11.052919, 106.666057)
    assert parse_latlon('33.86S, 151.21E') == (-33.86, 151.21)
    assert parse_latlon('-33.86, -70.5') == (-33.86, -70.5)
    assert parse_latlon('106.666057, 11.052919') is None        # nhap nguoc (lon, lat): vi do > 90
    assert parse_latlon('91, 10') is None and parse_latlon('10, 181') is None
    assert parse_latlon('abc') is None and parse_latlon('') is None and parse_latlon('11.05') is None


def test_count_tiles_is_capped_for_huge_areas():
    # Ca the gioi zoom 14-19: hang ty o -> phai dung som, khong tran int32 cua Qt
    assert count_tiles(-179, -80, 179, 80, 14, 19, limit=300000) == 300001


def test_client_closing_connection_is_silent(tmp_path, capsys):
    """QtLocation huy request khi keo ban do -> khong duoc in traceback BrokenPipe."""
    import socket
    server = TileServer([{'id': 'sat', 'url': 'http://127.0.0.1:9/{z}/{x}/{y}'}], tmp_path)
    server.layers['sat'].cache.put(15, 1, 2, b'x' * 200000)
    port = server.start()
    try:
        for _ in range(5):
            s = socket.create_connection(('127.0.0.1', port))
            s.setsockopt(socket.SOL_SOCKET, socket.SO_LINGER, b'\x01\x00\x00\x00\x00\x00\x00\x00')
            s.sendall(b'GET /sat/15/1/2.png HTTP/1.0\r\n\r\n')
            s.close()                                    # dong ngay, khong doc
        time.sleep(0.5)
        assert get(f'http://127.0.0.1:{port}/sat/15/1/2.png') == b'x' * 200000   # server van song
    finally:
        server.stop()
    err = capsys.readouterr().err
    assert 'BrokenPipe' not in err and 'ConnectionReset' not in err


def _keyed_layers(base_url):
    return [
        {'id': 'az', 'compose': ['az_img', 'az_road']},
        {'id': 'az_img', 'hidden': True, 'url': base_url + '/img/{z}/{x}/{y}?k={key}', 'key_env': 'TEST_MAP_KEY'},
        {'id': 'az_road', 'hidden': True, 'url': base_url + '/road/{z}/{x}/{y}?k={key}', 'key_env': 'TEST_MAP_KEY'},
    ]


def test_api_key_comes_from_env(tmp_path, upstream, monkeypatch):
    base, hits = upstream
    monkeypatch.setenv('TEST_MAP_KEY', 'abc123')
    ts = TileServer(_keyed_layers(base), tmp_path)
    assert ts.fetch_upstream(ts.layers['az_img'], 3, 1, 2) == b'/img/3/1/2?k=abc123'
    assert 'az' in ts.layers
    ts.stop()


def test_layer_without_key_is_dropped_unless_cached(tmp_path, upstream, monkeypatch):
    base, hits = upstream
    monkeypatch.delenv('TEST_MAP_KEY', raising=False)
    # chua co key, chua co cache -> bo ca lop nguon lan lop ghep
    ts = TileServer(_keyed_layers(base), tmp_path)
    assert not {'az', 'az_img', 'az_road'} & set(ts.layers)
    ts.stop()

    # da co cache tu truoc (vd tai luc con key) -> van dung offline, khong goi nguon
    MBTiles(tmp_path / 'az_img.mbtiles').put(3, 1, 2, b'cached')
    ts = TileServer(_keyed_layers(base), tmp_path)
    assert 'az' in ts.layers and 'az_road' not in ts.layers
    assert ts.get_tile('az_img', 3, 1, 2) == b'cached'
    assert ts.get_tile('az_img', 3, 1, 3) is None
    assert hits == []
    ts.stop()

