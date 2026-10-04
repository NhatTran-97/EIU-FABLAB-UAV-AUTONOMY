"""Tile server cuc bo (127.0.0.1) cho QtLocation -- cach QGC lam ban do offline.

    GET /<layer>/<z>/<x>/<y>.png
        co trong cache .mbtiles  -> tra ve (offline van chay)
        chua co + dang co mang   -> tai tu nguon online, LUU vao cache, tra ve
        chua co + mat mang       -> 404 (o trong)

Lop ghep (compose, vd Hybrid = ve tinh + nhan duong + dia danh): ghep o cua cac lop nguon
(moi nguon co cache rieng) thanh 1 anh. Lop nguon co `hidden: true` thi khong hien o nut chon lop.

Nguon can API key (vd Azure Maps): url co {key}, `key_env` = ten key -- lay tu bien moi truong
cung ten, khong co thi tu config/keys.yaml (file rieng, khong commit).
Khong co key -> lop chi doc cache (van bay offline duoc); cache cung trong -> bo lop.

Mat mang -> tam ngung tai online OFFLINE_BACKOFF_S giay de ban do khong cho timeout tung o.
"""

import io
import logging
import os
import threading
import time
import urllib.error
import urllib.request
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

from ground_station_ui.core.maps.mbtiles import MBTiles

log = logging.getLogger(__name__)

USER_AGENT = 'drone-gcs/0.1 (offline tile cache)'
FETCH_TIMEOUT_S = 5.0
OFFLINE_BACKOFF_S = 30.0


class MissingKey(Exception):
    """Lop can API key ma chua co key, cung chua co cache -> an lop (khong phai loi)."""


class MapLayer:
    def __init__(self, cfg, cache_dir, keys=None):
        self.id = cfg['id']
        self.title = cfg.get('title', self.id)
        self.url = cfg.get('url', '')                 # rong = chi offline (vd anh drone tu chup)
        self.compose = list(cfg.get('compose', []))   # lop ghep: [nen, overlay1, overlay2...]
        self.hidden = bool(cfg.get('hidden', False))
        self.max_zoom = int(cfg.get('max_zoom', 19))
        self.attribution = cfg.get('attribution', '')
        self.cache = None
        self.key_env = cfg.get('key_env', '')
        self._key = ''
        if self.key_env:
            self._key = (os.environ.get(self.key_env) or str((keys or {}).get(self.key_env) or '')).strip()
        if self.key_env and not self._key:
            self.url = ''               # khong co key -> chi offline tu cache
        self._auth_warned = False
        if not self.compose:
            path = Path(cfg.get('file', f'{self.id}.mbtiles'))
            if not path.is_absolute():
                path = Path(cache_dir).expanduser() / path
            if self.key_env and not self._key and not path.exists():
                raise MissingKey(f'chua co {self.key_env} (bien moi truong / config/keys.yaml)')
            self.cache = MBTiles(path, name=self.title, readonly=bool(cfg.get('readonly', not self.url)))

    def upstream_url(self, z, x, y):
        return self.url.format(z=z, x=x, y=y, key=self._key)


class TileServer:
    def __init__(self, layer_cfgs, cache_dir, keys=None):
        self.layers = {}
        no_key = set()
        for cfg in layer_cfgs:
            try:
                layer = MapLayer(cfg, cache_dir, keys)
            except MissingKey as e:
                log.info('An lop %s: %s', cfg.get('id'), e)
                no_key.add(cfg.get('id'))
                continue
            except Exception as e:      # vd file .mbtiles offline chua co -> bo qua lop do
                log.warning('Bo qua lop ban do %s: %s', cfg.get('id'), e)
                continue
            self.layers[layer.id] = layer
        for layer in list(self.layers.values()):
            if not layer.compose:
                continue
            if layer.compose[0] not in self.layers:
                # mat lop nen (nguon dau tien) thi khong ve duoc gi
                (log.info if layer.compose[0] in no_key else log.warning)(
                    'Bo lop ghep %s: thieu lop nen %s', layer.id, layer.compose[0])
                del self.layers[layer.id]
                continue
            missing = [s for s in layer.compose if s not in self.layers]
            if missing:
                log.warning('Lop ghep %s thieu lop nguon %s', layer.id, missing)
                layer.compose = [s for s in layer.compose if s in self.layers]
        self.stats = {'cache_hits': 0, 'downloaded': 0, 'missing': 0}
        self._offline_until = 0.0
        self._httpd = None

    # --- trang thai mang -------------------------------------------------------------------
    @property
    def online(self):
        return time.monotonic() >= self._offline_until

    def mark_offline(self):
        self._offline_until = time.monotonic() + OFFLINE_BACKOFF_S

    # --- lay 1 o ---------------------------------------------------------------------------
    def fetch_upstream(self, layer, z, x, y):
        """Tai 1 o tu nguon online va luu cache. None neu loi / mat mang."""
        if not layer.url or not self.online or z > layer.max_zoom:
            return None
        req = urllib.request.Request(layer.upstream_url(z, x, y), headers={'User-Agent': USER_AGENT})
        try:
            with urllib.request.urlopen(req, timeout=FETCH_TIMEOUT_S) as resp:
                data = resp.read()
        except urllib.error.HTTPError as e:
            if e.code in (401, 403) and not layer._auth_warned:
                layer._auth_warned = True
                log.warning('Lop %s: nguon tu choi (%d) -- kiem tra API key / quota', layer.id, e.code)
            return None                 # o khong ton tai o nguon (vd vung bien) -- van dang online
        except (urllib.error.URLError, OSError):
            self.mark_offline()
            return None
        if data:
            layer.cache.put(z, x, y, data)
        return data

    def sources(self, layer_id):
        """Cac lop co cache that su can tai cho 1 lop (lop ghep -> cac lop nguon)."""
        layer = self.layers.get(layer_id)
        if layer is None:
            return []
        return [self.layers[s] for s in layer.compose] if layer.compose else [layer]

    def get_tile(self, layer_id, z, x, y):
        layer = self.layers.get(layer_id)
        if layer is None:
            return None
        if layer.compose:
            return self._compose(layer, z, x, y)
        data = layer.cache.get(z, x, y)
        if data is not None:
            self.stats['cache_hits'] += 1
            return data
        data = self.fetch_upstream(layer, z, x, y)
        self.stats['downloaded' if data else 'missing'] += 1
        return data

    def _compose(self, layer, z, x, y):
        from PIL import Image        # chi can khi co lop ghep
        parts = [self.get_tile(src, z, x, y) for src in layer.compose]
        if not parts or parts[0] is None:
            return None                  # khong co anh nen thi khong ve
        img = Image.open(io.BytesIO(parts[0])).convert('RGBA')
        for data in parts[1:]:
            if data:
                ov = Image.open(io.BytesIO(data)).convert('RGBA')
                if ov.size != img.size:
                    ov = ov.resize(img.size)
                img = Image.alpha_composite(img, ov)
        out = io.BytesIO()
        # subsampling=0 (4:4:4): khong lam nhoe mau o net chu nhan duong (chi phuc vu localhost)
        img.convert('RGB').save(out, 'JPEG', quality=92, subsampling=0)
        return out.getvalue()

    # --- HTTP ------------------------------------------------------------------------------
    def start(self, host='127.0.0.1', port=0):
        server = self

        class Handler(BaseHTTPRequestHandler):
            def do_GET(self):
                parts = self.path.split('?')[0].strip('/').split('/')
                try:
                    layer_id, z, x, y = parts[0], int(parts[1]), int(parts[2]), int(parts[3].split('.')[0])
                except (IndexError, ValueError):
                    self.send_error(400)
                    return
                data = server.get_tile(layer_id, z, x, y)
                try:
                    if data is None:
                        self.send_error(404)
                        return
                    self.send_response(200)
                    self.send_header('Content-Type', 'image/png' if data[:4] == b'\x89PNG' else 'image/jpeg')
                    self.send_header('Content-Length', str(len(data)))
                    self.end_headers()
                    self.wfile.write(data)
                except (BrokenPipeError, ConnectionResetError):
                    # QtLocation huy o khong con can (keo / zoom nhanh) -- binh thuong, o da nam trong cache
                    pass

            def log_message(self, *args):     # khong in moi request ra terminal
                pass

        class Server(ThreadingHTTPServer):
            daemon_threads = True

            def handle_error(self, request, client_address):
                # Khong in traceback cho ket noi bi client dong giua chung
                import sys
                if not isinstance(sys.exc_info()[1], (BrokenPipeError, ConnectionResetError)):
                    super().handle_error(request, client_address)

        self._httpd = Server((host, port), Handler)
        self._httpd.daemon_threads = True
        threading.Thread(target=self._httpd.serve_forever, name='tile-server', daemon=True).start()
        return self._httpd.server_address[1]

    @property
    def port(self):
        return self._httpd.server_address[1] if self._httpd else 0

    def stop(self):
        if self._httpd:
            self._httpd.shutdown()
            self._httpd.server_close()
            self._httpd = None
        for layer in self.layers.values():
            if layer.cache:
                layer.cache.close()
