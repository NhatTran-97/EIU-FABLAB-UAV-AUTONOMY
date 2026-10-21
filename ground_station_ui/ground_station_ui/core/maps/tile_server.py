"""Local tile server (127.0.0.1) for QtLocation, the same approach QGC uses for offline maps.

    GET /<layer>/<z>/<x>/<y>.png
        in the .mbtiles cache     -> served (works offline)
        missing + online          -> downloaded from the online source, STORED in the cache, served
        missing + offline         -> 404 (empty tile)

Composite layers (compose, e.g. Hybrid = imagery + road labels + places) merge the tiles of their source
layers (each with its own cache) into one image. Source layers with `hidden: true` are not listed in the
layer selector.

Sources that need an API key (e.g. Azure Maps): the url contains {key} and `key_env` names the key, read
from the environment variable of that name, or else from config/keys.yaml (private file, not committed).
Without a key the layer is cache-only (offline flying still works); with an empty cache it is dropped.

A REAL network outage (DNS failure, unreachable network, connection refused) pauses online downloads for
OFFLINE_BACKOFF_S so the map does not wait for a timeout on every tile. Errors specific to one tile
(timeout, reset, SSL, 429...) fail only that tile and do NOT pause the server: at deep zoom QtLocation
requests many tiles at once, and pausing 30 s for one slow tile would 404 every uncached tile, QtLocation
gives up after 5 attempts, and the map stays full of empty tiles.
"""

import errno
import io
import logging
import os
import socket
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
OFFLINE_ERRNOS = (errno.ENETUNREACH, errno.EHOSTUNREACH, errno.ENETDOWN)


def is_offline_error(err):
    """True if the error means the network is down (not a problem with a single tile)."""
    reason = err.reason if isinstance(err, urllib.error.URLError) else err
    if isinstance(reason, (TimeoutError, socket.timeout)):
        return False
    if isinstance(reason, (socket.gaierror, ConnectionRefusedError)):
        return True
    return isinstance(reason, OSError) and reason.errno in OFFLINE_ERRNOS


class MissingKey(Exception):
    """Layer needs an API key, has none and has no cache -> hide it (not an error)."""


class MapLayer:
    def __init__(self, cfg, cache_dir, keys=None):
        self.id = cfg['id']
        self.title = cfg.get('title', self.id)
        self.url = cfg.get('url', '')                 # empty = offline only (e.g. own drone orthophoto)
        self.compose = list(cfg.get('compose', []))   # composite layer: [base, overlay1, overlay2...]
        self.hidden = bool(cfg.get('hidden', False))
        self.max_zoom = int(cfg.get('max_zoom', 19))
        self.attribution = cfg.get('attribution', '')
        self.cache = None
        self.key_env = cfg.get('key_env', '')
        self._key = ''
        if self.key_env:
            self._key = (os.environ.get(self.key_env) or str((keys or {}).get(self.key_env) or '')).strip()
        if self.key_env and not self._key:
            self.url = ''               # no key -> offline from cache only
        self._auth_warned = False
        if not self.compose:
            path = Path(cfg.get('file', f'{self.id}.mbtiles'))
            if not path.is_absolute():
                path = Path(cache_dir).expanduser() / path
            if self.key_env and not self._key and not path.exists():
                raise MissingKey(f'{self.key_env} not set (environment variable / config/keys.yaml)')
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
                log.info('Hiding layer %s: %s', cfg.get('id'), e)
                no_key.add(cfg.get('id'))
                continue
            except Exception as e:      # e.g. the offline .mbtiles file does not exist yet -> skip that layer
                log.warning('Skipping map layer %s: %s', cfg.get('id'), e)
                continue
            self.layers[layer.id] = layer
        for layer in list(self.layers.values()):
            if not layer.compose:
                continue
            if layer.compose[0] not in self.layers:
                # without the base layer (first source) nothing can be drawn
                (log.info if layer.compose[0] in no_key else log.warning)(
                    'Dropping composite layer %s: missing base layer %s', layer.id, layer.compose[0])
                del self.layers[layer.id]
                continue
            missing = [s for s in layer.compose if s not in self.layers]
            if missing:
                log.warning('Composite layer %s is missing source layer %s', layer.id, missing)
                layer.compose = [s for s in layer.compose if s in self.layers]
        self.stats = {'cache_hits': 0, 'downloaded': 0, 'missing': 0}
        self._offline_until = 0.0
        self._httpd = None
        self._inflight = {}            # (layer, z, x, y) -> [Event, result]: single-flight
        self._inflight_lock = threading.Lock()

    # --- network state ---------------------------------------------------------------------
    @property
    def online(self):
        return time.monotonic() >= self._offline_until

    def mark_offline(self):
        self._offline_until = time.monotonic() + OFFLINE_BACKOFF_S

    # --- fetch one tile ---------------------------------------------------------------------
    def fetch_upstream(self, layer, z, x, y):
        """Download one tile from the online source and cache it. None on error or when offline.

        Single-flight: if the tile server and prefetch both miss the same tile, only one request is made
        and the other caller waits for its result.
        """
        if not layer.url or not self.online or z > layer.max_zoom:
            return None
        key = (layer.id, z, x, y)
        with self._inflight_lock:
            slot = self._inflight.get(key)
            owner = slot is None
            if owner:
                slot = self._inflight[key] = [threading.Event(), None]
        if not owner:
            slot[0].wait(FETCH_TIMEOUT_S + 1.0)
            return slot[1]
        try:
            slot[1] = self._download(layer, z, x, y)
            return slot[1]
        finally:
            slot[0].set()
            with self._inflight_lock:
                self._inflight.pop(key, None)

    def _download(self, layer, z, x, y):
        req = urllib.request.Request(layer.upstream_url(z, x, y), headers={'User-Agent': USER_AGENT})
        try:
            with urllib.request.urlopen(req, timeout=FETCH_TIMEOUT_S) as resp:
                data = resp.read()
        except urllib.error.HTTPError as e:
            if e.code in (401, 403) and not layer._auth_warned:
                layer._auth_warned = True
                log.warning('Layer %s: source refused (%d) -- check API key / quota', layer.id, e.code)
            return None                 # tile does not exist at the source (e.g. at sea) -- still online
        except (urllib.error.URLError, OSError) as e:
            if is_offline_error(e):
                self.mark_offline()
            else:                       # one slow / failing tile fails alone; other tiles keep loading
                log.debug('Layer %s: tile %d/%d/%d temporary error: %s', layer.id, z, x, y, e)
            return None
        if data:
            layer.cache.put(z, x, y, data)
        return data

    def sources(self, layer_id):
        """Layers with a real cache needed to serve one layer (composite -> its source layers)."""
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
        from PIL import Image        # only needed for composite layers
        parts = [self.get_tile(src, z, x, y) for src in layer.compose]
        if not parts or parts[0] is None:
            return None                  # nothing to draw without the base image
        img = Image.open(io.BytesIO(parts[0])).convert('RGBA')
        for data in parts[1:]:
            if data:
                ov = Image.open(io.BytesIO(data)).convert('RGBA')
                if ov.size != img.size:
                    ov = ov.resize(img.size)
                img = Image.alpha_composite(img, ov)
        out = io.BytesIO()
        # subsampling=0 (4:4:4): keeps label edges sharp (served to localhost only)
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
                    # QtLocation cancels tiles it no longer needs (fast pan / zoom) -- normal, the tile is already cached
                    pass

            def log_message(self, *args):     # do not print every request to the terminal
                pass

        class Server(ThreadingHTTPServer):
            daemon_threads = True

            def handle_error(self, request, client_address):
                # no traceback for connections closed by the client mid-request
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
