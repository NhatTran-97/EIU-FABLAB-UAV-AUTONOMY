"""Tai truoc 1 vung ban do vao cache (giong 'Offline Maps' cua QGC).

Chay tren thread rieng, vai worker song song; o da co trong cache thi bo qua. Tien do doc qua
`progress()`; huy bang cancel().
"""

import threading
from concurrent.futures import ThreadPoolExecutor

from ground_station_ui.core.maps.tiles import count_tiles, tiles_in_bbox

WORKERS = 4


class Prefetcher:
    def __init__(self, server):
        self.server = server
        self._cancel = threading.Event()
        self._lock = threading.Lock()
        self._state = {'running': False, 'total': 0, 'done': 0, 'downloaded': 0, 'failed': 0,
                       'message': ''}

    def progress(self):
        with self._lock:
            return dict(self._state)

    def cancel(self):
        self._cancel.set()

    def start(self, layer_id, bbox, zmin, zmax, max_tiles):
        """bbox = (west, south, east, north). Tra ve thong bao loi (str) hoac '' neu bat dau."""
        # Lop ghep (Hybrid) -> tai tung lop nguon
        sources = [s for s in self.server.sources(layer_id) if s.url]
        if not sources:
            return 'This layer has no online source'
        if self.progress()['running']:
            return 'Another download is running'
        zmax = min([zmax] + [s.max_zoom for s in sources])
        total = count_tiles(*bbox, zmin, zmax) * len(sources)
        if total == 0:
            return 'Invalid area'
        if total > max_tiles:
            return f'Area too large: {total} tiles (max {max_tiles}). Zoom in or lower max zoom.'
        self._cancel.clear()
        with self._lock:
            self._state = {'running': True, 'total': total, 'done': 0, 'downloaded': 0, 'failed': 0,
                           'message': f'Downloading {total} tiles ({", ".join(s.title for s in sources)}, '
                                      f'zoom {zmin}-{zmax})'}
        threading.Thread(target=self._run, args=(sources, bbox, zmin, zmax),
                         name='map-prefetch', daemon=True).start()
        return ''

    def _one(self, layer, z, x, y):
        if self._cancel.is_set():
            return
        if layer.cache.has(z, x, y):
            result = 'cached'
        elif self.server.fetch_upstream(layer, z, x, y) is not None:
            result = 'downloaded'
        else:
            result = 'failed'
        with self._lock:
            self._state['done'] += 1
            if result != 'cached':
                self._state[result] += 1

    def _run(self, sources, bbox, zmin, zmax):
        with ThreadPoolExecutor(max_workers=WORKERS) as pool:
            for z in range(zmin, zmax + 1):
                for x, y in tiles_in_bbox(*bbox, z):
                    if self._cancel.is_set():
                        break
                    for layer in sources:
                        pool.submit(self._one, layer, z, x, y)
        with self._lock:
            s = self._state
            s['running'] = False
            if self._cancel.is_set():
                s['message'] = f'Cancelled ({s["done"]}/{s["total"]})'
            elif not self.server.online and s['failed']:
                s['message'] = f'Offline: downloaded {s["downloaded"]}, failed {s["failed"]}'
            else:
                s['message'] = f'Done: {s["done"]} tiles ({s["downloaded"]} new, {s["failed"]} failed)'
