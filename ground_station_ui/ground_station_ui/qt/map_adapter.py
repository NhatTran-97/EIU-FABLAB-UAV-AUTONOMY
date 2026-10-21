"""MapAdapter: offline map info for QML (tile server port, layers, area prefetch)."""

import time
from pathlib import Path

from PySide6.QtCore import Property, QObject, QSettings, QStandardPaths, Signal, Slot

from ground_station_ui.core.maps.geo import parse_latlon

STATS_PERIOD_S = 2.0      # counting tiles in SQLite is not needed every tick


class MapAdapter(QObject):
    changed = Signal()
    layerChanged = Signal()

    def __init__(self, server, prefetcher, map_cfg, parent=None):
        super().__init__(parent)
        self.server = server
        self.prefetcher = prefetcher
        self._max_tiles = int(map_cfg.get('max_prefetch_tiles', 30000))
        # On start, restore the last viewed area if any, otherwise use gcs.yaml
        self._settings = QSettings()
        center = map_cfg.get('default_center', [10.8231, 106.6297])
        self._center = [self._settings.value('map/lat', float(center[0]), type=float),
                        self._settings.value('map/lon', float(center[1]), type=float)]
        self._zoom = self._settings.value('map/zoom', float(map_cfg.get('default_zoom', 16)), type=float)
        # default text of the coordinate box (Map tab); empty when not configured
        sd = map_cfg.get('search_default')
        self._search_default = f'{float(sd[0]):.6f}, {float(sd[1]):.6f}' if sd else ''
        self._layers = []
        for l in server.layers.values():
            if l.hidden:
                continue            # source layers of a composite layer (road labels, places...)
            srcs = server.sources(l.id)
            self._layers.append({
                'id': l.id, 'title': l.title, 'attribution': l.attribution,
                'offlineOnly': not any(s.url for s in srcs),
                'maxZoom': min([l.max_zoom] + [s.max_zoom for s in srcs]),
            })
        # Current layer (chosen in Settings), remembered in QSettings; falls back to the first layer if it is gone
        ids = [l['id'] for l in self._layers]
        saved = self._settings.value('map/layer', '', type=str)
        self._current = saved if saved in ids else (ids[0] if ids else '')
        self._cache = {}
        self._progress = prefetcher.progress()
        self._online = True
        self._stats_at = 0.0
        cache_root = QStandardPaths.writableLocation(QStandardPaths.GenericCacheLocation)
        self._qt_cache = str(Path(cache_root) / 'drone-gcs' / 'qtlocation')

    def _get_port(self):
        return self.server.port

    def _get_layers(self):
        return self._layers

    def _get_center(self):
        return self._center

    def _get_zoom(self):
        return self._zoom

    def _get_search_default(self):
        return self._search_default

    def _get_max_tiles(self):
        return self._max_tiles

    def _get_online(self):
        return self._online

    def _get_cache(self):
        return self._cache

    def _get_progress(self):
        return self._progress

    def _get_qt_cache(self):
        return self._qt_cache

    def _get_current(self):
        return self._current

    def _set_current(self, layer_id):
        if layer_id != self._current and any(l['id'] == layer_id for l in self._layers):
            self._current = layer_id
            self._settings.setValue('map/layer', layer_id)
            self.layerChanged.emit()

    port = Property(int, _get_port, constant=True)
    layers = Property('QVariantList', _get_layers, constant=True)
    defaultCenter = Property('QVariantList', _get_center, constant=True)
    defaultZoom = Property(float, _get_zoom, constant=True)
    maxPrefetchTiles = Property(int, _get_max_tiles, constant=True)
    searchDefault = Property(str, _get_search_default, constant=True)   # "lat, lon" prefilled in the search box
    online = Property(bool, _get_online, notify=changed)
    cache = Property('QVariantMap', _get_cache, notify=changed)        # layer id -> {tiles, sizeMb}
    prefetch = Property('QVariantMap', _get_progress, notify=changed)
    qtCacheDir = Property(str, _get_qt_cache, constant=True)   # QtLocation's own cache
    currentLayer = Property(str, _get_current, _set_current, notify=layerChanged)

    @Slot(str, float, float, float, float, int, int, result=int)
    def estimateTiles(self, layer_id, west, south, east, north, zmin, zmax):
        from ground_station_ui.core.maps.tiles import count_tiles
        n_sources = len([s for s in self.server.sources(layer_id) if s.url]) or 1
        # Limit: values above Qt int32 raise OverflowError (e.g. zoomed out to the whole world)
        cap = self._max_tiles * 10
        return min(count_tiles(west, south, east, north, zmin, zmax, limit=cap) * n_sources, cap + 1)

    @Slot(str, float, float, float, float, int, int, result=str)
    def startPrefetch(self, layer_id, west, south, east, north, zmin, zmax):
        err = self.prefetcher.start(layer_id, (west, south, east, north), zmin, zmax, self._max_tiles)
        self.flush(force=True)
        return err

    @Slot(str, result='QVariantList')
    def parseLatLon(self, text):
        """'11.052919, 106.666057' -> [lat, lon]; [] if invalid."""
        ll = parse_latlon(text)
        return list(ll) if ll else []

    @Slot(float, float, float)
    def saveView(self, lat, lon, zoom):
        self._settings.setValue('map/lat', lat)
        self._settings.setValue('map/lon', lon)
        self._settings.setValue('map/zoom', zoom)

    @Slot()
    def cancelPrefetch(self):
        self.prefetcher.cancel()

    def flush(self, force=False):
        changed = False
        progress = self.prefetcher.progress()
        if progress != self._progress:
            self._progress, changed = progress, True
        if self.server.online != self._online:
            self._online, changed = self.server.online, True
        now = time.monotonic()
        if force or now - self._stats_at > STATS_PERIOD_S:
            self._stats_at = now
            cache = {}
            for lid in self.server.layers:
                tiles = size = 0
                for src in self.server.sources(lid):     # composite layer = sum of its source layers
                    t, s = src.cache.stats()
                    tiles, size = tiles + t, size + s
                cache[lid] = {'tiles': tiles, 'sizeMb': round(size / 1e6, 1)}
            if cache != self._cache:
                self._cache, changed = cache, True
        if changed:
            self.changed.emit()
