"""Waypoint list being edited on the map (pure Python).

Editing only (add / drag / delete / change altitude / task). Uploading is done by mission/uploader.py with
the MAVLink mission protocol, using this `items` list.

Item: {seq, lat, lon, alt, task, searchAlt, hookAlt}
    task       'none' (fly through), 'pick_auto' or 'pick_manual' (descend, align on the ArUco marker,
               hook the load automatically or by the pilot, climb back)
    searchAlt  height above ground (m) at which the marker is searched
    hookAlt    height above ground (m) at which the load is hooked
"""

from ground_station_ui.core.facts import FactGroup

TASKS = ('none', 'pick_auto', 'pick_manual')
DEFAULT_SEARCH_ALT = 3.0
DEFAULT_HOOK_ALT = 1.0


def _key(wp):
    return (round(float(wp['lat']), 7), round(float(wp['lon']), 7), round(float(wp['alt']), 2),
            wp.get('task', 'none'))


def flying_index(items, uploaded, progress):
    """Index in the edited list of the waypoint the drone is flying to, or -1.

    progress: missionProgress values {valid, state, seq (1-based), total}. The Jetson numbers the waypoints it
    received, so the index is used only while the edited list is the uploaded one (same points in the same
    order); after an edit without Send WP nothing is highlighted rather than the wrong row.
    """
    if not progress.get('valid') or progress.get('state') not in ('active', 'paused'):
        return -1
    seq = int(progress.get('seq') or 0)
    if not 1 <= seq <= len(items) or len(items) != len(uploaded):
        return -1
    if any(_key(a) != _key(b) for a, b in zip(items, uploaded)):
        return -1
    return seq - 1


def _item(lat, lon, alt):
    return {'seq': 0, 'lat': float(lat), 'lon': float(lon), 'alt': float(alt), 'task': 'none',
            'searchAlt': DEFAULT_SEARCH_ALT, 'hookAlt': DEFAULT_HOOK_ALT}


class Mission(FactGroup):
    """values: count, defaultAltitude (m, new waypoints), and options sent with Start mission for the whole
    mission: speed (m/s between waypoints), xyTolerance / zTolerance (m, distance at which a waypoint counts
    as reached)."""

    def __init__(self, default_altitude=20.0, speed=2.0, xy_tolerance=0.5, z_tolerance=0.5):
        super().__init__(defaultAltitude=float(default_altitude), count=0, speed=float(speed),
                         xyTolerance=float(xy_tolerance), zTolerance=float(z_tolerance))
        self.on_options_changed = lambda: None   # set by the Qt layer to push live settings to a running mission

    def restore(self, data):
        """Waypoints and options from a saved session ({'items': [...], 'values': {...}})."""
        items = []
        for wp in data.get('items') or []:
            item = _item(wp['lat'], wp['lon'], wp['alt'])
            if wp.get('task') in TASKS:
                item['task'] = wp['task']
            item['searchAlt'] = float(wp.get('searchAlt', DEFAULT_SEARCH_ALT))
            item['hookAlt'] = float(wp.get('hookAlt', DEFAULT_HOOK_ALT))
            items.append(item)
        self.items = items
        values = data.get('values') or {}
        self.set(**{k: float(values[k]) for k in ('defaultAltitude', 'speed', 'xyTolerance', 'zTolerance')
                    if k in values and float(values[k]) > 0})
        self._changed()

    def _changed(self):
        for i, wp in enumerate(self.items):
            wp['seq'] = i + 1
        self.set(count=len(self.items))
        self.dirty = True

    def add(self, lat, lon, alt=None):
        alt = self.values['defaultAltitude'] if alt is None else float(alt)
        self.items = self.items + [_item(lat, lon, alt)]
        self._changed()

    def insert(self, index, lat, lon):
        """Insert between two points (e.g. dragging the midpoint of a leg)."""
        index = max(0, min(index, len(self.items)))
        alt = self.items[index - 1]['alt'] if index > 0 else self.values['defaultAltitude']
        self.items = self.items[:index] + [_item(lat, lon, alt)] + self.items[index:]
        self._changed()

    def move(self, index, lat, lon):
        if 0 <= index < len(self.items):
            items = [dict(wp) for wp in self.items]
            items[index].update(lat=float(lat), lon=float(lon))
            self.items = items
            self._changed()

    def set_altitude(self, index, alt):
        if 0 <= index < len(self.items):
            items = [dict(wp) for wp in self.items]
            items[index]['alt'] = float(alt)
            self.items = items
            self._changed()

    def set_task(self, index, task):
        """Set the task at a waypoint: 'none', 'pick_auto' or 'pick_manual'."""
        if 0 <= index < len(self.items) and task in TASKS:
            items = [dict(wp) for wp in self.items]
            items[index]['task'] = task
            self.items = items
            self._changed()

    def set_task_heights(self, index, search_alt, hook_alt):
        """Heights above ground (m) for searching the marker and hooking the load."""
        if 0 <= index < len(self.items):
            items = [dict(wp) for wp in self.items]
            items[index].update(searchAlt=float(search_alt), hookAlt=float(hook_alt))
            self.items = items
            self._changed()

    def task_count(self):
        return sum(1 for wp in self.items if wp.get('task', 'none') != 'none')

    def remove(self, index):
        if 0 <= index < len(self.items):
            self.items = self.items[:index] + self.items[index + 1:]
            self._changed()

    def clear(self):
        self.items = []
        self._changed()

    def set_default_altitude(self, alt):
        self.set(defaultAltitude=float(alt))

    def set_tolerances(self, xy, z):
        """Arrival tolerances (m) sent with Start mission and live to a running mission; values <= 0 are ignored."""
        if float(xy) > 0:
            self.set(xyTolerance=round(float(xy), 2))
        if float(z) > 0:
            self.set(zTolerance=round(float(z), 2))
        self.on_options_changed()

    def set_speed(self, speed):
        """Speed between waypoints (m/s) sent with Start mission and live to a running mission."""
        if float(speed) > 0:
            self.set(speed=round(float(speed), 1))
            self.on_options_changed()
