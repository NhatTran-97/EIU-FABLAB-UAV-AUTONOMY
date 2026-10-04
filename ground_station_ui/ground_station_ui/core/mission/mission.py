"""Danh sach waypoint dang soan tren ban do (Python thuan).

Buoc nay chi soan (them / keo / xoa / sua do cao). Upload xuong PX4 qua giao thuc mission
(MISSION_COUNT -> MISSION_REQUEST_INT -> MISSION_ITEM_INT -> MISSION_ACK) se them o buoc sau,
dung chinh danh sach `items` nay.
"""

from ground_station_ui.core.facts import FactGroup


class Mission(FactGroup):
    def __init__(self, default_altitude=20.0):
        super().__init__(defaultAltitude=float(default_altitude), count=0)

    def _changed(self):
        for i, wp in enumerate(self.items):
            wp['seq'] = i + 1
        self.set(count=len(self.items))
        self.dirty = True

    def add(self, lat, lon, alt=None):
        alt = self.values['defaultAltitude'] if alt is None else float(alt)
        self.items = self.items + [{'seq': 0, 'lat': float(lat), 'lon': float(lon), 'alt': alt}]
        self._changed()

    def insert(self, index, lat, lon):
        """Chen giua 2 diem (vd keo diem giua tren duong bay)."""
        index = max(0, min(index, len(self.items)))
        alt = self.items[index - 1]['alt'] if index > 0 else self.values['defaultAltitude']
        self.items = self.items[:index] + [{'seq': 0, 'lat': lat, 'lon': lon, 'alt': alt}] + self.items[index:]
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

    def remove(self, index):
        if 0 <= index < len(self.items):
            self.items = self.items[:index] + self.items[index + 1:]
            self._changed()

    def clear(self):
        self.items = []
        self._changed()

    def set_default_altitude(self, alt):
        self.set(defaultAltitude=float(alt))
