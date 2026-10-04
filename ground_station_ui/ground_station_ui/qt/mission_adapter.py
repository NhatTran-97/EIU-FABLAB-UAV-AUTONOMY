"""MissionAdapter: soan waypoint tu QML (bam ban do de them, keo de di chuyen...)."""

from PySide6.QtCore import Slot

from ground_station_ui.qt.adapters import GroupAdapter


class MissionAdapter(GroupAdapter):
    """mission.items = [{seq, lat, lon, alt}], mission.values = {count, defaultAltitude}"""

    def __init__(self, mission, parent=None):
        super().__init__(mission, parent)
        self.mission = mission

    # Thao tac cua nguoi dung -> cap nhat ngay (khong doi tick) cho cam giac muot
    def _done(self):
        self.flush()

    @Slot(float, float)
    def add(self, lat, lon):
        self.mission.add(lat, lon)
        self._done()

    @Slot(int, float, float)
    def insert(self, index, lat, lon):
        self.mission.insert(index, lat, lon)
        self._done()

    @Slot(int, float, float)
    def move(self, index, lat, lon):
        self.mission.move(index, lat, lon)
        self._done()

    @Slot(int, float)
    def setAltitude(self, index, alt):
        self.mission.set_altitude(index, alt)
        self._done()

    @Slot(int)
    def remove(self, index):
        self.mission.remove(index)
        self._done()

    @Slot()
    def clear(self):
        self.mission.clear()
        self._done()

    @Slot(float)
    def setDefaultAltitude(self, alt):
        self.mission.set_default_altitude(alt)
        self._done()
