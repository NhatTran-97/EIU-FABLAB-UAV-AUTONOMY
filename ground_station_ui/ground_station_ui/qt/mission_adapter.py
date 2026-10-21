"""MissionAdapter: waypoint editing from QML (click the map to add, drag to move...)."""

from PySide6.QtCore import Slot

from ground_station_ui.qt.adapters import GroupAdapter


class MissionAdapter(GroupAdapter):
    """mission.items = [{seq, lat, lon, alt, task, searchAlt, hookAlt}],
    mission.values = {count, defaultAltitude, speed, xyTolerance, zTolerance}"""

    def __init__(self, mission, parent=None):
        super().__init__(mission, parent)
        self.mission = mission

    # UI edits flush immediately instead of waiting for the tick
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

    @Slot(int, str)
    def setTask(self, index, task):
        self.mission.set_task(index, task)
        self._done()

    @Slot(int, float, float)
    def setTaskHeights(self, index, search_alt, hook_alt):
        self.mission.set_task_heights(index, search_alt, hook_alt)
        self._done()

    @Slot(int)
    def remove(self, index):
        self.mission.remove(index)
        self._done()

    @Slot()
    def clear(self):
        self.mission.clear()
        self._done()

    @Slot(float, float)
    def setTolerances(self, xy, z):
        self.mission.set_tolerances(xy, z)
        self._done()

    @Slot(float)
    def setSpeed(self, speed):
        self.mission.set_speed(speed)
        self._done()

    @Slot(float)
    def setDefaultAltitude(self, alt):
        self.mission.set_default_altitude(alt)
        self._done()
