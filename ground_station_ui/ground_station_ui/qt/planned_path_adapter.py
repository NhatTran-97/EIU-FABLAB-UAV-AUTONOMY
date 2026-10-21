"""PlannedPathAdapter: the last uploaded waypoints (Send WP) in the local frame of the 3D view.

Recomputed on every UI flush from the anchor (drone GPS position + local pose, see
core/geometry/planned_path.py); signals are emitted only when the path moved by more than MOVE_M or the
upload changed, so the 3D geometry is not rebuilt at every tick.

QML: plannedPath.points [[e, n, u], ...], plannedPath.markers [{seq, e, n, u, task}], plannedPath.valid,
     plannedPath.reach (largest |e| / |n| of the path, for the grid size)
"""

from PySide6.QtCore import Property, QObject, Signal

from ground_station_ui.core.geometry.planned_path import planned_path_enu

MOVE_M = 0.3


class PlannedPathAdapter(QObject):
    changed = Signal()

    def __init__(self, uploader, vehicle, parent=None):
        super().__init__(parent)
        self.uploader = uploader
        self.vehicle = vehicle
        self._points, self._markers, self._reach = [], [], 0.0
        self._revision = -1

    def _anchor(self):
        g = self.vehicle.groups
        gps, pose, rel = g['gps'].values, g['localPose'].values, g['flight'].values.get('altHome')
        if gps.get('lat') is None or not pose.get('valid') or rel is None:
            return None
        return gps['lat'], gps['lon'], float(rel), pose['e'], pose['n'], pose['u']

    def flush(self):
        waypoints = self.uploader.uploaded
        anchor = self._anchor() if waypoints else None
        if anchor is None:
            if self._points and not waypoints:
                self._set([], [])
            return
        home = self.vehicle.groups['home'].values
        points, markers = planned_path_enu(waypoints, (home['lat'], home['lon']) if home.get('valid') else None,
                                           anchor)
        moved = (len(points) != len(self._points)
                 or max(abs(a - b) for p, q in zip(points, self._points) for a, b in zip(p, q)) > MOVE_M)
        if moved or self.uploader.revision != self._revision:
            self._revision = self.uploader.revision
            self._set(points, markers)

    def _set(self, points, markers):
        self._points, self._markers = points, markers
        self._reach = max((max(abs(p[0]), abs(p[1])) for p in points), default=0.0)
        self.changed.emit()

    points = Property('QVariantList', lambda self: self._points, notify=changed)
    markers = Property('QVariantList', lambda self: self._markers, notify=changed)
    valid = Property(bool, lambda self: len(self._points) > 1, notify=changed)
    reach = Property(float, lambda self: self._reach, notify=changed)
