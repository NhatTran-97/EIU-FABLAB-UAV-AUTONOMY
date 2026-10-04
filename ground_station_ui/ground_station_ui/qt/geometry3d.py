"""Hinh hoc tu dinh nghia cho trang 3D (Qt Quick 3D) -- thay RViz.

Quy uoc: du lieu trong app la ENU (met, Z len) nhu ROS. Qt Quick 3D la Y len, nen doi
DUY NHAT o day va trong View3DPage.qml:  scene = (E, U, -N) * SCALE   (1 m = 100 don vi).

    import DroneGcs3D
    Model { geometry: LineGrid { sizeM: 20; stepM: 1 } }
    Model { geometry: Polyline { points: vehicle.localPose.items } }   // [[e, n, u], ...]
"""

import struct

from PySide6.QtCore import Property, QByteArray, Signal
from PySide6.QtGui import QVector3D
from PySide6.QtQml import qmlRegisterType
from PySide6.QtQuick3D import QQuick3DGeometry

SCALE = 100.0          # 1 m = 100 don vi Qt Quick 3D


def enu_to_scene(e, n, u):
    return e * SCALE, u * SCALE, -n * SCALE


class _LineGeometry(QQuick3DGeometry):
    """Co so: nap danh sach diem (x, y, z) da o toa do scene vao GPU."""

    def _upload(self, pts, primitive):
        self.clear()
        if not pts:
            pts = [(0.0, 0.0, 0.0)]
        data = struct.pack(f'<{3 * len(pts)}f', *[c for p in pts for c in p])
        self.setVertexData(QByteArray(data))
        self.setStride(12)
        self.setPrimitiveType(primitive)
        self.addAttribute(QQuick3DGeometry.Attribute.Semantic.PositionSemantic, 0,
                          QQuick3DGeometry.Attribute.ComponentType.F32Type)
        xs, ys, zs = zip(*pts)
        self.setBounds(QVector3D(min(xs), min(ys), min(zs)), QVector3D(max(xs), max(ys), max(zs)))
        self.update()


class LineGrid(_LineGeometry):
    """Luoi san (mat phang E-N, U = 0), canh sizeM met, o stepM met, tam tai goc."""
    changed = Signal()

    def __init__(self, parent=None):
        super().__init__(parent)
        self._size = 20.0
        self._step = 1.0
        self._rebuild()

    def _rebuild(self):
        half = self._size / 2
        n = max(1, int(round(self._size / self._step)))
        pts = []
        for i in range(n + 1):
            t = -half + i * self._step
            pts += [enu_to_scene(t, -half, 0), enu_to_scene(t, half, 0),     # duong doc (Bac-Nam)
                    enu_to_scene(-half, t, 0), enu_to_scene(half, t, 0)]     # duong ngang (Dong-Tay)
        self._upload(pts, QQuick3DGeometry.PrimitiveType.Lines)

    def _set_size(self, v):
        if v > 0 and v != self._size:
            self._size = float(v)
            self._rebuild()
            self.changed.emit()

    def _set_step(self, v):
        if v > 0 and v != self._step:
            self._step = float(v)
            self._rebuild()
            self.changed.emit()

    sizeM = Property(float, lambda self: self._size, _set_size, notify=changed)
    stepM = Property(float, lambda self: self._step, _set_step, notify=changed)


class Polyline(_LineGeometry):
    """Duong noi cac diem ENU [[e, n, u], ...] (vet bay)."""
    changed = Signal()

    def __init__(self, parent=None):
        super().__init__(parent)
        self._points = []
        self._upload([], QQuick3DGeometry.PrimitiveType.LineStrip)

    def _set_points(self, pts):
        pts = [tuple(float(c) for c in p) for p in (pts or [])]
        self._points = pts
        self._upload([enu_to_scene(*p) for p in pts], QQuick3DGeometry.PrimitiveType.LineStrip)
        self.changed.emit()

    points = Property('QVariantList', lambda self: [list(p) for p in self._points], _set_points, notify=changed)


def register():
    """Goi 1 lan truoc khi nap QML."""
    for cls in (LineGrid, Polyline):
        qmlRegisterType(cls, 'DroneGcs3D', 1, 0, cls.__name__)
