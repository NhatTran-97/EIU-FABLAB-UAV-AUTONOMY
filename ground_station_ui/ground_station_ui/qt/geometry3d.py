"""Custom geometry for the 3D page (Qt Quick 3D), replacing RViz.

App data is ENU (m, Z up); Qt Quick 3D is Y up. The conversion happens only here and in View3DPage.qml:
    scene = (E, U, -N) * SCALE   (1 m = 100 units)

    import DroneGcs3D
    Model { geometry: LineGrid { sizeM: 20; stepM: 1 } }
    Model { geometry: Polyline { source: vehicle.trail3d; widthM: 0.05 } }   // incremental (PathAdapter)
"""

import struct

from PySide6.QtCore import Property, QByteArray, QObject, Signal
from PySide6.QtGui import QVector3D
from PySide6.QtQml import qmlRegisterType
from PySide6.QtQuick3D import QQuick3DGeometry

SCALE = 100.0          # 1 m = 100 Qt Quick 3D units


def enu_to_scene(e, n, u):
    return e * SCALE, u * SCALE, -n * SCALE


class _LineGeometry(QQuick3DGeometry):
    """Base class: uploads a list of (x, y, z) points, already in scene coordinates, to the GPU."""

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
    """Ground grid (E-N plane, U = 0), sizeM metres across, stepM metre cells, centred on the origin."""
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
            pts += [enu_to_scene(t, -half, 0), enu_to_scene(t, half, 0),     # north-south lines
                    enu_to_scene(-half, t, 0), enu_to_scene(half, t, 0)]     # east-west lines
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


def ribbon_triangles(pts, width):
    """Polyline (scene) -> triangles: each segment becomes two perpendicular ribbons of `width` (cross
    section). Replaces lineWidth, which many RHI backends draw as 1 px."""
    h = width / 2
    out = []
    for (x0, y0, z0), (x1, y1, z1) in zip(pts, pts[1:]):
        dx, dy, dz = x1 - x0, y1 - y0, z1 - z0
        ln = (dx * dx + dy * dy + dz * dz) ** 0.5
        if ln < 1e-9:
            continue
        dx, dy, dz = dx / ln, dy / ln, dz / ln
        ax, ay, az = dz, 0.0, -dx                       # d x up(Y): horizontal direction perpendicular to the segment
        la = (ax * ax + az * az) ** 0.5
        if la < 1e-6:                                    # vertical segment -> use the X axis
            ax, ay, az, la = 1.0, 0.0, 0.0, 1.0
        ax, ay, az = ax / la, ay / la, az / la
        bx, by, bz = dy * az - dz * ay, dz * ax - dx * az, dx * ay - dy * ax   # d x a
        for ox, oy, oz in ((ax * h, ay * h, az * h), (bx * h, by * h, bz * h)):
            p0a, p0b = (x0 - ox, y0 - oy, z0 - oz), (x0 + ox, y0 + oy, z0 + oz)
            p1a, p1b = (x1 - ox, y1 - oy, z1 - oz), (x1 + ox, y1 + oy, z1 + oz)
            out += [p0a, p0b, p1b, p0a, p1b, p1a]
    return out


class Polyline(_LineGeometry):
    """ENU polyline (m). widthM > 0: ribbon of width widthM (needs cullMode NoCulling); 0: 1 px Lines.

    Sources:
      source: PathAdapter -- appended: append the new segments' triangles to the buffer; reset: rebuild.
      points: [[e, n, u], ...] -- set the whole path (static paths).
    """
    changed = Signal()

    def __init__(self, parent=None):
        super().__init__(parent)
        self._points = []
        self._width = 0.0
        self._source = None
        self._segs = []                  # [[(x, y, z) scene], ...]
        self._buf = bytearray()
        self._lo = [float('inf')] * 3
        self._hi = [float('-inf')] * 3
        self._rebuild()

    # --- buffer building ---------------------------------------------------------------
    def _piece(self, a, b):
        """Geometry for one segment a -> b (scene)."""
        if self._width > 0:
            return ribbon_triangles([a, b], self._width * SCALE)
        return [a, b]                                       # Lines: two vertices per segment

    def _add_piece(self, pts):
        if not pts:
            return
        self._buf += struct.pack(f'<{3 * len(pts)}f', *[c for p in pts for c in p])
        for p in pts:
            for i in range(3):
                if p[i] < self._lo[i]:
                    self._lo[i] = p[i]
                if p[i] > self._hi[i]:
                    self._hi[i] = p[i]

    def _rebuild(self):
        self._buf = bytearray()
        self._lo, self._hi = [float('inf')] * 3, [float('-inf')] * 3
        for seg in self._segs:
            for a, b in zip(seg, seg[1:]):
                self._add_piece(self._piece(a, b))
        self._commit()

    def _commit(self):
        self.clear()
        n = len(self._buf) // 12
        if n == 0:
            self._buf = bytearray(struct.pack('<3f', 0.0, 0.0, 0.0))
            self._lo, self._hi = [0.0] * 3, [0.0] * 3
        self.setVertexData(QByteArray(bytes(self._buf)))
        self.setStride(12)
        self.setPrimitiveType(QQuick3DGeometry.PrimitiveType.Triangles if self._width > 0
                              else QQuick3DGeometry.PrimitiveType.Lines)
        self.addAttribute(QQuick3DGeometry.Attribute.Semantic.PositionSemantic, 0,
                          QQuick3DGeometry.Attribute.ComponentType.F32Type)
        self.setBounds(QVector3D(*self._lo), QVector3D(*self._hi))
        self.update()
        if n == 0:
            self._buf = bytearray()

    # --- incremental source (PathAdapter) ----------------------------------------------------
    def _from_source(self):
        log = self._source.log if self._source is not None else None
        self._segs = [[enu_to_scene(*p) for p in seg] for seg in log.segments] if log else []
        self._rebuild()

    def _on_appended(self, items):
        for it in items:
            si, p = int(it[0]), enu_to_scene(*it[1:4])
            while len(self._segs) <= si:
                self._segs.append([])
            seg = self._segs[si]
            if seg:
                self._add_piece(self._piece(seg[-1], p))
            seg.append(p)
        self._commit()

    def _set_source(self, src):
        if src is self._source:
            return
        if self._source is not None:
            self._source.reset.disconnect(self._from_source)
            self._source.appended.disconnect(self._on_appended)
        self._source = src
        if src is not None:
            src.reset.connect(self._from_source)
            src.appended.connect(self._on_appended)
        self._from_source()
        self.changed.emit()

    # --- whole path ---------------------------------------------------------------------------
    def _set_points(self, pts):
        self._points = [tuple(float(c) for c in p) for p in (pts or [])]
        self._segs = [[enu_to_scene(*p) for p in self._points]]
        self._rebuild()
        self.changed.emit()

    def _set_width(self, v):
        if v >= 0 and v != self._width:
            self._width = float(v)
            self._rebuild()
            self.changed.emit()

    points = Property('QVariantList', lambda self: [list(p) for p in self._points], _set_points, notify=changed)
    widthM = Property(float, lambda self: self._width, _set_width, notify=changed)
    source = Property(QObject, lambda self: self._source, _set_source, notify=changed)


def register():
    """Call once before loading QML."""
    for cls in (LineGrid, Polyline):
        qmlRegisterType(cls, 'DroneGcs3D', 1, 0, cls.__name__)
