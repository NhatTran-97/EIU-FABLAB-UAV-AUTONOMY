"""Hinh hoc tu dinh nghia cho trang 3D (Qt Quick 3D) -- thay RViz.

Du lieu app: ENU (m, Z len). Qt Quick 3D: Y len. Doi he chi o day va View3DPage.qml:
    scene = (E, U, -N) * SCALE   (1 m = 100 don vi)

    import DroneGcs3D
    Model { geometry: LineGrid { sizeM: 20; stepM: 1 } }
    Model { geometry: Polyline { source: vehicle.trail3d; widthM: 0.05 } }   // tang dan (PathAdapter)
"""

import struct

from PySide6.QtCore import Property, QByteArray, QObject, Signal
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


def ribbon_triangles(pts, width):
    """Polyline (scene) -> tam giac: moi doan = 2 dai rong `width` vuong goc nhau (mat cat chu thap).
    Thay cho lineWidth (nhieu backend RHI chi ve 1 px)."""
    h = width / 2
    out = []
    for (x0, y0, z0), (x1, y1, z1) in zip(pts, pts[1:]):
        dx, dy, dz = x1 - x0, y1 - y0, z1 - z0
        ln = (dx * dx + dy * dy + dz * dz) ** 0.5
        if ln < 1e-9:
            continue
        dx, dy, dz = dx / ln, dy / ln, dz / ln
        ax, ay, az = dz, 0.0, -dx                       # d x len(Y): huong ngang vuong goc doan
        la = (ax * ax + az * az) ** 0.5
        if la < 1e-6:                                    # doan thang dung -> lay truc X
            ax, ay, az, la = 1.0, 0.0, 0.0, 1.0
        ax, ay, az = ax / la, ay / la, az / la
        bx, by, bz = dy * az - dz * ay, dz * ax - dx * az, dx * ay - dy * ax   # d x a
        for ox, oy, oz in ((ax * h, ay * h, az * h), (bx * h, by * h, bz * h)):
            p0a, p0b = (x0 - ox, y0 - oy, z0 - oz), (x0 + ox, y0 + oy, z0 + oz)
            p1a, p1b = (x1 - ox, y1 - oy, z1 - oz), (x1 + ox, y1 + oy, z1 + oz)
            out += [p0a, p0b, p1b, p0a, p1b, p1a]
    return out


class Polyline(_LineGeometry):
    """Polyline ENU (m). widthM > 0: dai rong widthM (can cullMode NoCulling); 0: Lines 1 px.

    Nguon:
      source: PathAdapter -- appended: noi tam giac cua doan moi vao buffer; reset: dung lai toan bo.
      points: [[e, n, u], ...] -- dat ca duong (duong tinh).
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

    # --- dung buffer ---------------------------------------------------------------------
    def _piece(self, a, b):
        """Hinh hoc cho 1 doan a -> b (scene)."""
        if self._width > 0:
            return ribbon_triangles([a, b], self._width * SCALE)
        return [a, b]                                       # Lines: moi doan 2 dinh

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

    # --- nguon tang dan (PathAdapter) -------------------------------------------------------
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

    # --- dat ca duong ------------------------------------------------------------------------
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
    """Goi 1 lan truoc khi nap QML."""
    for cls in (LineGrid, Polyline):
        qmlRegisterType(cls, 'DroneGcs3D', 1, 0, cls.__name__)
