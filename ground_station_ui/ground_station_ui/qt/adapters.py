"""Adapter: dua FactGroup (Python thuan) len QML.

QML doc `<adapter>.values.<key>` va `<adapter>.items`. `changed` chi phat trong flush() khi group `dirty`
-> tan so danh gia lai binding <= tan so UiTicker.
"""

from PySide6.QtCore import Property, QObject, Signal, Slot


class GroupAdapter(QObject):
    changed = Signal()

    def __init__(self, group, parent=None):
        super().__init__(parent)
        self.group = group

    def _get_values(self):
        return self.group.values

    def _get_items(self):
        return self.group.items

    values = Property('QVariantMap', _get_values, notify=changed)
    items = Property('QVariantList', _get_items, notify=changed)

    def flush(self):
        if self.group.dirty:
            self.group.dirty = False
            self.changed.emit()


class PathAdapter(QObject):
    """PathLog -> QML / geometry, tang dan. flush() phat:
      reset()                         : consumer doc lai toan bo qua paths()
      appended([[seg, a, b(, c)], ...]): chi diem moi
    Khong truyen ca duong qua QVariantList (chi phi O(n) moi lan cap nhat)."""
    reset = Signal()
    appended = Signal('QVariantList')

    def __init__(self, path_log, parent=None):
        super().__init__(parent)
        self.log = path_log

    @Slot(result='QVariantList')
    def paths(self):
        return [[list(p) for p in seg] for seg in self.log.segments]

    def flush(self):
        res = self.log.take()
        if res is None:
            return
        kind, data = res
        if kind == 'reset':
            self.reset.emit()
        else:
            self.appended.emit([[seg, *p] for seg, p in data])


class VehicleAdapter(QObject):
    """vehicle.status / .gps / .home / .battery / .sensors / .flight / .attitude / .messages / .companion"""

    def __init__(self, vehicle, parent=None):
        super().__init__(parent)
        self.vehicle = vehicle
        self.adapters = {name: GroupAdapter(g, self) for name, g in vehicle.groups.items()}
        self._trail3d = PathAdapter(vehicle.groups['localPose'].trail, self)   # ENU, trang 3D
        self._track = PathAdapter(vehicle.groups['track'].path, self)         # lat/lon, ban do

    @Slot()
    def clearTrail(self):
        self.vehicle.groups['localPose'].clear_trail()
        self.adapters['localPose'].flush()

    @Slot()
    def clearTrack(self):
        """Xoa vet hien thi tren ban do (GPX khong doi)."""
        self.vehicle.groups['track'].clear()
        self.adapters['track'].flush()

    def flush(self):
        for a in self.adapters.values():
            a.flush()
        self._trail3d.flush()
        self._track.flush()

    def _a(name):   # noqa: N805 -- getter cho Property
        return lambda self: self.adapters[name]

    status = Property(QObject, _a('status'), constant=True)
    gps = Property(QObject, _a('gps'), constant=True)
    home = Property(QObject, _a('home'), constant=True)
    battery = Property(QObject, _a('battery'), constant=True)
    sensors = Property(QObject, _a('sensors'), constant=True)
    flight = Property(QObject, _a('flight'), constant=True)
    attitude = Property(QObject, _a('attitude'), constant=True)
    messages = Property(QObject, _a('messages'), constant=True)
    companion = Property(QObject, _a('companion'), constant=True)
    localPose = Property(QObject, _a('localPose'), constant=True)
    gimbal = Property(QObject, _a('gimbal'), constant=True)
    track = Property(QObject, _a('track'), constant=True)
    trail3d = Property(QObject, lambda self: self._trail3d, constant=True)
    trackPath = Property(QObject, lambda self: self._track, constant=True)
    del _a
