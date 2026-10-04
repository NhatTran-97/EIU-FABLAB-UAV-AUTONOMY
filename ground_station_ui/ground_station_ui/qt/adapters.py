"""Adapter: dua FactGroup (Python thuan) len QML.

QML doc `<adapter>.values.<key>` va `<adapter>.items`. Adapter chi phat `changed` trong flush()
khi group `dirty` -> so lan QML danh gia lai binding toi da = tan so UiTicker.
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


class VehicleAdapter(QObject):
    """vehicle.status / .gps / .home / .battery / .sensors / .flight / .attitude / .messages / .companion"""

    def __init__(self, vehicle, parent=None):
        super().__init__(parent)
        self.vehicle = vehicle
        self.adapters = {name: GroupAdapter(g, self) for name, g in vehicle.groups.items()}

    @Slot()
    def clearTrail(self):
        self.vehicle.groups['localPose'].clear_trail()
        self.adapters['localPose'].flush()

    def flush(self):
        for a in self.adapters.values():
            a.flush()

    def _a(name):   # noqa: N805 -- tao getter cho Property
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
    del _a
