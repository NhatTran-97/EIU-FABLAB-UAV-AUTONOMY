"""AirspaceAdapter: vung cam / han che bay cho QML (ve polygon, canh bao waypoint)."""

from pathlib import Path

from PySide6.QtCore import Property, QObject, QSettings, Signal, Slot

from ground_station_ui.core.maps.airspace import load_dirs, zones_at
from ground_station_ui.paths import data_path

# Thu tu ve: vung nhe truoc, vung cam ve sau cung (nam tren) -- giong cambay.mod.gov.vn
KIND_TITLES = {'prohibited': 'No-fly zone', 'military': 'Military area', 'restricted': 'Restricted zone',
               'danger': 'Danger zone', 'ctr': 'Airport zone', 'other': 'Other zone'}
DRAW_ORDER = {'ctr': 0, 'other': 1, 'danger': 2, 'restricted': 3, 'military': 4, 'prohibited': 5}


class AirspaceAdapter(QObject):
    changed = Signal()

    def __init__(self, cfg, parent=None):
        super().__init__(parent)
        self._dirs = [str(data_path(d)) for d in cfg.get('dirs', ['data/airspace'])]
        for d in self._dirs[:1]:
            Path(d).mkdir(parents=True, exist_ok=True)   # thu muc dau tien: noi nguoi dung chep file vao
        self._settings = QSettings()
        self._visible = self._settings.value('airspace/visible', True, type=bool)
        hidden = self._settings.value('airspace/hiddenKinds', [], type=list)
        self._hidden_kinds = [str(k) for k in (hidden or [])]
        self._zones = []
        self._items = []
        self._kinds = []
        self._revision = 0
        self.reload()

    @Slot()
    def reload(self):
        self._zones = load_dirs(self._dirs)
        self._items = [z.to_dict() for z in sorted(self._zones, key=lambda z: DRAW_ORDER.get(z.kind, 1))]
        self._kinds = sorted({z.kind for z in self._zones}, key=lambda k: -DRAW_ORDER.get(k, 1))
        self._revision += 1
        self.changed.emit()

    @Slot(float, float, result=str)
    def namesAt(self, lat, lon):
        """'' neu (lat, lon) khong nam trong vung nao, nguoc lai ten cac vung."""
        names = []
        for z in sorted(zones_at(self._zones, lat, lon), key=lambda z: -DRAW_ORDER.get(z.kind, 1)):
            n = z.name or KIND_TITLES.get(z.kind, z.kind)
            if n not in names:
                names.append(n)
        return ', '.join(names)

    @Slot(str)
    def toggleKind(self, kind):
        """An / hien 1 loai vung (o tick trong chu thich)."""
        if kind in self._hidden_kinds:
            self._hidden_kinds.remove(kind)
        else:
            self._hidden_kinds.append(kind)
        self._settings.setValue('airspace/hiddenKinds', self._hidden_kinds)
        self.changed.emit()

    def _set_visible(self, v):
        if v != self._visible:
            self._visible = v
            self._settings.setValue('airspace/visible', v)
            self.changed.emit()

    zones = Property('QVariantList', lambda self: self._items, notify=changed)
    count = Property(int, lambda self: len(self._items), notify=changed)
    revision = Property(int, lambda self: self._revision, notify=changed)   # binding namesAt() phu thuoc
    kinds = Property('QStringList', lambda self: self._kinds, notify=changed)            # loai co trong du lieu
    hiddenKinds = Property('QStringList', lambda self: self._hidden_kinds, notify=changed)
    dirs = Property('QStringList', lambda self: self._dirs, constant=True)
    visible = Property(bool, lambda self: self._visible, _set_visible, notify=changed)
