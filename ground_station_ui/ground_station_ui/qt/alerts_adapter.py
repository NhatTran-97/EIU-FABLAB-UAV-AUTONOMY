"""AlertsAdapter: danh sach van de cho QML (Alert Center, System Health, Pre-flight check)."""

from PySide6.QtCore import Property, QObject, Signal

from ground_station_ui.core.alerts.health import evaluate


class AlertsAdapter(QObject):
    changed = Signal()

    def __init__(self, vehicle, links, env, parent=None):
        super().__init__(parent)
        self.vehicle, self.links, self.env = vehicle, links, env
        self._items = []

    def flush(self):
        st = self.env.property('state') or {}
        items = evaluate(self.vehicle, self.links.values, st.get('effective', 'indoor'), st.get('mismatch', ''))
        if items != self._items:
            self._items = items
            self.changed.emit()

    items = Property('QVariantList', lambda self: self._items, notify=changed)
    critical = Property(int, lambda self: sum(a['level'] == 'critical' for a in self._items), notify=changed)
    warning = Property(int, lambda self: sum(a['level'] == 'warning' for a in self._items), notify=changed)
