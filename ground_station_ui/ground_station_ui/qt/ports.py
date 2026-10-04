"""Danh sach cong serial USB de chon trong tab Links."""

from PySide6.QtCore import Property, QObject, Signal, Slot
from serial.tools import list_ports


class PortList(QObject):
    changed = Signal()

    def __init__(self, parent=None):
        super().__init__(parent)
        self._ports = []
        self.refresh()

    def _get_ports(self):
        return self._ports

    ports = Property('QVariantList', _get_ports, notify=changed)

    @Slot()
    def refresh(self):
        self._ports = [
            {'device': p.device, 'label': f'{p.device}  ({p.description})'}
            for p in sorted(list_ports.comports(), key=lambda p: p.device)
            if 'USB' in p.device or 'ACM' in p.device
        ]
        self.changed.emit()
