"""LinksAdapter: tinh trang cac link + nut Connect/Disconnect cho QML.

Cong / baud nguoi dung chon duoc ghi nho (QSettings) va tu ket noi lai khi mo app.
"""

from PySide6.QtCore import QSettings, Slot

from ground_station_ui.qt.adapters import GroupAdapter


class LinksAdapter(GroupAdapter):
    def __init__(self, manager, parent=None):
        super().__init__(manager, parent)
        self.manager = manager
        self._settings = QSettings()
        for name, link in manager.links.items():
            if link.kind == 'serial':
                port = self._settings.value(f'links/{name}/port', link.port, type=str)
                baud = self._settings.value(f'links/{name}/baud', link.baud, type=int)
                link.configure(port, baud)
            autoconnect = self._settings.value(f'links/{name}/autoconnect',
                                               bool(link.cfg.get('autoconnect', False)), type=bool)
            if autoconnect and (link.kind != 'serial' or link.port):
                link.start()

    @Slot(str, str, int)
    def connectLink(self, name, port, baud):
        link = self.manager.get(name)
        if link is None:
            return
        if link.kind == 'serial':
            link.configure(port, baud)
            self._settings.setValue(f'links/{name}/port', port)
            self._settings.setValue(f'links/{name}/baud', baud)
        self._settings.setValue(f'links/{name}/autoconnect', True)
        link.start()

    @Slot(str)
    def disconnectLink(self, name):
        link = self.manager.get(name)
        if link is not None:
            self._settings.setValue(f'links/{name}/autoconnect', False)
            link.stop()
