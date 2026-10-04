#!/usr/bin/env python3
"""Drone GCS: van hanh drone trong luc bay (PySide6 + QML).

Nap firmware / cau hinh / hieu chuan PX4 van dung QGroundControl (tat app nay truoc).
Danh sach link: config/gcs.yaml.

    ros2 run ground_station_ui gcs                  # sau khi colcon build
    python3 ground_station_ui/main.py               # chay thang tu source
"""

import signal
import sys
from pathlib import Path

try:
    import ground_station_ui  # noqa: F401
except ImportError:   # chay thang tu source, chua build
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from PySide6.QtCore import QTimer
from PySide6.QtGui import QGuiApplication, QIcon
from PySide6.QtQml import QQmlApplicationEngine

from ground_station_ui.paths import resource_dir
from ground_station_ui.qt.app import GcsApp


def main():
    QGuiApplication.setOrganizationName('fablab')
    QGuiApplication.setApplicationName('drone-gcs')
    # ten khop file .desktop (scripts/install_desktop.sh) -> dock / thanh tac vu dung dung icon
    QGuiApplication.setDesktopFileName('drone-gcs')
    app = QGuiApplication(sys.argv)
    app.setApplicationDisplayName('EIU Drone GCS')
    icon = QIcon()
    for size in (32, 48, 64, 128, 256, 512):
        icon.addFile(str(resource_dir('images') / 'icons' / f'drone-gcs-{size}.png'))
    app.setWindowIcon(icon)
    # Ctrl+C trong terminal: vong lap Qt khong tra quyen cho Python -> timer nho de Python
    # kip xu ly SIGINT
    signal.signal(signal.SIGINT, lambda *_: app.quit())
    sigint_timer = QTimer()
    sigint_timer.timeout.connect(lambda: None)
    sigint_timer.start(200)

    gcs = GcsApp()
    qml = resource_dir('qml')
    engine = QQmlApplicationEngine()
    for name, obj in gcs.context_properties().items():
        engine.rootContext().setContextProperty(name, obj)
    engine.addImportPath(str(qml))
    engine.load(str(qml / 'Main.qml'))
    if not engine.rootObjects():
        sys.exit(1)

    ret = app.exec()
    # Huy QML truoc cac object Python: tranh binding doc object da bi xoa khi thoat
    del engine
    gcs.shutdown()
    sys.exit(ret)


if __name__ == '__main__':
    main()
