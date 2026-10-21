#!/usr/bin/env python3
"""Drone GCS: in-flight operation of the drone (PySide6 + QML).

Firmware upload, PX4 configuration and calibration still use QGroundControl (close this app first).
Links are listed in config/gcs.yaml.

    ros2 run ground_station_ui gcs                  # after colcon build
    python3 ground_station_ui/main.py               # directly from source
"""

import signal
import sys
from pathlib import Path

# Running from the source tree (config/ next to the package): prefer it over the copy in install/.
# The installed entry point (ros2 run ... gcs) lives in lib/<pkg>/ without config/, so sys.path is unchanged.
_SRC = Path(__file__).resolve().parents[1]
if (_SRC / 'config').is_dir():
    sys.path.insert(0, str(_SRC))

from PySide6.QtCore import QTimer
from PySide6.QtGui import QGuiApplication, QIcon
from PySide6.QtQml import QQmlApplicationEngine

from ground_station_ui.paths import resource_dir
from ground_station_ui.qt.app import GcsApp


def main():
    QGuiApplication.setOrganizationName('fablab')
    QGuiApplication.setApplicationName('drone-gcs')
    # matches the .desktop file name (scripts/install_desktop.sh) so the dock / taskbar shows the right icon
    QGuiApplication.setDesktopFileName('drone-gcs')
    app = QGuiApplication(sys.argv)
    app.setApplicationDisplayName('EIU Drone GCS')
    icon = QIcon()
    for size in (32, 48, 64, 128, 256, 512):
        icon.addFile(str(resource_dir('images') / 'icons' / f'drone-gcs-{size}.png'))
    app.setWindowIcon(icon)
    # SIGINT: the Qt event loop does not return control to Python, so an empty timer lets Python handle it
    signal.signal(signal.SIGINT, lambda *_: app.quit())
    sigint_timer = QTimer()
    sigint_timer.timeout.connect(lambda: None)
    sigint_timer.start(200)

    try:
        gcs = GcsApp()
    except ValueError as e:          # invalid gcs.yaml
        print(e, file=sys.stderr)
        sys.exit(2)
    qml = resource_dir('qml')
    engine = QQmlApplicationEngine()
    for name, obj in gcs.context_properties().items():
        engine.rootContext().setContextProperty(name, obj)
    engine.addImportPath(str(qml))
    engine.load(str(qml / 'Main.qml'))
    if not engine.rootObjects():
        sys.exit(1)

    ret = app.exec()
    # Destroy the QML engine before the Python objects so bindings never read deleted objects
    del engine
    gcs.shutdown()
    sys.exit(ret)


if __name__ == '__main__':
    main()
