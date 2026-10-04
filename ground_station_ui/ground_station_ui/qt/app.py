"""GcsApp: noi cac tang lai voi nhau.

    Link threads ──(lo message)──► LinkBridge (Signal, queued) ──► main thread:
        MessageRouter.dispatch ──► FactGroup (Vehicle, LinkManager)
    UiTicker (ui_rate_hz): tick() cac group theo thoi gian ──► adapter.flush() ──► QML

Toan bo domain chay tren main thread -> khong can khoa; thread chi o tang link.
"""

import time

import yaml
from PySide6.QtCore import QObject, Qt, QTimer, Signal

from ground_station_ui.core.commands.sender import CommandSender
from ground_station_ui.core.links.manager import LinkManager
from ground_station_ui.core.maps.prefetch import Prefetcher
from ground_station_ui.core.maps.tile_server import TileServer
from ground_station_ui.core.mission.mission import Mission
from ground_station_ui.core.mission.uploader import MissionUploader
from ground_station_ui.core.router import MessageRouter
from ground_station_ui.core.vehicle.params import Px4ParamWatcher
from ground_station_ui.core.vehicle.vehicle import Vehicle
from ground_station_ui.paths import data_path, resource_dir
from ground_station_ui.qt.adapters import VehicleAdapter
from ground_station_ui.qt.alerts_adapter import AlertsAdapter
from ground_station_ui.qt.telemetry_adapter import TelemetryAdapter
from ground_station_ui.qt.airspace_adapter import AirspaceAdapter
from ground_station_ui.qt.command_adapter import CommandAdapter
from ground_station_ui.qt.env_adapter import EnvironmentAdapter
from ground_station_ui.qt.voice import VoiceAdapter
from ground_station_ui.qt import geometry3d
from ground_station_ui.qt.features import FEATURES
from ground_station_ui.qt.links_adapter import LinksAdapter
from ground_station_ui.qt.map_adapter import MapAdapter
from ground_station_ui.qt.mission_adapter import MissionAdapter
from ground_station_ui.qt.ports import PortList


class LinkBridge(QObject):
    """Chuyen callback tu thread cua link ve main thread."""
    messages = Signal(object, object)    # (link, [msgs])
    state = Signal(object)               # link


def load_config():
    with open(resource_dir('config') / 'gcs.yaml') as f:
        return yaml.safe_load(f) or {}


def app_info():
    """Thong tin cho splash / About: phien ban (package.xml) + duong dan anh."""
    import re
    from PySide6.QtCore import QUrl
    version = '0.1.0'
    try:
        xml = (resource_dir('config').parent / 'package.xml').read_text()
        version = re.search(r'<version>([^<]+)</version>', xml).group(1)
    except Exception:
        pass
    return {'version': version,
            'imagesUrl': QUrl.fromLocalFile(str(resource_dir('images'))).toString() + '/'}


def load_keys():
    """API key cho nguon ban do (config/keys.yaml, khong commit). Khong co file -> {}."""
    path = resource_dir('config') / 'keys.yaml'
    if not path.exists():
        return {}
    with open(path) as f:
        return yaml.safe_load(f) or {}


class GcsApp:
    def __init__(self, config=None):
        cfg = config if config is not None else load_config()
        geometry3d.register()          # LineGrid / Polyline cho trang 3D
        self.router = MessageRouter()
        self.bridge = LinkBridge()
        self.bridge.messages.connect(self._dispatch, Qt.QueuedConnection)

        self.links = LinkManager(cfg.get('links', []),
                                 on_messages=self.bridge.messages.emit,
                                 on_state=self.bridge.state.emit)
        self.links.attach(self.router)
        self.vehicle = Vehicle(self.router)
        self.px4_params = Px4ParamWatcher(self.vehicle, self.links)
        self.px4_params.attach(self.router)
        self.commands = CommandSender(self.links)
        self.commands.attach(self.router)
        self.uploader = MissionUploader(self.links)
        self.uploader.attach(self.router)
        self.mission = Mission(cfg.get('mission', {}).get('default_altitude', 20.0))

        # Ban do offline: tile server cuc bo (cache .mbtiles), QtLocation doc qua localhost
        map_cfg = cfg.get('map', {})
        self.tiles = TileServer(map_cfg.get('layers', []),
                                data_path(map_cfg.get('cache_dir', 'data/maps')),
                                keys=load_keys())
        self.tiles.start()
        self.prefetcher = Prefetcher(self.tiles)

        # Adapter tao SAU vehicle/links; LinksAdapter co the tu ket noi link da nho
        self.vehicle_qml = VehicleAdapter(self.vehicle)
        self.links_qml = LinksAdapter(self.links)
        self.ports_qml = PortList()
        self.map_qml = MapAdapter(self.tiles, self.prefetcher, map_cfg)
        self.mission_qml = MissionAdapter(self.mission)
        self.airspace_qml = AirspaceAdapter(cfg.get('airspace', {}))
        self.voice_qml = VoiceAdapter(self.vehicle)
        self.env_qml = EnvironmentAdapter(self.vehicle, self.px4_params)
        self.telemetry_qml = TelemetryAdapter(self.vehicle, self.router, self.links)
        self.alerts_qml = AlertsAdapter(self.vehicle, self.links, self.env_qml)
        self.commands_qml = CommandAdapter(self.commands, self.uploader, self.mission, cfg.get('commands', {}))

        self._ticker = QTimer()
        self._ticker.timeout.connect(self._tick)
        self._ticker.start(int(1000 / float(cfg.get('ui_rate_hz', 5))))

    def context_properties(self):
        return {
            'vehicle': self.vehicle_qml,
            'links': self.links_qml,
            'ports': self.ports_qml,
            'mapService': self.map_qml,
            'mission': self.mission_qml,
            'airspace': self.airspace_qml,
            'commands': self.commands_qml,
            'flightEnv': self.env_qml,
            'voice': self.voice_qml,
            'alerts': self.alerts_qml,
            'telemetry': self.telemetry_qml,
            'features': [f for f in FEATURES if f['enabled']],
            'appInfo': app_info(),   # trang chua san sang: an khoi menu
        }

    def _dispatch(self, link, msgs):
        self.router.dispatch(link.name, msgs)

    def _tick(self):
        now = time.monotonic()
        self.vehicle.tick(now)
        self.links.tick(now)
        self.commands.tick(now)
        self.px4_params.tick(now)
        self.uploader.tick(now)
        self.vehicle_qml.flush()
        self.links_qml.flush()
        self.map_qml.flush()
        self.mission_qml.flush()
        self.commands_qml.flush()
        self.env_qml.flush()
        self.voice_qml.tick()
        self.alerts_qml.flush()
        self.telemetry_qml.flush()

    def shutdown(self):
        self._ticker.stop()
        self.prefetcher.cancel()
        self.links.stop_all()
        self.tiles.stop()
