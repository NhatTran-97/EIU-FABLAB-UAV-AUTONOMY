"""GcsApp: wires the layers together.

    Link threads ──(message batches)──► LinkBridge (Signal, queued) ──► main thread:
        MessageRouter.dispatch ──► FactGroup (Vehicle, LinkManager)
    UiTicker (ui_rate_hz): time-based tick() of the groups ──► adapter.flush() ──► QML

The whole domain runs on the main thread, so no locks are needed; threads exist only in the link layer.
"""

import logging
import time

import yaml
from PySide6.QtCore import QObject, Qt, QTimer, Signal

from ground_station_ui.core.commands.sender import CommandSender
from ground_station_ui.core.config import validate as validate_config
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
from ground_station_ui.qt.planned_path_adapter import PlannedPathAdapter
from ground_station_ui.qt.ports import PortList


log = logging.getLogger(__name__)


class LinkBridge(QObject):
    """Move link-thread callbacks to the main thread."""
    messages = Signal(object, object)    # (link, [msgs])
    state = Signal(object)               # link


def load_config():
    path = resource_dir('config') / 'gcs.yaml'
    with open(path) as f:
        cfg = yaml.safe_load(f) or {}
    errors, warnings = validate_config(cfg)
    for w in warnings:
        log.warning('%s: %s', path.name, w)
    if errors:
        raise ValueError(f'Invalid configuration {path}:\n  - ' + '\n  - '.join(errors))
    return cfg


def app_info():
    """Info for the splash / About: version (package.xml) and image path."""
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
    """API keys for map sources (config/keys.yaml, not committed). Missing file -> {}."""
    path = resource_dir('config') / 'keys.yaml'
    if not path.exists():
        return {}
    with open(path) as f:
        return yaml.safe_load(f) or {}


class GcsApp:
    def __init__(self, config=None):
        cfg = config if config is not None else load_config()
        geometry3d.register()          # LineGrid / Polyline for the 3D page
        self.router = MessageRouter()
        self.bridge = LinkBridge()
        self.bridge.messages.connect(self._dispatch, Qt.QueuedConnection)

        self.links = LinkManager(cfg.get('links', []),
                                 on_messages=self.bridge.messages.emit,
                                 on_state=self.bridge.state.emit)
        self.links.attach(self.router)
        # GPX directory: tracks.dir (default data/tracks)
        vcfg = dict(cfg.get('vehicle', {}))
        vcfg.setdefault('track_dir', str(data_path((cfg.get('tracks') or {}).get('dir', 'data/tracks'))))
        self.vehicle = Vehicle(self.router, vcfg)
        # autopilot changed -> clear trails (different source / frame)
        self.vehicle.on_identity_change.append(self.vehicle.groups['localPose'].clear_trail)
        self.vehicle.on_identity_change.append(self.vehicle.groups['track'].clear)
        self.px4_params = Px4ParamWatcher(self.vehicle, self.links)
        self.px4_params.attach(self.router)
        self.commands = CommandSender(self.links)           # commands via the companion (433)
        self.commands.attach(self.router)
        self.direct_commands = CommandSender(self.links)    # commands directly to the autopilot (915)
        self.direct_commands.attach(self.router)
        self.settings_commands = CommandSender(self.links)  # live mission settings; never replaces a flight command
        self.settings_commands.attach(self.router)
        self.uploader = MissionUploader(self.links)
        self.uploader.attach(self.router)
        mission_cfg = cfg.get('mission', {})
        self.mission = Mission(mission_cfg.get('default_altitude', 20.0), mission_cfg.get('default_speed', 2.0),
                               mission_cfg.get('xy_tolerance', 0.75), mission_cfg.get('z_tolerance', 0.5))

        # localhost tile server with .mbtiles cache (works offline)
        map_cfg = cfg.get('map', {})
        self.tiles = TileServer(map_cfg.get('layers', []),
                                data_path(map_cfg.get('cache_dir', 'data/maps')),
                                keys=load_keys())
        self.tiles.start()
        self.prefetcher = Prefetcher(self.tiles)

        # adapters are created after vehicle / links (LinksAdapter may auto-connect)
        self.vehicle_qml = VehicleAdapter(self.vehicle)
        self.planned_path_qml = PlannedPathAdapter(self.uploader, self.vehicle)   # 3D view reference path
        self.links_qml = LinksAdapter(self.links)
        self.ports_qml = PortList()
        self.map_qml = MapAdapter(self.tiles, self.prefetcher, map_cfg)
        self.mission_qml = MissionAdapter(self.mission)
        self.airspace_qml = AirspaceAdapter(cfg.get('airspace', {}))
        self.voice_qml = VoiceAdapter(self.vehicle)
        self.env_qml = EnvironmentAdapter(self.vehicle, self.px4_params)
        self.telemetry_qml = TelemetryAdapter(self.vehicle, self.router, self.links)
        self.alerts_qml = AlertsAdapter(self.vehicle, self.links, self.env_qml)
        self.commands_qml = CommandAdapter(self.commands, self.direct_commands, self.settings_commands,
                                           self.uploader, self.mission,
                                           self.vehicle, lambda: self.env_qml.state.get('effective', 'indoor'),
                                           cfg.get('commands', {}))

        self._tick_errors = {}            # step -> time of the last logged error
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
            'plannedPath': self.planned_path_qml,
            'features': [f for f in FEATURES if f['enabled']],
            'appInfo': app_info(),   # pages not ready yet are hidden from the menu
        }

    def _dispatch(self, link, msgs):
        self.router.dispatch(link.name, msgs)

    def _tick(self):
        """UiTicker (ui_rate_hz): tick the domain, then flush the adapters. Each step has its own try/except,
        so an error in one step does not skip the others."""
        now = time.monotonic()
        steps = (
            ('vehicle', lambda: self.vehicle.tick(now)),
            ('links', lambda: self.links.tick(now)),
            ('commands', lambda: self.commands.tick(now)),
            ('direct_commands', lambda: self.direct_commands.tick(now)),
            ('settings_commands', lambda: self.settings_commands.tick(now)),
            ('commands_qml.tick', lambda: self.commands_qml.tick(now)),
            ('px4_params', lambda: self.px4_params.tick(now)),
            ('uploader', lambda: self.uploader.tick(now)),
            ('vehicle_qml', self.vehicle_qml.flush),
            ('planned_path_qml', self.planned_path_qml.flush),
            ('links_qml', self.links_qml.flush),
            ('map_qml', self.map_qml.flush),
            ('mission_qml', self.mission_qml.flush),
            ('commands_qml', self.commands_qml.flush),
            ('env_qml', self.env_qml.flush),
            ('voice', self.voice_qml.tick),
            ('alerts_qml', self.alerts_qml.flush),
            ('telemetry_qml', self.telemetry_qml.flush),
        )
        for name, step in steps:
            try:
                step()
            except Exception:                            # log at most once every 10 s per step
                last = self._tick_errors.get(name, 0.0)
                if now - last > 10.0:
                    self._tick_errors[name] = now
                    log.exception('UI tick: step %s failed (other steps still run)', name)

    def shutdown(self):
        self._ticker.stop()
        # stop prefetching before the tile server closes SQLite
        if not self.prefetcher.cancel_and_wait(6.0):
            log.warning('Prefetch did not stop in time on shutdown')
        self.links.stop_all()
        self.tiles.stop()
