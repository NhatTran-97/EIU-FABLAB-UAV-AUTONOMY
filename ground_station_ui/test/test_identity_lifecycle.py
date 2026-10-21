"""Review scenarios: two autopilots / SITL + real drone, Jetson data from the wrong source,
closing the app while maps download, duplicate tile downloads, bad configuration."""

import threading
import time

from ground_station_ui.core.config import validate
from ground_station_ui.core.mavlink import drone_dialect as d
from ground_station_ui.core.router import MessageRouter
from ground_station_ui.core.vehicle.vehicle import Vehicle

REAL = d.MAVLink(None, srcSystem=1, srcComponent=1)
SITL = d.MAVLink(None, srcSystem=1, srcComponent=1)
JET = d.MAVLink(None, srcSystem=1, srcComponent=191)
ROGUE = d.MAVLink(None, srcSystem=9, srcComponent=191)


def rt(mav, m):
    return d.MAVLink(None).parse_buffer(m.pack(mav))[0]


def hb(mav, mode=3 << 16):
    return rt(mav, mav.heartbeat_encode(d.MAV_TYPE_QUADROTOR, d.MAV_AUTOPILOT_PX4,
                                        d.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, mode, 3))


def make(cfg=None):
    r = MessageRouter()
    return r, Vehicle(r, cfg or {'autopilot_links': ['px4', 'sitl'], 'companion_link': 'jetson'})


def test_real_radio_preferred_over_sitl_and_stale_reidentify(monkeypatch):
    now = [100.0]
    monkeypatch.setattr('ground_station_ui.core.vehicle.vehicle.time.monotonic', lambda: now[0])
    r, v = make()
    r.dispatch('sitl', [hb(SITL, 7 << 16)])                       # SITL comes up first (Stabilized)
    assert v.link == 'sitl'
    changed = []
    v.on_identity_change.append(lambda: changed.append(v.link))
    r.dispatch('px4', [hb(REAL, 3 << 16)])                        # real radio has priority -> switch
    assert v.link == 'px4' and changed == ['px4']
    assert v.groups['status'].values['mode'] == 'Position'
    r.dispatch('sitl', [hb(SITL, 7 << 16)])                       # SITL still sending: ignored
    assert v.groups['status'].values['mode'] == 'Position' and v.link == 'px4'
    att = rt(SITL, SITL.attitude_encode(0, 0.5, 0, 0, 0, 0, 0))
    r.dispatch('sitl', [att])                                     # same sysid/compid but wrong link
    assert v.groups['attitude'].values['valid'] is False
    now[0] += 6                                                   # real radio silent > 5 s
    r.dispatch('sitl', [hb(SITL, 7 << 16)])
    assert v.link == 'sitl'


def test_link_not_allowed_and_fixed_sysid():
    r, v = make({'autopilot_links': ['px4'], 'sysid': 1})
    r.dispatch('sitl', [hb(SITL)])
    assert v.sysid is None                                        # link not allowed
    other = d.MAVLink(None, srcSystem=2, srcComponent=1)
    r.dispatch('px4', [hb(other)])
    assert v.sysid is None                                        # wrong fixed sysid
    r.dispatch('px4', [hb(REAL)])
    assert (v.link, v.sysid) == ('px4', 1)


def test_companion_data_only_from_jetson_link_and_ids():
    r, v = make()
    r.dispatch('px4', [hb(REAL)])
    st = lambda mav: rt(mav, mav.drone_component_status_encode(1, 0, 7.0, b'STREAMING'))  # noqa: E731
    gim = lambda mav: rt(mav, mav.drone_gimbal_state_encode(10.0, -30.0, 0.0, 1, 1))     # noqa: E731
    r.dispatch('sitl', [st(JET), gim(JET)])                      # right id but wrong link
    r.dispatch('jetson', [st(ROGUE), gim(ROGUE)])                # right link but wrong sysid
    v.tick(time.monotonic())
    assert v.groups['companion'].items == [] and v.groups['gimbal'].values['valid'] is False
    r.dispatch('jetson', [st(JET), gim(JET)])
    v.tick(time.monotonic())
    assert len(v.groups['companion'].items) == 1 and v.groups['gimbal'].values['pitch'] == -30.0


def test_shutdown_while_prefetching_waits_then_closes(tmp_path):
    from ground_station_ui.core.maps.prefetch import Prefetcher
    from ground_station_ui.core.maps.tile_server import TileServer

    started = threading.Event()
    ts = TileServer([{'id': 'sat', 'url': 'http://127.0.0.1:9/{z}/{x}/{y}'}], tmp_path)

    def slow(layer, z, x, y):
        started.set()
        time.sleep(0.2)
        layer.cache.put(z, x, y, b'x')
        return b'x'
    ts.fetch_upstream = slow
    pre = Prefetcher(ts)
    assert pre.start('sat', (106.0, 11.0, 106.05, 11.05), 14, 16, 10000) == ''
    assert started.wait(2)
    assert pre.progress()['phase'] == 'running'
    assert pre.cancel_and_wait(5.0)                               # stopped in time before closing the DB
    assert pre.progress()['phase'] == 'idle'
    ts.stop()
    assert ts.layers['sat'].cache.has(14, 0, 0) is False          # DB closed: no exception


def test_single_flight_one_request_per_tile(tmp_path):
    from ground_station_ui.core.maps.tile_server import TileServer
    ts = TileServer([{'id': 'sat', 'url': 'http://127.0.0.1:9/{z}/{x}/{y}'}], tmp_path)
    calls = []

    def dl(layer, z, x, y):
        calls.append((z, x, y))
        time.sleep(0.2)
        return b'tile'
    ts._download = dl
    out = []
    th = [threading.Thread(target=lambda: out.append(ts.fetch_upstream(ts.layers['sat'], 5, 1, 2)))
          for _ in range(4)]
    [t.start() for t in th]
    [t.join() for t in th]
    assert calls == [(5, 1, 2)] and out == [b'tile'] * 4
    ts.stop()


def test_config_validation_reports_specific_errors():
    bad = {
        'links': [{'name': 'px4', 'type': 'serial'}, {'name': 'px4', 'type': 'tcp'},
                  {'name': 'sitl', 'type': 'udp', 'listen': '14550'}],
        'vehicle': {'autopilot_links': ['px4', 'radio2']},
        'commands': {'link': 'jetson'},
        'map': {'layers': [{'id': 'h', 'compose': ['sat']}, {'title': 'no id'}]},
        'ui_rate_hz': 0,
    }
    errors, warnings = validate(bad)
    text = '\n'.join(errors)
    for needle in ('duplicate link name', 'type "tcp"', 'host:port', 'radio2', 'commands.link',
                   'unknown layer "sat"', 'missing "id"', 'ui_rate_hz'):
        assert needle in text, needle
    assert any('companion_link' in w for w in warnings)
