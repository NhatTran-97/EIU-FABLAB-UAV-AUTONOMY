"""Waypoint tasks, mission options and mission progress (pure Python)."""

import math

from ground_station_ui.core.commands.flight import jetson_steps
from ground_station_ui.core.mavlink import drone_dialect as d
from ground_station_ui.core.mission.mission import Mission
from ground_station_ui.core.mission.uploader import CMD_PICK, to_items
from ground_station_ui.core.router import MessageRouter
from ground_station_ui.core.vehicle.vehicle import Vehicle

PX4 = d.MAVLink(None, srcSystem=1, srcComponent=1)
JETSON = d.MAVLink(None, srcSystem=1, srcComponent=191)


def roundtrip(mav, msg):
    return d.MAVLink(None).parse_buffer(msg.pack(mav))[0]


def test_waypoint_tasks_are_encoded_as_mission_items():
    m = Mission(default_altitude=8.0)
    m.add(11.0, 106.0)
    m.add(11.001, 106.0)
    m.add(11.002, 106.0)
    m.set_task(1, 'pick_auto')
    m.set_task(2, 'pick_manual')
    m.set_task_heights(2, 4.0, 1.5)
    m.set_task(0, 'unknown')                                          # ignored
    assert m.task_count() == 2
    items = to_items(m.items, 1, 191)
    assert items[0].command == d.MAV_CMD_NAV_WAYPOINT
    assert items[1].command == CMD_PICK and (items[1].param1, items[1].param2, items[1].param3) == (0, 3.0, 1.0)
    assert items[2].command == CMD_PICK and (items[2].param1, items[2].param2, items[2].param3) == (1, 4.0, 1.5)
    assert all(it.frame == d.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT for it in items)


def test_start_mission_options_and_continue_commands():
    (cmd, params), = jetson_steps('mission', math.nan)
    assert cmd == d.MAV_CMD_MISSION_START and params[2] == 0
    (cmd, params), = jetson_steps('mission', math.nan, {'avoidance': True})
    assert params[2] == 1
    (cmd, params), = jetson_steps('continue', math.nan)
    assert cmd == d.MAV_CMD_DO_PAUSE_CONTINUE and params[0] == 1


def test_mission_progress_from_the_companion_only():
    router = MessageRouter()
    v = Vehicle(router)
    g = v.groups['missionProgress']
    hb = roundtrip(PX4, PX4.heartbeat_encode(d.MAV_TYPE_QUADROTOR, d.MAV_AUTOPILOT_PX4,
                                             d.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, 3 << 16, d.MAV_STATE_STANDBY))
    router.dispatch('px4', [hb])
    router.dispatch('px4', [roundtrip(PX4, PX4.mission_current_encode(3, 9, 3, 0))])   # PX4's own mission
    assert not g.values['valid']
    router.dispatch('jetson', [roundtrip(JETSON, JETSON.mission_current_encode(2, 3, 4, 0)),
                               roundtrip(JETSON, JETSON.statustext_encode(4, b'WP2 pick: take control, then Continue'))])
    assert g.values['valid'] and g.values['state'] == 'paused'
    assert (g.values['seq'], g.values['total']) == (2, 3) and 'take control' in g.values['text']


def test_mission_speed_is_sent_with_start_mission():
    m = Mission(default_altitude=8.0, speed=2.0)
    m.set_speed(3.25)
    assert m.values['speed'] == 3.2
    m.set_speed(0)                                                    # ignored
    assert m.values['speed'] == 3.2
    (cmd, params), = jetson_steps('mission', math.nan, {'speed': m.values['speed']})
    assert cmd == d.MAV_CMD_MISSION_START and params[3] == 3.2


def test_planned_path_is_anchored_on_the_drone():
    from ground_station_ui.core.geometry.planned_path import planned_path_enu
    lat0, lon0 = 11.0529, 106.6661
    m_lat = 1 / 111195.0                                              # degrees per metre north
    m_lon = m_lat / math.cos(math.radians(lat0))
    anchor = (lat0, lon0, 10.0, 5.0, -30.0, 10.2)                     # drone: 10 m above Home, ENU (5, -30, 10.2)
    wps = [{'lat': lat0 + 20 * m_lat, 'lon': lon0, 'alt': 8.0},
           {'lat': lat0, 'lon': lon0 + 15 * m_lon, 'alt': 12.0, 'task': 'pick_manual'}]
    points, markers = planned_path_enu(wps, (lat0, lon0 - 5 * m_lon), anchor)
    assert len(points) == 3 and len(markers) == 2
    assert points[0] == [0.0, -30.0, 0.2]                             # Home on the ground (u_home = 10.2 - 10)
    assert abs(points[1][0] - 5.0) < 0.05 and abs(points[1][1] + 10.0) < 0.05 and points[1][2] == 8.2
    assert abs(points[2][0] - 20.0) < 0.05 and abs(points[2][1] + 30.0) < 0.05 and points[2][2] == 12.2
    assert markers[1]['task'] == 'pick_manual' and markers[0]['seq'] == 1


def test_uploader_keeps_the_last_successful_upload():
    from ground_station_ui.core.commands.sender import CommandTarget
    from ground_station_ui.core.mission.uploader import MissionUploader

    class Link:
        def send(self, m):
            return True

    class Links:
        def get(self, name):
            return Link()

    up = MissionUploader(Links())
    target = CommandTarget('jetson', 1, 191, 1.5, 3, name='Jetson')
    wps = [{'lat': 11.0, 'lon': 106.0, 'alt': 8.0}]
    up.upload(target, wps)
    assert up.uploaded == [] and up.revision == 0
    ack = roundtrip(JETSON, JETSON.mission_ack_encode(255, 190, 0, 0))
    up.on_ack(ack, 'jetson')
    assert up.uploaded == wps and up.revision == 1


def test_arrival_tolerances_are_sent_with_start_mission():
    m = Mission(default_altitude=8.0, xy_tolerance=0.75, z_tolerance=0.5)
    m.set_tolerances(0.4, 0)                                          # z unchanged
    assert (m.values['xyTolerance'], m.values['zTolerance']) == (0.4, 0.5)
    (cmd, params), = jetson_steps('mission', math.nan, {'speed': 2.0, 'xy_tolerance': 0.4, 'z_tolerance': 0.5})
    assert params[3:6] == [2.0, 0.4, 0.5]


def test_option_changes_notify_the_live_settings_hook():
    m = Mission(default_altitude=8.0)
    calls = []
    m.on_options_changed = lambda: calls.append(1)
    m.set_speed(1.5)
    m.set_speed(0)                                                    # ignored, no notification
    m.set_tolerances(0.5, 0.3)
    assert len(calls) == 2
