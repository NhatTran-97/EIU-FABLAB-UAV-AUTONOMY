"""Session file: waypoints, uploaded waypoints and the flown path survive a restart of the app."""

import time

from ground_station_ui.core.mavlink import drone_dialect as d
from ground_station_ui.core.mission.mission import Mission
from ground_station_ui.core.mission.uploader import MissionUploader
from ground_station_ui.core.router import MessageRouter
from ground_station_ui.core.session import SessionStore
from ground_station_ui.core.vehicle.vehicle import Vehicle


def make(path):
    vehicle = Vehicle(MessageRouter())
    mission, uploader = Mission(10.0), MissionUploader(links=None)
    return SessionStore(path, mission, uploader, vehicle.groups['track']), mission, uploader, vehicle.groups['track']


def test_waypoints_options_upload_and_track_are_restored(tmp_path):
    path = tmp_path / 'session.json'
    store, mission, uploader, track = make(path)
    mission.add(11.0, 106.0)
    mission.add(11.001, 106.0, 8.0)
    mission.set_task(1, 'pick_manual')
    mission.set_speed(3.0)
    mission.set_tolerances(0.4, 0.3)
    uploader.uploaded, uploader.revision = [dict(w) for w in mission.items], 2
    track.path.start_segment()
    for i in range(5):
        track.path.add((11.0 + i * 1e-4, 106.0))
    track.set(lengthM=44.0, points=5)
    store.save_if_changed(10.0)

    store2, mission2, uploader2, track2 = make(path)
    assert store2.load()
    assert [(w['lat'], w['alt'], w['task']) for w in mission2.items] == [(11.0, 10.0, 'none'), (11.001, 8.0, 'pick_manual')]
    assert (mission2.values['speed'], mission2.values['xyTolerance'], mission2.values['zTolerance']) == (3.0, 0.4, 0.3)
    assert len(uploader2.uploaded) == 2 and uploader2.revision == 2
    assert track2.path.count == 5 and track2.values['lengthM'] == 44.0
    assert track2.path.take()[0] == 'reset'                       # the map reloads the restored path


def test_missing_or_broken_file_starts_empty(tmp_path):
    store, mission, _, _ = make(tmp_path / 'none.json')
    assert not store.load() and mission.items == []
    bad = tmp_path / 'bad.json'
    bad.write_text('{not json')
    store, mission, _, _ = make(bad)
    assert not store.load() and mission.items == []


def test_flight_in_progress_is_continued_after_a_restart(tmp_path):
    store, _, _, track = make(tmp_path / 's.json')
    now = time.time()
    track.restore({'segments': [[[11.0, 106.0], [11.0001, 106.0]]], 'lengthM': 11.0, 'points': 2,
                   'flight': [[11.0, 106.0, 20.0, now - 5], [11.0001, 106.0, 20.0, now - 1]]})
    track._armed = False
    track.vehicle.accept_global = lambda m, link: True
    track.vehicle.is_autopilot = lambda m, link: True
    track.vehicle.groups['status'].values.update(armed=True)
    mav = d.MAVLink(None, srcSystem=1, srcComponent=1)
    m = d.MAVLink(None).parse_buffer(mav.global_position_int_encode(0, 110002000, 1060000000, 20000, 5000,
                                                                   0, 0, 0, 0).pack(mav))[0]
    track.on_global(m, 'px4')
    assert len(track._flight) == 3 and len(track.path.segments) == 1      # same flight, same segment


def test_last_command_results_and_messages_are_restored(tmp_path):
    from ground_station_ui.core.commands.sender import CommandSender
    path = tmp_path / 's.json'

    def build():
        vehicle = Vehicle(MessageRouter())
        mission, uploader, sender = Mission(10.0), MissionUploader(links=None), CommandSender(links=None)
        store = SessionStore(path, mission, uploader, vehicle.groups['track'],
                             status_groups={'commands': sender, 'uploader': uploader},
                             messages=vehicle.groups['messages'])
        return store, sender, uploader, vehicle.groups['messages']

    store, sender, uploader, messages = build()
    sender.set(label='Continue', state='accepted', route='Jetson', elapsed=0.2)
    uploader.set(state='done', sent=7, total=7)
    messages.set_items([{'time': '09:10:00', 'severity': 'INFO', 'level': 6, 'text': 'Resumed'}])
    store.save()
    store2, sender2, uploader2, messages2 = build()
    store2.load()
    assert (sender2.values['label'], sender2.values['state'], sender2.values['busy']) == ('Continue', 'accepted', False)
    assert uploader2.values['state'] == 'done' and uploader2.values['total'] == 7
    assert messages2.items[0]['text'] == 'Resumed'
    # a command still running when the app closed is not shown as running after a restart
    sender2.set(label='Land', state='in_progress', busy=True)
    store2.save()
    store3, sender3, _, _ = build()
    store3.load()
    assert sender3.values['state'] == 'idle' and not sender3.values['busy']


def test_3d_trail_restored_and_dropped_after_an_ekf_reset(tmp_path):
    path = tmp_path / 's.json'

    def build():
        vehicle = Vehicle(MessageRouter())
        store = SessionStore(path, Mission(10.0), MissionUploader(links=None), vehicle.groups['track'],
                             pose=vehicle.groups['localPose'])
        return store, vehicle.groups['localPose']

    store, pose = build()
    for i in range(4):
        pose.trail.add((float(i), 0.0, 5.0))
    pose._reset = 3
    pose.set(extentM=4.0)
    store.save()
    store2, pose2 = build()
    store2.load()
    assert pose2.trail.count == 4 and pose2.values['trailPoints'] == 4 and pose2._reset == 3
    pose2.vehicle.is_companion = lambda m, link: True

    class Pose:                                   # first DRONE_LOCAL_POSE after the restart, EKF was reset
        reset_counter, x, y, z, q, vx, vy, vz = 4, 0.0, 0.0, -5.0, (1, 0, 0, 0), 0.0, 0.0, 0.0
    pose2.on_pose(Pose(), 'jetson')
    assert pose2.trail.count == 1                 # old trail dropped, new one started


def test_recording_closes_when_positions_stop_and_far_resume_is_a_new_flight(tmp_path):
    _, _, _, track = make(tmp_path / 's.json')
    track._armed, track._last_rx = True, 100.0
    track._flight = [[11.0, 106.0, 20.0, time.time()], [11.0001, 106.0, 20.0, time.time()]]
    track.set(recording=True)
    track.tick(103.0)
    assert track.values['recording']                                   # short gap: still recording
    track.tick(106.0)
    assert not track.values['recording'] and not track._armed           # positions stopped: flight closed
    # restored flight but the drone is now 1 km away (SITL restart, other site): not continued
    now = time.time()
    track.restore({'segments': [], 'flight': [[11.0, 106.0, 20.0, now - 5]]})
    track.vehicle.accept_global = lambda m, link: True
    track.vehicle.is_autopilot = lambda m, link: True
    track.vehicle.groups['status'].values.update(armed=True)
    mav = d.MAVLink(None, srcSystem=1, srcComponent=1)
    m = d.MAVLink(None).parse_buffer(mav.global_position_int_encode(0, 110090000, 1060000000, 20000, 5000,
                                                                   0, 0, 0, 0).pack(mav))[0]
    track.on_global(m, 'px4')
    assert len(track._flight) == 1                                      # new flight from the current point


def test_clear_flown_path_removes_map_track_and_3d_trail_and_is_saved(tmp_path):
    from ground_station_ui.qt.adapters import VehicleAdapter
    path = tmp_path / 's.json'
    vehicle = Vehicle(MessageRouter())
    store = SessionStore(path, Mission(10.0), MissionUploader(links=None), vehicle.groups['track'],
                         pose=vehicle.groups['localPose'])
    track, pose = vehicle.groups['track'], vehicle.groups['localPose']
    track.path.start_segment()
    for i in range(3):
        track.path.add((11.0 + i * 1e-4, 106.0))
        pose.trail.add((float(i), 0.0, 5.0))
    track.set(points=3, lengthM=22.0)
    VehicleAdapter(vehicle).clearFlownPath()
    assert track.path.count == 0 and track.values['lengthM'] == 0 and pose.trail.count == 0
    store.save()
    vehicle2 = Vehicle(MessageRouter())
    SessionStore(path, Mission(10.0), MissionUploader(links=None), vehicle2.groups['track'],
                 pose=vehicle2.groups['localPose']).load()
    assert vehicle2.groups['track'].path.count == 0 and vehicle2.groups['localPose'].trail.count == 0
