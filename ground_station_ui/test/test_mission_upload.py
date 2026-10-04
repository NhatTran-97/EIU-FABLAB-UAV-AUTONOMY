from ground_station_ui.core.commands.sender import CommandTarget
from ground_station_ui.core.mavlink import drone_dialect as mavlink
from ground_station_ui.core.mission.uploader import MissionUploader

from test_commands import FakeLink, FakeLinks

TARGET = CommandTarget('jetson', 1, 191)
WPS = [{'lat': 11.052919, 'lon': 106.666057, 'alt': 20}, {'lat': 11.0535, 'lon': 106.667, 'alt': 25.5}]


def from_jetson(m, sysid=1, compid=191):
    m._header = mavlink.MAVLink_header(m.id, srcSystem=sysid, srcComponent=compid)
    return m


def req(seq):
    return from_jetson(mavlink.MAVLink_mission_request_int_message(255, 190, seq))


def test_full_upload_like_jetson():
    link = FakeLink()
    up = MissionUploader(FakeLinks(jetson=link))
    up.upload(TARGET, WPS)
    count = link.sent[0]
    assert count.get_type() == 'MISSION_COUNT' and count.count == 2 and count.target_component == 191
    for seq in (0, 1):
        up.on_request(req(seq), 'jetson')
        item = link.sent[-1]
        assert item.get_type() == 'MISSION_ITEM_INT' and item.seq == seq
    assert (link.sent[1].x, link.sent[1].y, link.sent[1].z) == (110529190, 1066660570, 20.0)
    assert link.sent[2].z == 25.5 and link.sent[1].current == 1 and link.sent[2].current == 0
    assert link.sent[1].frame == mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT
    assert up.values['sent'] == 2 and up.values['busy']
    up.on_ack(from_jetson(mavlink.MAVLink_mission_ack_message(255, 190, mavlink.MAV_MISSION_ACCEPTED)), 'jetson')
    assert up.values['state'] == 'done' and not up.values['busy']


def test_resend_last_on_timeout_then_give_up(monkeypatch):
    now = [10.0]
    monkeypatch.setattr('ground_station_ui.core.mission.uploader.time.monotonic', lambda: now[0])
    link = FakeLink()
    up = MissionUploader(FakeLinks(jetson=link), timeout_s=1.0, retries=3)
    up.upload(TARGET, WPS)
    up.on_request(req(0), 'jetson')
    for t in (11.1, 12.2):              # mat goi -> gui lai ITEM 0
        now[0] = t
        up.tick(t)
    assert [m.get_type() for m in link.sent] == ['MISSION_COUNT'] + ['MISSION_ITEM_INT'] * 3
    now[0] = 13.3
    up.tick(13.3)
    assert up.values['state'] == 'timeout' and not up.values['busy']


def test_rejected_and_foreign_messages_ignored():
    link = FakeLink()
    up = MissionUploader(FakeLinks(jetson=link))
    up.upload(TARGET, WPS)
    up.on_request(from_jetson(mavlink.MAVLink_mission_request_int_message(255, 190, 0), compid=1), 'jetson')  # PX4
    up.on_request(req(0), 'px4')
    assert len(link.sent) == 1
    up.on_ack(from_jetson(mavlink.MAVLink_mission_ack_message(255, 190, mavlink.MAV_MISSION_DENIED)), 'jetson')
    assert up.values['state'] == 'error' and up.values['message'] == 'Denied'


def test_snapshot_not_affected_by_edits():
    link = FakeLink()
    up = MissionUploader(FakeLinks(jetson=link))
    wps = [dict(w) for w in WPS]
    up.upload(TARGET, wps)
    wps[0]['alt'] = 99
    up.on_request(req(0), 'jetson')
    assert link.sent[-1].z == 20.0
