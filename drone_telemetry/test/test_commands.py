import math

from drone_telemetry import drone_dialect as mav
from drone_telemetry.commands import (ACCEPTED, CMD_HOLD, CMD_LAND, CMD_RTL, CMD_TAKEOFF, DENIED, FAILED,
                                      IN_PROGRESS, PX4_ARM, PX4_SET_MODE, TEMP_REJECTED, CommandBridge,
                                      MissionReceiver)


class Rec:
    def __init__(self):
        self.px4, self.acks, self.texts = [], [], []

    def bridge(self):
        return CommandBridge(lambda c, p: self.px4.append((c, p)),
                             lambda c, r: self.acks.append((c, r)),
                             lambda t, s: self.texts.append(t))


def test_takeoff_arms_then_takes_off_to_amsl_plus_rel():
    r = Rec(); b = r.bridge()
    b.update_px4(0.0, landed=True, armed=False, amsl=48.0, amsl_valid=True)
    b.handle(CMD_TAKEOFF, [0, 0, 0, 0, 0, 0, 1.5], 0.1)
    assert r.acks == [(CMD_TAKEOFF, IN_PROGRESS)] and r.px4[0][0] == PX4_ARM
    b.on_px4_ack(PX4_ARM, ACCEPTED, 0.3)
    assert r.px4[1][0] == CMD_TAKEOFF and r.px4[1][1][6] == 49.5
    b.on_px4_ack(CMD_TAKEOFF, ACCEPTED, 0.5)
    assert r.acks[-1] == (CMD_TAKEOFF, ACCEPTED) and not b.busy


def test_takeoff_indoor_uses_mis_takeoff_alt_and_skips_arm_if_armed():
    r = Rec(); b = r.bridge()
    b.update_px4(0.0, landed=True, armed=True, amsl=None, amsl_valid=False)
    b.handle(CMD_TAKEOFF, [0, 0, 0, 0, 0, 0, 1.5], 0.1)
    assert r.px4[0][0] == CMD_TAKEOFF and math.isnan(r.px4[0][1][6])
    assert any('MIS_TAKEOFF_ALT' in t for t in r.texts)


def test_safety_rules():
    r = Rec(); b = r.bridge()
    b.handle(CMD_LAND, [], 0.0)
    assert r.acks[-1] == (CMD_LAND, TEMP_REJECTED)                 # chua co du lieu PX4
    b.update_px4(1.0, landed=False, armed=True)
    b.handle(CMD_TAKEOFF, [0] * 7, 1.1)
    assert r.acks[-1] == (CMD_TAKEOFF, DENIED)                     # dang bay
    b.update_px4(2.0, landed=True, armed=False)
    for c in (CMD_LAND, CMD_HOLD, CMD_RTL):
        b.handle(c, [], 2.1)
        assert r.acks[-1] == (c, DENIED)                           # o mat dat
    b.handle(CMD_LAND, [], 9.0)
    assert r.acks[-1] == (CMD_LAND, TEMP_REJECTED)                 # du lieu PX4 cu
    assert r.px4 == []


def test_hold_maps_to_set_mode_and_retry_dedup_keepalive_timeout():
    r = Rec(); b = r.bridge()
    b.update_px4(0.0, landed=False, armed=True)
    b.handle(CMD_HOLD, [], 0.1)
    assert r.px4 == [(PX4_SET_MODE, [1.0, 4.0, 3.0, 0, 0, 0, 0])]
    b.handle(CMD_HOLD, [], 0.2)                                     # ground gui lai
    assert len(r.px4) == 1 and r.acks[-1] == (CMD_HOLD, IN_PROGRESS)
    b.tick(0.7)
    assert r.acks[-1] == (CMD_HOLD, IN_PROGRESS)                    # keepalive
    b.tick(3.2)
    assert len(r.px4) == 2                                          # gui lai PX4 1 lan
    b.tick(6.3)
    assert r.acks[-1] == (CMD_HOLD, FAILED) and not b.busy


def test_px4_rejection_and_supersede():
    r = Rec(); b = r.bridge()
    b.update_px4(0.0, landed=False, armed=True)
    b.handle(CMD_RTL, [], 0.1)
    b.on_px4_ack(CMD_RTL, DENIED, 0.2)                              # vd khong co GPS
    assert r.acks[-1] == (CMD_RTL, DENIED)
    b.handle(CMD_HOLD, [], 0.3)
    b.handle(CMD_LAND, [], 0.4)                                     # Land thay Hold
    assert r.px4[-1][0] == CMD_LAND
    b.on_px4_ack(PX4_SET_MODE, ACCEPTED, 0.5)                      # ack cua lenh cu: bo qua
    assert b.busy


def test_mission_receiver():
    gcs = mav.MAVLink(None, srcSystem=255, srcComponent=190)
    me = mav.MAVLink(None, srcSystem=1, srcComponent=191)
    sent, got = [], []
    rt = lambda m: mav.MAVLink(None).parse_buffer(m.pack(gcs))[0]
    rx = MissionReceiver(me, sent.append, got.append)
    rx.on_count(rt(gcs.mission_count_encode(1, 191, 2, 0)), 0.0)
    assert sent[-1].get_type() == 'MISSION_REQUEST_INT' and sent[-1].seq == 0 and sent[-1].target_system == 255
    rx.tick(2.0)
    assert len(sent) == 2 and sent[-1].seq == 0                     # mat goi -> xin lai
    for seq, (lat, lon, alt) in enumerate([(11.0, 106.0, 20.0), (11.1, 106.1, 25.0)]):
        rx.on_item(rt(gcs.mission_item_int_encode(1, 191, seq, 6, 16, 0, 1, 0, 0, 0, 0,
                                                  int(lat * 1e7), int(lon * 1e7), alt, 0)), 2.1)
    assert sent[-1].get_type() == 'MISSION_ACK' and sent[-1].type == 0
    assert [round(w['lat'], 1) for w in got[0]] == [11.0, 11.1] and got[0][1]['alt'] == 25.0


def test_lost_final_ack_retry_gets_cached_result_not_rerun():
    """ACK ACCEPTED rot tren radio -> ground gui lai (confirmation>0): tra lai ket qua, KHONG cat canh lan 2."""
    r = Rec(); b = r.bridge()
    b.update_px4(0.0, landed=True, armed=False)
    b.handle(CMD_TAKEOFF, [0] * 7, 0.1)
    b.on_px4_ack(PX4_ARM, ACCEPTED, 0.2)
    b.on_px4_ack(CMD_TAKEOFF, ACCEPTED, 0.3)
    assert r.acks[-1] == (CMD_TAKEOFF, ACCEPTED) and len(r.px4) == 2
    b.update_px4(1.0, landed=True, armed=True)                     # land detector chua kip doi
    b.handle(CMD_TAKEOFF, [0] * 7, 1.8, confirmation=1)            # ban gui lai
    assert r.acks[-1] == (CMD_TAKEOFF, ACCEPTED) and len(r.px4) == 2
    # lan bam moi (confirmation=0) van duoc xet binh thuong
    b.update_px4(20.0, landed=False, armed=True)
    b.handle(CMD_TAKEOFF, [0] * 7, 20.1)
    assert r.acks[-1] == (CMD_TAKEOFF, DENIED)
    # ket qua cu het han -> ban gui lai cung duoc xet lai
    b.update_px4(39.9, landed=False, armed=True)
    b.handle(CMD_TAKEOFF, [0] * 7, 40.0, confirmation=2)
    assert r.acks[-1] == (CMD_TAKEOFF, DENIED) and len(r.px4) == 2


def test_mission_lost_ack_resent_when_last_item_repeated():
    gcs = mav.MAVLink(None, srcSystem=255, srcComponent=190)
    me = mav.MAVLink(None, srcSystem=1, srcComponent=191)
    sent, got = [], []
    rt = lambda m: mav.MAVLink(None).parse_buffer(m.pack(gcs))[0]
    item = lambda seq: rt(gcs.mission_item_int_encode(1, 191, seq, 6, 16, 0, 1, 0, 0, 0, 0,
                                                      110000000, 1060000000, 20.0, 0))
    rx = MissionReceiver(me, sent.append, got.append)
    rx.on_count(rt(gcs.mission_count_encode(1, 191, 2, 0)), 0.0)
    rx.on_item(item(0), 0.1)
    rx.on_item(item(1), 0.2)
    assert sent[-1].get_type() == 'MISSION_ACK' and len(got) == 1
    n = len(sent)
    rx.on_item(item(1), 1.8)                                        # ground khong thay ACK, gui lai
    assert len(sent) == n + 1 and sent[-1].get_type() == 'MISSION_ACK' and len(got) == 1
    rx.on_item(item(0), 1.9)                                        # khong phai diem cuoi: bo qua
    assert len(sent) == n + 1


def test_ping_replies_without_px4():
    """Nut Test tren ground: khong co du lieu PX4 van tra ACK + STATUSTEXT, khong gui gi cho PX4."""
    from drone_telemetry.commands import CMD_PING
    r = Rec(); b = r.bridge()
    b.handle(CMD_PING, [0] * 7, 0.0)
    assert r.acks == [(CMD_PING, ACCEPTED)] and r.px4 == [] and 'ping #1' in r.texts[-1]
    assert not b.busy


def mission_bridge():
    r = Rec()
    r.mission = []
    b = CommandBridge(lambda c, p: r.px4.append((c, p)), lambda c, res: r.acks.append((c, res)),
                      lambda t, s: r.texts.append(t), send_mission=r.mission.append)
    return r, b


def test_takeoff_via_mission_manager_waits_for_px4_offboard_and_arm():
    r, b = mission_bridge()
    b.update_px4(0.0, landed=True, armed=False)
    b.handle(CMD_TAKEOFF, [0, 0, 0, 0, 0, 0, 3.0], 0.1)
    assert r.mission == [{'id': 1, 'cmd': 'takeoff', 'alt': 3.0}] and r.px4 == []   # khong gui thang PX4
    assert r.acks == [(CMD_TAKEOFF, IN_PROGRESS)]
    b.on_mission_ack(1, 'accepted', 'take-off to 3.0 m', 0.2)
    assert b.busy
    b.on_px4_ack(PX4_SET_MODE, ACCEPTED, 1.3)                       # offboard_control: OFFBOARD
    assert b.busy
    b.on_px4_ack(PX4_ARM, ACCEPTED, 1.3)                            # ... + ARM
    assert r.acks[-1] == (CMD_TAKEOFF, ACCEPTED) and not b.busy


def test_mission_manager_denies_or_silent_or_px4_rejects():
    r, b = mission_bridge()
    b.update_px4(0.0, landed=True, armed=False)
    b.handle(CMD_TAKEOFF, [0] * 7, 0.1)
    assert r.mission[-1]['alt'] == 0                                # 0 -> mission_manager dung takeoff_altitude
    b.on_mission_ack(1, 'denied', 'no local position yet', 0.2)
    assert r.acks[-1] == (CMD_TAKEOFF, DENIED) and 'no local position' in r.texts[-1]

    b.update_px4(1.0, landed=True, armed=False)
    b.handle(CMD_TAKEOFF, [0] * 7, 1.1)
    b.tick(2.2)
    assert len(r.mission) == 3 and r.mission[-1]['id'] == 2         # gui lai cung id
    b.update_px4(3.0, landed=True, armed=False)
    b.tick(3.3)
    assert r.acks[-1] == (CMD_TAKEOFF, FAILED) and 'mission_manager not responding' in r.texts[-1]

    b.update_px4(4.0, landed=True, armed=False)
    b.handle(CMD_TAKEOFF, [0] * 7, 4.1)
    b.on_mission_ack(3, 'accepted', 'ok', 4.2)
    b.on_px4_ack(PX4_SET_MODE, ACCEPTED, 5.0)
    b.on_px4_ack(PX4_ARM, DENIED, 5.0)                              # pre-arm check fail
    assert r.acks[-1] == (CMD_TAKEOFF, DENIED) and 'arming' in r.texts[-1]


def test_land_via_mission_and_abort_takeoff_on_ground():
    r, b = mission_bridge()
    b.update_px4(0.0, landed=False, armed=True)
    b.handle(CMD_LAND, [0] * 7, 0.1)
    assert r.mission[-1] == {'id': 1, 'cmd': 'land'}
    b.on_mission_ack(1, 'accepted', 'landing', 0.2)
    b.on_px4_ack(CMD_LAND, ACCEPTED, 1.0)                           # offboard_control gui NAV_LAND
    assert r.acks[-1] == (CMD_LAND, ACCEPTED)
    # da ARM (OFFBOARD) nhung chua roi dat -> Land van duoc (huy cat canh)
    b.update_px4(2.0, landed=True, armed=True)
    b.handle(CMD_LAND, [0] * 7, 2.1)
    assert r.mission[-1]['cmd'] == 'land'
    b.on_mission_ack(2, 'accepted', 'landing', 2.2)
    b.on_px4_ack(CMD_LAND, ACCEPTED, 2.5)
    # o dat + disarm -> tu choi nhu cu
    b.update_px4(10.0, landed=True, armed=False)
    b.handle(CMD_LAND, [0] * 7, 10.1)
    assert r.acks[-1] == (CMD_LAND, DENIED)


def test_hold_still_direct_to_px4_in_mission_mode():
    r, b = mission_bridge()
    b.update_px4(0.0, landed=False, armed=True)
    b.handle(CMD_HOLD, [], 0.1)
    assert r.px4[0][0] == PX4_SET_MODE and r.mission == []


def test_land_cancels_takeoff_before_arming():
    """Bam Land ngay sau Take-off (chua ARM, drone van o dat): khong bi chan 'on the ground'."""
    r, b = mission_bridge()
    b.update_px4(0.0, landed=True, armed=False)
    b.handle(CMD_TAKEOFF, [0, 0, 0, 0, 0, 0, 2.0], 0.1)
    b.on_mission_ack(1, 'accepted', 'take-off to 2.0 m', 0.15)
    b.handle(CMD_LAND, [0] * 7, 0.3)
    assert r.mission[-1] == {'id': 2, 'cmd': 'land'}
    b.on_mission_ack(2, 'accepted', 'landing', 0.35)
    assert r.acks[-1] == (CMD_LAND, ACCEPTED) and 'cancelled' in r.texts[-1]


def test_start_mission_via_mission_manager():
    from drone_telemetry.commands import CMD_MISSION_START, UNSUPPORTED
    r, b = mission_bridge()
    b.update_px4(0.0, landed=True, armed=False)
    b.handle(CMD_MISSION_START, [0] * 7, 0.1)
    assert r.acks[-1] == (CMD_MISSION_START, DENIED) and r.mission == []     # dang o dat
    b.update_px4(1.0, landed=False, armed=True)
    b.handle(CMD_MISSION_START, [0] * 7, 1.1)
    assert r.mission[-1] == {'id': 1, 'cmd': 'mission'} and r.px4 == []
    b.on_mission_ack(1, 'accepted', 'mission started: 3 waypoints', 1.2)
    assert r.acks[-1] == (CMD_MISSION_START, ACCEPTED) and '3 waypoints' in r.texts[-1]
    b.handle(CMD_MISSION_START, [0] * 7, 2.0)
    b.on_mission_ack(2, 'denied', 'no waypoints uploaded (Send WP first)', 2.1)
    assert r.acks[-1] == (CMD_MISSION_START, DENIED) and 'Send WP' in r.texts[-1]
    # duong px4 (khong co mission_manager) -> khong ho tro
    r2 = Rec(); b2 = r2.bridge()
    b2.update_px4(0.0, landed=False, armed=True)
    b2.handle(CMD_MISSION_START, [0] * 7, 0.1)
    assert r2.acks[-1] == (CMD_MISSION_START, UNSUPPORTED)
