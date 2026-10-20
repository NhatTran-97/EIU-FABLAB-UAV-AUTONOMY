"""Lenh bay 2 duong (o PX4 -> thang PX4 qua 915 / 3 nut duoi -> Jetson 433): khoa cheo, dieu kien
tung nut, lenh nhieu buoc."""

import math

from ground_station_ui.core.commands.flight import (CommandLock, FlightCommander, check_state,
                                                    px4_steps)
from ground_station_ui.core.commands.sender import CommandSender, CommandTarget
from ground_station_ui.core.mavlink import drone_dialect as mavlink
from ground_station_ui.core.vehicle.groups import LINK_TIMEOUT_S

JETSON = CommandTarget('jetson', 1, 191, ack_timeout_s=1.0, retries=3, name='Jetson · 433')


class FakeLink:
    def __init__(self):
        self.sent = []

    def send(self, msg, force_v1=False):
        self.sent.append(msg)
        return True


class FakeLinks:
    def __init__(self, **links):
        self.links = links

    def get(self, name):
        return self.links.get(name)


class Group:
    def __init__(self, items=(), **values):
        self.values = values
        self.items = list(items)


class FakeVehicle:
    def __init__(self, px4_alive=True, jetson_alive=True, landed='ON GROUND', armed=False,
                 fix=3, alt_global=None, dds='ok'):
        self.sysid, self.compid, self.link = (1, 1, 'px4') if px4_alive else (None, None, None)
        self.companion_link = 'jetson'
        comps = [{'id': mavlink.DRONE_COMPONENT_PX4, 'level': dds}] if dds else []
        self.groups = {'status': Group(linkAlive=px4_alive, landed=landed, armed=armed),
                       'companion': Group(comps, linkAlive=jetson_alive),
                       'gps': Group(fixType=fix),
                       'flight': Group(altGlobal=alt_global)}


def ack(command, result, sysid=1, compid=1):
    m = mavlink.MAVLink_command_ack_message(command, result)
    m._header = mavlink.MAVLink_header(m.id, srcSystem=sysid, srcComponent=compid)
    return m


def make(env='outdoor', **kw):
    px4, jet = FakeLink(), FakeLink()
    links = FakeLinks(px4=px4, jetson=jet)
    senders = {'px4': CommandSender(links), 'jetson': CommandSender(links)}
    fc = FlightCommander(senders, FakeVehicle(**kw), JETSON, lambda: env)
    return fc, senders, px4, jet


def test_check_state():
    assert check_state('takeoff', 'ON GROUND') == ''
    assert check_state('takeoff', 'IN AIR') == 'already flying'
    assert check_state('land', 'ON GROUND') == 'vehicle is on the ground'
    assert check_state('hold', 'IN AIR') == ''
    assert check_state('land', '—') == ''                    # chua biet -> de PX4 / Jetson quyet


def test_px4_takeoff_steps():
    steps = px4_steps('takeoff', 5.0, armed=False, amsl=None)
    assert [c for c, _ in steps] == [mavlink.MAV_CMD_COMPONENT_ARM_DISARM, mavlink.MAV_CMD_NAV_TAKEOFF]
    assert math.isnan(steps[1][1][6])                         # khong GPS -> NaN = MIS_TAKEOFF_ALT
    steps = px4_steps('takeoff', 5.0, armed=True, amsl=12.0)
    assert [c for c, _ in steps] == [mavlink.MAV_CMD_NAV_TAKEOFF] and steps[0][1][6] == 17.0  # AMSL


def test_direct_takeoff_arm_then_takeoff():
    fc, s, px4, jet = make(alt_global=20.0)
    assert fc.execute('px4', 'takeoff', 5.0)
    assert jet.sent == [] and len(px4.sent) == 1
    m = px4.sent[0]
    assert (m.target_system, m.target_component, m.command, m.param1) == (1, 1, 400, 1.0)
    d = s['px4'].values
    assert d['route'] == 'PX4 · 915' and (d['step'], d['steps']) == (1, 2)

    s['px4'].on_ack(ack(400, mavlink.MAV_RESULT_ACCEPTED), 'px4')    # ARM xong -> NAV_TAKEOFF
    m = px4.sent[-1]
    assert (m.command, m.param7, m.confirmation) == (22, 25.0, 0)
    s['px4'].on_ack(ack(22, mavlink.MAV_RESULT_ACCEPTED), 'px4')
    assert s['px4'].values['state'] == 'accepted'
    assert s['jetson'].values['state'] == 'idle'               # 2 sender rieng


def test_direct_arm_denied_stops_sequence():
    fc, s, px4, _ = make()
    fc.execute('px4', 'takeoff', 1.5)
    s['px4'].on_ack(ack(400, mavlink.MAV_RESULT_DENIED), 'px4')   # pre-arm check fail
    assert s['px4'].values['state'] == 'denied' and len(px4.sent) == 1


def test_jetson_buttons_send_to_jetson():
    fc, s, px4, jet = make()
    assert fc.execute('jetson', 'takeoff', 1.5)
    m = jet.sent[0]
    assert px4.sent == [] and (m.target_component, m.command, m.param7) == (191, 22, 1.5)


def test_jetson_gates():
    fc, *_ = make(jetson_alive=False)
    assert fc.blocker('jetson', 'takeoff') == 'Jetson not connected (radio 433)'
    assert fc.blocker('px4', 'takeoff') == ''                 # duong PX4 khong phu thuoc Jetson
    fc, *_ = make(dds=None)
    assert fc.blocker('jetson', 'land') == 'Jetson has no PX4 data (DDS)'
    assert fc.blocker('jetson', 'upload') == ''               # Send WP khong can DDS
    fc, *_ = make(dds='stale')
    assert fc.blocker('jetson', 'takeoff') == 'Jetson has no PX4 data (DDS)'
    # GPS: ngoai troi can fix de bat dau (Take-off / Send WP); Land / Hold khong bao gio bi chan vi GPS
    fc, *_ = make(fix=0)
    g = fc.gates()['jetson']
    assert g['takeoff'] == g['upload'] == 'no GPS fix' and g['rtl'] == 'RTL needs GPS fix'
    assert g['land'] == g['hold'] == ''
    fc, *_ = make(fix=0, env='indoor')                        # trong nha: khong can GPS (tru RTL)
    g = fc.gates()['jetson']
    assert g['takeoff'] == g['upload'] == '' and g['rtl'] == 'RTL needs GPS fix'


def test_cross_lock_after_takeoff():
    fc, s, px4, jet = make()
    fc.execute('jetson', 'takeoff', 5.0)
    assert fc.lock.owner == 'jetson'
    assert fc.blocker('px4', 'land') == 'flight controlled from Jetson · 433'
    assert not fc.execute('px4', 'land')                      # bi chan, khong gui
    assert px4.sent == [] and s['px4'].values['state'] == 'blocked'
    assert fc.blocker('jetson', 'land') == ''                 # duong giu quyen van dung duoc

    # duong giu quyen mat ket noi -> duong kia duoc dung (khong de drone khong ai dieu khien)
    fc.vehicle.groups['companion'].values['linkAlive'] = False
    assert fc.blocker('px4', 'land') == ''


def test_lock_release_after_flight():
    lock = CommandLock()
    lock.claim('px4')
    lock.update(0.0, armed=False, landed='ON GROUND', owner_state='sending')
    assert lock.owner == 'px4'                                 # dang gui
    lock.update(1.0, armed=True, landed='TAKING OFF', owner_state='accepted')
    lock.update(30.0, armed=True, landed='IN AIR', owner_state='accepted')
    assert lock.owner == 'px4'                                 # dang bay: giu quyen
    lock.update(60.0, armed=False, landed='ON GROUND', owner_state='accepted')
    assert lock.owner is None                                  # ve dat + disarm: nha


def test_lock_release_when_command_fails_or_never_arms():
    lock = CommandLock()
    lock.claim('jetson')
    lock.update(0.0, armed=False, landed='ON GROUND', owner_state='denied')
    assert lock.owner is None                                  # bi tu choi: chua bay
    lock.claim('px4')
    lock.update(0.0, armed=False, landed='ON GROUND', owner_state='accepted')
    assert lock.owner == 'px4'
    lock.update(LINK_TIMEOUT_S + 0.5, armed=False, landed='ON GROUND', owner_state='accepted')
    assert lock.owner is None                                  # chap nhan nhung khong arm


def test_state_check_blocks_both_routes():
    fc, s, px4, jet = make(landed='IN AIR', armed=True)
    assert not fc.execute('px4', 'takeoff', 5.0)
    assert s['px4'].values['message'] == 'already flying'
    fc, s, px4, jet = make(landed='ON GROUND')
    assert not fc.execute('jetson', 'land')
    assert s['jetson'].values['message'] == 'vehicle is on the ground' and jet.sent == []


def test_test_mode_bypasses_gcs_gates_and_ping():
    """Nut 🧪: trong phong thi nghiem (khong DDS, khong GPS) van gui duoc toi Jetson."""
    fc, s, px4, jet = make(dds=None, fix=0, landed='—')
    assert fc.blocker('jetson', 'takeoff')                     # nut thuong bi khoa
    assert fc.execute('jetson', 'takeoff', 1.5, test=True)
    m = jet.sent[-1]
    assert (m.command, m.param7) == (22, 1.5) and s['jetson'].values['label'] == 'Take-off 1.5 m [TEST]'
    fc.ping()
    assert jet.sent[-1].command == mavlink.MAV_CMD_USER_1 and px4.sent == []
    s['jetson'].on_ack(ack(mavlink.MAV_CMD_USER_1, mavlink.MAV_RESULT_ACCEPTED, compid=191), 'jetson')
    assert s['jetson'].values['state'] == 'accepted'


def test_start_mission_gates_and_command():
    fc, s, px4, jet = make(landed='ON GROUND')
    assert not fc.execute('jetson', 'mission')                 # o dat
    assert s['jetson'].values['message'] == 'vehicle is on the ground'
    fc, s, px4, jet = make(landed='IN AIR', armed=True, fix=0)
    assert fc.blocker('jetson', 'mission') == 'mission needs GPS fix'   # waypoint la toa do GPS
    fc, s, px4, jet = make(landed='IN AIR', armed=True, fix=3)
    assert fc.execute('jetson', 'mission')
    m = jet.sent[-1]
    assert (m.target_component, m.command) == (191, mavlink.MAV_CMD_MISSION_START) and px4.sent == []
    assert s['jetson'].values['label'] == 'Start mission'
