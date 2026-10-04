from ground_station_ui.core.commands.sender import CommandSender, CommandTarget
from ground_station_ui.core.mavlink import drone_dialect as mavlink


class FakeLink:
    def __init__(self, ok=True):
        self.ok = ok
        self.sent = []

    def send(self, msg, force_v1=False):
        self.sent.append(msg)
        return self.ok


class FakeLinks:
    def __init__(self, **links):
        self.links = links

    def get(self, name):
        return self.links.get(name)


def ack(command, result, sysid=1, compid=191):
    m = mavlink.MAVLink_command_ack_message(command, result)
    m._header = mavlink.MAVLink_header(m.id, srcSystem=sysid, srcComponent=compid)
    return m


TARGET = CommandTarget('jetson', 1, 191, ack_timeout_s=1.0, retries=3)


def test_takeoff_accepted():
    link = FakeLink()
    s = CommandSender(FakeLinks(jetson=link))
    s.send(TARGET, mavlink.MAV_CMD_NAV_TAKEOFF, (0, 0, 0, 0, 0, 0, 12.5), label='Take-off')
    m = link.sent[0]
    assert (m.target_system, m.target_component, m.command, m.param7, m.confirmation) == (1, 191, 22, 12.5, 0)
    assert s.values['busy'] and s.values['state'] == 'sending'

    s.on_ack(ack(22, mavlink.MAV_RESULT_ACCEPTED, compid=1), 'jetson')    # PX4 (compid 1): bo qua
    s.on_ack(ack(21, mavlink.MAV_RESULT_ACCEPTED), 'jetson')              # lenh khac: bo qua
    s.on_ack(ack(22, mavlink.MAV_RESULT_ACCEPTED), 'px4')                 # link khac: bo qua
    assert s.values['busy']
    s.on_ack(ack(22, mavlink.MAV_RESULT_ACCEPTED), 'jetson')
    assert not s.values['busy'] and s.values['state'] == 'accepted'


def test_rejected():
    link = FakeLink()
    s = CommandSender(FakeLinks(jetson=link))
    s.send(TARGET, mavlink.MAV_CMD_NAV_LAND)
    s.on_ack(ack(21, mavlink.MAV_RESULT_DENIED), 'jetson')
    assert s.values['state'] == 'denied'


def test_retries_then_timeout(monkeypatch):
    now = [100.0]
    monkeypatch.setattr('ground_station_ui.core.commands.sender.time.monotonic', lambda: now[0])
    link = FakeLink()
    s = CommandSender(FakeLinks(jetson=link))
    s.send(TARGET, mavlink.MAV_CMD_NAV_LAND)
    s.tick(100.5)
    assert len(link.sent) == 1                       # chua het han ACK
    s.tick(101.1)
    s.tick(102.2)
    assert [m.confirmation for m in link.sent] == [0, 1, 2]
    assert s.values['attempt'] == 3 and s.values['busy']
    s.tick(103.3)
    assert len(link.sent) == 3 and s.values['state'] == 'timeout' and not s.values['busy']


def test_no_link():
    s = CommandSender(FakeLinks())
    s.send(TARGET, mavlink.MAV_CMD_NAV_LAND)
    assert s.values['state'] == 'no_link' and not s.values['busy']
    s2 = CommandSender(FakeLinks(jetson=FakeLink(ok=False)))   # link chua mo cong
    s2.send(TARGET, mavlink.MAV_CMD_NAV_LAND)
    assert s2.values['state'] == 'no_link'
