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


def test_in_progress_silence_polls_instead_of_timing_out(monkeypatch):
    """Dang IN_PROGRESS ma rot vai keepalive -> gui lai de hoi (khong bao 'not responding' ngay)."""
    now = [100.0]
    monkeypatch.setattr('ground_station_ui.core.commands.sender.time.monotonic', lambda: now[0])
    link = FakeLink()
    s = CommandSender(FakeLinks(jetson=link))
    s.send(TARGET, mavlink.MAV_CMD_NAV_TAKEOFF)
    s.on_ack(ack(22, mavlink.MAV_RESULT_IN_PROGRESS), 'jetson')
    assert s.values['state'] == 'in_progress'
    for t in (101.1, 102.2):                         # im lang -> hoi lai, tang confirmation
        now[0] = t
        s.tick(t)
    assert [m.confirmation for m in link.sent] == [0, 1, 2] and s.values['busy']
    s.on_ack(ack(22, mavlink.MAV_RESULT_IN_PROGRESS), 'jetson')   # Jetson tra loi -> dem lai
    assert s.values['attempt'] == 0
    for t in (103.3, 104.4):
        now[0] = t
        s.tick(t)
    assert s.values['busy']                           # moi 2 lan im lang lien tiep
    now[0] = 104.9
    s.on_ack(ack(22, mavlink.MAV_RESULT_ACCEPTED), 'jetson')     # ket qua (co the la ban tra lai)
    assert s.values['state'] == 'accepted' and s.values['elapsed'] == 4.9


def test_timeout_only_after_consecutive_silence(monkeypatch):
    now = [0.0]
    monkeypatch.setattr('ground_station_ui.core.commands.sender.time.monotonic', lambda: now[0])
    s = CommandSender(FakeLinks(jetson=FakeLink()))
    s.send(TARGET, mavlink.MAV_CMD_NAV_LAND)
    for t in (1.1, 2.2, 3.3):
        now[0] = t
        s.tick(t)
    assert s.values['state'] == 'timeout' and s.values['elapsed'] == 3.3
