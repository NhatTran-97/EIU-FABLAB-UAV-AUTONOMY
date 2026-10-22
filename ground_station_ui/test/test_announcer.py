from ground_station_ui.core.alerts.announcer import Announcer


class G:
    def __init__(self, **v):
        self.values = v


class V:
    def __init__(self):
        self.groups = {'status': G(linkAlive=True, mode='Stabilized', armed=False),
                       'companion': G(linkAlive=True), 'battery': G(percent=80)}


def test_mode_arm_link_battery():
    v = V()
    a = Announcer(v)
    assert a.tick() == []                                   # first sample: remembered, not announced
    v.groups['status'].values['mode'] = 'Position'
    assert a.tick() == ['Position mode']
    v.groups['status'].values['armed'] = True
    assert a.tick() == ['Armed']
    v.groups['battery'].values['percent'] = 29
    assert a.tick() == ['Battery 29 percent']
    v.groups['battery'].values['percent'] = 28
    assert a.tick() == []                                   # same 30 % level -> not repeated
    v.groups['battery'].values['percent'] = 19
    assert a.tick() == ['Battery low, 19 percent']
    v.groups['companion'].values['linkAlive'] = False
    assert a.tick() == ['Jetson connection lost']
    v.groups['status'].values['linkAlive'] = False
    assert a.tick() == ['PX4 connection lost']
    v.groups['status'].values.update(linkAlive=True, mode='Hold')
    assert a.tick() == ['PX4 connected', 'Hold mode']


def test_english():
    v = V()
    a = Announcer(v)
    a.tick()
    v.groups['status'].values['mode'] = 'Land'
    assert a.tick() == ['Land mode']


class Up(G):
    revision = 0


def test_command_results_upload_mission_and_low_battery():
    v = V()
    v.groups['missionProgress'] = G(valid=False, state='idle')
    sender, upload = G(label='', state='idle'), Up(state='idle', total=0)
    a = Announcer(v, [sender], upload)
    assert a.tick() == []
    sender.values.update(label='Start mission 2 m/s + avoidance', state='sending')
    assert a.tick() == []                                   # not spoken while sending
    sender.values['state'] = 'accepted'
    assert a.tick() == ['Start mission accepted']
    sender.values.update(label='RTL', state='timeout')
    assert a.tick() == ['Return to launch: no response']
    sender.values.update(label='Land', state='accepted')
    assert a.tick() == ['Land accepted']
    upload.values.update(state='done', total=7); upload.revision = 1
    assert a.tick() == ['7 waypoints sent']
    v.groups['missionProgress'].values.update(valid=True, state='active', seq=1, total=7)
    assert a.tick() == ['Mission started, flying to waypoint 1 of 7']
    v.groups['missionProgress'].values['state'] = 'paused'
    assert a.tick() == ['Mission paused']
    v.groups['missionProgress'].values['state'] = 'active'
    assert a.tick() == ['Mission resumed']
    v.groups['missionProgress'].values['state'] = 'complete'
    assert a.tick() == ['Mission complete. Hovering at the last waypoint']
    v.groups['status'].values['armed'] = True
    a.tick()
    v.groups['battery'].values['percent'] = 14
    assert a.tick() == ['Battery critical, 14 percent. Land now']
