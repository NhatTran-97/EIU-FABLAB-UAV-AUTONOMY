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
    assert a.tick() == []                                   # lan dau: chi ghi nho, khong doc
    v.groups['status'].values['mode'] = 'Position'
    assert a.tick() == ['Position mode']
    v.groups['status'].values['armed'] = True
    assert a.tick() == ['Armed']
    v.groups['battery'].values['percent'] = 29
    assert a.tick() == ['Battery 29 percent']
    v.groups['battery'].values['percent'] = 28
    assert a.tick() == []                                   # cung muc 30 -> khong lap
    v.groups['battery'].values['percent'] = 19
    assert a.tick() == ['Battery 19 percent']
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
