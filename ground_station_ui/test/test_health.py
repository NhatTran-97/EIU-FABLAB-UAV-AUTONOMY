from ground_station_ui.core.alerts.health import evaluate


class G:
    def __init__(self, values=None, items=None):
        self.values = values or {}
        self.items = items or []


class V:
    def __init__(self):
        self.groups = {
            'status': G({'linkAlive': True, 'armed': False, 'heartbeatAge': 0.5}),
            'gps': G({'fixType': 0}),
            'battery': G({'percent': 80}),
            'sensors': G(items=[{'name': 'Gyro', 'status': 'ok'}]),
            'companion': G({'linkAlive': True, 'heartbeatAge': 0.3},
                           [{'id': 1, 'name': 'Lidar', 'level': 'ok', 'state': 'STREAMING'}]),
        }


LINKS = {'px4': {'isOpen': True, 'txbuf': 90}, 'jetson': {'isOpen': True, 'txbuf': 95}}


def test_all_ok_indoor_no_gps_is_fine():
    assert evaluate(V(), LINKS, 'indoor') == []


def test_problems_sorted_critical_first():
    v = V()
    v.groups['battery'].values['percent'] = 12
    v.groups['companion'].items[0].update(level='stale')
    v.groups['sensors'].items[0]['status'] = 'error'
    out = evaluate(v, LINKS, 'outdoor')
    levels = [a['level'] for a in out]
    assert levels == sorted(levels, key=lambda lv: lv != 'critical')
    texts = ' | '.join(a['text'] for a in out)
    assert 'Gyro' in texts and 'Battery critical: 12%' in texts and 'GPS' in texts and 'Lidar' in texts


def test_link_lost():
    v = V()
    v.groups['status'].values['linkAlive'] = False
    v.groups['companion'].values['linkAlive'] = False
    out = evaluate(v, LINKS, 'indoor')
    assert out[0] == {'level': 'critical', 'source': 'PX4', 'text': 'PX4 link lost (radio 915)'}
    assert any(a['source'] == 'Jetson' for a in out)
