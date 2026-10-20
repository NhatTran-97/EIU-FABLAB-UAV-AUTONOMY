from drone_telemetry.health import (LEVEL_ERROR, LEVEL_OK, LEVEL_STALE, LEVEL_WARN,
                                    ComponentMonitor)


def test_no_data_is_stale():
    m = ComponentMonitor(1, 'lidar', 3.0, rate_key='scan_rate_hz')
    assert m.snapshot(0.0) == (LEVEL_STALE, 0.0, 'NO_DATA')


def test_diag_rate_key():
    m = ComponentMonitor(1, 'lidar', 3.0, rate_key='scan_rate_hz')
    m.update_from_diag(10.0, LEVEL_OK, 'Streaming', {'state': 'STREAMING', 'scan_rate_hz': '7.00'})
    assert m.snapshot(10.5) == (LEVEL_OK, 7.0, 'STREAMING')


def test_diag_goes_stale():
    m = ComponentMonitor(1, 'lidar', 3.0, rate_key='scan_rate_hz')
    m.update_from_diag(10.0, LEVEL_OK, '', {'state': 'STREAMING', 'scan_rate_hz': '7'})
    assert m.snapshot(13.5)[0] == LEVEL_STALE


def test_diag_count_key_rate():
    m = ComponentMonitor(2, 'gimbal', 3.0, count_key='total_attitudes')
    m.update_from_diag(0.0, LEVEL_OK, '', {'state': 'READY', 'total_attitudes': '100'})
    m.update_from_diag(1.0, LEVEL_OK, '', {'state': 'READY', 'total_attitudes': '110'})
    assert m.snapshot(1.0) == (LEVEL_OK, 10.0, 'READY')


def test_count_reset_is_ignored():
    m = ComponentMonitor(2, 'gimbal', 3.0, count_key='total_attitudes')
    m.update_from_diag(0.0, LEVEL_OK, '', {'total_attitudes': '100'})
    m.update_from_diag(1.0, LEVEL_OK, '', {'total_attitudes': '110'})
    m.update_from_diag(2.0, LEVEL_OK, '', {'total_attitudes': '5'})    # node restart
    assert m.snapshot(2.0)[1] == 10.0
    m.update_from_diag(3.0, LEVEL_OK, '', {'total_attitudes': '15'})
    assert m.snapshot(3.0)[1] == 10.0


def test_state_falls_back_to_message_and_is_truncated():
    m = ComponentMonitor(3, 'camera', 3.0)
    m.update_from_diag(0.0, LEVEL_WARN, 'A very long status message', {})
    level, _, state = m.snapshot(0.0)
    assert level == LEVEL_WARN and state == 'A very long stat'


def test_unknown_level_maps_to_error():
    m = ComponentMonitor(3, 'camera', 3.0)
    m.update_from_diag(0.0, 7, 'x', {'state': 'WEIRD'})
    assert m.snapshot(0.0)[0] == LEVEL_ERROR


def test_topic_rate_and_low_rate():
    m = ComponentMonitor(4, 'px4', 3.0, min_rate_hz=10.0)
    for i in range(50):
        m.on_message(i * 0.02)          # 50 Hz
    m.tick(1.0)
    assert m.snapshot(1.0) == (LEVEL_OK, 50.0, 'STREAMING')
    for i in range(5):
        m.on_message(1.0 + i * 0.2)     # 5 Hz
    m.tick(2.0)
    assert m.snapshot(2.0) == (LEVEL_WARN, 5.0, 'LOW_RATE')


def test_topic_stops_goes_stale():
    m = ComponentMonitor(4, 'px4', 3.0)
    m.on_message(0.0)
    m.tick(1.0)
    assert m.snapshot(5.0) == (LEVEL_STALE, 0.0, 'NO_DATA')
