"""Speed from the stopping distance: v*T + v^2/(2a) <= D."""
import math

import pytest

from obstacle_avoidance.motion_limits import stopping_distance, stopping_speed


@pytest.mark.parametrize('D, T, a', [(1.5, 0.25, 2.0), (6.5, 0.4, 3.0), (0.2, 1.0, 1.0)])
def test_speed_stops_exactly_within_distance(D, T, a):
    v = stopping_speed(D, T, a)
    assert v > 0.0
    assert stopping_distance(v, T, a) == pytest.approx(D)


def test_no_room_means_no_speed():
    assert stopping_speed(0.0, 0.2, 2.0) == 0.0
    assert stopping_speed(-1.0, 0.2, 2.0) == 0.0


def test_open_space_unbounded():
    assert stopping_speed(math.inf, 0.2, 2.0) == math.inf


def test_more_latency_means_slower():
    fast = stopping_speed(5.0, 0.1, 2.0)
    slow = stopping_speed(5.0, 0.5, 2.0)
    assert slow < fast


def test_array_input():
    import numpy as np
    v = stopping_speed(np.array([-1.0, 0.0, 1.5, np.inf]), 0.25, 2.0)
    assert v[0] == 0.0 and v[1] == 0.0 and v[3] == math.inf
    assert v[2] == pytest.approx(stopping_speed(1.5, 0.25, 2.0))


def test_approach_limit_only_counts_closing_component():
    import numpy as np
    from obstacle_avoidance.motion_limits import approach_speed_limit
    wall_east = np.array([[0.0, 3.0]])                       # point 3 m east
    along = approach_speed_limit(wall_east, 0.0, 1.5, 0.25, 2.0)          # flying north: parallel
    away = approach_speed_limit(wall_east, math.pi / 2 + math.pi, 1.5, 0.25, 2.0)  # flying west
    toward = approach_speed_limit(wall_east, math.pi / 2, 1.5, 0.25, 2.0)          # flying east
    oblique = approach_speed_limit(wall_east, math.pi / 4, 1.5, 0.25, 2.0)
    assert along == math.inf and away == math.inf
    assert toward == pytest.approx(stopping_speed(1.5, 0.25, 2.0))
    assert oblique == pytest.approx(toward / math.cos(math.pi / 4))


def test_must_brake_when_closing_too_fast():
    import numpy as np
    from obstacle_avoidance.motion_limits import must_brake
    pts = np.array([[3.0, 0.0]])                             # obstacle 3 m north
    v_ok = stopping_speed(1.5, 0.25, 2.0)
    assert not must_brake(pts, (0.9 * v_ok, 0.0), 1.5, 0.25, 2.0)
    assert must_brake(pts, (1.1 * v_ok, 0.0), 1.5, 0.25, 2.0)
    assert not must_brake(pts, (0.0, 5.0), 1.5, 0.25, 2.0)   # fast sideways: not closing in
