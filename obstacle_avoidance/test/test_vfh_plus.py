"""Kiem tra VFH+ bang vat can gia va mo phong vong kin (drone diem).

    colcon test --packages-select obstacle_avoidance
"""
import math

import numpy as np
import pytest

from obstacle_avoidance.vfh_plus import (AVOIDING, BLOCKED, CLEAR, VfhParams,
                                         VfhPlus, angle_diff)

P = VfhParams()
R_RS = P.drone_radius + P.safety_margin


def circle(center, radius, n=72):
    t = np.linspace(0.0, 2.0 * math.pi, n, endpoint=False)
    return np.stack([center[0] + radius * np.cos(t), center[1] + radius * np.sin(t)], axis=1)


def segment(a, b, step=0.05):
    a, b = np.asarray(a, float), np.asarray(b, float)
    n = max(2, int(np.linalg.norm(b - a) / step) + 1)
    return np.linspace(a, b, n)


def test_invalid_params_rejected():
    with pytest.raises(ValueError):
        VfhPlus(VfhParams(mu_goal=3.0, mu_motion=2.0, mu_prev=2.0))
    with pytest.raises(ValueError):
        VfhPlus(VfhParams(block_distance=1.0))          # < drone_radius + safety_margin


def test_sector_of_wraps():
    v = VfhPlus(P)
    assert v.sector_of(0.0) == 0
    assert v.sector_of(math.radians(-1.0)) == 0
    assert v.sector_of(math.radians(-3.0)) == v.n_sectors - 1
    assert v.sector_of(math.radians(90.0)) == 18


def test_no_obstacle_goes_straight_to_goal():
    res = VfhPlus(P).plan(np.empty((0, 2)), (10.0, 10.0))
    assert res.state == CLEAR
    assert res.direction == pytest.approx(math.radians(45.0))
    assert res.speed_scale == 1.0


def test_obstacle_on_the_side_does_not_trigger_avoidance():
    res = VfhPlus(P).plan(circle((0.0, 3.0), 0.3), (10.0, 0.0))   # cot o phia Dong 3m
    assert res.state == CLEAR
    assert res.direction == pytest.approx(0.0)


def test_obstacle_ahead_deflects_by_at_least_enlargement_angle():
    obstacle = segment((2.5, -0.5), (2.5, 0.5))      # vat can rong 1m, cach 2.5m phia Bac
    res = VfhPlus(P).plan(obstacle, (10.0, 0.0))
    assert res.state == AVOIDING
    assert angle_diff(res.direction, 0.0) >= math.asin(R_RS / 2.5)
    assert res.min_distance == pytest.approx(2.5)
    assert res.min_bearing == pytest.approx(0.0)


def test_surrounded_is_blocked():
    res = VfhPlus(P).plan(circle((0.0, 0.0), 1.2, n=360), (10.0, 0.0))
    assert res.state == BLOCKED
    assert res.direction is None
    assert res.speed_scale == 0.0


def test_goal_in_front_of_wall_is_reachable():
    wall = segment((2.5, -5.0), (2.5, 5.0))
    res = VfhPlus(P).plan(wall, (1.0, 0.0))            # goal 1m, tuong 2.5m
    assert res.state == CLEAR
    assert res.direction == pytest.approx(0.0)


def test_block_hysteresis():
    v = VfhPlus(P)
    goal = (20.0, 0.0)
    assert v.plan(circle((2.9, 0.0), 0.05), goal).state == AVOIDING      # < block 3.0
    assert v.plan(circle((3.3, 0.0), 0.05), goal).state == AVOIDING      # < free 3.5: van chan
    assert v.plan(circle((3.7, 0.0), 0.05), goal).state == CLEAR         # > free: mo lai


def test_symmetric_obstacle_choice_is_stable():
    v = VfhPlus(P)
    obstacle = segment((2.5, -0.5), (2.5, 0.5))
    first = v.plan(obstacle, (10.0, 0.0)).direction
    for _ in range(10):
        assert v.plan(obstacle, (10.0, 0.0)).direction == pytest.approx(first)


# --------------------------------------------------------------------------- #
# Mo phong vong kin: drone diem bay theo huong VFH+ (giong carrot cua node).
def fly(obstacles_ne, goal, start=(0.0, 0.0), v_max=1.5, dt=0.1, steps=600, sensor_range=8.0):
    v = VfhPlus(P)
    pos = np.asarray(start, float)
    goal = np.asarray(goal, float)
    heading = None
    closest = math.inf
    for _ in range(steps):
        rel = obstacles_ne - pos
        rel = rel[np.hypot(rel[:, 0], rel[:, 1]) <= sensor_range]
        closest = min(closest, float(np.hypot(rel[:, 0], rel[:, 1]).min(initial=math.inf)))
        to_goal = goal - pos
        if np.hypot(*to_goal) < 0.3:
            return True, closest
        res = v.plan(rel, to_goal, heading)
        if res.state == BLOCKED:
            return False, closest
        step = min(v_max * res.speed_scale * dt, float(np.hypot(*to_goal)))
        pos = pos + step * np.array([math.cos(res.direction), math.sin(res.direction)])
        heading = res.direction
    return False, closest


def test_sim_flies_through_pole_field():
    poles = np.concatenate([circle(c, 0.3) for c in [(6.0, 0.0), (10.0, 1.5), (14.0, -1.0),
                                                     (11.0, -2.5)]])
    reached, closest = fly(poles, (20.0, 0.0))
    assert reached
    assert closest > P.drone_radius + 0.3


def test_sim_goes_around_wall():
    wall = segment((8.0, -3.0), (8.0, 3.0))
    reached, closest = fly(wall, (16.0, 0.0))
    assert reached
    assert closest > P.drone_radius + 0.3
