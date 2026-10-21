"""Closed-loop simulation with LATENCY and an ACCELERATION LIMIT.

Velocity commands reach PX4 `true_latency` seconds late, and the drone changes velocity by at most
`accel` m/s^2 (vector). The planner uses the node's own safe_speed with the latency the node measures
(transport delay + one scan period).

This simulation caught: (1) assuming a lower latency than the real one -> hitting the wall;
(2) limiting only along the flight direction -> entering the bubble when turning along a wall.

    colcon test --packages-select obstacle_avoidance
"""
import math
from collections import deque

import numpy as np
import pytest

from obstacle_avoidance.motion_limits import safe_speed
from obstacle_avoidance.vfh_plus import BLOCKED, VfhParams, VfhPlus

P = VfhParams()
R_RS = P.drone_radius + P.safety_margin
SENSOR_RANGE = 8.0


def circle(center, radius, n=72):
    t = np.linspace(0.0, 2.0 * math.pi, n, endpoint=False)
    return np.stack([center[0] + radius * np.cos(t), center[1] + radius * np.sin(t)], axis=1)


def segment(a, b, step=0.05):
    a, b = np.asarray(a, float), np.asarray(b, float)
    return np.linspace(a, b, max(2, int(np.linalg.norm(b - a) / step) + 1))


def fly(obstacles, goal, true_latency, scan_period, accel=2.0, v_max=4.0, dt=0.02, t_max=60.0):
    assumed_latency = true_latency + scan_period
    planner = VfhPlus(P)
    pos, vel, cmd = np.zeros(2), np.zeros(2), np.zeros(2)
    goal = np.asarray(goal, float)
    queue, closest, t, next_scan = deque(), math.inf, 0.0, 0.0
    while t < t_max:
        closest = min(closest, float(np.hypot(*(obstacles - pos).T).min()))
        if np.hypot(*(goal - pos)) < 0.3:
            return True, closest
        if t >= next_scan:
            next_scan += scan_period
            rel = obstacles - pos
            rel = rel[np.hypot(*rel.T) <= SENSOR_RANGE]
            heading = math.atan2(vel[1], vel[0]) if np.hypot(*vel) > 0.3 else None
            res = planner.plan(rel, goal - pos, heading)
            c = np.zeros(2)
            if res.state != BLOCKED:
                s = safe_speed(rel, res.direction, vel, R_RS, SENSOR_RANGE - R_RS,
                               assumed_latency, accel, v_max)
                c = s * np.array([math.cos(res.direction), math.sin(res.direction)])
            queue.append((t, c))
        while queue and queue[0][0] <= t - true_latency + 1e-9:
            cmd = queue.popleft()[1]
        dv = cmd - vel
        n = float(np.hypot(*dv))
        vel = vel + (dv if n <= accel * dt else dv / n * accel * dt)
        pos = pos + vel * dt
        t += dt
    return False, closest


# Up to 25 % of safety_margin may be entered (scan / command discretisation). = 1.25 m with the defaults.
MIN_CLEARANCE = R_RS - 0.25 * P.safety_margin


@pytest.mark.parametrize('scan_period', [0.1, 0.18])      # lidar 10 Hz and 5.5 Hz
@pytest.mark.parametrize('true_latency', [0.2, 0.4, 0.8])
def test_head_on_wide_wall_keeps_bubble(true_latency, scan_period):
    _, closest = fly(segment((10.0, -15.0), (10.0, 15.0)), (20.0, 0.0),
                     true_latency, scan_period, t_max=40.0)
    assert closest > MIN_CLEARANCE


@pytest.mark.parametrize('scan_period', [0.1, 0.18])
@pytest.mark.parametrize('true_latency', [0.2, 0.4, 0.8])
def test_pole_field_reached_safely(true_latency, scan_period):
    poles = np.concatenate([circle(c, 0.3) for c in
                            [(6.0, 0.0), (10.0, 1.5), (14.0, -1.0), (11.0, -2.5)]])
    reached, closest = fly(poles, (20.0, 0.0), true_latency, scan_period)
    assert reached
    assert closest > MIN_CLEARANCE
