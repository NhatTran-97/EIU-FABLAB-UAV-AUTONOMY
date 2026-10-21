"""Speed limit from the stopping distance -- pure numpy.

The drone must be able to stop before touching the safety bubble:

    v * T + v^2 / (2 * a)  <=  D

    T: total latency from the lidar seeing an obstacle to PX4 starting to brake (s)
    a: horizontal braking deceleration (m/s^2), <= MPC_ACC_HOR
    D: free distance before the bubble is touched (m)

Solved for v: v = a * (-T + sqrt(T^2 + 2 D / a)).
T comes from the latency MEASURED at runtime, so a slower or more delayed lidar makes the drone slow down
on its own, without manual tuning.

The constraint applies to the CLOSING component towards each obstacle point, not only along the flight
direction: flying at an angle into a wall still closes in on it. Simulation (with latency and an
acceleration limit) shows that checking only the flight direction lets the drone enter the bubble while
sliding along a wall.
"""
import math

import numpy as np


def stopping_distance(speed: float, latency: float, decel: float) -> float:
    return speed * latency + speed * speed / (2.0 * decel)


def stopping_speed(distance, latency: float, decel: float):
    """Highest speed that can still stop within `distance` m (scalar or array)."""
    d = np.asarray(distance, dtype=float)
    with np.errstate(invalid='ignore'):
        v = decel * (-latency + np.sqrt(latency * latency + 2.0 * np.maximum(d, 0.0) / decel))
    v = np.where(d <= 0.0, 0.0, np.where(np.isinf(d), np.inf, v))
    return float(v) if v.ndim == 0 else v


def _approach(points_ne: np.ndarray, unit_ne: np.ndarray):
    """(distance, cosine of the angle between `unit` and the direction to each point)."""
    d = np.hypot(points_ne[:, 0], points_ne[:, 1])
    cos = (points_ne @ unit_ne) / np.maximum(d, 1e-9)
    return d, cos


def approach_speed_limit(points_ne, direction: float, bubble: float,
                         latency: float, decel: float) -> float:
    """Highest speed along `direction` such that, for EVERY point, the closing component s*cos(angle)
    can still stop before the bubble of radius `bubble`."""
    pts = np.asarray(points_ne, dtype=float).reshape(-1, 2)
    if pts.shape[0] == 0:
        return math.inf
    d, cos = _approach(pts, np.array([math.cos(direction), math.sin(direction)]))
    closing = cos > 1e-6
    if not closing.any():
        return math.inf
    allowed = stopping_speed(d[closing] - bubble, latency, decel) / cos[closing]
    return float(np.min(allowed))


def must_brake(points_ne, velocity_ne, bubble: float, latency: float, decel: float) -> bool:
    """True if the CURRENT velocity closes in on a point faster than it can still stop.
    The drone should then be commanded to stop, so PX4 uses its full deceleration to brake."""
    pts = np.asarray(points_ne, dtype=float).reshape(-1, 2)
    vel = np.asarray(velocity_ne, dtype=float)
    speed = float(np.hypot(vel[0], vel[1]))
    if pts.shape[0] == 0 or speed < 1e-6:
        return False
    d, cos = _approach(pts, vel / speed)
    closing_speed = speed * cos
    return bool(np.any(closing_speed > stopping_speed(d - bubble, latency, decel)))


def safe_speed(points_ne, direction: float, velocity_ne, bubble: float, horizon: float,
               latency: float, decel: float, max_speed: float) -> float:
    """Safe speed along `direction`.

    - 0 if the current velocity already exceeds the stopping limit (brake first, turn afterwards).
    - Able to stop within the sensing `horizon` (obstacles beyond it are not seen yet).
    - Every obstacle point: the closing component can stop before the bubble.
    """
    if velocity_ne is not None and must_brake(points_ne, velocity_ne, bubble, latency, decel):
        return 0.0
    return min(max_speed,
               stopping_speed(horizon, latency, decel),
               approach_speed_limit(points_ne, direction, bubble, latency, decel))
