"""Filter a LaserScan and convert the lidar points to the local NED frame (relative to the drone).

Pure numpy, no rclpy, so it can be tested without ROS.

Frame chain of a beam at angle theta (REP-103, counter-clockwise):

    lidar (z up)  --flip if mounted upside down-->  FLU  --(x, -y, -z)-->  FRD  --R(q)-->  NED

    - lidar_yaw_offset: angle of the lidar x axis relative to the drone nose, in FLU (positive = to the left).
    - upside_down: a lidar mounted upside down (z down) mirrors y, so theta changes sign.
    - q: PX4 VehicleAttitude.q, quaternion [w, x, y, z] FRD -> NED.

The full quaternion (not only the heading) is used, so points keep their horizontal position when the
drone tilts, and beams that hit the ground can be detected.
"""
import math
from dataclasses import dataclass, field

import numpy as np


@dataclass
class ScanConfig:
    lidar_yaw_offset: float = 0.0     # rad
    upside_down: bool = False
    min_range: float = 0.3            # m, closer points are dropped (propellers, legs)
    max_range: float = 12.0           # m, farther points are dropped
    # Arcs hidden by the drone body, in the LIDAR frame (raw /scan angles), rad.
    # Each entry (a, b) masks from a counter-clockwise to b.
    self_mask: list = field(default_factory=list)
    # A beam is kept only if at least one neighbour differs by less than despeckle_tol (m).
    # Removes single-beam noise (dust, rain drops). 0 = off.
    despeckle_tol: float = 0.3
    # Points whose "down" offset is >= agl - ground_margin are ground and are dropped.
    ground_margin: float = 0.3        # m


@dataclass
class ProcessedScan:
    points_ne: np.ndarray      # (M, 2) obstacles: [north, east] relative to the drone (m)
    free_bearings: np.ndarray  # (K,) NED directions of beams that hit no obstacle
    ground_hits: int           # number of beams dropped because they hit the ground


def wrap_pi(a):
    return (np.asarray(a) + math.pi) % (2.0 * math.pi) - math.pi


def quat_to_rotmat(q_wxyz) -> np.ndarray:
    """Hamilton quaternion [w, x, y, z] -> rotation matrix (body -> world)."""
    w, x, y, z = (float(v) for v in q_wxyz)
    n = math.sqrt(w * w + x * x + y * y + z * z)
    if n < 1e-9:
        raise ValueError('quaternion has zero length')
    w, x, y, z = w / n, x / n, y / n, z / n
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
        [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
        [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
    ])


def yaw_to_quat(yaw: float):
    """Yaw-only quaternion [w, x, y, z] (NED), for tests and simulation."""
    return (math.cos(yaw / 2.0), 0.0, 0.0, math.sin(yaw / 2.0))


def beam_directions_frd(angles, lidar_yaw_offset=0.0, upside_down=False) -> np.ndarray:
    """Beam angles in the lidar frame -> unit vectors in the body FRD frame, shape (N, 3)."""
    angles = np.asarray(angles, dtype=float)
    phi = (-angles if upside_down else angles) + lidar_yaw_offset   # angle in FLU
    return np.stack([np.cos(phi), -np.sin(phi), np.zeros_like(phi)], axis=1)


def angle_mask(angles, arcs) -> np.ndarray:
    """True for angles inside one of the arcs (a, b) (counter-clockwise from a to b)."""
    angles = np.asarray(angles, dtype=float)
    masked = np.zeros(angles.shape, dtype=bool)
    two_pi = 2.0 * math.pi
    for a, b in arcs:
        span = (b - a) % two_pi
        masked |= ((angles - a) % two_pi) <= span
    return masked


def despeckle(ranges, tol) -> np.ndarray:
    """True for beams with at least one neighbour (circular) of nearly the same range."""
    r = np.asarray(ranges, dtype=float)
    if tol <= 0.0 or r.size < 3:
        return np.ones(r.shape, dtype=bool)
    with np.errstate(invalid='ignore'):
        near_prev = np.abs(r - np.roll(r, 1)) < tol
        near_next = np.abs(r - np.roll(r, -1)) < tol
    return near_prev | near_next


def process_scan(ranges, angle_min, angle_increment, range_min, range_max,
                 q_wxyz, agl, cfg: ScanConfig) -> ProcessedScan:
    """LaserScan -> obstacle points in the NED frame (relative to the drone).

    agl: height above ground (m, positive). None = no ground filter.
    """
    r = np.asarray(ranges, dtype=float)
    angles = angle_min + angle_increment * np.arange(r.size)

    lo = max(float(range_min), cfg.min_range)
    hi = min(float(range_max), cfg.max_range) if range_max > 0 else cfg.max_range

    usable = ~angle_mask(angles, cfg.self_mask)
    with np.errstate(invalid='ignore'):
        # "Free" beam: inf or beyond range_max -> nothing in that direction.
        # NaN and values below range_min (many drivers report 0) are "unknown" -> dropped.
        no_return = usable & (np.isposinf(r) | (r > hi))
        hit = usable & np.isfinite(r) & (r >= lo) & (r <= hi)
    hit &= despeckle(r, cfg.despeckle_tol)

    dirs_ned = beam_directions_frd(angles, cfg.lidar_yaw_offset, cfg.upside_down) \
        @ quat_to_rotmat(q_wxyz).T

    pts = dirs_ned[hit] * r[hit, None]
    ground = np.zeros(pts.shape[0], dtype=bool)
    if agl is not None:
        ground = pts[:, 2] >= (agl - cfg.ground_margin)
    obstacles = pts[~ground]

    free_dirs = np.concatenate([dirs_ned[no_return], dirs_ned[hit][ground]])
    free_bearings = np.arctan2(free_dirs[:, 1], free_dirs[:, 0])

    return ProcessedScan(points_ne=obstacles[:, :2].copy(),
                         free_bearings=free_bearings,
                         ground_hits=int(ground.sum()))
