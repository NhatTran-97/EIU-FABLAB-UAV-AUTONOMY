"""Loc LaserScan va dua diem lidar ve he local NED (lech so voi drone).

Thuan numpy, khong import rclpy -> test duoc khong can ROS.

Chuoi he toa do cua mot tia co goc theta (REP-103, nguoc chieu kim dong ho):

    lidar (z len)  --lat neu lap up-->  FLU  --(x, -y, -z)-->  FRD  --R(q)-->  NED

    - lidar_yaw_offset: goc cua truc x lidar so voi mui drone, trong FLU
      (duong = quay sang trai).
    - upside_down: lidar lap up (z xuong) thi y bi dao -> theta doi dau.
    - q: VehicleAttitude.q cua PX4, quaternion [w, x, y, z] FRD -> NED.

Dung ca quaternion (chu khong chi heading) nen khi drone nghieng, diem van dung
vi tri ngang, va biet duoc tia nao dang cham mat dat.
"""
import math
from dataclasses import dataclass, field

import numpy as np


@dataclass
class ScanConfig:
    lidar_yaw_offset: float = 0.0     # rad
    upside_down: bool = False
    min_range: float = 0.3            # m, bo diem gan hon (canh quat, chan)
    max_range: float = 12.0           # m, bo diem xa hon
    # Cac cung goc bi than drone che, trong he LIDAR (goc tho cua /scan), rad.
    # Moi phan tu (a, b): che tu a quay nguoc chieu kim dong ho toi b.
    self_mask: list = field(default_factory=list)
    # Tia chi duoc giu neu co it nhat 1 tia ke ben lech < despeckle_tol (m).
    # Loai nhieu 1 tia (bui, hat mua). 0 = tat.
    despeckle_tol: float = 0.3
    # Diem co do cao "xuong" >= agl - ground_margin coi la mat dat va bi bo.
    ground_margin: float = 0.3        # m


@dataclass
class ProcessedScan:
    points_ne: np.ndarray      # (M, 2) vat can: [north, east] lech so voi drone (m)
    free_bearings: np.ndarray  # (K,) huong NED cua cac tia khong cham vat can nao
    ground_hits: int           # so tia bi loai vi cham dat


def wrap_pi(a):
    return (np.asarray(a) + math.pi) % (2.0 * math.pi) - math.pi


def quat_to_rotmat(q_wxyz) -> np.ndarray:
    """Quaternion Hamilton [w, x, y, z] -> ma tran quay (body -> world)."""
    w, x, y, z = (float(v) for v in q_wxyz)
    n = math.sqrt(w * w + x * x + y * y + z * z)
    if n < 1e-9:
        raise ValueError('quaternion co do dai bang 0')
    w, x, y, z = w / n, x / n, y / n, z / n
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
        [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
        [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
    ])


def yaw_to_quat(yaw: float):
    """Quaternion [w, x, y, z] chi co yaw (NED) -- tien cho test va mo phong."""
    return (math.cos(yaw / 2.0), 0.0, 0.0, math.sin(yaw / 2.0))


def beam_directions_frd(angles, lidar_yaw_offset=0.0, upside_down=False) -> np.ndarray:
    """Goc tia trong he lidar -> vector don vi trong he body FRD, shape (N, 3)."""
    angles = np.asarray(angles, dtype=float)
    phi = (-angles if upside_down else angles) + lidar_yaw_offset   # goc trong FLU
    return np.stack([np.cos(phi), -np.sin(phi), np.zeros_like(phi)], axis=1)


def angle_mask(angles, arcs) -> np.ndarray:
    """True voi goc nam trong mot trong cac cung (a, b) (nguoc chieu kim dong ho a -> b)."""
    angles = np.asarray(angles, dtype=float)
    masked = np.zeros(angles.shape, dtype=bool)
    two_pi = 2.0 * math.pi
    for a, b in arcs:
        span = (b - a) % two_pi
        masked |= ((angles - a) % two_pi) <= span
    return masked


def despeckle(ranges, tol) -> np.ndarray:
    """True voi tia co it nhat mot tia ke ben (vong tron) gan bang no."""
    r = np.asarray(ranges, dtype=float)
    if tol <= 0.0 or r.size < 3:
        return np.ones(r.shape, dtype=bool)
    with np.errstate(invalid='ignore'):
        near_prev = np.abs(r - np.roll(r, 1)) < tol
        near_next = np.abs(r - np.roll(r, -1)) < tol
    return near_prev | near_next


def process_scan(ranges, angle_min, angle_increment, range_min, range_max,
                 q_wxyz, agl, cfg: ScanConfig) -> ProcessedScan:
    """LaserScan -> diem vat can trong he NED (lech so voi drone).

    agl: do cao so voi mat dat (m, duong). None = khong loc mat dat.
    """
    r = np.asarray(ranges, dtype=float)
    angles = angle_min + angle_increment * np.arange(r.size)

    lo = max(float(range_min), cfg.min_range)
    hi = min(float(range_max), cfg.max_range) if range_max > 0 else cfg.max_range

    usable = ~angle_mask(angles, cfg.self_mask)
    with np.errstate(invalid='ignore'):
        # Tia "trong": inf hoac vuot range_max -> khong co gi tren huong nay.
        # NaN va gia tri < range_min (nhieu driver tra 0) la "khong biet" -> bo.
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
