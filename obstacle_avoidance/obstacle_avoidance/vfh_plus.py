"""VFH+ (Ulrich & Borenstein, 1998) cho multirotor voi lidar 2D 360 do.

Thuan numpy, khong import rclpy. Moi goc la goc NED: 0 = Bac, duong = theo
chieu kim dong ho, huong (cos a, sin a) = (north, east).

Khac voi VFH+ goc (viet cho xe):
  - Bo "masked polar histogram" theo ban kinh quay: multirotor bay ngang duoc
    moi huong.
  - Histogram luu KHOANG CACH chu khong luu do lon m = a - b*d. Voi m tuyen
    tinh va lay max theo sector thi hai cach tuong duong, nhung nguong dat
    bang met (block_distance) de chinh hon nguong tau khong co don vi, va khong
    phu thuoc do phan giai lidar.

Cac buoc:
  1. Polar histogram: moi diem o khoang cach d duoc phong to thanh non
     +-gamma, gamma = asin((drone_radius + safety_margin) / d).
     clearance[k] = khoang cach nho nhat cua diem co non phu sector k.
  2. Binary histogram co hysteresis: bi chan khi clearance < block_distance,
     da bi chan thi chi mo lai khi clearance > free_distance.
  3. Valley = day sector trong lien tiep. Valley hep -> ung vien o giua.
     Valley rong -> 2 ung vien cach mep wide_valley_sectors/2, cong huong goal
     neu sector goal trong.
  4. Cost g = mu_goal*D(c, goal) + mu_motion*D(c, huong van toc)
            + mu_prev*D(c, huong chon lan truoc), chon g nho nhat.
     Giu mu_goal > mu_motion + mu_prev thi khi huong goal trong, no LUON thang
     (bat dang thuc tam giac) -> khong di vong khi khong can.

Toc do KHONG tinh o day: xem motion_limits.py (theo quang dung + do tre do duoc).
"""
import math
from dataclasses import dataclass
from typing import Optional

import numpy as np

CLEAR = 0
AVOIDING = 1
BLOCKED = 2

TWO_PI = 2.0 * math.pi


def wrap_pi(a):
    return (np.asarray(a) + math.pi) % TWO_PI - math.pi


def angle_diff(a, b):
    """|a - b| tren vong tron, trong [0, pi]."""
    return np.abs(wrap_pi(np.asarray(a) - np.asarray(b)))


@dataclass
class VfhParams:
    sector_deg: float = 5.0
    window_radius: float = 8.0      # m, chi xet vat can trong ban kinh nay
    drone_radius: float = 0.5       # m, ban kinh drone tinh ca canh quat
    safety_margin: float = 1.0      # m, khoang dem them quanh drone
    block_distance: float = 3.0     # m, sector bi chan khi clearance < muc nay
    free_distance: float = 3.5      # m, sector da chan chi mo lai khi > muc nay
    wide_valley_sectors: int = 8    # valley rong hon muc nay la valley rong
    min_valley_sectors: int = 1     # valley hep hon muc nay bi bo qua
    mu_goal: float = 5.0
    mu_motion: float = 2.0
    mu_prev: float = 2.0

    def validate(self) -> None:
        if not 0.0 < self.sector_deg <= 45.0 or (360.0 / self.sector_deg) % 1.0 > 1e-6:
            raise ValueError('sector_deg phai chia het 360 va trong (0, 45]')
        if not (self.drone_radius + self.safety_margin
                < self.block_distance <= self.free_distance <= self.window_radius):
            raise ValueError('can: drone_radius + safety_margin < block_distance'
                             ' <= free_distance <= window_radius')
        if self.mu_goal <= self.mu_motion + self.mu_prev:
            raise ValueError('can mu_goal > mu_motion + mu_prev')


@dataclass
class VfhResult:
    state: int                    # CLEAR / AVOIDING / BLOCKED
    direction: Optional[float]    # huong bay (rad NED), None neu BLOCKED
    goal_bearing: float
    clearance_ahead: float        # clearance theo huong bay (m), 0 neu BLOCKED
    min_distance: float           # vat can gan nhat (m), inf neu khong co
    min_bearing: float
    clearance: np.ndarray         # (S,) clearance tung sector
    blocked: np.ndarray           # (S,) binary histogram sau hysteresis


class VfhPlus:
    def __init__(self, params: VfhParams) -> None:
        params.validate()
        self.p = params
        self.alpha = math.radians(params.sector_deg)
        self.n_sectors = int(round(360.0 / params.sector_deg))
        self.centers = np.arange(self.n_sectors) * self.alpha
        self.reset()

    def reset(self) -> None:
        """Goi khi bat lai avoidance: xoa hysteresis va huong cu."""
        self._prev_blocked = None
        self._prev_direction = None

    def sector_of(self, angle: float) -> int:
        return int(math.floor(((angle + self.alpha / 2.0) % TWO_PI) / self.alpha)) % self.n_sectors

    # ------------------------------------------------------------------ #
    def clearance_histogram(self, points_ne: np.ndarray) -> np.ndarray:
        """Buoc 1: clearance[k] (m) sau khi phong to vat can."""
        clearance = np.full(self.n_sectors, np.inf)
        if points_ne.size == 0:
            return clearance
        d = np.hypot(points_ne[:, 0], points_ne[:, 1])
        keep = d <= self.p.window_radius
        if not keep.any():
            return clearance
        d = d[keep]
        beta = np.arctan2(points_ne[keep, 1], points_ne[keep, 0])
        r_rs = self.p.drone_radius + self.p.safety_margin
        gamma = np.arcsin(np.clip(r_rs / np.maximum(d, 1e-6), 0.0, 1.0))
        # Sector [c - a/2, c + a/2] giao non [beta - gamma, beta + gamma]
        covered = angle_diff(self.centers[None, :], beta[:, None]) \
            <= gamma[:, None] + self.alpha / 2.0
        return np.where(covered, d[:, None], np.inf).min(axis=0)

    def _valleys(self, free: np.ndarray):
        """Cac day sector trong lien tiep (vong tron): list (start, length)."""
        S = free.size
        if free.all():
            return [(0, S)]
        if not free.any():
            return []
        shift = int(np.argmax(~free))          # xoay de index 0 la sector bi chan
        rolled = np.roll(free, -shift)
        valleys, i = [], 0
        while i < S:
            if rolled[i]:
                j = i
                while j < S and rolled[j]:
                    j += 1
                valleys.append(((i + shift) % S, j - i))
                i = j
            else:
                i += 1
        return valleys

    def _candidates(self, free: np.ndarray, goal_bearing: float, goal_free: bool):
        cands = []
        if goal_free:
            cands.append(goal_bearing)
        s_max = self.p.wide_valley_sectors
        for start, length in self._valleys(free):
            if length == self.n_sectors:
                continue                          # toan bo trong: chi can goal
            if length < self.p.min_valley_sectors:
                continue
            if length <= s_max:
                cands.append(self.centers[start] + (length - 1) / 2.0 * self.alpha)
            else:
                cands.append(self.centers[start] + (s_max / 2.0) * self.alpha)
                cands.append(self.centers[start] + (length - 1 - s_max / 2.0) * self.alpha)
        return [float(wrap_pi(c)) for c in cands]

    def _cost(self, c, goal_bearing, motion_heading):
        g = self.p.mu_goal * angle_diff(c, goal_bearing)
        if motion_heading is not None:
            g += self.p.mu_motion * angle_diff(c, motion_heading)
        if self._prev_direction is not None:
            g += self.p.mu_prev * angle_diff(c, self._prev_direction)
        return float(g) / self.alpha

    # ------------------------------------------------------------------ #
    def plan(self, points_ne: np.ndarray, goal_ne, motion_heading: Optional[float] = None
             ) -> VfhResult:
        """points_ne: (M, 2) vat can lech so voi drone. goal_ne: (dn, de) goal lech so voi drone.
        motion_heading: huong van toc ngang (rad NED) hoac None khi gan nhu dung yen."""
        p = self.p
        points_ne = np.asarray(points_ne, dtype=float).reshape(-1, 2)
        goal_bearing = math.atan2(goal_ne[1], goal_ne[0])
        goal_dist = math.hypot(goal_ne[0], goal_ne[1])

        if points_ne.shape[0]:
            d_all = np.hypot(points_ne[:, 0], points_ne[:, 1])
            i_min = int(np.argmin(d_all))
            min_distance = float(d_all[i_min])
            min_bearing = math.atan2(points_ne[i_min, 1], points_ne[i_min, 0])
        else:
            min_distance, min_bearing = math.inf, 0.0

        clearance = self.clearance_histogram(points_ne)

        blocked = clearance < p.block_distance
        if self._prev_blocked is not None:
            blocked |= self._prev_blocked & (clearance < p.free_distance)
        self._prev_blocked = blocked.copy()

        # Goal nam truoc vat can (vd goal sat tuong) thi van toi duoc.
        k_goal = self.sector_of(goal_bearing)
        goal_free = (not blocked[k_goal]) or \
            (goal_dist + p.drone_radius <= clearance[k_goal])

        cands = self._candidates(~blocked, goal_bearing, goal_free)
        if not cands:
            self._prev_direction = None
            return VfhResult(BLOCKED, None, goal_bearing, 0.0,
                             min_distance, min_bearing, clearance, blocked)

        costs = [self._cost(c, goal_bearing, motion_heading) for c in cands]
        direction = cands[int(np.argmin(costs))]
        self._prev_direction = direction

        heading_to_goal = goal_free and angle_diff(direction, goal_bearing) < 1e-6
        ahead = float(clearance[self.sector_of(direction)])

        return VfhResult(CLEAR if heading_to_goal else AVOIDING, direction, goal_bearing,
                         ahead, min_distance, min_bearing, clearance, blocked)
