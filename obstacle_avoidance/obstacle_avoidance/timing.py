"""Do tan so va do tre luc chay -- thuan python, khong import rclpy.

Khong hardcode tan so cua lidar / PX4: moi timeout suy ra tu chu ky DO DUOC.
"""
import math
from collections import deque


def percentile(values, q):
    """Phan vi q (0..1) theo kieu nearest-rank. values rong -> None."""
    if not values:
        return None
    s = sorted(values)
    return s[max(0, math.ceil(q * len(s)) - 1)]


class RateMonitor:
    """Chu ky den cua mot luong message va timeout suy ra tu no.

    timeout = periods * p90(chu ky), kep trong (0, max_timeout].
    Dung p90 chu khong dung trung binh: jitter binh thuong khong gay STALE gia,
    con mot lan rot message le te khong keo timeout len.

    max_timeout la gioi han cung "mu bao lau thi chap nhan duoc". Nguon cham
    toi muc periods * chu ky > max_timeout thi saturated() = True -> nen canh bao.
    """

    def __init__(self, periods: float = 3.0, max_timeout: float = 1.0,
                 window: int = 20, min_samples: int = 5) -> None:
        if periods <= 1.0:
            raise ValueError('periods phai > 1')
        self.periods = periods
        self.max_timeout = max_timeout
        self.min_samples = min_samples
        self.intervals = deque(maxlen=window)
        self.last = None
        self.hint = None

    def tick(self, t: float) -> None:
        if self.last is not None and t > self.last:
            self.intervals.append(t - self.last)
        self.last = t

    def set_hint(self, period: float) -> None:
        """Chu ky nguon tu khai bao (vd LaserScan.scan_time), dung khi chua du mau."""
        self.hint = period if period and period > 0.0 else None

    def period(self):
        if len(self.intervals) >= self.min_samples:
            return percentile(self.intervals, 0.9)
        return self.hint

    def rate(self) -> float:
        p = self.period()
        return 1.0 / p if p else 0.0

    def timeout(self) -> float:
        p = self.period()
        if p is None:
            return self.max_timeout
        return min(self.max_timeout, self.periods * p)

    def saturated(self) -> bool:
        p = self.period()
        return p is not None and self.periods * p > self.max_timeout

    def age(self, now: float) -> float:
        return math.inf if self.last is None else now - self.last

    def is_stale(self, now: float) -> bool:
        return self.age(now) > self.timeout()


class RecentStats:
    """N mau gan nhat cua mot dai luong (vd do tre), lay p90 de uoc luong than trong."""

    def __init__(self, window: int = 20) -> None:
        self.values = deque(maxlen=window)

    def add(self, v: float) -> None:
        self.values.append(v)

    def p90(self):
        return percentile(self.values, 0.9)

    def __len__(self) -> int:
        return len(self.values)
