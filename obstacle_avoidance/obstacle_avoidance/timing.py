"""Rate and latency measured at runtime -- pure Python, no rclpy.

Lidar and PX4 rates are not hardcoded: every timeout is derived from the MEASURED period.
"""
import math
from collections import deque


def percentile(values, q):
    """Nearest-rank percentile q (0..1). Empty values -> None."""
    if not values:
        return None
    s = sorted(values)
    return s[max(0, math.ceil(q * len(s)) - 1)]


class RateMonitor:
    """Arrival period of a message stream and the timeout derived from it.

    timeout = periods * p90(period), clamped to (0, max_timeout].
    p90 rather than the mean: normal jitter does not cause a false STALE, and a single dropped message
    does not inflate the timeout.

    max_timeout is the hard limit on how long being blind is acceptable. A source so slow that
    periods * period > max_timeout makes saturated() True, which should be reported.
    """

    def __init__(self, periods: float = 3.0, max_timeout: float = 1.0,
                 window: int = 20, min_samples: int = 5) -> None:
        if periods <= 1.0:
            raise ValueError('periods must be > 1')
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
        """Period declared by the source (e.g. LaserScan.scan_time), used until there are enough samples."""
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
    """Last N samples of a quantity (e.g. latency); p90 gives a conservative estimate."""

    def __init__(self, window: int = 20) -> None:
        self.values = deque(maxlen=window)

    def add(self, v: float) -> None:
        self.values.append(v)

    def p90(self):
        return percentile(self.values, 0.9)

    def __len__(self) -> int:
        return len(self.values)
