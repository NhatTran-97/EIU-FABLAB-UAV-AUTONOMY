"""RateMonitor / RecentStats: timeouts must come from the measured rate, not hardcoded values."""
import math

import pytest

from obstacle_avoidance.timing import RateMonitor, RecentStats, percentile


def feed(m, period, n, t0=0.0):
    t = t0
    for _ in range(n):
        m.tick(t)
        t += period
    return t - period


def test_percentile_nearest_rank():
    assert percentile([], 0.9) is None
    assert percentile([5.0], 0.9) == 5.0
    assert percentile(list(range(1, 11)), 0.9) == 9


def test_unknown_rate_uses_max_timeout_then_hint():
    m = RateMonitor(periods=3.0, max_timeout=1.0)
    assert m.timeout() == 1.0
    assert m.is_stale(0.0)                     # nothing received yet
    m.set_hint(0.1)                            # e.g. LaserScan.scan_time
    assert m.timeout() == pytest.approx(0.3)
    m.set_hint(0.0)                            # driver leaves scan_time empty
    assert m.timeout() == 1.0


@pytest.mark.parametrize('hz', [5.5, 10.0, 40.0])
def test_timeout_follows_measured_rate(hz):
    m = RateMonitor(periods=3.0, max_timeout=1.0)
    last = feed(m, 1.0 / hz, 30)
    assert m.rate() == pytest.approx(hz, rel=1e-6)
    assert m.timeout() == pytest.approx(min(1.0, 3.0 / hz))
    assert not m.is_stale(last + 2.0 / hz)     # one period late: still ok
    assert m.is_stale(last + 4.0 / hz)         # 3 periods missing: STALE


def test_single_dropout_does_not_inflate_timeout():
    m = RateMonitor(periods=3.0, max_timeout=1.0)
    t = feed(m, 0.1, 15)
    m.tick(t + 0.5)                            # 4 scans dropped
    feed(m, 0.1, 5, t0=t + 0.6)
    assert m.timeout() == pytest.approx(0.3)


def test_slow_source_is_capped_and_flagged():
    m = RateMonitor(periods=3.0, max_timeout=1.0)
    feed(m, 0.5, 10)                           # 2 Hz
    assert m.timeout() == 1.0
    assert m.saturated()


def test_recent_stats_p90():
    s = RecentStats(window=10)
    assert s.p90() is None
    for v in [0.1] * 9 + [0.5]:
        s.add(v)
    assert s.p90() == pytest.approx(0.1)
    s.add(0.5)                                 # 2 large values in 10 -> p90 catches them
    assert s.p90() == pytest.approx(0.5)
    assert len(s) == 10
    assert not math.isnan(s.p90())
