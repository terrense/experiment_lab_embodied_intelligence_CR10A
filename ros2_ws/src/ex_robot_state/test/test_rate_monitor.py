import math

import pytest

from ex_robot_state.core.rate_monitor import Freshness, RateMonitor


def test_rate_and_freshness():
    m = RateMonitor(stale_after=0.1, lost_after=1.0)
    assert m.freshness(0.0) == Freshness.NEVER_RECEIVED
    for i in range(126):
        m.tick(i * 0.008)
    s = m.stats(1.0)
    assert s.rate_hz == pytest.approx(125.0)
    assert s.mean_dt == pytest.approx(0.008) and s.std_dt == pytest.approx(0.0, abs=1e-12)
    assert m.freshness(1.0 + 0.05) == Freshness.FRESH
    assert m.freshness(1.0 + 0.2) == Freshness.STALE
    assert m.freshness(1.0 + 2.0) == Freshness.LOST


def test_gap_shows_in_max_dt():
    m = RateMonitor()
    for t in (0.0, 0.01, 0.02, 0.5, 0.51):
        m.tick(t)
    assert m.stats(0.51).max_dt == pytest.approx(0.48)


def test_backwards_time_rejected():
    m = RateMonitor()
    m.tick(1.0)
    with pytest.raises(ValueError):
        m.tick(0.5)


def test_bad_thresholds():
    with pytest.raises(ValueError):
        RateMonitor(stale_after=1.0, lost_after=0.5)
