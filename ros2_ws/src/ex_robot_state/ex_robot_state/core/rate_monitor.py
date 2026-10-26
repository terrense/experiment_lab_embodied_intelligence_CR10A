"""Stream health: rate, inter-arrival jitter, staleness.

Times are passed in explicitly (seconds, monotonic clock) so the logic is
deterministic under test; the ROS node feeds it ``time.monotonic()`` on receive,
which deliberately measures *host receive* timing, independent of whatever the
message header stamp claims.
"""

from __future__ import annotations

import enum
import math
from collections import deque
from dataclasses import dataclass
from typing import Deque, Optional


class Freshness(enum.Enum):
    NEVER_RECEIVED = "NEVER_RECEIVED"
    FRESH = "FRESH"
    STALE = "STALE"  # late, but may recover (e.g. one dropped packet)
    LOST = "LOST"  # treat as disconnected; freeze display + mark invalid


@dataclass(frozen=True)
class RateStats:
    count: int
    rate_hz: float
    mean_dt: float
    max_dt: float
    std_dt: float
    age: float  # seconds since last sample, inf if none
    freshness: Freshness


class RateMonitor:
    def __init__(self, stale_after: float = 0.1, lost_after: float = 1.0, window: int = 200) -> None:
        if not 0 < stale_after < lost_after:
            raise ValueError("require 0 < stale_after < lost_after")
        self.stale_after = stale_after
        self.lost_after = lost_after
        self._stamps: Deque[float] = deque(maxlen=window)
        self.total = 0

    def tick(self, now: float) -> None:
        if self._stamps and now < self._stamps[-1]:
            raise ValueError("monotonic time went backwards")
        self._stamps.append(now)
        self.total += 1

    def last(self) -> Optional[float]:
        return self._stamps[-1] if self._stamps else None

    def reset(self) -> None:
        self._stamps.clear()

    def freshness(self, now: float) -> Freshness:
        last = self.last()
        if last is None:
            return Freshness.NEVER_RECEIVED
        age = now - last
        if age >= self.lost_after:
            return Freshness.LOST
        if age >= self.stale_after:
            return Freshness.STALE
        return Freshness.FRESH

    def stats(self, now: float) -> RateStats:
        s = list(self._stamps)
        last = self.last()
        age = (now - last) if last is not None else math.inf
        if len(s) < 2:
            return RateStats(len(s), 0.0, math.nan, math.nan, math.nan, age, self.freshness(now))
        dts = [b - a for a, b in zip(s, s[1:])]
        mean = sum(dts) / len(dts)
        var = sum((d - mean) ** 2 for d in dts) / len(dts)
        rate = (len(s) - 1) / (s[-1] - s[0]) if s[-1] > s[0] else math.inf
        return RateStats(len(s), rate, mean, max(dts), math.sqrt(var), age, self.freshness(now))
