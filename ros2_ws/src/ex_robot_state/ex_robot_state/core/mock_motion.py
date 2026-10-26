"""Bounded, smooth multi-frequency joint motion for offline pipeline checks.

q_i(t) = home_i + s(t) * A_i * sin(2*pi*f_i*t + phi_i)

* Distinct, mutually non-integer frequencies per joint so a joint-mapping error
  (e.g. joint2 and joint3 swapped) is visually obvious in RViz and detectable in
  recorded data via the dominant frequency of each channel.
* s(t) is a smoothstep ramp from 0 to 1 over ``ramp_time`` so motion starts at
  home with zero velocity (no step on startup).
* Amplitudes are clipped so home +- A stays inside the joint limits minus a
  margin. This is SIMULATION data only — never a command to hardware.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import List, Sequence

from .urdf_model import JointSpec

DEFAULT_FREQS_HZ = (0.10, 0.13, 0.17, 0.23, 0.29, 0.37)
DEFAULT_AMPS_RAD = tuple(math.radians(a) for a in (30, 20, 25, 35, 30, 45))


@dataclass
class MockMotionConfig:
    home: Sequence[float] = field(default_factory=lambda: [0.0] * 6)
    amplitudes: Sequence[float] = DEFAULT_AMPS_RAD
    frequencies: Sequence[float] = DEFAULT_FREQS_HZ
    phases: Sequence[float] = field(default_factory=lambda: [0.0] * 6)
    ramp_time: float = 2.0
    limit_margin: float = math.radians(10.0)


def _smoothstep(x: float) -> float:
    x = min(max(x, 0.0), 1.0)
    return x * x * (3 - 2 * x)


class MockMotion:
    def __init__(self, joints: Sequence[JointSpec], cfg: MockMotionConfig) -> None:
        self.joints = [j for j in joints if j.is_actuated]
        n = len(self.joints)
        for name in ("home", "amplitudes", "frequencies", "phases"):
            if len(getattr(cfg, name)) != n:
                raise ValueError(f"{name} must have {n} entries")
        self.cfg = cfg
        self.amplitudes: List[float] = []
        for spec, home, amp in zip(self.joints, cfg.home, cfg.amplitudes):
            a = abs(float(amp))
            if not spec.is_continuous and math.isfinite(spec.lower):
                lo = spec.lower + cfg.limit_margin
                hi = spec.upper - cfg.limit_margin
                if not lo <= home <= hi:
                    raise ValueError(f"home {home} of {spec.name} not inside limits-with-margin")
                a = min(a, home - lo, hi - home)
            self.amplitudes.append(a)

    @property
    def names(self) -> List[str]:
        return [j.name for j in self.joints]

    def positions(self, t: float) -> List[float]:
        s = _smoothstep(t / self.cfg.ramp_time) if self.cfg.ramp_time > 0 else 1.0
        return [
            h + s * a * math.sin(2 * math.pi * f * t + p)
            for h, a, f, p in zip(self.cfg.home, self.amplitudes, self.cfg.frequencies, self.cfg.phases)
        ]

    def velocities(self, t: float, eps: float = 1e-4) -> List[float]:
        a, b = self.positions(max(t - eps, 0.0)), self.positions(t + eps)
        span = (t + eps) - max(t - eps, 0.0)
        return [(y - x) / span for x, y in zip(a, b)]
