"""JointState validation and canonicalisation.

The adapter between ``/joint_states_robot`` (raw, from a driver or bag) and
``/joint_states`` (consumed by robot_state_publisher) runs every sample through
:class:`JointStateValidator`. Policy (enforced by the node, not here):

* ERROR  -> sample is NOT forwarded; display freezes on the last valid state and
            is marked invalid in diagnostics.
* WARN   -> sample is forwarded, issue reported.
"""

from __future__ import annotations

import enum
import math
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Sequence

from .urdf_model import JointSpec

TWO_PI = 2.0 * math.pi


class Level(enum.IntEnum):
    OK = 0
    WARN = 1
    ERROR = 2


@dataclass(frozen=True)
class Issue:
    level: Level
    code: str
    message: str
    joint: Optional[str] = None


@dataclass
class ValidationResult:
    issues: List[Issue] = field(default_factory=list)
    # positions in canonical (URDF chain) order; None if not reconstructible
    positions: Optional[List[float]] = None

    @property
    def level(self) -> Level:
        return max((i.level for i in self.issues), default=Level.OK)

    @property
    def accepted(self) -> bool:
        return self.level < Level.ERROR and self.positions is not None

    def add(self, level: Level, code: str, message: str, joint: Optional[str] = None) -> None:
        self.issues.append(Issue(level, code, message, joint))


def wrap_to_pi(a: float) -> float:
    return (a + math.pi) % TWO_PI - math.pi


@dataclass
class ValidatorConfig:
    limit_tolerance: float = math.radians(0.5)  # encoder noise at a hard limit
    near_limit_margin: float = math.radians(5.0)
    # Any |q| above this is almost certainly degrees, not radians. CR-series
    # joints span about +-360 deg, so a radian value can legitimately reach
    # ~6.28; anything above 2*pi + slack is suspicious.
    degree_suspect_threshold: float = TWO_PI + 0.2
    # Max plausible |dq| between consecutive samples = velocity_limit * dt * factor
    velocity_factor: float = 1.5
    # The official CR10 URDF declares velocity="100" rad/s (a placeholder), which
    # would make jump detection useless. When set, this replaces URDF velocity.
    velocity_limit_override: Optional[float] = math.radians(225.0)
    max_jump_floor: float = math.radians(2.0)  # tolerate quantisation at tiny dt
    resync_gap: float = 1.0  # after this gap, a large change is RESYNC, not JUMP


class JointStateValidator:
    def __init__(self, joints: Sequence[JointSpec], config: Optional[ValidatorConfig] = None) -> None:
        self.joints = [j for j in joints if j.is_actuated]
        if not self.joints:
            raise ValueError("no actuated joints to validate")
        self.order = [j.name for j in self.joints]
        self.cfg = config or ValidatorConfig()
        self._last_positions: Optional[List[float]] = None
        self._last_time: Optional[float] = None
        self._last_stamp: Optional[float] = None

    def reset_reference(self) -> None:
        self._last_positions = None
        self._last_time = None
        self._last_stamp = None

    def validate(
        self,
        names: Sequence[str],
        positions: Sequence[float],
        recv_time: float,
        stamp: Optional[float] = None,
    ) -> ValidationResult:
        r = ValidationResult()
        cfg = self.cfg

        # ---- structure ------------------------------------------------------
        if len(names) != len(positions):
            r.add(Level.ERROR, "LENGTH_MISMATCH", f"{len(names)} names vs {len(positions)} positions")
            return r
        if len(set(names)) != len(names):
            r.add(Level.ERROR, "DUPLICATE_NAMES", f"duplicate joint names: {list(names)}")
            return r
        by_name: Dict[str, float] = dict(zip(names, positions))
        missing = [n for n in self.order if n not in by_name]
        extra = [n for n in names if n not in self.order]
        if missing:
            r.add(Level.ERROR, "MISSING_JOINTS", f"missing {missing}; got {list(names)}")
            return r
        if extra:
            r.add(Level.WARN, "EXTRA_JOINTS", f"ignoring unknown joints {extra}")
        if list(names[: len(self.order)]) != self.order:
            # reordering is fine (JointState is name-keyed), but worth knowing
            r.add(Level.OK, "REORDERED", f"input order {list(names)} != URDF order {self.order}")

        q = [float(by_name[n]) for n in self.order]

        # ---- values ---------------------------------------------------------
        bad = False
        for spec, v in zip(self.joints, q):
            if not math.isfinite(v):
                r.add(Level.ERROR, "NON_FINITE", f"{spec.name}={v}", spec.name)
                bad = True
                continue
            if abs(v) > cfg.degree_suspect_threshold:
                r.add(
                    Level.ERROR,
                    "UNIT_SUSPECT_DEGREES",
                    f"{spec.name}={v:.3f} exceeds {cfg.degree_suspect_threshold:.2f}; degrees sent as radians?",
                    spec.name,
                )
                bad = True
                continue
            if not spec.is_continuous and math.isfinite(spec.lower):
                if v < spec.lower - cfg.limit_tolerance or v > spec.upper + cfg.limit_tolerance:
                    r.add(
                        Level.ERROR,
                        "OUT_OF_LIMITS",
                        f"{spec.name}={math.degrees(v):.2f} deg outside "
                        f"[{math.degrees(spec.lower):.1f}, {math.degrees(spec.upper):.1f}]",
                        spec.name,
                    )
                    bad = True
                elif v < spec.lower + cfg.near_limit_margin or v > spec.upper - cfg.near_limit_margin:
                    r.add(Level.WARN, "NEAR_LIMIT", f"{spec.name}={math.degrees(v):.2f} deg", spec.name)
        if bad:
            return r

        # ---- temporal -------------------------------------------------------
        if stamp is not None:
            if stamp == 0.0:
                r.add(Level.WARN, "ZERO_STAMP", "header.stamp is zero")
            elif self._last_stamp is not None and stamp < self._last_stamp:
                r.add(Level.WARN, "STAMP_REGRESSED", f"stamp went back {self._last_stamp - stamp:.4f}s")

        if self._last_positions is not None and self._last_time is not None:
            dt = recv_time - self._last_time
            if dt > cfg.resync_gap:
                r.add(Level.WARN, "RESYNC", f"{dt:.2f}s gap since last accepted sample")
            else:
                for spec, v, prev in zip(self.joints, q, self._last_positions):
                    d = abs(wrap_to_pi(v - prev)) if spec.is_continuous else abs(v - prev)
                    if cfg.velocity_limit_override is not None:
                        vmax = cfg.velocity_limit_override
                    else:
                        vmax = spec.velocity if math.isfinite(spec.velocity) else math.radians(360)
                    allowed = max(cfg.max_jump_floor, vmax * max(dt, 0.0) * cfg.velocity_factor)
                    if d > allowed:
                        r.add(
                            Level.ERROR,
                            "JUMP",
                            f"{spec.name} moved {math.degrees(d):.2f} deg in {dt * 1000:.1f} ms "
                            f"(allowed {math.degrees(allowed):.2f})",
                            spec.name,
                        )
                if r.level >= Level.ERROR:
                    return r

        r.positions = q
        self._last_positions = q
        self._last_time = recv_time
        if stamp is not None and stamp != 0.0:
            self._last_stamp = stamp
        return r
