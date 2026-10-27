"""Excitation trajectory for dynamics identification: small, slow, multi-frequency joint
motion around the robot's CURRENT pose, as a list of MovJ waypoints. Validated before any
command is allowed out (pure function, unit tested).

q_k = q0 + A * sin(2 pi f_i t_k)  (per joint distinct f_i -> joints separable in the data)
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Callable, Dict, List, Sequence

import numpy as np

FREQS = (0.05, 0.07, 0.09, 0.11, 0.13, 0.17)  # cycles per "waypoint time unit"
# Proximal joints swing the whole arm (J1 +-15 deg moves the TCP > 0.35 m at full reach), so they get
# smaller amplitudes than the wrist; the plan is still rejected if the envelope is violated.
DEFAULT_AMPLITUDES_DEG = (8.0, 10.0, 10.0, 15.0, 15.0, 15.0)


@dataclass
class SafetyEnvelope:
    max_joint_excursion_deg: float = 20.0  # from the start pose, per joint
    min_tcp_z_m: float = 0.25              # above the base mounting plane
    min_radius_m: float = 0.30             # TCP distance from the J1 axis (keeps it off the base/column)
    max_tcp_excursion_m: float = 0.35      # from the start TCP
    joint_limit_margin_deg: float = 10.0
    checked_links: List[str] = field(default_factory=lambda: ["Link3", "Link4", "Link5", "Link6", "tcp"])


@dataclass
class ExcitationPlan:
    q0_deg: List[float]
    waypoints_deg: List[List[float]]
    amplitudes_deg: List[float]
    report: Dict[str, float]


def make_plan(q0_deg: Sequence[float], amplitude_deg=DEFAULT_AMPLITUDES_DEG, n_waypoints: int = 60,
              cycles: float = 1.0) -> ExcitationPlan:
    q0 = np.asarray(q0_deg, float)
    amps = np.broadcast_to(np.asarray(amplitude_deg, float), (6,)).copy()
    # common time axis: the slowest joint completes `cycles` periods over n waypoints
    t = np.arange(1, n_waypoints + 1) / n_waypoints * cycles / min(FREQS)
    W = q0[None, :] + amps[None, :] * np.sin(2 * math.pi * np.outer(t, FREQS))
    W = np.vstack([W, q0])  # end where we started
    return ExcitationPlan(list(q0), W.tolist(), amps.tolist(), {})


def validate_plan(plan: ExcitationPlan, link_positions: Callable[[Sequence[float]], Dict[str, np.ndarray]],
                  lower_deg: Sequence[float], upper_deg: Sequence[float], env: SafetyEnvelope,
                  interp_steps: int = 10) -> List[str]:
    """Return a list of violations (empty = OK). Checks every waypoint AND linear joint-space
    interpolation between them (MovJ moves along a joint-space path)."""
    problems: List[str] = []
    q0 = np.asarray(plan.q0_deg)
    lo = np.asarray(lower_deg) + env.joint_limit_margin_deg
    hi = np.asarray(upper_deg) - env.joint_limit_margin_deg
    tcp0 = link_positions(q0)["tcp"]
    pts = [q0] + [np.asarray(w) for w in plan.waypoints_deg]
    for i in range(1, len(pts)):
        for s in np.linspace(0, 1, interp_steps + 1)[1:]:
            q = pts[i - 1] + s * (pts[i] - pts[i - 1])
            tag = f"waypoint {i} (s={s:.1f})"
            if np.any(np.abs(q - q0) > env.max_joint_excursion_deg + 1e-9):
                problems.append(f"{tag}: joint excursion > {env.max_joint_excursion_deg} deg")
            if np.any(q < lo) or np.any(q > hi):
                problems.append(f"{tag}: within {env.joint_limit_margin_deg} deg of a joint limit")
            P = link_positions(q)
            for name in env.checked_links:
                p = P[name]
                if p[2] < env.min_tcp_z_m:
                    problems.append(f"{tag}: {name} z={p[2]:.3f} m < {env.min_tcp_z_m}")
                if math.hypot(p[0], p[1]) < env.min_radius_m and name == "tcp":
                    problems.append(f"{tag}: tcp radius {math.hypot(p[0], p[1]):.3f} m < {env.min_radius_m}")
            if np.linalg.norm(P["tcp"] - tcp0) > env.max_tcp_excursion_m:
                problems.append(f"{tag}: tcp moved {np.linalg.norm(P['tcp'] - tcp0):.3f} m > {env.max_tcp_excursion_m}")
            if len(problems) > 20:
                return problems
    return problems
