"""Path-quality metrics for planned joint trajectories (ROS-free)."""

from __future__ import annotations

import math
from typing import Callable, Dict, List, Sequence

import numpy as np


def joint_path_length(Q: np.ndarray) -> float:
    """Sum of Euclidean joint-space step lengths (rad)."""
    if len(Q) < 2:
        return 0.0
    return float(np.linalg.norm(np.diff(Q, axis=0), axis=1).sum())


def cartesian_path_length(P: np.ndarray) -> float:
    if len(P) < 2:
        return 0.0
    return float(np.linalg.norm(np.diff(P, axis=0), axis=1).sum())


def densify(Q: np.ndarray, max_step: float = 0.02) -> np.ndarray:
    """Linearly interpolate so that no joint-space step exceeds max_step (rad).
    Needed for clearance: the planner's waypoints can be far apart in free space."""
    out = [Q[0]]
    for a, b in zip(Q[:-1], Q[1:]):
        n = max(1, int(math.ceil(np.max(np.abs(b - a)) / max_step)))
        for k in range(1, n + 1):
            out.append(a + (b - a) * k / n)
    return np.array(out)


def min_clearance(Q: np.ndarray, link_poses: Callable[[np.ndarray], Dict[str, np.ndarray]],
                  capsules, boxes, box_distance) -> float:
    """Min over the (densified) path of capsule-box distances. inf if no boxes."""
    best = math.inf
    for q in Q:
        poses = link_poses(q)
        for cap in capsules:
            T = poses[cap.link]
            for b in boxes:
                best = min(best, box_distance(T, cap, b.lo, b.hi))
    return best


def summarize(values: Sequence[float]) -> Dict[str, float]:
    v = np.array([x for x in values if x is not None and math.isfinite(x)], dtype=float)
    if len(v) == 0:
        return {"n": 0}
    return {"n": int(len(v)), "median": float(np.median(v)), "mean": float(v.mean()),
            "p10": float(np.percentile(v, 10)), "p90": float(np.percentile(v, 90)),
            "min": float(v.min()), "max": float(v.max())}


def estimate_time_offset(t_plan: np.ndarray, Q_plan: np.ndarray, t_exec: np.ndarray, Q_exec: np.ndarray,
                         search: float = 0.5, step: float = 0.002) -> float:
    """Shift d (s) minimising RMS |q_exec(t) - q_plan(t - d)| - i.e. the delay between
    the planned timeline and what the controller actually did (start latency)."""
    best_d, best_e = 0.0, math.inf
    for d in np.arange(-search, search + step / 2, step):
        m = (t_exec - d >= t_plan[0]) & (t_exec - d <= t_plan[-1])
        if m.sum() < 5:
            continue
        Qp = np.stack([np.interp(t_exec[m] - d, t_plan, Q_plan[:, j]) for j in range(Q_plan.shape[1])], axis=1)
        e = float(np.sqrt(((Q_exec[m] - Qp) ** 2).mean()))
        if e < best_e:
            best_d, best_e = float(d), e
    return best_d


def tracking_error(t_plan: np.ndarray, Q_plan: np.ndarray, t_exec: np.ndarray, Q_exec: np.ndarray) -> Dict[str, List[float]]:
    """Per-joint RMS / max |q_exec(t) - q_plan(t)|, planned trajectory linearly
    interpolated at the execution sample times (inside the planned time span)."""
    m = (t_exec >= t_plan[0]) & (t_exec <= t_plan[-1])
    if m.sum() == 0:
        return {"rms": [], "max": [], "n": 0}
    te, Qe = t_exec[m], Q_exec[m]
    Qp = np.stack([np.interp(te, t_plan, Q_plan[:, j]) for j in range(Q_plan.shape[1])], axis=1)
    e = Qe - Qp
    return {"rms": np.sqrt((e ** 2).mean(axis=0)).tolist(), "max": np.abs(e).max(axis=0).tolist(), "n": int(m.sum())}
