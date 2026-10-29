"""Classical reference for the window task: RRT-Connect in joint space + path following (SIMULATION ONLY).

Uses the SAME collision model as the environment (env.clearance(), conservative mesh point clouds) and
the goal joint configuration of the episode (a planner gets it from IK; here the episode's sampled goal
configuration plays that role - privileged information the RL policy does not get).

  plan:    RRT-Connect (Kuffner & LaValle 2000), states valid if clearance > `margin`, edges checked
           every `edge_res` rad (max-norm), random shortcut smoothing; budget = number of collision checks
  execute: the path is followed through the env (servo dynamics, 50 Hz) by a waypoint tracker,
           so planning success and *executed* success are reported separately
"""

from __future__ import annotations

import math
import time
from typing import Callable, List, Optional, Tuple

import numpy as np


class JointRRTConnect:
    def __init__(self, env, margin: float = 0.005, step: float = 0.25, edge_res: float = 0.015,
                 max_checks: int = 60_000, rng: Optional[np.random.Generator] = None) -> None:
        # budget in collision CHECKS, not seconds: independent of CPU load, reproducible
        self.env, self.margin, self.step, self.edge_res, self.max_checks = env, margin, step, edge_res, max_checks
        self.rng = rng or np.random.default_rng(0)
        lo, hi = env.lo.copy(), env.hi.copy()
        lo[[0, 3, 5]] = np.maximum(lo[[0, 3, 5]], [-math.pi / 2, -math.pi, -math.pi])
        hi[[0, 3, 5]] = np.minimum(hi[[0, 3, 5]], [math.pi / 2, math.pi, math.pi])
        self.lo, self.hi = lo, hi
        self.checks = 0

    def valid(self, q: np.ndarray) -> bool:
        self.checks += 1
        if np.any(q < self.env.lo) or np.any(q > self.env.hi):
            return False
        self.env._set_q(q)
        return self.env.clearance() > self.margin

    def edge_valid(self, a: np.ndarray, b: np.ndarray) -> bool:
        n = int(math.ceil(np.max(np.abs(b - a)) / self.edge_res))
        return all(self.valid(a + (b - a) * (i / n)) for i in range(1, n + 1))

    def _extend(self, nodes: List[np.ndarray], parents: List[int], target: np.ndarray) -> Tuple[str, int]:
        i = int(np.argmin([np.max(np.abs(n - target)) for n in nodes]))
        d = target - nodes[i]
        dist = np.max(np.abs(d))
        new = target if dist <= self.step else nodes[i] + d * (self.step / dist)
        if not self.edge_valid(nodes[i], new):
            return "trapped", -1
        nodes.append(new)
        parents.append(i)
        return ("reached" if dist <= self.step else "advanced"), len(nodes) - 1

    def plan(self, q_start: np.ndarray, q_goal: np.ndarray) -> Optional[List[np.ndarray]]:
        if not (self.valid(q_start) and self.valid(q_goal)):
            return None
        A, pa, B, pb = [q_start.copy()], [-1], [q_goal.copy()], [-1]
        a_is_start = True
        while self.checks < self.max_checks:
            q_rand = self.rng.uniform(self.lo, self.hi)
            status, ia = self._extend(A, pa, q_rand)
            if status != "trapped":
                while True:  # connect B towards the new node
                    status_b, ib = self._extend(B, pb, A[ia])
                    if status_b != "advanced":
                        break
                if status_b == "reached":  # B's last node == A[ia]: the trees meet there
                    from_a = self._trace(A, pa, ia)   # meeting node -> root of A
                    from_b = self._trace(B, pb, ib)   # meeting node -> root of B
                    start_half, goal_half = (from_a, from_b) if a_is_start else (from_b, from_a)
                    return self.shortcut(start_half[::-1] + goal_half[1:])
            A, pa, B, pb = B, pb, A, pa
            a_is_start = not a_is_start
        return None

    @staticmethod
    def _trace(nodes, parents, i) -> List[np.ndarray]:
        out = []
        while i != -1:
            out.append(nodes[i])
            i = parents[i]
        return out

    def shortcut(self, path: List[np.ndarray], tries: int = 60) -> List[np.ndarray]:
        path = list(path)
        for _ in range(tries):
            if len(path) < 3:
                break
            i, j = sorted(self.rng.choice(len(path), 2, replace=False))
            if j - i < 2:
                continue
            if self.edge_valid(path[i], path[j]):
                path = path[:i + 1] + path[j:]
        return path


def path_follower(path: List[np.ndarray], env, speed: float = 0.6, tol: float = 0.03) -> Callable:
    """Waypoint tracker: joint velocity toward the current waypoint, at most `speed` rad/s."""
    state = {"k": 1}

    def act(obs):
        q = env.q()
        k = state["k"]
        while k < len(path) - 1 and np.max(np.abs(path[k] - q)) < tol:
            k += 1
        state["k"] = k
        d = path[k] - q
        m = np.max(np.abs(d))
        v = d / m * min(speed, 3.0 * m) if m > 1e-9 else np.zeros(6)
        return v / env.cfg.max_joint_speed
    return act


def evaluate_planner(env, episodes: int, seed: int, max_checks: int = 60_000) -> dict:
    from .baselines import evaluate
    plans = {}

    def make(e):
        ep_q0, ep_qg = e.q().copy(), e.q_goal.copy()
        planner = JointRRTConnect(e, max_checks=max_checks, rng=np.random.default_rng(len(plans)))
        t0 = time.time()
        path = planner.plan(ep_q0, ep_qg)
        dt = time.time() - t0
        e._set_q(ep_q0)  # planning moved the model: restore the episode start exactly
        e.q_ref = ep_q0.copy()
        plans[len(plans)] = {"solved": path is not None, "plan_s": round(dt, 2), "checks": planner.checks,
                             "waypoints": 0 if path is None else len(path)}
        if path is None:
            return lambda obs: np.zeros(6)  # no plan: stay
        return path_follower(path, e)

    rep = evaluate(env, make, episodes, seed)
    solved = [p for p in plans.values() if p["solved"]]
    rep.update({"plan_success_rate": len(solved) / episodes,
                "plan_time_s_median": float(np.median([p["plan_s"] for p in solved])) if solved else None,
                "plans": list(plans.values()), "max_checks": max_checks, "margin_m": 0.005})
    return rep


def main(argv=None) -> int:
    import argparse
    import json
    from pathlib import Path
    from .envs import make_env
    ap = argparse.ArgumentParser()
    ap.add_argument("--task", default="window")
    ap.add_argument("--episodes", type=int, default=50)
    ap.add_argument("--seed", type=int, default=1000)
    ap.add_argument("--max-checks", type=int, default=60_000)
    ap.add_argument("--exec-steps", type=int, default=None, help="episode length for EXECUTION (default: the task's)")
    ap.add_argument("--out", default="../data/reports/learning/rrt_window.json")
    a = ap.parse_args(argv)
    env = make_env(a.task)
    if a.exec_steps:
        env.cfg.max_steps = a.exec_steps
    rep = evaluate_planner(env, a.episodes, a.seed, a.max_checks)
    rep["exec_steps"] = env.cfg.max_steps
    Path(a.out).write_text(json.dumps(rep, indent=1))
    print(f"RRT-Connect ({a.max_checks} checks): planned {rep['plan_success_rate']*100:.0f}%  executed success "
          f"{rep['success_rate']*100:.0f}%  collisions {rep['collision_rate']*100:.0f}%  "
          f"final dist median {rep['final_dist_m_median']*1000:.0f} mm  plan time median {rep['plan_time_s_median']} s")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
