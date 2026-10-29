"""Baselines for the reach task (no learning): the bar any learned policy must beat.

  random      uniform actions
  dls_ik      damped-least-squares Jacobian controller: dq = J^T (J J^T + lambda^2 I)^-1 k (goal - tcp),
              velocity-limited - the classical controller a robotics engineer writes first
  apf         dls_ik + artificial potential field repulsion (Khatib): the nearest robot point to an
              obstacle is pushed away along the clearance normal through its point Jacobian
              (wall task only; known to get stuck in local minima)
"""

from __future__ import annotations

import json
import time
from pathlib import Path
from typing import Callable, Dict, List

import numpy as np

from .envs import TASKS
from .envs.cr10a_reach import ReachEnv


def random_policy(env: ReachEnv, rng: np.random.Generator) -> Callable:
    return lambda obs: rng.uniform(-1, 1, 6)


def dls_policy(env: ReachEnv, gain: float = 3.0, damping: float = 0.05) -> Callable:
    def act(obs):
        err = obs[18:21]
        J = env.jacobian()
        dq = J.T @ np.linalg.solve(J @ J.T + damping ** 2 * np.eye(3), gain * err)
        a = dq / env.cfg.max_joint_speed
        m = np.max(np.abs(a))
        return a / m if m > 1 else a
    return act


def apf_policy(env, gain: float = 3.0, damping: float = 0.05, k_rep: float = 0.02, d0: float = 0.10) -> Callable:
    # k_rep, d0: grid-searched on TUNING seeds 0..19 (80 % success, 0 collisions); evaluation uses seeds 1000+
    """Repulsion from EVERY link capsule within d0 of an obstacle (its closest obstacle point),
    mapped to joint space through the point Jacobian, summed with the DLS attraction."""
    import mujoco
    att = dls_policy(env, gain, damping)
    jp = np.zeros((3, env.m.nv))

    def act(obs):
        dq = att(obs) * env.cfg.max_joint_speed
        for d, p_robot, n, body in env.repulsion_points(d0):
            mujoco.mj_jac(env.m, env.d, jp, None, p_robot, body)
            dd = max(d, 0.005)
            dq = dq + jp[:, env.dadr].T @ (k_rep * (1.0 / dd - 1.0 / d0) / dd ** 2 * n)
        a = dq / env.cfg.max_joint_speed
        m = np.max(np.abs(a))
        return a / m if m > 1 else a
    return act


def joint_interp_policy(env, gain: float = 3.0) -> Callable:
    """DIAGNOSTIC, PRIVILEGED: knows the goal joint configuration (env.q_goal) and moves straight to it
    in joint space. Not a competitor (no policy gets q_goal); it measures how many episodes a naive
    joint-space path already solves, i.e. how hard the episodes are."""
    def act(obs):
        a = gain * (env.q_goal - env.q()) / env.cfg.max_joint_speed
        m = np.max(np.abs(a))
        return a / m if m > 1 else a
    return act


def evaluate(env: ReachEnv, make_policy: Callable[[ReachEnv], Callable], episodes: int, seed: int) -> Dict:
    results: List[Dict] = []
    for ep in range(episodes):
        obs, info = env.reset(seed=seed + ep)
        policy = make_policy(env)
        path, steps_to_success, limit_hits, min_clear = 0.0, None, 0, float("inf")
        prev = env.tcp()
        for k in range(env.cfg.max_steps):
            obs, r, term, trunc, inf = env.step(policy(obs))
            path += float(np.linalg.norm(env.tcp() - prev))
            prev = env.tcp()
            limit_hits += int(inf["limit_hit"])
            min_clear = min(min_clear, inf.get("clearance", float("inf")))
            if inf["success"] and steps_to_success is None:
                steps_to_success = k + 1
            if term or trunc:
                break
        results.append({"success": bool(inf["success"]), "final_dist_m": inf["dist"], "path_m": path,
                        "time_to_reach_s": None if steps_to_success is None else steps_to_success * env.cfg.control_dt,
                        "limit_hit_steps": limit_hits, "collision": bool(inf.get("collision", False)),
                        "min_clearance_m": None if min_clear == float("inf") else min_clear})
    succ = [r for r in results if r["success"]]
    return {"episodes": episodes, "success_rate": len(succ) / episodes,
            "final_dist_m_median": float(np.median([r["final_dist_m"] for r in results])),
            "time_to_reach_s_median": float(np.median([r["time_to_reach_s"] for r in succ])) if succ else None,
            "path_m_median": float(np.median([r["path_m"] for r in results])),
            "limit_hit_episodes": sum(r["limit_hit_steps"] > 0 for r in results),
            "collision_rate": sum(r["collision"] for r in results) / episodes, "per_episode": results}


def main() -> None:
    import argparse
    ap = argparse.ArgumentParser()
    ap.add_argument("--episodes", type=int, default=50)
    ap.add_argument("--seed", type=int, default=1000)
    ap.add_argument("--task", choices=TASKS, default="reach")
    ap.add_argument("--out", default=None)
    a = ap.parse_args()
    a.out = a.out or f"data/reports/learning/baselines_{a.task}.json"
    rng = np.random.default_rng(0)
    rep = {}
    policies = [("random", lambda e: random_policy(e, rng)), ("dls_ik", dls_policy)]
    from .envs import make_env
    env = make_env(a.task)
    if a.task != "reach":
        policies.append(("apf", apf_policy))
        policies.append(("joint_interp_PRIVILEGED", joint_interp_policy))
    for name, mk in policies:
        t0 = time.time()
        rep[name] = evaluate(env, mk, a.episodes, a.seed)
        rep[name]["wall_s"] = round(time.time() - t0, 1)
        r = rep[name]
        print(f"{name:8s} success {r['success_rate']*100:5.1f}%  final dist median {r['final_dist_m_median']*1000:7.1f} mm  "
              f"time-to-reach {r['time_to_reach_s_median']}  path {r['path_m_median']:.2f} m  limit-hit eps {r['limit_hit_episodes']}  "
              f"collisions {r['collision_rate']*100:.0f}%  ({r['wall_s']} s)")
    Path(a.out).parent.mkdir(parents=True, exist_ok=True)
    Path(a.out).write_text(json.dumps(rep, indent=1))


if __name__ == "__main__":
    main()
