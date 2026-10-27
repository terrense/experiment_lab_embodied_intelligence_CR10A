"""Baselines for the reach task (no learning): the bar any learned policy must beat.

  random      uniform actions
  dls_ik      damped-least-squares Jacobian controller: dq = J^T (J J^T + lambda^2 I)^-1 k (goal - tcp),
              velocity-limited - the classical controller a robotics engineer writes first
"""

from __future__ import annotations

import json
import time
from pathlib import Path
from typing import Callable, Dict, List

import numpy as np

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


def evaluate(env: ReachEnv, make_policy: Callable[[ReachEnv], Callable], episodes: int, seed: int) -> Dict:
    results: List[Dict] = []
    for ep in range(episodes):
        obs, info = env.reset(seed=seed + ep)
        policy = make_policy(env)
        path, steps_to_success, limit_hits = 0.0, None, 0
        prev = env.tcp()
        for k in range(env.cfg.max_steps):
            obs, r, term, trunc, inf = env.step(policy(obs))
            path += float(np.linalg.norm(env.tcp() - prev))
            prev = env.tcp()
            limit_hits += int(inf["limit_hit"])
            if inf["success"] and steps_to_success is None:
                steps_to_success = k + 1
            if term or trunc:
                break
        results.append({"success": bool(inf["success"]), "final_dist_m": inf["dist"], "path_m": path,
                        "time_to_reach_s": None if steps_to_success is None else steps_to_success * env.cfg.control_dt,
                        "limit_hit_steps": limit_hits})
    succ = [r for r in results if r["success"]]
    return {"episodes": episodes, "success_rate": len(succ) / episodes,
            "final_dist_m_median": float(np.median([r["final_dist_m"] for r in results])),
            "time_to_reach_s_median": float(np.median([r["time_to_reach_s"] for r in succ])) if succ else None,
            "path_m_median": float(np.median([r["path_m"] for r in results])),
            "limit_hit_episodes": sum(r["limit_hit_steps"] > 0 for r in results), "per_episode": results}


def main() -> None:
    import argparse
    ap = argparse.ArgumentParser()
    ap.add_argument("--episodes", type=int, default=50)
    ap.add_argument("--seed", type=int, default=1000)
    ap.add_argument("--out", default="data/reports/learning/baselines.json")
    a = ap.parse_args()
    env = ReachEnv()
    rng = np.random.default_rng(0)
    rep = {}
    for name, mk in (("random", lambda e: random_policy(e, rng)), ("dls_ik", dls_policy)):
        t0 = time.time()
        rep[name] = evaluate(env, mk, a.episodes, a.seed)
        rep[name]["wall_s"] = round(time.time() - t0, 1)
        r = rep[name]
        print(f"{name:8s} success {r['success_rate']*100:5.1f}%  final dist median {r['final_dist_m_median']*1000:7.1f} mm  "
              f"time-to-reach {r['time_to_reach_s_median']}  path {r['path_m_median']:.2f} m  limit-hit eps {r['limit_hit_episodes']}  ({r['wall_s']} s)")
    Path(a.out).parent.mkdir(parents=True, exist_ok=True)
    Path(a.out).write_text(json.dumps(rep, indent=1))


if __name__ == "__main__":
    main()
