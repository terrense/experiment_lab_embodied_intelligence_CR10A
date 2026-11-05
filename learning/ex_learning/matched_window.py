"""MATCHED comparison on the narrow-passage (window) scene (SIMULATION ONLY).

The archived comparison "RRT-Connect 38 % vs PPO 0 %" is not controlled: the planner was given
the goal JOINT configuration (privileged - its own docstring says so) and 30 s of execution
against the reactive policies' 6 s, and it used a different collision model.

This module runs every method under the same conditions:

  * the same environment, the same collision model, the same start and goal pools, the same
    success criterion and the same execution deadline for everybody;
  * the same INFORMATION: the episode's CARTESIAN goal. The planner may not use the privileged
    goal joint configuration - it has to realise the Cartesian goal itself, by picking the pool
    member whose TCP is closest to it (an explicit, cheap stand-in for IK, and the honest
    statement of what a deployed system would have to do);
  * the planning time is reported separately AND added to the time-to-success, so a planner that
    needs 21 s to plan cannot hide that.

    cd learning && ~/ex_rlenv/bin/python -m ex_learning.matched_window --episodes 50
"""

from __future__ import annotations

import argparse
import json
import time
from pathlib import Path
from typing import Callable, Dict, List

import numpy as np

from .baselines import apf_policy, dls_policy, evaluate
from .envs import make_env
from .planner import JointRRTConnect, path_follower


def _planner_with_goal_realisation(env, q_goal_pool: np.ndarray, cartesian_goal: np.ndarray,
                                   max_checks: int, rng_seed: int):
    """RRT-Connect whose goal configuration is REALISED from the Cartesian goal.

    The archived planner used the episode's privileged goal joint configuration. Here the planner
    only gets the Cartesian goal and has to pick a configuration that reaches it - the pool member
    with the closest TCP. That is the step a deployed system would perform with IK, and it is
    visible in the numbers instead of being hidden.
    """
    tcp_of = []
    for q in q_goal_pool:
        env._set_q(q)
        tcp_of.append(env.tcp().copy())
    tcp_of = np.array(tcp_of)
    idx = int(np.argmin(np.linalg.norm(tcp_of - cartesian_goal[None, :], axis=1)))
    q_goal = q_goal_pool[idx]
    env._set_q(q_goal)
    goal_error_m = float(np.linalg.norm(env.tcp() - cartesian_goal))
    planner = JointRRTConnect(env, max_checks=max_checks, rng=np.random.default_rng(rng_seed))
    return planner, q_goal, goal_error_m


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--episodes", type=int, default=50)
    ap.add_argument("--seed", type=int, default=1000)
    ap.add_argument("--deadline-s", type=float, default=30.0,
                    help="the execution deadline EVERY method gets (the reactive policies are given "
                         "the same deadline, so the comparison is about the method, not the clock)")
    ap.add_argument("--max-checks", type=int, default=60_000)
    ap.add_argument("--out", type=Path, default=Path("../data/reports/learning/window_matched.json"))
    a = ap.parse_args(argv)

    env = make_env("window")
    steps = int(round(a.deadline_s / env.cfg.control_dt))
    env.cfg.max_steps = steps

    results: Dict[str, Dict] = {}

    # --- the reactive methods: Cartesian goal only, same deadline -----------------------
    for name, mk in (("dls_ik", dls_policy), ("apf", apf_policy)):
        t0 = time.time()
        rep = evaluate(env, mk, a.episodes, a.seed)
        rep["wall_s"] = round(time.time() - t0, 1)
        rep["goal_information"] = "cartesian"
        rep["deadline_s"] = a.deadline_s
        results[name] = rep
        print(f"{name:14s} success {rep['success_rate']*100:5.1f}%  collisions {rep['collision_rate']*100:5.1f}%  "
              f"final {rep['final_dist_m_median']*1000:6.0f} mm")

    # --- the planner: same deadline, same Cartesian goal, planning time counted ---------
    plans: Dict[str, Dict] = {}

    def make_planner(e):
        ep_q0 = e.q().copy()
        # the Cartesian goal of THIS episode (the same information the reactive methods get)
        cartesian_goal = e.goal.copy()
        planner, q_goal, goal_err = _planner_with_goal_realisation(e, e.goal_pool, cartesian_goal,
                                                                   a.max_checks, len(plans))
        t0 = time.time()
        path = planner.plan(ep_q0, q_goal)
        plan_s = time.time() - t0
        e._set_q(ep_q0)
        e.q_ref = ep_q0.copy()
        plans[len(plans)] = {"solved": path is not None, "plan_s": round(plan_s, 2),
                             "checks": planner.checks, "goal_error_m": round(goal_err, 4),
                             "waypoints": 0 if path is None else len(path)}
        if path is None:
            return lambda obs: np.zeros(6)
        return path_follower(path, e)

    t0 = time.time()
    rep = evaluate(env, make_planner, a.episodes, a.seed)
    solved = [p for p in plans.values() if p["solved"]]
    rep["plan_success_rate"] = len(solved) / a.episodes
    rep["plan_time_s_median"] = float(np.median([p["plan_s"] for p in solved])) if solved else None
    rep["goal_realisation_error_m_median"] = float(np.median([p["goal_error_m"] for p in plans.values()]))
    rep["goal_information"] = "cartesian (realised from the goal pool by nearest TCP)"
    rep["deadline_s"] = a.deadline_s
    rep["wall_s"] = round(time.time() - t0, 1)
    rep["plans"] = list(plans.values())
    results["rrt_connect_matched"] = rep
    print(f"{'rrt_connect':14s} success {rep['success_rate']*100:5.1f}%  collisions {rep['collision_rate']*100:5.1f}%  "
          f"planned {rep['plan_success_rate']*100:5.1f}%  plan {rep['plan_time_s_median']} s median  "
          f"goal realisation error {rep['goal_realisation_error_m_median']*1000:.1f} mm")

    a.out.parent.mkdir(parents=True, exist_ok=True)
    a.out.write_text(json.dumps({"episodes": a.episodes, "seed": a.seed, "deadline_s": a.deadline_s,
                                 "max_checks": a.max_checks,
                                 "note": "MATCHED protocol: same environment, collision model, pools, "
                                         "success criterion, deadline and goal information for every "
                                         "method. The planner realises the Cartesian goal itself.",
                                 "results": results}, indent=1))
    print(f"wrote {a.out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
