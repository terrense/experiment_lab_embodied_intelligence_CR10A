"""Legacy vs corrected collision evaluation on the historical dev set (SIMULATION ONLY).

The Week-1 gate of docs/RESEARCH_PLAN.md: how much of the published M5 table is an artefact of
the evaluator rather than of the method?

Three verdicts are produced for the SAME episodes and the SAME policy:

  legacy     the environment as it was used for the published numbers: the capsule clearance
             sampled at 32 points along each axis, checked once per policy step, no
             self-collision check.
  corrected  the same environment with `conservative_geometry` (the clearance is a rigorous lower
             bound) and `substep_collision_check` (the clearance is checked after every physics
             substep, so an intersection inside a step is caught).
  independent a SECOND, separate checker (evaluation/independent_collision.py) that uses the mesh
             surface instead of the enclosing capsules and its own distance code, and that also
             checks the robot against ITSELF over the link pairs the MoveIt SRDF does not disable.

The legacy JSONs are never overwritten. This writes a new comparison file.

    cd learning && ~/ex_rlenv/bin/python ../evaluation/compare_evaluators.py
"""

from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path
from typing import Dict, List

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "learning"))

from ex_learning.envs import make_env  # noqa: E402
from ex_learning.envs.cr10a_reach_wall import WallConfig  # noqa: E402

TASKS = ("wall_obs", "wall_rand", "wall_ood")
SEEDS = (0, 1, 2)


def _make(task: str, corrected: bool):
    """The environment for one evaluator variant."""
    if not corrected:
        return make_env(task)
    return make_env(task, conservative_geometry=True, substep_collision_check=True)


def _boxes_from_env(env) -> List[Dict]:
    """The scene boxes AS THEY ARE for the current episode.

    This matters: the random-wall tasks move the wall at every reset, so a checker fed the static
    scene file would test the trajectory against the WRONG obstacle.
    """
    out = []
    for i, name in enumerate(env.box_names):
        lo, hi = np.asarray(env.box_lo[i], float), np.asarray(env.box_hi[i], float)
        out.append({"name": name, "center": 0.5 * (lo + hi), "size": hi - lo})
    return out


def run_and_record(env, make_policy, episodes: int, seed: int) -> List[Dict]:
    """Run the episodes in the LEGACY environment and record the action sequence and the
    joint trajectory of every episode.

    The trajectory is what makes the comparison a controlled one: the same actions are replayed
    through the corrected environment, where the physics is identical (the servo torque depends
    only on q, qd, q_ref and the command) and only the DETECTOR differs.
    """
    results: List[Dict] = []
    for ep in range(episodes):
        obs, info = env.reset(seed=seed + ep)
        boxes = _boxes_from_env(env)          # the obstacles of THIS episode
        policy = make_policy(env)
        actions: List[np.ndarray] = []
        traj: List[np.ndarray] = []
        collided = False
        success = False
        final_dist = float("inf")
        for _ in range(env.cfg.max_steps):
            a = np.asarray(policy(obs), float)
            actions.append(a.copy())
            traj.append(env.q().copy())
            obs, r, term, trunc, inf = env.step(a)
            collided = collided or bool(inf.get("collision", False))
            success = success or bool(inf.get("success", False))
            final_dist = float(inf.get("dist", final_dist))
            if term or trunc:
                break
        traj.append(env.q().copy())
        results.append({"episode": ep, "seed": seed + ep, "success": success, "collision": collided,
                        "final_dist_m": final_dist, "actions": actions, "trajectory": traj, "boxes": boxes})
    return results


def replay(env, actions: List[np.ndarray], ep_seed: int) -> Dict:
    """Replay recorded actions in another environment: identical physics, possibly different
    detector. `ep_seed` is the episode's seed (the one the original run used).

    The environment must be reset with the SAME seed, and the episode sampling must not depend on
    the evaluator - otherwise the replay would run a different episode than the original.
    """
    env.reset(seed=ep_seed)
    collided = False
    worst = float("inf")
    for a in actions:
        obs, r, term, trunc, inf = env.step(a)
        collided = collided or bool(inf.get("collision", False))
        c = inf.get("min_clearance_episode", inf.get("clearance"))
        if c is not None:
            worst = min(worst, c)
        if term or trunc:
            break
    return {"collision": collided, "min_clearance_m": worst}


def main(argv=None) -> int:
    from ex_learning.eval_ppo import ppo_policy_factory
    from ex_learning.safety_filter import cbf_policy, cbf_policy_legacy

    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--tasks", nargs="*", default=list(TASKS))
    ap.add_argument("--seeds", nargs="*", type=int, default=list(SEEDS))
    ap.add_argument("--filters", nargs="*", default=["raw", "cbf"], choices=["raw", "cbf", "cbf_legacy"])
    ap.add_argument("--episodes", type=int, default=50)
    ap.add_argument("--seed", type=int, default=1000)
    ap.add_argument("--cbf-margin", type=float, default=0.05)
    ap.add_argument("--cbf-alpha", type=float, default=5.0)
    ap.add_argument("--independent-episode-stride", type=int, default=5,
                    help="check every Nth episode with the independent checker (it is slower); 0 disables it")
    ap.add_argument("--out", type=Path, default=ROOT / "data" / "reports" / "learning" / "evaluator_comparison.json")
    a = ap.parse_args(argv)

    from independent_collision import IndependentCollisionChecker

    out: Dict = {"episodes": a.episodes, "seed_start": a.seed, "tasks": a.tasks,
                 "policy_seeds": a.seeds, "filters": a.filters,
                 "cbf_margin": a.cbf_margin, "cbf_alpha": a.cbf_alpha,
                 "independent_episode_stride": a.independent_episode_stride,
                 "note": "legacy = the evaluator used for the published numbers; the archived JSONs "
                         "are NOT overwritten by this run.",
                 "rows": []}
    checker = IndependentCollisionChecker()
    # the obstacles are read from the ENVIRONMENT per episode, not from the static scene file:
    # the random-wall tasks move the wall at every reset, so a static box list would test the
    # trajectory against the wrong obstacle

    def make_filtered(env, base, filt):
        if filt == "raw":
            return base(env)
        if filt == "cbf":
            return cbf_policy(env, base(env), margin=a.cbf_margin, alpha=a.cbf_alpha)
        return cbf_policy_legacy(env, base(env), margin=a.cbf_margin, alpha=a.cbf_alpha)

    print(f"{'seed':>4} {'task':9} {'filter':11} {'legacy':>16} {'corrected':>16} {'independent':>18}")
    for s in a.seeds:
        model = ROOT / "data" / "reports" / "learning" / f"ppo_wall_rand_s{s}"
        for task in a.tasks:
            base = ppo_policy_factory(model, task)
            for filt in a.filters:
                row = {"policy_seed": s, "task": task, "filter": filt}

                legacy_env = _make(task, False)
                corrected_env = _make(task, True)
                rec = run_and_record(legacy_env, lambda e: make_filtered(e, base, filt), a.episodes, a.seed)

                # self-consistency: replaying the recorded actions in the SAME (legacy) environment
                # must reproduce the legacy verdict exactly - otherwise the comparison is not
                # comparing detectors but something else
                sanity = [replay(_make(task, False), r["actions"], r["seed"])["collision"] for r in rec]
                legacy_coll = [r["collision"] for r in rec]
                assert sanity == legacy_coll, ("the replay is not reproducing the legacy run - the "
                                               "episode sampling or the physics differ")

                corr = [replay(corrected_env, r["actions"], r["seed"]) for r in rec]
                row["legacy"] = {"success_rate": sum(r["success"] for r in rec) / len(rec),
                                 "collision_rate": sum(legacy_coll) / len(rec)}
                row["corrected"] = {"collision_rate": sum(c["collision"] for c in corr) / len(corr),
                                    "min_clearance_m_median": float(np.median(
                                        [c["min_clearance_m"] for c in corr if c["min_clearance_m"] is not None]))}

                # the independent checker on a subsample of episodes
                ind = {"episodes": 0, "collision_rate": None, "min_clearance_m_median": None,
                       "min_self_clearance_m_median": None, "self_collision_rate": None}
                if a.independent_episode_stride > 0:
                    idx = list(range(0, len(rec), a.independent_episode_stride))
                    coll, mins, selfmins, selfcoll = 0, [], [], 0
                    for i in idx:
                        res = checker.check_trajectory(rec[i]["trajectory"], rec[i]["boxes"])
                        coll += int(res["collision"])
                        mins.append(res["min_clearance_m"])
                        selfmins.append(res["min_self_clearance_m"])
                        selfcoll += int(res["self_collision"])
                    ind = {"episodes": len(idx), "collision_rate": coll / len(idx),
                           "min_clearance_m_median": float(np.median(mins)),
                           "min_self_clearance_m_median": float(np.median(selfmins)),
                           "self_collision_rate": selfcoll / len(idx)}
                row["independent"] = ind
                row["verdict_changes"] = {
                    "legacy_to_corrected": sum(1 for c, l in zip(corr, legacy_coll)
                                               if c["collision"] != l),
                    "episodes": len(rec)}
                out["rows"].append(row)
                f = lambda d: f"{d['success_rate']*100:4.0f}%/{d['collision_rate']*100:3.0f}%"  # noqa: E731
                fc = f"{row['corrected']['collision_rate']*100:4.0f}%            "
                fi = f"{ind['collision_rate']*100:3.0f}%" if ind["collision_rate"] is not None else "   -"
                print(f"{s:>4} {task:9} {filt:11} {f(row['legacy']):>16} {fc:>16} {fi:>18}   "
                      f"(changed {row['verdict_changes']['legacy_to_corrected']}/{len(rec)})")

    a.out.parent.mkdir(parents=True, exist_ok=True)
    a.out.write_text(json.dumps(out, indent=1))
    print(f"\nwrote {a.out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
