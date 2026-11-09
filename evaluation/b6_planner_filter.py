"""B6: does PLANNER GUIDANCE beat a reactive policy out of distribution? (SIMULATION ONLY).

The plan's B6 arm, and the question §5.6 of the manuscript asks: *does the filter work across
proposal sources*, and *does planning add measurable benefit beyond the filter*?

Six arms, all with the same deadline, the same goal information and the same detector:

    raw        the reactive PPO policy
    qp         the reactive policy + the bounded all-pair kinematic QP
    exec       the reactive policy + the execution-aware filter
    plan       RRT-Connect + a waypoint follower
    plan_qp    the same path, followed through the bounded QP
    plan_exec  the same path, followed through the execution-aware filter

**What the planner is given** (and what it is not): the scene geometry, because the task is
"known geometry"; the start configuration, because the robot knows where it is; and the
**CARTESIAN** goal. It has to realise that goal into a joint configuration itself, with a
damped-least-squares IK. It does NOT get the episode's privileged goal configuration
(`env.q_goal`) and it does not get a goal pool.

**The plan is computed once per episode and shared by the three planner arms**, so the
comparison between them is not confounded by the planner's randomness. The plan does not depend
on the policy seed either, so the plans are computed ONCE and cached (`--stage plan`), then
reused for every arm and every seed.

The stages:

    --stage plan   compute and cache one plan per episode (multi-IK + RRT-Connect)
    --stage run    execute the arms, using the cached plans
    --stage both   (default) plan, then run

    cd learning && ~/ex_rlenv/bin/python ../evaluation/b6_planner_filter.py --task wall_ood
"""

from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "learning"))
sys.path.insert(0, str(ROOT / "evaluation"))

from episode_manifest import splits_for  # noqa: E402


# ----------------------------------------------------------------------- goal realisation
def dls_realise_goal(env, cartesian_goal: np.ndarray, q_start: np.ndarray,
                     rng: np.random.Generator, tries: int = 60, steps: int = 300,
                     tol: float = 0.002) -> List[Tuple[float, float, np.ndarray]]:
    """Realise the CARTESIAN goal into joint configurations, with a damped-least-squares IK.

    The planner of a deployed system would do exactly this: the Cartesian target is given, the
    joint configuration that realises it is NOT. Several candidate realisations are produced,
    because the IK is multi-valued and only some of them are collision-free - the planner needs
    a collision-free goal to connect to.

    Returns a list of (ik_error_m, clearance_m, q) sorted by the IK error.
    """
    out: List[Tuple[float, float, np.ndarray]] = []
    for i in range(tries):
        q = np.array(q_start if i == 0 else rng.uniform(env.lo, env.hi), float)
        env._set_q(q)
        for _ in range(steps):
            e = cartesian_goal - env.tcp()
            if np.linalg.norm(e) < tol:
                break
            J = env.jacobian()
            try:
                dq = J.T @ np.linalg.solve(J @ J.T + 0.05 ** 2 * np.eye(3), e)
            except np.linalg.LinAlgError:
                break
            q = np.clip(q + dq, env.lo, env.hi)
            env._set_q(q)
        err = float(np.linalg.norm(cartesian_goal - env.tcp()))
        env._set_q(q)
        out.append((err, float(env.clearance()), q.copy()))
    out.sort(key=lambda t: t[0])
    return out


# ------------------------------------------------------------------------------- planning
def plan_episode(env, seed: int, args) -> Dict:
    """One episode: realise the goal, then plan. The planning time is reported separately."""
    from ex_learning.planner import JointRRTConnect

    env.reset(seed=seed)
    q_start = env.q().copy()
    cart = env.goal.copy()
    rng = np.random.default_rng(1000 + seed)

    t_ik = time.time()
    cands = dls_realise_goal(env, cart, q_start, rng, tries=args.ik_candidates)
    ik_s = time.time() - t_ik
    free = [c for c in cands if c[1] > args.plan_margin and c[0] < args.ik_tol]

    rec = {"seed": seed, "solved": False, "path": None, "ik_s": round(ik_s, 2),
           "ik_candidates": len(cands), "ik_collision_free": len(free),
           "ik_error_m": cands[0][0] if cands else None,
           "best_goal_clearance_m": max((c[1] for c in cands), default=None),
           "plan_s": 0.0, "checks": 0, "attempts": 0, "goal_config_error_m": None}
    if not free:
        rec["fail_reason"] = "no collision-free IK realisation of the Cartesian goal"
        return rec

    t0 = time.time()
    for err, clr, q_goal in free[:args.plan_candidates]:
        if time.time() - t0 > args.plan_time_budget_s:
            rec["fail_reason"] = (f"the planning time budget ({args.plan_time_budget_s} s) was "
                                  "exhausted before a plan was found")
            break
        planner = JointRRTConnect(env, margin=args.plan_margin, step=args.plan_step,
                                  edge_res=args.edge_res, max_checks=args.plan_checks, rng=rng)
        path = planner.plan(q_start, q_goal)
        rec["attempts"] += 1
        rec["checks"] += planner.checks
        if path is not None:
            rec.update({"solved": True, "path": np.asarray(path, float).tolist(),
                        "plan_s": round(time.time() - t0, 2),
                        "goal_config_error_m": round(float(err), 5),
                        "goal_clearance_m": round(float(clr), 4)})
            return rec
    rec["plan_s"] = round(time.time() - t0, 2)
    rec.setdefault("fail_reason",
                   f"RRT-Connect did not connect within {args.plan_candidates} attempts")
    return rec


# ------------------------------------------------------------------------------ execution
def _cell_worker(job) -> Dict:
    """One (policy seed, arm) cell. It runs in its own process.

    The cells are independent, so they are run in parallel: the execution-aware filter solves a
    bounded program per control step and is by far the slowest arm, and a serial run of the 18
    cells would take hours.
    """
    args, seed, arm, plans = job
    from compare_evaluators import _make, _boxes_from_env
    from ex_learning.eval_ppo import ppo_policy_factory
    from ex_learning.execution_model import ServoExecutionModel
    from ex_learning.planner import path_follower
    from ex_learning.safety_filter import cbf_policy_exec, cbf_policy_qp

    m = json.loads((ROOT / "data" / "reports" / "learning" / "execution_channel.json")
                   .read_text())["identification"]["model"]
    model = ServoExecutionModel(omega=m["omega"], zeta=m["zeta"], v_gain=m["v_gain"],
                                dt=m["dt"], sub=m.get("sub", 10))

    from independent_collision import IndependentCollisionChecker
    checker = IndependentCollisionChecker()

    t0 = time.time()
    model_dir = ROOT / "data" / "reports" / "learning" / f"ppo_wall_rand_s{seed}"
    if not (model_dir / "ppo.zip").exists():
        raise SystemExit(f"missing policy {model_dir}")
    base = ppo_policy_factory(model_dir, args.task)

    env = _make(args.task, False)
    corrected = _make(args.task, True)
    env.cfg.max_steps = int(args.deadline_s / env.cfg.control_dt)
    corrected.cfg.max_steps = env.cfg.max_steps
    n_short = int(6.0 / env.cfg.control_dt)
    per_ep = []

    for i in range(args.episodes):
        ep_seed = args.seed0 + i
        obs, _ = env.reset(seed=ep_seed)
        boxes = _boxes_from_env(env)
        plan = plans[i]
        path = [np.asarray(p, float) for p in plan["path"]] if plan["solved"] else None

        if arm == "raw":
            policy = base(env)
        elif arm == "qp":
            policy = cbf_policy_qp(env, base(env), margin=args.qp_margin, alpha=5.0, rho=0.0)
        elif arm == "exec":
            policy = cbf_policy_exec(env, base(env), model, margin=args.exec_margin)
        else:
            if path is None:
                policy = lambda obs: np.zeros(6)      # no plan: the arm does not move
            else:
                follower = path_follower(path, env, speed=args.plan_speed)
                if arm == "plan":
                    policy = follower
                elif arm == "plan_qp":
                    policy = cbf_policy_qp(env, follower, margin=args.qp_margin,
                                          alpha=5.0, rho=0.0)
                else:
                    policy = cbf_policy_exec(env, follower, model, margin=args.exec_margin)

        actions, traj = [], []
        collided = success = False
        first_success = None
        final_dist = float("inf")
        for k in range(env.cfg.max_steps):
            a = np.asarray(policy(obs), float)
            actions.append(a.copy())
            traj.append(env.q().copy())
            obs, r, term, trunc, inf = env.step(a)
            collided = collided or bool(inf.get("collision", False))
            if first_success is None and bool(inf.get("success", False)):
                first_success = k + 1
            success = success or bool(inf.get("success", False))
            final_dist = float(inf.get("dist", final_dist))
            if term or trunc:
                break
        traj.append(env.q().copy())
        per_ep.append({"episode": i, "seed": ep_seed, "success": success,
                       "success_short": bool(first_success is not None and first_success <= n_short),
                       "first_success_step": first_success, "collision": collided,
                       "final_dist_m": final_dist, "actions": actions,
                       "trajectory": traj, "boxes": boxes})

    # the corrected detector: the same actions, replayed
    corr = []
    for r in per_ep:
        corrected.reset(seed=r["seed"])
        coll = False
        for a in r["actions"]:
            _, _, term, trunc, inf = corrected.step(a)
            coll = coll or bool(inf.get("collision", False))
            if term or trunc:
                break
        corr.append(coll)

    mesh = {"collision_rate": None, "min_clearance_m_median": None, "episodes": 0}
    idx = list(range(0, len(per_ep), args.independent_stride))
    if idx:
        coll, mins = 0, []
        for i in idx:
            res = checker.check_trajectory(per_ep[i]["trajectory"], per_ep[i]["boxes"])
            coll += int(res["collision"])
            mins.append(res["min_clearance_m"])
        mesh = {"episodes": len(idx), "collision_rate": coll / len(idx),
                "min_clearance_m_median": float(np.median(mins))}

    return {"policy_seed": seed, "arm": arm,
            "success_rate": sum(r["success"] for r in per_ep) / len(per_ep),
            "success_rate_6s": sum(r["success_short"] for r in per_ep) / len(per_ep),
            "legacy_collision_rate": sum(r["collision"] for r in per_ep) / len(per_ep),
            "corrected_collision_rate": sum(corr) / len(corr),
            "final_dist_m_median": float(np.median([r["final_dist_m"] for r in per_ep])),
            # the step at which the goal was first reached (-1: never). With this, the completion
            # at ANY deadline can be recomputed from the committed report - which matters here,
            # because the planner is slow and the reactive policy is fast, and the deadline is
            # the axis on which they differ.
            "first_success_steps": [(-1 if r["first_success_step"] is None else r["first_success_step"])
                                    for r in per_ep],
            "mesh": mesh, "wall_s": round(time.time() - t0, 1)}


def run_arms(args, plans: List[Dict]) -> Dict:
    import concurrent.futures as cf

    out: Dict = {"task": args.task, "split": args.split, "seed_start": args.seed0,
                 "episodes": args.episodes, "seeds": args.seeds, "deadline_s": args.deadline_s,
                 "qp_margin": args.qp_margin, "exec_margin": args.exec_margin,
                 "plan_margin": args.plan_margin, "plan_checks": args.plan_checks,
                 "plan_candidates": args.plan_candidates,
                 "goal_information": "cartesian only; the planner realises it with a DLS IK and "
                                    "never sees the privileged goal configuration",
                 "arms": list(args.arms), "rows": []}

    print(f"{'seed':>4} {'arm':10} {'success':>8} {'success@6s':>10} {'legacy':>8} "
          f"{'corrected':>10} {'mesh':>7} {'final mm':>9}")

    jobs = [(args, s, arm, plans) for s in args.seeds for arm in args.arms]
    workers = max(1, min(args.workers, len(jobs)))
    rows: List[Dict] = []
    with cf.ProcessPoolExecutor(max_workers=workers) as pool:
        for row in pool.map(_cell_worker, jobs):
            # the completion at any deadline, recomputed from the recorded first-success steps
            steps = row["first_success_steps"]
            row["completion_by_deadline_s"] = {
                f"{d}s": (sum(1 for s in steps if 0 <= s <= int(d / 0.02)) / len(steps)
                          if steps else None)
                for d in (6, 12, 20, 30)}
            rows.append(row)
            mesh = row["mesh"]
            print(f"{row['policy_seed']:>4} {row['arm']:10} {row['success_rate']*100:7.0f}% "
                  f"{row['success_rate_6s']*100:9.0f}% "
                  f"{row['legacy_collision_rate']*100:7.0f}% {row['corrected_collision_rate']*100:9.0f}% "
                  f"{(mesh['collision_rate'] or 0)*100:6.0f}% "
                  f"{row['final_dist_m_median']*1000:8.0f}", flush=True)
            out["rows"].append(row)
            args.out.parent.mkdir(parents=True, exist_ok=True)
            args.out.write_text(json.dumps(out, indent=1))       # written as the cells finish
    out["rows"].sort(key=lambda r: (r["policy_seed"], args.arms.index(r["arm"])))
    return out


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--task", default="wall_ood")
    ap.add_argument("--split", default="test")
    ap.add_argument("--stage", default="both", choices=("plan", "run", "both"))
    ap.add_argument("--seeds", nargs="*", type=int, default=[0, 1, 2])
    ap.add_argument("--arms", nargs="*", default=["raw", "qp", "exec", "plan", "plan_qp", "plan_exec"])
    ap.add_argument("--episodes", type=int, default=50)
    ap.add_argument("--deadline-s", type=float, default=30.0)
    ap.add_argument("--qp-margin", type=float, default=0.04)
    ap.add_argument("--exec-margin", type=float, default=0.01)
    ap.add_argument("--plan-margin", type=float, default=0.02)
    ap.add_argument("--plan-checks", type=int, default=20000)
    ap.add_argument("--plan-candidates", type=int, default=4)
    ap.add_argument("--plan-time-budget-s", type=float, default=120.0,
                    help="the planning time per episode: a planner that cannot plan within this "
                         "budget is reported as a planning FAILURE, not as a slow success")
    ap.add_argument("--plan-step", type=float, default=0.15)
    ap.add_argument("--edge-res", type=float, default=0.015)
    ap.add_argument("--plan-speed", type=float, default=0.6)
    ap.add_argument("--ik-candidates", type=int, default=60)
    ap.add_argument("--ik-tol", type=float, default=0.002)
    ap.add_argument("--independent-stride", type=int, default=5)
    ap.add_argument("--workers", type=int, default=6,
                    help="the cells are independent and are run in parallel processes")
    ap.add_argument("--plans-out", type=Path,
                    default=ROOT / "data" / "reports" / "learning" / "b6_plans.json")
    ap.add_argument("--out", type=Path,
                    default=ROOT / "data" / "reports" / "learning" / "b6_planner_filter.json")
    a = ap.parse_args(argv)

    splits = splits_for()
    if a.split not in splits:
        raise SystemExit(f"unknown split {a.split!r}; known: {sorted(splits)}")
    a.seed0 = splits[a.split][0]

    if a.stage in ("plan", "both"):
        from compare_evaluators import _make
        env = _make(a.task, True)          # the planner uses the RIGOROUS lower-bound clearance
        print(f"planning {a.episodes} episodes of {a.task} (split {a.split}, from seed {a.seed0})")
        plans = []
        t0 = time.time()
        for i in range(a.episodes):
            rec = plan_episode(env, a.seed0 + i, a)
            plans.append(rec)
            print(f"  ep {i:3d} seed {a.seed0+i}: "
                  f"{'PLANNED' if rec['solved'] else 'no plan '}  "
                  f"{rec.get('plan_s', 0):7.1f} s  checks {rec['checks']:6d}  "
                  f"attempts {rec['attempts']}  ik_free {rec['ik_collision_free']}  "
                  f"{rec.get('fail_reason', '')}", flush=True)
        a.plans_out.parent.mkdir(parents=True, exist_ok=True)
        a.plans_out.write_text(json.dumps({"task": a.task, "split": a.split, "seed_start": a.seed0,
                                           "episodes": a.episodes, "plan_margin": a.plan_margin,
                                           "plan_checks": a.plan_checks,
                                           "plan_candidates": a.plan_candidates,
                                           "ik_candidates": a.ik_candidates,
                                           "wall_s": round(time.time() - t0, 1),
                                           "plans": plans}, indent=1))
        print(f"wrote {a.plans_out}  ({time.time()-t0:.0f} s)")
    else:
        plans = None

    if a.stage in ("run", "both"):
        if plans is None:
            if not a.plans_out.exists():
                raise SystemExit(f"{a.plans_out} not found - run --stage plan first")
            plans = json.loads(a.plans_out.read_text())["plans"]
        n = sum(p["solved"] for p in plans)
        print(f"\n{len(plans)} cached plans, {n} solved ({100*n/len(plans):.0f} %)")
        out = run_arms(a, plans)
        out["plan_summary"] = {"solved": n, "episodes": len(plans),
                               "plan_time_s_median": float(np.median(
                                   [p["plan_s"] for p in plans if p["solved"]])) if n else None,
                               "plan_time_s_total": sum(p["plan_s"] for p in plans)}
        a.out.parent.mkdir(parents=True, exist_ok=True)
        a.out.write_text(json.dumps(out, indent=1))
        print(f"\nwrote {a.out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
