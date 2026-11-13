"""FINAL benchmark on the frozen, untouched test range (SIMULATION ONLY).

This is the run the paper's main table comes from. The protocol is frozen in
`evaluation/episode_manifest.py`:

  * the TEST seed range 20000-20199 has never been looked at (the published M5 numbers were
    computed on 1000-1049, which are now historical DEV data);
  * the policy runs in the LEGACY environment, so the archived policies behave exactly as they
    were trained - the corrected detector is applied afterwards by REPLAYING the recorded
    actions, so only the detector differs and never the behaviour;
  * every episode is judged by three independent detectors: the legacy capsule detector, the
    corrected detector (rigorous lower-bound clearance + per-substep checking) and the mesh
    geometry checker, which also checks the robot against itself.

Methods
  raw        the policy alone
  cbf_legacy the archived single-constraint projection (project-then-clip) - reproduces the
             published filter, including its defect
  qp         the bounded ALL-PAIR kinematic QP with the fixed margin chosen on the tuning seeds
             (the B3/B4 baseline of the research plan)
  exec       the EXECUTION-AWARE filter (B5): it constrains the predicted motion, coast included,
             with the margin chosen on the tuning seeds
  apf        the classical potential field (deterministic, a reference, not a competitor)

    cd learning && ~/ex_rlenv/bin/python ../evaluation/final_benchmark.py
"""

from __future__ import annotations

import argparse
import json
import concurrent.futures as cf
import sys
import time
from pathlib import Path
from typing import Dict

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "learning"))
sys.path.insert(0, str(ROOT / "evaluation"))

from compare_evaluators import _make, replay, run_and_record  # noqa: E402
from episode_manifest import manifest_hash, build_manifest, splits_for  # noqa: E402

TASKS = ("wall_obs", "wall_rand", "wall_ood")
SEEDS = (0, 1, 2)


def _cell_worker(job):
    """One (policy seed, task, method) cell, in its own process.

    The independent mesh checker is the expensive part (it re-derives the link geometry and the
    distances for every configuration of every trajectory), so the cells run in parallel.
    """
    (a, s, task, method, seed, exec_params) = job
    from ex_learning.eval_ppo import ppo_policy_factory
    from ex_learning.safety_filter import cbf_policy_legacy, cbf_policy_qp

    exec_model = None
    if exec_params is not None:
        from ex_learning.execution_model import ServoExecutionModel
        exec_model = ServoExecutionModel(**exec_params)

    def make_filtered(env, base, method, stats=None):
        if method == "raw":
            return base(env)
        if method == "cbf_legacy":
            return cbf_policy_legacy(env, base(env), margin=a.cbf_margin, alpha=a.cbf_alpha)
        if method == "qp":
            return cbf_policy_qp(env, base(env), margin=a.qp_margin, alpha=a.cbf_alpha, rho=0.0,
                                 stats=stats)
        if method == "qp0":
            return cbf_policy_qp(env, base(env), margin=a.qp0_margin, alpha=a.cbf_alpha, rho=0.0,
                                 stats=stats)
        if method == "exec":
            from ex_learning.safety_filter import cbf_policy_exec
            return cbf_policy_exec(env, base(env), exec_model, margin=a.exec_margin, stats=stats)
        if method == "qp_speed":
            from ex_learning.safety_filter import speed_dependent_margin
            return speed_dependent_margin(env, base(env), margin0=a.speed_margin0,
                                          tau=a.speed_tau, a_min=a.speed_a_min,
                                          margin_max=a.speed_margin_max, stats=stats)
        raise ValueError(method)

    t0 = time.time()
    model = ROOT / "data" / "reports" / "learning" / f"ppo_wall_rand_s{s}"
    base = ppo_policy_factory(model, task)
    legacy_env = _make(task, False, a.dynamics)
    corrected_env = _make(task, True, a.dynamics)
    stats: Dict = {}
    rec = run_and_record(legacy_env, lambda e: make_filtered(e, base, method, stats),
                         a.episodes, seed)
    corr = [replay(corrected_env, r["actions"], r["seed"]) for r in rec]

    from independent_collision import IndependentCollisionChecker
    checker = IndependentCollisionChecker()
    mesh = {"collision_rate": None, "min_clearance_m_median": None,
            "min_self_clearance_m_median": None, "self_collision_rate": None,
            "episodes": 0}
    idx = list(range(0, len(rec), a.independent_stride))
    if idx:
        coll, mins, selfmins, selfcoll = 0, [], [], 0
        for i in idx:
            res = checker.check_trajectory(rec[i]["trajectory"], rec[i]["boxes"])
            coll += int(res["collision"])
            mins.append(res["min_clearance_m"])
            selfmins.append(res["min_self_clearance_m"])
            selfcoll += int(res["self_collision"])
        mesh = {"episodes": len(idx), "collision_rate": coll / len(idx),
                "min_clearance_m_median": float(np.median(mins)),
                "min_self_clearance_m_median": float(np.median(selfmins)),
                "self_collision_rate": selfcoll / len(idx)}

    # The solver latency is a headline number (a filter that cannot run inside the control
    # period is not deployable), so it is summarised here: the median, P95 and P99 of the
    # per-step solve times, plus a fixed histogram. The raw per-step samples are NOT kept in the
    # report - there are thousands of them per cell and they would make the file unreadable - but
    # the histogram preserves the distribution, and the summary is what the documents quote.
    from ex_learning.safety_filter import latency_summary
    latency = latency_summary(stats)
    hist = None
    if stats.get("solve_times_s"):
        arr = np.asarray(stats["solve_times_s"], float)
        edges = np.arange(0.0, 0.0205, 0.0005)
        counts, _ = np.histogram(arr, bins=edges)
        hist = {"bin_edges_s": edges.tolist(), "counts": counts.tolist(),
                "note": "the solver latency per control step"}
    filter_stats = {k: v for k, v in stats.items() if k != "solve_times_s"}
    if latency:
        filter_stats["latency"] = latency
    if hist is not None:
        filter_stats["latency_histogram"] = hist

    return {"policy_seed": s, "task": task, "method": method,
            "success_rate": sum(r["success"] for r in rec) / len(rec),
            "legacy_collision_rate": sum(r["collision"] for r in rec) / len(rec),
            "corrected_collision_rate": sum(c["collision"] for c in corr) / len(corr),
            "final_dist_m_median": float(np.median([r["final_dist_m"] for r in rec])),
            "filter_stats": filter_stats,
            "mesh": mesh, "wall_s": round(time.time() - t0, 1)}


def main(argv=None) -> int:

    from ex_learning.eval_ppo import ppo_policy_factory
    from ex_learning.safety_filter import cbf_policy_legacy, cbf_policy_qp

    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--tasks", nargs="*", default=list(TASKS))
    ap.add_argument("--seeds", nargs="*", type=int, default=list(SEEDS))
    ap.add_argument("--episodes", type=int, default=50)
    ap.add_argument("--split", default="test", choices=sorted(splits_for()),
                    help="which manifest split to evaluate on: 'test' (the confirmatory range) or "
                         "'replication' (a second untouched range, manifest v2)")
    ap.add_argument("--manifest-version", type=int, default=None,
                    help="the manifest version whose split boundaries are used (default: current)")
    ap.add_argument("--qp-margin", type=float, default=0.04,
                    help="the margin chosen on the TUNING seeds (docs/LEARNING.md, the B4 frontier)")
    ap.add_argument("--exec-margin", type=float, default=0.005,
                    help="the margin of the execution-aware filter, chosen on the TUNING seeds")
    ap.add_argument("--cbf-margin", type=float, default=0.05)
    ap.add_argument("--cbf-alpha", type=float, default=5.0)
    ap.add_argument("--methods", nargs="*", default=["raw", "cbf_legacy", "qp"],
                    choices=["raw", "cbf_legacy", "qp", "qp0", "qp_speed", "exec"])
    ap.add_argument("--qp0-margin", type=float, default=0.0,
                    help="the second QP operating point (margin 0 = the proportional approach "
                         "barrier, the best completion on the tuning seeds)")
    ap.add_argument("--exec-report", type=Path,
                    default=ROOT / "data" / "reports" / "learning" / "execution_channel.json")
    ap.add_argument("--dynamics", default="placeholder",
                    choices=["placeholder", "identified_run02"],
                    help="the plant: the one the archived policies were trained on, or the one "
                         "identified on the real CR10A (a robustness check)")
    ap.add_argument("--independent-stride", type=int, default=5)
    ap.add_argument("--workers", type=int, default=6,
                    help="the cells are independent and run in parallel processes")
    ap.add_argument("--speed-margin0", type=float, default=0.01,
                    help="B4+: the base clearance margin, before the speed-dependent part")
    ap.add_argument("--speed-tau", type=float, default=0.02,
                    help="B4+: the command-to-motion lag [s] in the stopping-distance margin")
    ap.add_argument("--speed-a-min", type=float, default=2.0,
                    help="B4+: the smallest Cartesian deceleration the arm can achieve [m/s^2]")
    ap.add_argument("--speed-margin-max", type=float, default=0.04,
                    help="B4+: the cap on the speed-dependent margin. It is the SAME margin the "
                         "tuned fixed-margin QP uses, so B4+ can only be safer than B4, never "
                         "more conservative - without the cap the margin reaches half a metre, "
                         "the constraint is infeasible everywhere and the filter is harmful")
    ap.add_argument("--out", type=Path, default=ROOT / "data" / "reports" / "learning" / "final_benchmark.json")
    a = ap.parse_args(argv)

    splits = splits_for(a.manifest_version) if a.manifest_version else splits_for()
    if a.split not in splits:
        raise SystemExit(f"unknown split {a.split!r}; known: {sorted(splits)}")
    test_start, test_end = splits[a.split]
    seed = test_start
    if seed + a.episodes > test_end:
        raise SystemExit(f"{a.episodes} episodes from {seed} would run past the {a.split} range")

    from independent_collision import IndependentCollisionChecker
    checker = IndependentCollisionChecker()

    exec_model = None
    exec_params = None
    if "exec" in a.methods:
        from ex_learning.execution_model import ServoExecutionModel
        m = json.loads(a.exec_report.read_text())["identification"]["model"]
        exec_model = ServoExecutionModel(omega=m["omega"], zeta=m["zeta"], v_gain=m["v_gain"],
                                         dt=m["dt"], sub=m.get("sub", 10))
        exec_params = {"omega": m["omega"], "zeta": m["zeta"], "v_gain": m["v_gain"],
                       "dt": m["dt"], "sub": m.get("sub", 10)}

    man = build_manifest(tuple(a.tasks), a.manifest_version) if a.manifest_version \
        else build_manifest(tuple(a.tasks))
    out: Dict = {
        "protocol": "frozen test range, three detectors, legacy policy behaviour",
        "episode_manifest_hash": manifest_hash(man),
        "episode_manifest_version": man["version"],
        "split": a.split, "seed_start": seed, "seed_end_exclusive": seed + a.episodes,
        "episodes": a.episodes, "tasks": a.tasks, "policy_seeds": a.seeds,
        "methods": a.methods,
        "qp_margin": a.qp_margin, "qp0_margin": a.qp0_margin, "cbf_margin": a.cbf_margin, "cbf_alpha": a.cbf_alpha,
        "exec_margin": a.exec_margin,
        "dynamics": a.dynamics,
        "independent_stride": a.independent_stride, "rows": [],
    }

    def make_filtered(env, base, method, stats=None):
        if method == "raw":
            return base(env)
        if method == "cbf_legacy":
            return cbf_policy_legacy(env, base(env), margin=a.cbf_margin, alpha=a.cbf_alpha)
        if method == "qp":
            return cbf_policy_qp(env, base(env), margin=a.qp_margin, alpha=a.cbf_alpha, rho=0.0,
                                 stats=stats)
        if method == "qp0":
            return cbf_policy_qp(env, base(env), margin=a.qp0_margin, alpha=a.cbf_alpha, rho=0.0,
                                 stats=stats)
        if method == "exec":
            from ex_learning.safety_filter import cbf_policy_exec
            return cbf_policy_exec(env, base(env), exec_model, margin=a.exec_margin, stats=stats)
        if method == "qp_speed":
            # B4+: the fixed-margin QP with a SPEED-DEPENDENT margin (the stopping-distance
            # margin). This is the cheap strong alternative that a reviewer will ask for: if it
            # matches the execution-aware filter, the look-ahead prediction is not needed.
            from ex_learning.safety_filter import speed_dependent_margin
            return speed_dependent_margin(env, base(env), margin0=a.speed_margin0,
                                          tau=a.speed_tau, a_min=a.speed_a_min,
                                          margin_max=a.speed_margin_max, stats=stats)
        raise ValueError(method)

    print(f"{'seed':>4} {'task':9} {'method':11} {'legacy':>15} {'corrected':>11} {'mesh':>8} {'self':>7} "
          f"{'dist mm':>9} {'t_succ':>8}")
    # The cells are independent, so they run in PARALLEL processes, and each cell is written as
    # it finishes. Both matter on this machine: the independent mesh checker is the expensive
    # part, and the laptop suspends (which restarts WSL and kills every running job - that has
    # happened more than once on this project). A cell that is already in the output file is not
    # re-run.
    jobs = [(s, task, method) for s in a.seeds for task in a.tasks for method in a.methods]
    done = {(r["policy_seed"], r["task"], r["method"]) for r in out["rows"]}
    jobs = [j for j in jobs if j not in done]
    if jobs:
        workers = max(1, min(a.workers, len(jobs)))
        with cf.ProcessPoolExecutor(max_workers=workers) as pool:
            for row in pool.map(_cell_worker, [(a, j[0], j[1], j[2], seed, exec_params)
                                               for j in jobs]):
                out["rows"].append(row)
                a.out.parent.mkdir(parents=True, exist_ok=True)
                a.out.write_text(json.dumps(out, indent=1))
                print(f"{row['policy_seed']:>4} {row['task']:9} {row['method']:11} "
                      f"{row['success_rate']*100:6.0f}%/{row['legacy_collision_rate']*100:3.0f}% "
                      f"{row['corrected_collision_rate']*100:9.0f}% "
                      f"{(row['mesh']['collision_rate'] or 0)*100:6.0f}% "
                      f"{(row['mesh']['self_collision_rate'] or 0)*100:5.0f}% "
                      f"{'':>9} {row['wall_s']:7.0f}")
                sys.stdout.flush()

    out["rows"].sort(key=lambda r: (r["policy_seed"], r["task"], a.methods.index(r["method"])))
    a.out.parent.mkdir(parents=True, exist_ok=True)
    a.out.write_text(json.dumps(out, indent=1))
    print(f"\nwrote {a.out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
