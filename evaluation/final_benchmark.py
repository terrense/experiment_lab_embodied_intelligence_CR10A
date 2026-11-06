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
import sys
import time
from pathlib import Path
from typing import Dict

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "learning"))
sys.path.insert(0, str(ROOT / "evaluation"))

from compare_evaluators import _make, replay, run_and_record  # noqa: E402
from episode_manifest import SPLITS, manifest_hash, build_manifest  # noqa: E402

TASKS = ("wall_obs", "wall_rand", "wall_ood")
SEEDS = (0, 1, 2)


def main(argv=None) -> int:

    from ex_learning.eval_ppo import ppo_policy_factory
    from ex_learning.safety_filter import cbf_policy_legacy, cbf_policy_qp

    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--tasks", nargs="*", default=list(TASKS))
    ap.add_argument("--seeds", nargs="*", type=int, default=list(SEEDS))
    ap.add_argument("--episodes", type=int, default=50)
    ap.add_argument("--qp-margin", type=float, default=0.04,
                    help="the margin chosen on the TUNING seeds (docs/LEARNING.md, the B4 frontier)")
    ap.add_argument("--exec-margin", type=float, default=0.005,
                    help="the margin of the execution-aware filter, chosen on the TUNING seeds")
    ap.add_argument("--cbf-margin", type=float, default=0.05)
    ap.add_argument("--cbf-alpha", type=float, default=5.0)
    ap.add_argument("--methods", nargs="*", default=["raw", "cbf_legacy", "qp"],
                    choices=["raw", "cbf_legacy", "qp", "qp0", "exec"])
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
    ap.add_argument("--out", type=Path, default=ROOT / "data" / "reports" / "learning" / "final_benchmark.json")
    a = ap.parse_args(argv)

    test_start, test_end = SPLITS["test"]
    seed = test_start
    if seed + a.episodes > test_end:
        raise SystemExit(f"{a.episodes} episodes from {seed} would run past the frozen test range")

    from independent_collision import IndependentCollisionChecker
    checker = IndependentCollisionChecker()

    exec_model = None
    if "exec" in a.methods:
        from ex_learning.execution_model import ServoExecutionModel
        m = json.loads(a.exec_report.read_text())["identification"]["model"]
        exec_model = ServoExecutionModel(omega=m["omega"], zeta=m["zeta"], v_gain=m["v_gain"],
                                         dt=m["dt"], sub=m.get("sub", 10))

    man = build_manifest(tuple(a.tasks))
    out: Dict = {
        "protocol": "frozen test range, three detectors, legacy policy behaviour",
        "episode_manifest_hash": manifest_hash(man),
        "split": "test", "seed_start": seed, "seed_end_exclusive": seed + a.episodes,
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
        raise ValueError(method)

    print(f"{'seed':>4} {'task':9} {'method':11} {'legacy':>15} {'corrected':>11} {'mesh':>8} {'self':>7} "
          f"{'dist mm':>9} {'t_succ':>8}")
    for s in a.seeds:
        model = ROOT / "data" / "reports" / "learning" / f"ppo_wall_rand_s{s}"
        for task in a.tasks:
            base = ppo_policy_factory(model, task)
            for method in a.methods:
                t0 = time.time()
                legacy_env = _make(task, False, a.dynamics)
                corrected_env = _make(task, True, a.dynamics)
                stats: Dict = {}
                rec = run_and_record(legacy_env, lambda e: make_filtered(e, base, method, stats),
                                     a.episodes, seed)
                corr = [replay(corrected_env, r["actions"], r["seed"]) for r in rec]

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

                row = {"policy_seed": s, "task": task, "method": method,
                       "success_rate": sum(r["success"] for r in rec) / len(rec),
                       "legacy_collision_rate": sum(r["collision"] for r in rec) / len(rec),
                       "corrected_collision_rate": sum(c["collision"] for c in corr) / len(corr),
                       "final_dist_m_median": float(np.median([r["final_dist_m"] for r in rec])),
                       "filter_stats": {k: v for k, v in stats.items()},
                       "mesh": mesh, "wall_s": round(time.time() - t0, 1)}
                out["rows"].append(row)
                print(f"{s:>4} {task:9} {method:11} "
                      f"{row['success_rate']*100:6.0f}%/{row['legacy_collision_rate']*100:3.0f}% "
                      f"{row['corrected_collision_rate']*100:9.0f}% "
                      f"{(mesh['collision_rate'] or 0)*100:6.0f}% {(mesh['self_collision_rate'] or 0)*100:5.0f}% "
                      f"{'':>9} {row['wall_s']:7.0f}")
                sys.stdout.flush()

    a.out.parent.mkdir(parents=True, exist_ok=True)
    a.out.write_text(json.dumps(out, indent=1))
    print(f"\nwrote {a.out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
