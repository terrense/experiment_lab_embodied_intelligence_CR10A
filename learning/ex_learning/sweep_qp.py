"""Safety-completion frontier of the kinematic filters (SIMULATION ONLY).

The B3/B4 baseline of the research plan: a BOUNDED, ALL-PAIR kinematic QP whose only knob is a
FIXED clearance margin. Sweeping that margin traces the safety-completion frontier, and the
best point on it is the baseline the proposed execution-aware filter (B5) has to beat.

The protocol matters more than the curve:

  * the sweep runs on the TUNING seeds (0-19) - the operating point is chosen there,
  * the chosen point is then evaluated on the TEST range (20000+), never on the tuning seeds,
  * the frontier is therefore not chosen on the data it is reported on.

    cd learning && ~/ex_rlenv/bin/python -m ex_learning.sweep_qp --model ../data/reports/learning/ppo_wall_rand_s0 \\
        --task wall_ood --margins 0.0,0.02,0.04,0.06,0.10,0.15
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path

from .baselines import evaluate
from .envs import make_env
from .eval_ppo import ppo_policy_factory
from .safety_filter import cbf_policy, cbf_policy_qp


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--model", type=Path, required=True)
    ap.add_argument("--task", default="wall_ood")
    ap.add_argument("--margins", default="0.0,0.02,0.04,0.06,0.10,0.15")
    ap.add_argument("--alpha", type=float, default=5.0)
    ap.add_argument("--episodes", type=int, default=20)
    ap.add_argument("--seed", type=int, default=0, help="tuning seeds start here")
    ap.add_argument("--mode", choices=("qp", "single"), default="qp",
                    help="'qp' = bounded all-pair QP (B3/B4), 'single' = the single-constraint filter")
    ap.add_argument("--corrected-evaluator", action="store_true")
    ap.add_argument("--include-raw", action="store_true",
                    help="also evaluate the UNFILTERED policy on the same seeds, as the reference "
                         "point of the frontier")
    ap.add_argument("--out", type=Path, default=None)
    a = ap.parse_args(argv)

    base_factory = ppo_policy_factory(a.model, a.task)

    def env_for():
        if not a.corrected_evaluator:
            return make_env(a.task)
        return make_env(a.task, conservative_geometry=True, substep_collision_check=True)

    rows = []
    raw = None
    if a.include_raw:
        raw = evaluate(env_for(), lambda e: base_factory(e), a.episodes, a.seed)
        print(f"{'raw':>7} {raw['success_rate']*100:7.0f}% {raw['collision_rate']*100:9.0f}% "
              f"{raw['final_dist_m_median']*1000:9.0f}        -")
    print(f"{'margin':>7} {'success':>8} {'collision':>10} {'dist mm':>9} {'solve ms':>9}")
    for margin in [float(x) for x in a.margins.split(",")]:
        stats = {}
        if a.mode == "qp":
            mk = lambda e, m=margin: cbf_policy_qp(e, base_factory(e), margin=m, alpha=a.alpha,  # noqa: E731
                                                   rho=0.0, stats=stats)
        else:
            mk = lambda e, m=margin: cbf_policy(e, base_factory(e), margin=m, alpha=a.alpha)  # noqa: E731
        rep = evaluate(env_for(), mk, a.episodes, a.seed)
        rows.append({"margin": margin, "mode": a.mode, "alpha": a.alpha,
                     "success_rate": rep["success_rate"], "collision_rate": rep["collision_rate"],
                     "final_dist_m_median": rep["final_dist_m_median"],
                     "infeasible": stats.get("infeasible", 0), "max_violation": stats.get("max_violation", 0.0)})
        print(f"{margin:7.3f} {rep['success_rate']*100:7.0f}% {rep['collision_rate']*100:9.0f}% "
              f"{rep['final_dist_m_median']*1000:9.0f} {stats.get('solve_time_s_median', 0.0)*1000:9.2f}")
    if a.out:
        a.out.parent.mkdir(parents=True, exist_ok=True)
        out = {"task": a.task, "model": str(a.model), "mode": a.mode,
               "alpha": a.alpha, "episodes": a.episodes, "seed": a.seed,
               "rows": rows}
        if raw is not None:
            out["raw"] = {"success_rate": raw["success_rate"], "collision_rate": raw["collision_rate"],
                          "final_dist_m_median": raw["final_dist_m_median"]}
        a.out.write_text(json.dumps(out, indent=1))
        print(f"wrote {a.out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
