"""Frontier sweep of the EXECUTION-AWARE filter (SIMULATION ONLY).

The B5 arm of the research plan. `sweep_qp.py` traces the frontier of the bounded kinematic QP
(B3/B4); this traces the frontier of the filter that reasons about the EXECUTION of the command
(B5), so the two can be compared on the same axes.

Three modes, so the paper can attribute the result:

  exec         the full execution-aware filter: the lookahead constraint uses the predicted
               motion, including the COAST (the displacement with a zero command);
  exec_nocoast the ABLATION: the same lookahead, but the coast term is dropped - i.e. the
               constraint is built from an execution-blind model;
  qp           the bounded kinematic QP, re-run here so the comparison is on the SAME seeds with
               the SAME code path (it is the B4 baseline).

The protocol is the one of the research plan: the sweep runs on the TUNING seeds, the operating
point is chosen there, and only then is the frozen TEST range touched.

The execution model is loaded from `data/reports/learning/execution_channel.json` - the measured,
identified parameters - so no number in the sweep is hand-entered.

    cd learning && ~/ex_rlenv/bin/python -m ex_learning.sweep_exec --model ../data/reports/learning/ppo_wall_rand_s0 \\
        --task wall_ood --margins 0.005,0.01,0.02,0.04 --mode exec
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path

from .baselines import evaluate
from .envs import make_env
from .eval_ppo import ppo_policy_factory
from .execution_model import ServoExecutionModel
from .safety_filter import cbf_policy_exec, cbf_policy_qp

ROOT = Path(__file__).resolve().parents[2]
DEFAULT_EXEC_REPORT = ROOT / "data" / "reports" / "learning" / "execution_channel.json"


def load_exec_model(path: Path) -> tuple:
    """Load the IDENTIFIED execution model (never hand-entered parameters)."""
    rep = json.loads(path.read_text())
    m = rep["identification"]["model"]
    model = ServoExecutionModel(omega=m["omega"], zeta=m["zeta"], v_gain=m["v_gain"],
                                dt=m["dt"], sub=m.get("sub", 10))
    return model, rep


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--model", type=Path, required=True)
    ap.add_argument("--task", default="wall_ood")
    ap.add_argument("--margins", default="0.005,0.01,0.02,0.04,0.06")
    ap.add_argument("--mode", choices=("exec", "exec_nocoast", "qp"), default="exec")
    ap.add_argument("--episodes", type=int, default=20)
    ap.add_argument("--seed", type=int, default=0, help="tuning seeds start here")
    ap.add_argument("--horizon", type=int, default=5)
    ap.add_argument("--sample-every", type=int, default=2)
    ap.add_argument("--alpha", type=float, default=5.0)
    ap.add_argument("--exec-report", type=Path, default=DEFAULT_EXEC_REPORT)
    ap.add_argument("--include-raw", action="store_true")
    ap.add_argument("--out", type=Path, default=None)
    a = ap.parse_args(argv)

    base_factory = ppo_policy_factory(a.model, a.task)
    model, exec_report = load_exec_model(a.exec_report)
    print(f"execution model (identified): omega {model.omega:.2f} rad/s, zeta {model.zeta:.2f}, "
          f"v_gain {model.v_gain:.2f}  [prediction RMS "
          f"{exec_report['identification']['prediction_error']['rms_rad']*1000:.2f} mrad]")

    rows = []
    raw = None
    if a.include_raw:
        raw = evaluate(make_env(a.task), lambda e: base_factory(e), a.episodes, a.seed)
        print(f"{'raw':>7} {raw['success_rate']*100:7.0f}% {raw['collision_rate']*100:9.0f}% "
              f"{raw['final_dist_m_median']*1000:9.0f}        -")
    print(f"{'margin':>7} {'success':>8} {'collision':>10} {'dist mm':>9} {'relax mm':>9} {'solve ms':>9}")
    for margin in [float(x) for x in a.margins.split(",")]:
        stats = {}
        if a.mode == "qp":
            mk = lambda e, m=margin: cbf_policy_qp(e, base_factory(e), margin=m, alpha=a.alpha,  # noqa: E731
                                                   rho=0.0, stats=stats)
        else:
            mk = lambda e, m=margin: cbf_policy_exec(  # noqa: E731
                e, base_factory(e), model, margin=m, horizon=a.horizon,
                sample_every=a.sample_every, stats=stats,
                account_execution=(a.mode == "exec"))
        rep = evaluate(make_env(a.task), mk, a.episodes, a.seed)
        rows.append({"margin": margin, "mode": a.mode, "alpha": a.alpha,
                     "success_rate": rep["success_rate"], "collision_rate": rep["collision_rate"],
                     "final_dist_m_median": rep["final_dist_m_median"],
                     "infeasible": stats.get("infeasible", 0),
                     "max_relaxation": stats.get("max_relaxation", 0.0),
                     "max_violation": stats.get("max_violation", 0.0),
                     "interventions": stats.get("interventions", 0),
                     "steps": stats.get("steps", 0)})
        print(f"{margin:7.3f} {rep['success_rate']*100:7.0f}% {rep['collision_rate']*100:9.0f}% "
              f"{rep['final_dist_m_median']*1000:9.0f} {stats.get('max_relaxation', 0.0)*1000:9.2f} "
              f"{stats.get('solve_time_s_median', 0.0)*1000:9.2f}")
    if a.out:
        a.out.parent.mkdir(parents=True, exist_ok=True)
        out = {"task": a.task, "model": str(a.model), "mode": a.mode, "alpha": a.alpha,
               "episodes": a.episodes, "seed": a.seed, "horizon": a.horizon,
               "sample_every": a.sample_every,
               "execution_model": exec_report["identification"]["model"],
               "execution_prediction_rms_rad": exec_report["identification"]["prediction_error"]["rms_rad"],
               "rows": rows}
        if raw is not None:
            out["raw"] = {"success_rate": raw["success_rate"], "collision_rate": raw["collision_rate"],
                          "final_dist_m_median": raw["final_dist_m_median"]}
        a.out.write_text(json.dumps(out, indent=1))
        print(f"wrote {a.out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
