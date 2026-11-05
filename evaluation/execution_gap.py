"""The EXECUTION GAP: how often the filter's own safety claim fails in execution (SIMULATION ONLY).

This is the measurement behind the research main line. A kinematic filter constrains the
COMMAND: it guarantees that the command it issues does not drive the link closer to the obstacle
than the barrier allows. What it does NOT guarantee is that the ARM stays clear, because the
command is integrated into a position reference and tracked by a delayed, bandwidth-limited
servo.

The diagnostic therefore compares, at every control step of a real episode:

  * the filter's own claim - were all of its constraints satisfied by the command it returned?
    (for the kinematic QP the constraint is the barrier condition on the command; for the
    execution-aware filter it is the predicted-motion condition);
  * what the plant actually did - the clearance one control step later.

Three numbers come out:

  claim_violations   steps where the filter's constraints were NOT satisfied. Those are the
                     filter's own violations and are counted as failures of the filter.
  prediction_failures
                     steps where the filter was SATISFIED but the actual clearance one step
                     later fell below the margin. These are the cases the kinematic reasoning
                     cannot see: the command was fine, the execution was not.
  gap_mm            the median and the worst of (actual clearance - predicted clearance) over
                     the steps where the filter was satisfied. A negative bias means the
                     prediction is systematically optimistic.

The comparison is made between the kinematic QP (B3/B4) and the execution-aware filter (B5) on
the SAME episodes, so the difference is the filter and nothing else.

    cd learning && ~/ex_rlenv/bin/python ../evaluation/execution_gap.py --episodes 20
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "learning"))

from ex_learning.envs import make_env  # noqa: E402
from ex_learning.eval_ppo import ppo_policy_factory  # noqa: E402
from ex_learning.execution_model import ServoExecutionModel  # noqa: E402
from ex_learning.safety_filter import cbf_policy_exec, cbf_policy_qp  # noqa: E402

EXEC_REPORT = ROOT / "data" / "reports" / "learning" / "execution_channel.json"


def _load_exec_model(path: Path) -> ServoExecutionModel:
    m = json.loads(path.read_text())["identification"]["model"]
    return ServoExecutionModel(omega=m["omega"], zeta=m["zeta"], v_gain=m["v_gain"],
                                dt=m["dt"], sub=m.get("sub", 10))


def _run(env, make_policy, episodes: int, seed: int, margin: float) -> Dict:
    """Run the episodes and measure the gap between the filter's claim and the plant.

    At every control step:
      * ``d_before`` is the clearance the filter reasoned about (the rigorous lower bound);
      * the filter returns a command and its own residuals;
      * ``claimed_after`` is what the filter's KINEMATIC reasoning implies for the next step,
        ``d_before + (c . a*vmax) * dt`` for the closest constrained pair;
      * ``d_after`` is what the plant actually did.
    """
    steps = claim_violations = prediction_failures = compared = 0
    breaches = 0
    gaps = []
    for ep in range(episodes):
        obs, _ = env.reset(seed=seed + ep)
        trace: list = []
        policy = make_policy(env, trace)
        for _ in range(env.cfg.max_steps):
            d_before = float(env.clearance(conservative=True))
            a = policy(obs)
            rec = trace[-1] if trace else {"feasible": True, "pairs": [], "a": a}
            obs, _, term, trunc, info = env.step(a)
            d_after = float(env.clearance(conservative=True))
            steps += 1
            if not rec["feasible"]:
                claim_violations += 1
                continue
            compared += 1
            # the kinematic one-step prediction of the closest constrained pair
            best = None
            vmax = env.cfg.max_joint_speed
            for di, c in rec["pairs"]:
                pred = di + float(c @ (np.asarray(rec["a"], float) * vmax)) * env.cfg.control_dt
                if best is None or pred < best:
                    best = pred
            if best is not None:
                gaps.append(d_after - best)
            if d_after < margin:
                prediction_failures += 1
            if d_after < 0.0:
                breaches += 1
            if term or trunc:
                break
    g = np.array(gaps) if gaps else np.zeros(1)
    # a fixed histogram, so the figure can be redrawn from the JSON without re-running anything
    edges = np.arange(-160.0, 5.0, 5.0)
    counts, _ = np.histogram(np.clip(g * 1000.0, edges[0], edges[-1]), bins=edges)
    return {"steps": steps, "compared": compared,
            "claim_violations": claim_violations,
            "prediction_failures": prediction_failures,
            "prediction_failure_rate": prediction_failures / max(compared, 1),
            "envelope_breaches": breaches,
            "gap_mm_median": float(np.median(g) * 1000),
            "gap_mm_p05": float(np.percentile(g, 5) * 1000),
            "gap_mm_worst": float(g.min() * 1000),
            "gap_histogram": {"bin_edges_mm": edges.tolist(), "counts": counts.tolist()}}


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--task", default="wall_ood")
    ap.add_argument("--model", type=Path, default=ROOT / "data" / "reports" / "learning" / "ppo_wall_rand_s0")
    ap.add_argument("--episodes", type=int, default=20)
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--qp-margin", type=float, default=0.02)
    ap.add_argument("--exec-margin", type=float, default=0.02)
    ap.add_argument("--sweep-margins", default="",
                    help="comma-separated margins; each method is run at each of them")
    ap.add_argument("--alpha", type=float, default=5.0)
    ap.add_argument("--exec-report", type=Path, default=EXEC_REPORT)
    ap.add_argument("--out", type=Path, default=ROOT / "data" / "reports" / "learning" / "execution_gap.json")
    a = ap.parse_args(argv)

    base = ppo_policy_factory(a.model, a.task)
    model = _load_exec_model(a.exec_report)
    out = {"task": a.task, "episodes": a.episodes, "seed": a.seed,
           "qp_margin": a.qp_margin, "exec_margin": a.exec_margin,
           "note": "SIMULATED plant. The gap is the difference between what the filter's own "
                   "reasoning guarantees and what the plant actually did.",
           "rows": []}

    margins = [float(x) for x in a.sweep_margins.split(",") if x.strip()] or [None]
    print(f"{'margin':>8} {'method':>12} {'steps':>7} {'claim viol':>11} {'pred fail':>10} {'rate':>7} "
          f"{'gap med mm':>12} {'gap p05 mm':>12} {'gap worst mm':>13}")
    for m in margins:
        qm = a.qp_margin if m is None else m
        em = a.exec_margin if m is None else m
        for name, maker, margin in (
                ("qp", lambda e, t, m=qm: cbf_policy_qp(e, base(e), margin=m, alpha=a.alpha, trace=t),
                 qm),
                ("exec_nocoast", lambda e, t, m=em: cbf_policy_exec(e, base(e), model, margin=m,
                                                                   trace=t, account_execution=False),
                 em),
                ("exec", lambda e, t, m=em: cbf_policy_exec(e, base(e), model, margin=m, trace=t),
                 em)):
            env = make_env(a.task)
            rep = _run(env, maker, a.episodes, a.seed, margin)
            rep["method"] = name
            rep["margin"] = margin
            out["rows"].append(rep)
            print(f"{(margin or 0)*100:7.1f}c {name:>12} {rep['steps']:>7} "
                  f"{rep['claim_violations']:>11} {rep['prediction_failures']:>10} "
                  f"{rep['prediction_failure_rate']*100:>6.1f}% "
                  f"{rep['gap_mm_median']:>12.2f} {rep['gap_mm_p05']:>12.2f} {rep['gap_mm_worst']:>13.2f}")

    a.out.parent.mkdir(parents=True, exist_ok=True)
    a.out.write_text(json.dumps(out, indent=1))
    print(f"\nwrote {a.out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
