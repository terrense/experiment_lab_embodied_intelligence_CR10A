"""The confirmatory comparison, with the statistics the protocol promises (SIMULATION ONLY).

The manuscript's section 7.7 reports a per-seed completion difference of +22, +2 and +2
percentage points between the execution-aware filter and the bounded kinematic QP, and says the
three seeds cannot separate that from the between-seed spread. This script produces the
statistical version of that statement:

  * the same scenario seeds for every method, so the methods are PAIRED;
  * the per-episode success and collision flags are kept, not just the aggregates;
  * the paired difference is bootstrapped over the SCENARIOS;
  * the hierarchical bootstrap resamples the TRAINING SEEDS and then the scenarios inside a
    seed, which is the interval that answers "would this hold on a new training seed and a new
    set of scenarios";
  * the naive interval (every cell independent) is computed too, so the report can show how much
    a naive analysis would over-claim;
  * `separability` gives an explicit verdict rather than a p-value.

The output is `data/reports/learning/confirmatory_stats.json`.

    cd learning && ~/ex_rlenv/bin/python ../evaluation/confirmatory_stats.py --seeds 0 1 2
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path
from typing import Dict, List

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "learning"))
sys.path.insert(0, str(ROOT / "evaluation"))

from compare_evaluators import _make  # noqa: E402
from episode_manifest import splits_for  # noqa: E402
from stats import (hierarchical_bootstrap, naive_independent_interval,  # noqa: E402
                   paired_difference, seed_spread, separability)


def main(argv=None) -> int:
    from ex_learning.eval_ppo import ppo_policy_factory
    from ex_learning.execution_model import ServoExecutionModel
    from ex_learning.safety_filter import cbf_policy_exec, cbf_policy_qp

    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--split", default="test")
    ap.add_argument("--task", default="wall_ood")
    ap.add_argument("--seeds", nargs="*", type=int, default=[0, 1, 2])
    ap.add_argument("--episodes", type=int, default=50)
    ap.add_argument("--qp-margin", type=float, default=0.04)
    ap.add_argument("--qp0-margin", type=float, default=0.0)
    ap.add_argument("--exec-margin", type=float, default=0.01)
    ap.add_argument("--n-boot", type=int, default=4000)
    ap.add_argument("--out", type=Path,
                    default=ROOT / "data" / "reports" / "learning" / "confirmatory_stats.json")
    a = ap.parse_args(argv)

    splits = splits_for()
    if a.split not in splits:
        raise SystemExit(f"unknown split {a.split!r}; known: {sorted(splits)}")
    seed0 = splits[a.split][0]

    m = json.loads((ROOT / "data" / "reports" / "learning" / "execution_channel.json")
                   .read_text())["identification"]["model"]
    model = ServoExecutionModel(omega=m["omega"], zeta=m["zeta"], v_gain=m["v_gain"],
                                dt=m["dt"], sub=m.get("sub", 10))

    METHODS = ("qp", "qp0", "exec")

    def build(env, method, base):
        if method == "qp":
            return cbf_policy_qp(env, base(env), margin=a.qp_margin, alpha=5.0, rho=0.0)
        if method == "qp0":
            return cbf_policy_qp(env, base(env), margin=a.qp0_margin, alpha=5.0, rho=0.0)
        return cbf_policy_exec(env, base(env), model, margin=a.exec_margin)

    # per_seed[method][training_seed][scenario_seed] = 1.0 / 0.0
    per_seed: Dict[str, Dict[int, Dict[int, float]]] = {mth: {} for mth in METHODS}
    per_seed_coll: Dict[str, Dict[int, Dict[int, float]]] = {mth: {} for mth in METHODS}

    for s in a.seeds:
        model_dir = ROOT / "data" / "reports" / "learning" / f"ppo_wall_rand_s{s}"
        if not (model_dir / "ppo.zip").exists():
            raise SystemExit(f"missing policy {model_dir}")
        base = ppo_policy_factory(model_dir, a.task)
        for method in METHODS:
            env = _make(a.task, False)
            succ: Dict[int, float] = {}
            coll: Dict[int, float] = {}
            for ep in range(a.episodes):
                es = seed0 + ep
                env.reset(seed=es)
                policy = build(env, method, base)
                hit = False
                ok = False
                for _ in range(env.cfg.max_steps):
                    obs, r, term, trunc, info = env.step(policy(env.obs()))
                    hit = hit or bool(info.get("collision", False))
                    ok = ok or bool(info.get("success", False))
                    if term or trunc:
                        break
                succ[es] = 1.0 if ok else 0.0
                coll[es] = 1.0 if hit else 0.0
            per_seed[method][s] = succ
            per_seed_coll[method][s] = coll
            print(f"seed {s} {method:5s}: success {100*sum(succ.values())/len(succ):5.1f}%  "
                  f"collision {100*sum(coll.values())/len(coll):5.1f}%", flush=True)

    out: Dict = {"split": a.split, "task": a.task, "seeds": a.seeds, "episodes": a.episodes,
                 "seed_start": seed0,
                 "qp_margin": a.qp_margin, "qp0_margin": a.qp0_margin,
                 "exec_margin": a.exec_margin, "n_boot": a.n_boot, "comparisons": {}}

    for other in ("qp", "qp0"):
        if other == "exec":
            continue
        for a_m, b_m, label in (("exec", other, f"exec - {other}"), (other, "exec", f"{other} - exec")):
            key = label.replace(" ", "_")
            hier = hierarchical_bootstrap(per_seed[a_m], per_seed[b_m], n_boot=a.n_boot, seed=1)
            naive = naive_independent_interval(per_seed[a_m], per_seed[b_m], n_boot=a.n_boot, seed=1)
            pooled_a = [v for s in a.seeds for v in per_seed[a_m][s].values()]
            pooled_b = [v for s in a.seeds for v in per_seed[b_m][s].values()]
            paired = paired_difference(pooled_a, pooled_b, n_boot=a.n_boot, seed=1)
            spread_a = seed_spread(per_seed[a_m])
            spread_b = seed_spread(per_seed[b_m])
            sd = float(np.hypot(spread_a["sd"], spread_b["sd"]))
            sep = separability(hier["mean_difference"], sd, len(a.seeds),
                               hier["ci_low"], hier["ci_high"])
            out["comparisons"][key] = {
                "completion": {"hierarchical": hier, "naive": naive, "paired_pooled": paired,
                               "seed_spread_a": spread_a, "seed_spread_b": spread_b,
                               "separability": sep},
                "collision": {
                    "hierarchical": hierarchical_bootstrap(per_seed_coll[a_m], per_seed_coll[b_m],
                                                            n_boot=a.n_boot, seed=1),
                    "naive": naive_independent_interval(per_seed_coll[a_m], per_seed_coll[b_m],
                                                        n_boot=a.n_boot, seed=1),
                },
            }
            print(f"\n== {label} (completion)")
            print(f"   mean difference {100*hier['mean_difference']:+.1f} points   "
                  f"hierarchical 95 % CI [{100*hier['ci_low']:+.1f}, {100*hier['ci_high']:+.1f}]   "
                  f"(width {100*(hier['ci_high']-hier['ci_low']):.1f})")
            print(f"   NAIVE interval (cells treated as independent): "
                  f"[{100*naive['ci_low']:+.1f}, {100*naive['ci_high']:+.1f}]   "
                  f"(width {100*(naive['ci_high']-naive['ci_low']):.1f}) - over-claims by "
                  f"{100*(hier['ci_high']-hier['ci_low'])/(naive['ci_high']-naive['ci_low']):.1f}x")
            print(f"   between-seed spread: {sd:.3f}   per-seed differences (points): "
                  f"{[round(100*v, 1) for v in hier['per_seed_mean_difference']]}")
            print(f"   VERDICT: {sep['verdict']} - {sep['why']}")

    a.out.parent.mkdir(parents=True, exist_ok=True)
    a.out.write_text(json.dumps(out, indent=1))
    print(f"\nwrote {a.out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
