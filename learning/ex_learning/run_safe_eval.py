"""Full PPO + safety-filter evaluation: {wall_obs, wall_rand, wall_ood} x 3 seeds x 2 filters (SIMULATION ONLY).

For every generalist model (trained on wall_rand, wall observed) run the standard 50-episode protocol
(seeds 1000-1049) with:
  additive  - the naive always-on potential-field repulsion added to the learned action (d0, k_rep)
  cbf       - the single-constraint control barrier projection (margin, alpha)
writes one JSON per (seed, task, filter) to gen_safe/ and prints a compact table. The raw (no-filter)
numbers are the existing gen/ppo_wall_rand_s{seed}_on_{task}.json files, run with the same seeds.

    cd learning && ~/ex_rlenv/bin/python -m ex_learning.run_safe_eval
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path

from .baselines import evaluate
from .envs import make_env
from .eval_ppo import ppo_policy_factory
from .safety_filter import cbf_policy, safe_policy

TASKS = ("wall_obs", "wall_rand", "wall_ood")
SEEDS = (0, 1, 2)


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--out-dir", default="../data/reports/learning/gen_safe")
    ap.add_argument("--episodes", type=int, default=50)
    ap.add_argument("--seed", type=int, default=1000)
    ap.add_argument("--add-d0", type=float, default=0.10)
    ap.add_argument("--add-k", type=float, default=0.02)
    ap.add_argument("--cbf-margin", type=float, default=0.05)
    ap.add_argument("--cbf-alpha", type=float, default=5.0)
    a = ap.parse_args(argv)
    out = Path(a.out_dir).resolve()
    out.mkdir(parents=True, exist_ok=True)
    print(f"{'seed':>4} {'task':9} {'raw':>18} {'additive':>18} {'cbf':>18}")
    for s in SEEDS:
        model = Path("../data/reports/learning/ppo_wall_rand_s%d" % s)
        for task in TASKS:
            base_factory = ppo_policy_factory(model, task)
            raw = json.loads((Path("../data/reports/learning/gen/ppo_wall_rand_s%d_on_%s.json" % (s, task))).read_text())
            add = evaluate(make_env(task), lambda e: safe_policy(e, base_factory(e), a.add_d0, a.add_k), a.episodes, a.seed)
            cbf = evaluate(make_env(task), lambda e: cbf_policy(e, base_factory(e), a.cbf_margin, a.cbf_alpha), a.episodes, a.seed)
            for name, rep in (("additive", add), ("cbf", cbf)):
                rep.update({"model": str(model), "task": task, "filter": name})
                (out / f"ppo_wall_rand_s{s}_on_{task}_{name}.json").write_text(json.dumps(rep, indent=1))
            f = lambda r: f"{r['success_rate']*100:3.0f}%/{r['collision_rate']*100:3.0f}%"  # noqa: E731
            print(f"{s:>4} {task:9} {f(raw):>18} {f(add):>18} {f(cbf):>18}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
