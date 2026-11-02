"""Evaluate a trained PPO policy wrapped in the safety filter (SIMULATION ONLY).

    cd learning && ~/ex_rlenv/bin/python -m ex_learning.eval_ppo_safe --model ../data/reports/learning/ppo_wall_rand_s0 \\
        --task wall_ood --d0 0.10 --k-rep 0.02 --out ../data/reports/learning/gen_safe/ppo_wall_rand_s0_on_wall_ood_safe.json

The observation must match what the model was trained on (wall_rand policies: wall_obs / wall_rand / wall_ood).
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path

from .baselines import evaluate
from .envs import TASKS, make_env
from .eval_ppo import ppo_policy_factory
from .safety_filter import safe_policy


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--model", type=Path, required=True)
    ap.add_argument("--task", choices=TASKS, required=True)
    ap.add_argument("--d0", type=float, default=0.10, help="repulsion activation distance [m]")
    ap.add_argument("--k-rep", type=float, default=0.02, help="repulsion gain")
    ap.add_argument("--episodes", type=int, default=50)
    ap.add_argument("--seed", type=int, default=1000)
    ap.add_argument("--out", type=Path, required=True)
    a = ap.parse_args(argv)
    base_factory = ppo_policy_factory(a.model, a.task)
    rep = evaluate(make_env(a.task), lambda e: safe_policy(e, base_factory(e), a.d0, a.k_rep), a.episodes, a.seed)
    rep.update({"model": str(a.model), "task": a.task, "d0": a.d0, "k_rep": a.k_rep})
    a.out.parent.mkdir(parents=True, exist_ok=True)
    a.out.write_text(json.dumps(rep, indent=1))
    print(f"{a.model.name} on {a.task} (safe d0={a.d0} k_rep={a.k_rep}): success {rep['success_rate']*100:.0f}%  "
          f"collisions {rep['collision_rate']*100:.0f}%  final dist median {rep['final_dist_m_median']*1000:.0f} mm")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
