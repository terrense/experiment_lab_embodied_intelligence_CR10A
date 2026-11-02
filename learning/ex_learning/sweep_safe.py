"""Sweep safety-filter parameters on tuning seeds, for one model and task (SIMULATION ONLY).

    cd learning && ~/ex_rlenv/bin/python -m ex_learning.sweep_safe --model ../data/reports/learning/ppo_wall_rand_s0 --task wall_ood --mode cbf
"""

from __future__ import annotations

import argparse
from pathlib import Path

from .baselines import evaluate
from .envs import make_env
from .eval_ppo import ppo_policy_factory
from .safety_filter import cbf_policy, safe_policy


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--model", type=Path, required=True)
    ap.add_argument("--task", default="wall_ood")
    ap.add_argument("--episodes", type=int, default=20)
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--mode", choices=("additive", "cbf"), default="cbf")
    ap.add_argument("--p1", default="0.02,0.05", help="cbf: margin; additive: d0")
    ap.add_argument("--p2", default="1,3,5,10", help="cbf: alpha; additive: k_rep")
    a = ap.parse_args(argv)
    base_factory = ppo_policy_factory(a.model, a.task)
    for p1 in [float(x) for x in a.p1.split(",")]:
        for p2 in [float(x) for x in a.p2.split(",")]:
            if a.mode == "cbf":
                mk = lambda e, m=p1, al=p2: cbf_policy(e, base_factory(e), margin=m, alpha=al)  # noqa: E731
            else:
                mk = lambda e, d=p1, k=p2: safe_policy(e, base_factory(e), d0=d, k_rep=k)  # noqa: E731
            rep = evaluate(make_env(a.task), mk, a.episodes, a.seed)
            p1n, p2n = ("margin", "alpha") if a.mode == "cbf" else ("d0", "k_rep")
            print(f"{p1n}={p1:.2f} {p2n}={p2:5.2f}: success {rep['success_rate']*100:4.0f}%  "
                  f"coll {rep['collision_rate']*100:4.0f}%  dist {rep['final_dist_m_median']*1000:4.0f} mm")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
