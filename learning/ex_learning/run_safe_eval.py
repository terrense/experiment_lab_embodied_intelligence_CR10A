"""Full PPO + safety-filter evaluation: {wall_obs, wall_rand, wall_ood} x 3 seeds x 2 filters (SIMULATION ONLY).

For every generalist model (trained on wall_rand, wall observed) run the standard 50-episode protocol
(seeds 1000-1049) with:
  additive  - the naive always-on potential-field repulsion added to the learned action (d0, k_rep)
  cbf       - the single-constraint control barrier projection (margin, alpha)
writes one JSON per (seed, task, filter) to gen_safe/ and prints a compact table. The raw (no-filter)
numbers are the existing gen/ppo_wall_rand_s{seed}_on_{task}.json files, run with the same seeds.

EVERY output records a full manifest: the filter settings, the environment configuration, the
plant preset, the collision-evaluator variant, the episode seed range, the git commit and the
SHA-256 of the checkpoint. Without those, an archived number cannot be reproduced from itself -
which is exactly the defect P1-d of the 2026-11-02 audit.

    cd learning && ~/ex_rlenv/bin/python -m ex_learning.run_safe_eval
"""

from __future__ import annotations

import argparse
import hashlib
import json
import subprocess
from dataclasses import asdict
from pathlib import Path

from .baselines import evaluate
from .envs import make_env
from .eval_ppo import ppo_policy_factory
from .safety_filter import cbf_policy, cbf_policy_legacy, safe_policy

TASKS = ("wall_obs", "wall_rand", "wall_ood")
SEEDS = (0, 1, 2)


def _sha256(path: Path) -> str:
    h = hashlib.sha256()
    with open(path, "rb") as f:
        for chunk in iter(lambda: f.read(1 << 20), b""):
            h.update(chunk)
    return h.hexdigest()[:16]


def _git_commit(repo_root: Path) -> str:
    try:
        return subprocess.check_output(["git", "-C", str(repo_root), "rev-parse", "HEAD"],
                                       text=True, stderr=subprocess.DEVNULL).strip()
    except Exception:
        return "UNKNOWN"


def _env_manifest(env) -> dict:
    cfg = env.cfg
    out = {"cfg_class": type(cfg).__name__}
    for k, v in asdict(cfg).items():
        out[k] = v
    out["dynamics_preset"] = getattr(env, "dynamics_preset", "unknown")
    out["observation_dim"] = env.observation_dim
    out["collision_evaluator"] = ("corrected" if getattr(cfg, "conservative_geometry", False)
                                  or getattr(cfg, "substep_collision_check", False) else "legacy")
    return out


def build_manifest(model: Path, task: str, filter_name: str, params: dict, env, episodes: int,
                   seed: int, repo_root: Path) -> dict:
    """Everything needed to reproduce this number."""
    ckpt = model / "ppo.zip"
    m = {"model": str(model), "task": task, "filter": filter_name,
         "filter_params": dict(params),
         "env": _env_manifest(env),
         "episodes": episodes, "seed_start": seed, "seed_end_exclusive": seed + episodes,
         "checkpoint_sha256_16": _sha256(ckpt) if ckpt.exists() else "MISSING",
         "git_commit": _git_commit(repo_root)}
    try:
        import sys
        sys.path.insert(0, str(repo_root / "evaluation"))
        from episode_manifest import manifest_hash, build_manifest as build_ep_manifest
        m["episode_manifest_hash"] = manifest_hash(build_ep_manifest((task,)))
    except Exception as exc:                                  # pragma: no cover
        m["episode_manifest_hash"] = f"UNAVAILABLE ({exc})"
    return m


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--out-dir", default="../data/reports/learning/gen_safe")
    ap.add_argument("--episodes", type=int, default=50)
    ap.add_argument("--seed", type=int, default=1000)
    ap.add_argument("--add-d0", type=float, default=0.10)
    ap.add_argument("--add-k", type=float, default=0.02)
    ap.add_argument("--cbf-margin", type=float, default=0.05)
    ap.add_argument("--cbf-alpha", type=float, default=5.0)
    ap.add_argument("--cbf-d-max", type=float, default=0.30)
    ap.add_argument("--cbf-variant", default="bounded", choices=["bounded", "legacy"],
                    help="'bounded' is the corrected projection; 'legacy' reproduces the archived "
                         "2026-10-29 numbers (the filter could violate its own constraint)")
    ap.add_argument("--corrected-evaluator", action="store_true",
                    help="use the conservative geometry and the per-substep collision check")
    a = ap.parse_args(argv)
    out = Path(a.out_dir).resolve()
    out.mkdir(parents=True, exist_ok=True)
    repo_root = Path(__file__).resolve().parents[2]

    print(f"{'seed':>4} {'task':9} {'raw':>18} {'additive':>18} {'cbf':>18}")
    for s in SEEDS:
        model = Path("../data/reports/learning/ppo_wall_rand_s%d" % s)
        for task in TASKS:
            base_factory = ppo_policy_factory(model, task)
            raw = json.loads((Path("../data/reports/learning/gen/ppo_wall_rand_s%d_on_%s.json" % (s, task))).read_text())

            def env_for(task=task):
                if not a.corrected_evaluator:
                    return make_env(task)
                return make_env(task, conservative_geometry=True, substep_collision_check=True)

            add_env = env_for()
            add = evaluate(add_env, lambda e: safe_policy(e, base_factory(e), a.add_d0, a.add_k),
                           a.episodes, a.seed)
            cbf_env = env_for()
            cbf_fn = cbf_policy if a.cbf_variant == "bounded" else cbf_policy_legacy
            cbf = evaluate(cbf_env, lambda e: cbf_fn(e, base_factory(e), a.cbf_margin, a.cbf_alpha,
                                                     a.cbf_d_max), a.episodes, a.seed)
            for name, rep, params, menv in (
                    ("additive", add, {"d0": a.add_d0, "k_rep": a.add_k}, add_env),
                    ("cbf", cbf, {"margin": a.cbf_margin, "alpha": a.cbf_alpha, "d_max": a.cbf_d_max,
                                  "variant": a.cbf_variant}, cbf_env)):
                rep.update(build_manifest(model, task, name, params, menv, a.episodes, a.seed, repo_root))
                (out / f"ppo_wall_rand_s{s}_on_{task}_{name}.json").write_text(json.dumps(rep, indent=1))
            f = lambda r: f"{r['success_rate']*100:3.0f}%/{r['collision_rate']*100:3.0f}%"  # noqa: E731
            print(f"{s:>4} {task:9} {f(raw):>18} {f(add):>18} {f(cbf):>18}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
