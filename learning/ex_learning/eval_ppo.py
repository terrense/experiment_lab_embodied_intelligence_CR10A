"""Evaluate a trained PPO model on any task with the standard protocol (SIMULATION ONLY).

    cd learning && ~/ex_rlenv/bin/python -m ex_learning.eval_ppo --model ../data/reports/learning/ppo_wall_s0 \
        --task wall_rand_blind --out ../data/reports/learning/gen/ppo_wall_s0_on_wall_rand.json

The task's observation must match what the model was trained on (fixed-wall models: *_blind tasks).
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np

from .baselines import evaluate
from .envs import TASKS, make_env


def ppo_policy_factory(model_dir: Path, task: str):
    from stable_baselines3 import PPO
    from stable_baselines3.common.vec_env import DummyVecEnv, VecNormalize
    from .train_ppo import GymWrap
    model = PPO.load(model_dir / "ppo", device="cpu")
    vn = VecNormalize.load(str(model_dir / "vecnormalize.pkl"), DummyVecEnv([lambda: GymWrap(task)]))
    vn.training = False
    if model.observation_space.shape != vn.observation_space.shape:
        raise ValueError(f"model expects obs {model.observation_space.shape}, task {task} gives {vn.observation_space.shape}")
    return lambda env: (lambda obs: model.predict(vn.normalize_obs(obs.astype(np.float32)[None]), deterministic=True)[0][0])


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--model", type=Path, required=True)
    ap.add_argument("--task", choices=TASKS, required=True)
    ap.add_argument("--episodes", type=int, default=50)
    ap.add_argument("--seed", type=int, default=1000)
    ap.add_argument("--out", type=Path, required=True)
    a = ap.parse_args(argv)
    rep = evaluate(make_env(a.task), ppo_policy_factory(a.model, a.task), a.episodes, a.seed)
    rep.update({"model": str(a.model), "task": a.task})
    a.out.parent.mkdir(parents=True, exist_ok=True)
    a.out.write_text(json.dumps(rep, indent=1))
    print(f"{a.model.name} on {a.task}: success {rep['success_rate']*100:.0f}%  collisions {rep['collision_rate']*100:.0f}%  "
          f"final dist median {rep['final_dist_m_median']*1000:.0f} mm")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
