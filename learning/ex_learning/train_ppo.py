"""PPO on the CR10A MuJoCo tasks (SIMULATION ONLY), stable-baselines3.

    cd learning && python3 -m ex_learning.train_ppo --task wall --timesteps 2000000 --n-envs 12 \
        --out ../data/reports/learning/ppo_wall_s0

Evaluation uses the SAME protocol and seeds as ex_learning.baselines (deterministic policy,
`--eval-episodes` episodes from seed 1000), so the numbers are directly comparable.
"""

from __future__ import annotations

import argparse
import json
import time
from pathlib import Path

import gymnasium as gym
import numpy as np

from .envs import TASKS
from .baselines import evaluate


class GymWrap(gym.Env):
    metadata = {"render_modes": []}

    def __init__(self, task: str, seed: int = 0) -> None:
        from .envs import make_env
        self.env = make_env(task, seed)
        self.observation_space = gym.spaces.Box(-np.inf, np.inf, (self.env.observation_dim,), np.float32)
        self.action_space = gym.spaces.Box(-1.0, 1.0, (6,), np.float32)

    def reset(self, *, seed=None, options=None):
        obs, info = self.env.reset(seed=seed)
        return obs.astype(np.float32), info

    def step(self, action):
        obs, r, term, trunc, info = self.env.step(action)
        return obs.astype(np.float32), float(r), bool(term), bool(trunc), info


def main(argv=None) -> int:
    from stable_baselines3 import PPO
    from stable_baselines3.common.monitor import Monitor
    from stable_baselines3.common.vec_env import SubprocVecEnv, VecNormalize

    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--task", choices=TASKS, default="wall")
    ap.add_argument("--timesteps", type=int, default=2_000_000)
    ap.add_argument("--n-envs", type=int, default=12)
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--eval-episodes", type=int, default=50)
    ap.add_argument("--out", type=Path, required=True)
    a = ap.parse_args(argv)
    a.out.mkdir(parents=True, exist_ok=True)

    def make(rank):
        def _f():
            return Monitor(GymWrap(a.task, seed=a.seed * 1000 + rank), str(a.out / f"monitor_{rank}"))
        return _f
    venv = VecNormalize(SubprocVecEnv([make(i) for i in range(a.n_envs)]), norm_obs=True, norm_reward=True, clip_obs=10.0)
    model = PPO("MlpPolicy", venv, n_steps=1024, batch_size=4096, n_epochs=10, learning_rate=3e-4, gamma=0.99,
                gae_lambda=0.95, clip_range=0.2, ent_coef=0.0, policy_kwargs={"net_arch": [256, 256]},
                seed=a.seed, verbose=0, device="cpu")
    t0 = time.time()
    model.learn(total_timesteps=a.timesteps, progress_bar=False)
    train_s = time.time() - t0
    model.save(a.out / "ppo")
    venv.save(str(a.out / "vecnormalize.pkl"))
    venv.training = False

    eval_env = GymWrap(a.task).env
    def make_policy(env):
        def act(obs):
            o = venv.normalize_obs(obs.astype(np.float32)[None])
            return model.predict(o, deterministic=True)[0][0]
        return act
    rep = evaluate(eval_env, make_policy, a.eval_episodes, 1000)
    rep.update({"task": a.task, "timesteps": a.timesteps, "n_envs": a.n_envs, "seed": a.seed, "train_wall_s": round(train_s, 1)})
    (a.out / "eval.json").write_text(json.dumps(rep, indent=1))
    print(f"PPO {a.task} seed {a.seed}: success {rep['success_rate']*100:.1f}%  collisions {rep['collision_rate']*100:.0f}%  "
          f"final dist median {rep['final_dist_m_median']*1000:.1f} mm  time-to-reach {rep['time_to_reach_s_median']}  "
          f"train {train_s/60:.1f} min")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
