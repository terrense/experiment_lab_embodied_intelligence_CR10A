"""Watch a policy on the CR10A MuJoCo tasks in the interactive viewer (SIMULATION ONLY).

    cd learning && python3 -m ex_learning.view_policy --task wall --policy apf
    cd learning && ~/ex_rlenv/bin/python -m ex_learning.view_policy --task wall --policy ppo \
        --model ../data/reports/learning/ppo_wall_s0

Episodes run in real time, one after another (seed 1000, 1001, ...: the evaluation episodes).
The goal is drawn as a green sphere (red when the episode ended in a collision), obstacles are
translucent red boxes. Viewer: left-drag rotate, right-drag pan, scroll zoom, double-click select,
Space pause, Tab/Shift-Tab hide the side panels, Esc/close window to quit.
"""

from __future__ import annotations

import argparse
import os
import sys
import time
from pathlib import Path

import mujoco
import mujoco.viewer
import numpy as np

from .baselines import apf_policy, dls_policy


def load_ppo(path: Path, env):
    from stable_baselines3 import PPO
    from stable_baselines3.common.vec_env import DummyVecEnv, VecNormalize
    from .train_ppo import GymWrap
    model = PPO.load(path / "ppo", device="cpu")
    vn = VecNormalize.load(str(path / "vecnormalize.pkl"), DummyVecEnv([lambda: GymWrap("wall")]))
    vn.training = False
    return lambda e: (lambda obs: model.predict(vn.normalize_obs(obs.astype(np.float32)[None]), deterministic=True)[0][0])


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--task", choices=("reach", "wall"), default="wall")
    ap.add_argument("--policy", choices=("dls", "apf", "ppo", "random"), default="apf")
    ap.add_argument("--model", type=Path, default=Path("../data/reports/learning/ppo_wall_s0"))
    ap.add_argument("--episodes", type=int, default=20)
    ap.add_argument("--seed", type=int, default=1000)
    ap.add_argument("--slowmo", type=float, default=1.0, help=">1 = slower than real time")
    a = ap.parse_args(argv)

    if a.task == "wall":
        from .envs.cr10a_reach_wall import ReachWallEnv
        env = ReachWallEnv()
    else:
        from .envs.cr10a_reach import ReachEnv
        env = ReachEnv()
    rng = np.random.default_rng(0)
    make = {"dls": dls_policy, "apf": apf_policy, "random": lambda e: (lambda o: rng.uniform(-1, 1, 6))}.get(a.policy)
    if a.policy == "ppo":
        make = load_ppo(a.model, env)

    v = mujoco.viewer.launch_passive(env.m, env.d)
    v.cam.azimuth, v.cam.elevation, v.cam.distance, v.cam.lookat[:] = 135, -25, 2.6, (-0.4, 0.0, 0.4)
    for ep in range(a.episodes):
        obs, _ = env.reset(seed=a.seed + ep)
        pol = make(env)
        outcome = "timeout"
        for _ in range(env.cfg.max_steps):
            t0 = time.time()
            obs, r, term, trunc, inf = env.step(pol(obs))
            with v.lock():
                v.user_scn.ngeom = 0
                rgba = np.array([0.9, 0.2, 0.2, 0.9] if inf.get("collision") else [0.2, 0.8, 0.3, 0.9])
                mujoco.mjv_initGeom(v.user_scn.geoms[0], mujoco.mjtGeom.mjGEOM_SPHERE, np.array([0.025, 0, 0]),
                                    env.goal, np.eye(3).flatten(), rgba)
                v.user_scn.ngeom = 1
            v.sync()
            if not v.is_running():
                return _quit(v)
            time.sleep(max(0.0, env.cfg.control_dt * a.slowmo - (time.time() - t0)))
            if term or trunc:
                outcome = "COLLISION " + str(getattr(env, "last_pair", "")) if inf.get("collision") else \
                    ("success" if inf["success"] else f"missed by {inf['dist'] * 1000:.0f} mm")
                break
        print(f"episode {ep} (seed {a.seed + ep}): {outcome}", flush=True)
        time.sleep(1.0 * a.slowmo)
    return _quit(v)


def _quit(v) -> int:
    # Under WSLg the passive viewer segfaults when it is closed (exit 139 from v.close() and from
    # the context-manager exit; observed 2026-10-28, MuJoCo 3.3.7). Output is flushed; leave
    # without closing - the OS releases the window and the GL context.
    sys.stdout.flush()
    os._exit(0)


if __name__ == "__main__":
    raise SystemExit(main())
