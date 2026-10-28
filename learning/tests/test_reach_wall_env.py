import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
pytest.importorskip("mujoco")
pytest.importorskip("xacro")

from ex_learning.envs.cr10a_reach_wall import ReachWallEnv, WallConfig  # noqa: E402


@pytest.fixture(scope="module")
def env():
    return ReachWallEnv(WallConfig(max_steps=150))


def test_obstacles_registered_and_start_is_free(env):
    assert "wall" in env.box_names and "table" in env.box_names and len(env.capsules) == 7
    for s in range(5):
        obs, info = env.reset(seed=s)
        assert obs.shape == (22,) and obs[-1] > 0.0, f"seed {s}: start pose in collision"


def test_goal_is_across_the_wall(env):
    for s in range(5):
        env.reset(seed=s)
        assert np.sign(env.tcp()[1]) != np.sign(env.goal[1])  # wall lies in the x-z plane at y = 0


def test_clearance_detects_penetration(env):
    env.reset(seed=0)
    w = env.box_names.index("wall")
    lo, hi = env.box_lo[w].copy(), env.box_hi[w].copy()
    tcp = env.tcp()  # move the wall box onto the TCP: clearance must turn negative
    env.box_lo[w], env.box_hi[w] = tcp - 0.05, tcp + 0.05
    try:
        assert env.clearance() < 0.0
    finally:
        env.box_lo[w], env.box_hi[w] = lo, hi


def test_clearance_is_continuous_along_a_path(env):
    # regression: mj_geomDistance on meshes gave spurious exact zeros; capsules must not
    env.reset(seed=0)
    prev = None
    for t in np.linspace(0, 1, 200):
        env._set_q(env.q_a + t * (env.q_b - env.q_a))
        c = env.clearance()
        if prev is not None:
            assert abs(c - prev) < 0.05
        prev = c


def test_straight_line_controller_hits_the_wall(env):
    from ex_learning.baselines import dls_policy, evaluate
    rep = evaluate(env, dls_policy, episodes=4, seed=1000)
    assert rep["collision_rate"] >= 0.5  # the task is not solvable by going straight


def test_repulsion_alone_increases_wall_clearance(env):
    # regression: swapped mesh distance endpoints once made the repulsion push INTO the wall
    from ex_learning.baselines import apf_policy
    w = env.box_names.index("wall")
    wall_clear = lambda: min(d for d, (ci, bi) in zip(env.pair_distances()[0], env.pairs) if bi == w)  # noqa: E731
    obs, _ = env.reset(seed=0)
    c0 = wall_clear()
    pol = apf_policy(env, gain=0.0, k_rep=0.1, d0=0.3)
    for _ in range(20):
        obs, *_ = env.step(pol(obs))
    assert wall_clear() > c0
