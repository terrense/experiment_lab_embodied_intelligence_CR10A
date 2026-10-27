import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
pytest.importorskip("mujoco")
pytest.importorskip("xacro")

from ex_learning.baselines import dls_policy, evaluate  # noqa: E402
from ex_learning.envs.cr10a_reach import ReachConfig, ReachEnv  # noqa: E402


@pytest.fixture(scope="module")
def env():
    return ReachEnv(ReachConfig(max_steps=150))


def test_api_shapes_and_determinism(env):
    o1, info = env.reset(seed=3)
    assert o1.shape == (21,) and np.isfinite(o1).all()
    o2, r, term, trunc, inf = env.step(np.zeros(6))
    assert o2.shape == (21,) and isinstance(r, float) and not term
    o3, _ = env.reset(seed=3)
    assert np.allclose(o1, o3) and np.allclose(info["goal"], env.goal)


def test_zero_action_holds_position(env):
    env.reset(seed=4)
    p0 = env.tcp()
    for _ in range(25):
        env.step(np.zeros(6))
    assert np.linalg.norm(env.tcp() - p0) < 0.005  # servo + gravity compensation hold the arm


def test_dls_baseline_reaches_most_goals(env):
    rep = evaluate(env, dls_policy, episodes=6, seed=100)
    assert rep["success_rate"] >= 0.8, rep["per_episode"]
