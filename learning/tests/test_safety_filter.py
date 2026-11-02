import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
pytest.importorskip("mujoco")
pytest.importorskip("xacro")

from ex_learning.envs.cr10a_reach_wall import ReachWallEnv, WallConfig  # noqa: E402
from ex_learning.safety_filter import cbf_policy, safe_policy  # noqa: E402


@pytest.fixture
def env():
    return ReachWallEnv(WallConfig(max_steps=150))


def _park_all_obstacles_far(env):
    # move every registered box far away so no link point is within the filter's d0
    for i in range(len(env.box_names)):
        env.box_lo[i], env.box_hi[i] = np.array([9.0, 9.0, 9.0]), np.array([9.1, 9.1, 9.1])


def test_filter_is_transparent_far_from_obstacle(env):
    # far from every obstacle the repulsion set is empty and the filter returns the base action unchanged
    env.reset(seed=0)
    _park_all_obstacles_far(env)
    assert len(env.repulsion_points(0.10)) == 0
    base = lambda obs: np.full(6, 0.5)  # noqa: E731
    a = safe_policy(env, base, d0=0.10, k_rep=0.02)(env.obs())
    assert np.allclose(a, 0.5)


def test_filter_repels_near_wall(env):
    # near the wall, a zero base action + repulsion must push the closest point away (clearance grows)
    w = env.box_names.index("wall")
    wall_clear = lambda: min(d for d, (ci, bi) in zip(env.pair_distances()[0], env.pairs) if bi == w)  # noqa: E731
    env.reset(seed=0)
    c0 = wall_clear()
    pol = safe_policy(env, lambda obs: np.zeros(6), d0=0.3, k_rep=0.1)
    obs = env.obs()
    for _ in range(20):
        obs, *_ = env.step(pol(obs))
    assert wall_clear() > c0


def test_cbf_is_transparent_far_from_obstacle(env):
    # far from every obstacle the CBF constraint is inactive: the base action passes through unchanged
    env.reset(seed=0)
    _park_all_obstacles_far(env)
    base = lambda obs: np.full(6, 0.3)  # noqa: E731
    a = cbf_policy(env, base, margin=0.05, alpha=5.0)(env.obs())
    assert np.allclose(a, 0.3)


def test_cbf_prevents_collision_while_dls_collides(env):
    # a straight-line controller drives into the wall and collides; the CBF-wrapped version must not
    from ex_learning.baselines import dls_policy
    env.reset(seed=0)
    raw = dls_policy(env)
    obs = env.obs()
    for _ in range(env.cfg.max_steps):
        obs, r, term, trunc, _ = env.step(raw(obs))
        if term or trunc:
            break
    assert env.collided  # the test is only meaningful if the raw controller actually collides here

    env.reset(seed=0)
    safe = cbf_policy(env, dls_policy(env), margin=0.05, alpha=5.0)
    obs = env.obs()
    for _ in range(env.cfg.max_steps):
        obs, r, term, trunc, _ = env.step(safe(obs))
        if term or trunc:
            break
    assert not env.collided
