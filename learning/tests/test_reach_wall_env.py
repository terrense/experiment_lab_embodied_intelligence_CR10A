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


def test_crashing_early_never_pays(env):
    # regression (PPO v1 learned to crash): standing still for a whole episode must earn more
    # than any collision, whatever happened before it
    obs, _ = env.reset(seed=3)
    still = 0.0
    for _ in range(env.cfg.max_steps):
        obs, r, term, trunc, _ = env.step(np.zeros(6))
        still += r
        if term or trunc:
            break
    assert not env.collided and still > -env.cfg.collision_penalty / 2


def test_window_reward_never_penalises_valid_starts_or_goals():
    # regression (window v1): a 5 cm shaping margin penalised standing AT 68 % of the goals
    from ex_learning.envs import make_env
    win = make_env("window")
    for pool in (win.start_pool, win.goal_pool):
        for q in pool[:64]:
            win._set_q(q)
            assert win.clearance() >= win.cfg.shaping_margin


def test_random_walls_change_every_episode_and_are_observed():
    from ex_learning.envs import make_env
    from ex_learning.envs.cr10a_reach_wall import OOD_WALLS, TRAIN_WALLS
    env = make_env("wall_rand")
    walls = []
    for s in range(6):
        obs, _ = env.reset(seed=s)
        assert obs.shape == (28,) and obs[21] > 0.0          # start collision-free
        walls.append(obs[22:].copy())
        lo, hi = env.box_lo[env.wall_i], env.box_hi[env.wall_i]
        assert np.allclose(obs[22:25], 0.5 * (lo + hi)) and np.allclose(obs[25:], 0.5 * (hi - lo))
        env._set_q(env.q_goal)
        assert env.clearance() > 0.02                         # goal collision-free too
    assert len({tuple(np.round(w, 4)) for w in walls}) == 6
    assert make_env("wall_rand_blind").reset(seed=0)[0].shape == (22,)
    assert OOD_WALLS["height"][0] >= TRAIN_WALLS["height"][1] and OOD_WALLS["width"][0] >= TRAIN_WALLS["width"][1]


def test_fixed_wall_task_unchanged_by_randomisation_code(env):
    # the fixed-wall results must stay reproducible: wall box as in the M2 scene, obs 22
    w = env.box_names.index("wall")
    assert np.allclose(env.box_lo[w], [-0.95, -0.02, 0.0]) and np.allclose(env.box_hi[w], [-0.45, 0.02, 0.9])
    assert env.reset(seed=0)[0].shape == (22,)


@pytest.mark.parametrize("task", ["wall", "wall_rand", "wall_rand_blind", "window"])
def test_reported_clearance_is_the_true_clearance(task):
    # regression: step() once read obs[-1] as the clearance; with the wall in the observation that
    # is the wall half-height, so collisions were never detected (DLS "100 % success" on wall_rand)
    from ex_learning.envs import make_env
    e = make_env(task)
    e.reset(seed=0)
    _, _, _, _, info = e.step(np.zeros(6))
    assert abs(info["clearance"] - min(e.clearance(), e.cfg.clearance_cap)) < 1e-9


def test_straight_line_hits_random_walls_too():
    from ex_learning.baselines import dls_policy, evaluate
    from ex_learning.envs import make_env
    rep = evaluate(make_env("wall_rand"), dls_policy, episodes=4, seed=1000)
    assert rep["collision_rate"] >= 0.5
