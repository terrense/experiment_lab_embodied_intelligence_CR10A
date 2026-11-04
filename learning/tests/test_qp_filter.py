"""Tests for the bounded multi-constraint QP (the B3 baseline of the research plan).

The point of the QP is that the command BOUNDS are inside the optimisation: the original filter
projected onto one half-space and clipped afterwards, which can leave the constraint violated.
"""

import sys
import time
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
pytest.importorskip("mujoco")
pytest.importorskip("xacro")
pytest.importorskip("scipy")

from ex_learning.envs.cr10a_reach_wall import ReachWallEnv, WallConfig  # noqa: E402
from ex_learning.qp_filter import project_box_halfspace, solve_bounded_qp  # noqa: E402
from ex_learning.safety_filter import cbf_policy_qp  # noqa: E402


def test_the_qp_respects_the_bounds_and_every_constraint():
    rng = np.random.default_rng(0)
    for _ in range(200):
        a = rng.uniform(-1, 1, 6)
        k = int(rng.integers(1, 6))
        C = rng.normal(size=(k, 6))
        g = rng.uniform(-1.0, 0.3, k)
        x, info = solve_bounded_qp(a, list(zip(C, g)))
        assert np.all(x >= -1.0 - 1e-9) and np.all(x <= 1.0 + 1e-9), "the command left the box"
        # every constraint must hold unless the solver reported infeasibility
        if info["feasible"]:
            assert np.all(C @ x >= g - 1e-6), f"a constraint was violated: {info['residuals']}"


def test_the_qp_reports_infeasibility_instead_of_hiding_it():
    # no command in the box can satisfy c.x >= 5 with c = (1, 0, ...): the QP must say so
    a = np.zeros(6)
    C = np.zeros((1, 6))
    C[0, 0] = 1.0
    x, info = solve_bounded_qp(a, [(C[0], 5.0)])
    assert not info["feasible"]
    assert info["residuals"][0] > 0.0


def test_the_qp_is_the_minimum_deviation_from_the_nominal_command():
    a = np.array([1.0, -1.0, 0.0, 0.0, 0.0, 0.0])
    C = np.array([[1.0, 0.1, 0.0, 0.0, 0.0, 0.0]])
    g = np.array([1.05])
    x, info = solve_bounded_qp(a, [(C[0], g[0])])
    assert info["feasible"]
    assert float(C[0] @ x) >= 1.05 - 1e-6
    assert np.all(x >= -1.0) and np.all(x <= 1.0)
    # it must be at least as good as the analytic single-constraint projection
    ref, _, _ = project_box_halfspace(a, C[0], g[0])
    assert float(np.sum((x - a) ** 2)) <= float(np.sum((ref - a) ** 2)) + 1e-9


def test_the_all_pair_qp_filter_never_returns_a_violating_command():
    env = ReachWallEnv(WallConfig(max_steps=40))
    env.reset(seed=3)
    stats = {}
    pol = cbf_policy_qp(env, lambda obs: np.full(6, 0.9), margin=0.05, alpha=5.0, stats=stats)
    obs = env.obs()
    for _ in range(40):
        obs, *_ = env.step(pol(obs))
    assert stats.get("solves", 0) > 0, "the QP was never engaged"
    assert stats.get("infeasible", 0) == 0, f"the QP reported an infeasible step: {stats}"
    assert stats.get("max_violation", 0.0) <= 1e-6, f"the QP returned a violating command: {stats}"


def test_the_qp_filter_is_fast_enough_for_the_control_rate():
    env = ReachWallEnv(WallConfig(max_steps=20))
    env.reset(seed=1)
    stats = {}
    pol = cbf_policy_qp(env, lambda obs: np.full(6, 0.5), margin=0.05, alpha=5.0, stats=stats)
    obs = env.obs()
    t0 = time.perf_counter()
    for _ in range(20):
        obs, *_ = env.step(pol(obs))
    dt = (time.perf_counter() - t0) / 20
    assert dt < 0.02, f"one control step took {dt*1000:.1f} ms - too slow for 50 Hz"
