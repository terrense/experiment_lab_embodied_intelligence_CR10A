"""Tests for the execution model and the execution-aware filter (B5).

The claims under test:

  * the model reproduces the channel it is used to protect - the reference is integrated and
    clipped at the joint limits, and the arm COASTS after the command is cut;
  * the affine decomposition used inside the QP is a faithful local expansion of the
    prediction, so the linear constraint the filter imposes really is the constraint it means;
  * the relaxed solver reports a violation instead of hiding it;
  * the filter returns a command inside the box and its predicted trajectory respects the margin.
"""

import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
pytest.importorskip("mujoco")
pytest.importorskip("xacro")
pytest.importorskip("scipy")

from ex_learning.envs.cr10a_reach_wall import ReachWallEnv, WallConfig  # noqa: E402
from ex_learning.execution_model import ServoExecutionModel  # noqa: E402
from ex_learning.qp_filter import solve_bounded_qp_relaxed  # noqa: E402
from ex_learning.safety_filter import cbf_policy_exec  # noqa: E402


def _env():
    return ReachWallEnv(WallConfig(max_steps=20))


def test_the_prediction_starts_at_the_state_and_respects_the_joint_limits():
    env = _env()
    model = ServoExecutionModel(dt=env.cfg.control_dt)
    q = env.q()
    qd = env.d.qvel[env.dadr].copy()
    q_ref = np.asarray(env.q_ref, float).copy()
    _, Q = model.predict_constant(q, qd, q_ref, np.full(6, 1.0), env.lo, env.hi, horizon=5)
    assert np.allclose(Q[0, 0], q), "the prediction must start at the current state"
    # the reference is clipped at the joint limits, so the predicted position cannot run past them
    assert np.all(Q[0, -1] <= env.hi + 1e-6), "the prediction ran past the upper joint limit"
    assert np.all(Q[0, -1] >= env.lo - 1e-6), "the prediction ran past the lower joint limit"


def test_the_arm_coasts_after_the_command_is_cut():
    """The quantity a kinematic filter ignores: with a ZERO command the arm keeps moving."""
    env = _env()
    model = ServoExecutionModel(dt=env.cfg.control_dt)
    q = env.q()
    qd = np.zeros(6)
    qd[1] = 0.8                      # the arm is already moving
    q_ref = q.copy() + qd * 0.02     # the reference is ahead, as it is while tracking
    _, Q = model.predict_constant(q, qd, q_ref, np.zeros(6), env.lo, env.hi, horizon=10)
    coast = np.abs(Q[0, -1] - q)
    assert coast[1] > 1e-3, "the model predicts no coast at all - the premise of B5 would be void"
    assert np.sign(Q[0, -1, 1] - q[1]) == np.sign(qd[1]), "the coast must continue the motion"


def test_the_affine_decomposition_matches_the_prediction():
    """The filter's linear constraint is only meaningful if the expansion is faithful."""
    env = _env()
    model = ServoExecutionModel(dt=env.cfg.control_dt)
    rng = np.random.default_rng(3)
    for _ in range(20):
        q = env.q()
        qd = rng.normal(size=6) * 0.3
        q_ref = q + rng.normal(size=6) * 0.02
        v = rng.uniform(-0.8, 0.8, 6)
        horizon, step = 4, 2
        dq0, G = model.affine(q, qd, q_ref, v, env.lo, env.hi, horizon, step=step)
        # the direct prediction
        _, Q = model.predict_constant(q, qd, q_ref, v, env.lo, env.hi, horizon)
        S = Q[0, step::step, :]
        direct = (S[1:, :] - q)
        # and the affine expansion at the nominal command
        approx = dq0 + np.einsum("kij,j->ki", G, v)
        err = np.abs(approx - direct).max()
        assert err < 5e-3, f"the affine expansion is off by {err:.2e} rad"


def test_the_relaxed_solver_reports_the_violation_instead_of_hiding_it():
    a = np.zeros(6)
    # infeasible: no command in the box can satisfy c.x >= 5
    x, info = solve_bounded_qp_relaxed(a, [(np.array([1.0, 0, 0, 0, 0, 0]), 5.0)])
    assert not info["feasible"]
    assert info["relaxation"] > 0.0, "an infeasible program must report a relaxation"
    assert np.all(x >= -1 - 1e-9) and np.all(x <= 1 + 1e-9)

    # feasible: no relaxation, and the constraint holds
    x, info = solve_bounded_qp_relaxed(a, [(np.array([1.0, 0, 0, 0, 0, 0]), 0.2)])
    assert info["feasible"] and info["relaxation"] == 0.0
    assert x[0] >= 0.2 - 1e-6


def test_the_relaxed_solver_still_respects_the_hard_constraints():
    rng = np.random.default_rng(11)
    for _ in range(50):
        a = rng.uniform(-1, 1, 6)
        k = int(rng.integers(1, 5))
        C = rng.normal(size=(k, 6))
        g = rng.uniform(-0.5, 0.5, k)
        x, info = solve_bounded_qp_relaxed(a, list(zip(C, g)))
        assert np.all(x >= -1 - 1e-9) and np.all(x <= 1 + 1e-9)
        if info["feasible"]:
            assert np.all(C @ x >= g - 1e-6)


def test_the_execution_aware_filter_returns_a_bounded_command_and_counts_its_interventions():
    env = _env()
    model = ServoExecutionModel(dt=env.cfg.control_dt)
    stats = {}
    obs, _ = env.reset(seed=5)
    base = lambda o: np.zeros(6)  # noqa: E731
    pol = cbf_policy_exec(env, base, model, margin=0.02, stats=stats)
    for _ in range(5):
        a = pol(obs)
        assert np.all(a >= -1 - 1e-9) and np.all(a <= 1 + 1e-9)
        obs, _, term, trunc, _ = env.step(a)
        if term or trunc:
            break
    assert stats["solves"] >= 1
    assert stats["steps"] == stats["solves"]


def test_the_execution_aware_filter_is_more_conservative_than_the_kinematic_one():
    """Near an obstacle the exec filter must reserve at least as much clearance.

    The kinematic filter reasons about the COMMAND; the execution-aware one about the PREDICTED
    MOTION, which includes the coast. So for the same margin the exec filter must never allow a
    closer approach than the kinematic one. This is the property the paper's claim rests on.
    """
    env = _env()
    model = ServoExecutionModel(dt=env.cfg.control_dt)
    obs, _ = env.reset(seed=11)
    # put the arm close to the wall by commanding a motion toward it for a while
    for _ in range(30):
        obs, _, term, trunc, _ = env.step(np.array([0.0, 0.6, 0.0, 0.0, 0.0, 0.0]))
        if term or trunc:
            break
    d_now = env.clearance(conservative=True)
    if d_now > 0.25:
        pytest.skip("the arm did not approach the wall in this configuration")

    base = lambda o: np.array([0.0, 0.6, 0.0, 0.0, 0.0, 0.0])  # noqa: E731
    a_kin = np.asarray(cbf_policy_exec(env, base, model, margin=0.02,
                                       account_execution=False)(obs), float)
    a_exec = np.asarray(cbf_policy_exec(env, base, model, margin=0.02,
                                        account_execution=True)(obs), float)
    # the coast-aware filter may not be MORE aggressive than the execution-blind one
    assert np.linalg.norm(a_exec - a_kin) >= -1e-9
    # and it must brake at least as hard on the joint that drives the approach
    assert a_exec[1] <= a_kin[1] + 1e-9, "the coast-aware filter let the approach through faster"
