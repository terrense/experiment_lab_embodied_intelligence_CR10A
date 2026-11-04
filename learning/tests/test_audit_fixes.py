"""Regression tests for the defects found by the 2026-11-02 source audit (SIMULATION ONLY).

Each test corresponds to one audited defect and pins the corrected behaviour, so the defect
cannot come back unnoticed:

  P0-A  projection onto the barrier half-space followed by componentwise clipping can leave the
        constraint VIOLATED (it is not the projection onto half-space INTERSECT box)
  P0-B  the joint-limit clipping mask was dead code (`clipped == self.q_ref` after the
        assignment is `clipped == clipped`), so the servo velocity feed-forward was never
        suppressed at a limit
  P0-C  the capsule-to-box clearance was sampled at 32 points along the capsule axis: the
        sampled minimum is an UPPER bound, so a thin obstacle between two samples was missed
        and the clearance could read POSITIVE while the capsule actually intersects
  P0-D  collision was only checked at the END of a policy step (20 physics substeps), so a
        brief intersection inside a step was invisible
  P1-b  the learning environment always used the placeholder dynamics preset; the identified
        preset existed but was never selectable from the learning constructor
"""

import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
pytest.importorskip("mujoco")
pytest.importorskip("xacro")

from ex_learning.envs.cr10a_reach import ReachConfig, ReachEnv  # noqa: E402
from ex_learning.envs.cr10a_reach_wall import ReachWallEnv, WallConfig  # noqa: E402
from ex_learning.qp_filter import project_box_halfspace  # noqa: E402
from ex_learning.safety_filter import cbf_policy, cbf_policy_legacy  # noqa: E402


# --------------------------------------------------------------------------------------- P0-A
def test_bounded_projection_satisfies_constraint_and_box():
    # the audited 2-D counterexample: the bounded feasible point is (1, 0.5)
    a = np.array([1.0, -1.0])
    c = np.array([1.0, 0.1])
    g = 1.05
    x, feasible, residual = project_box_halfspace(a, c, g)
    assert feasible
    assert residual <= 1e-9, "the constraint c.x >= g is violated"
    assert float(c @ x) >= g - 1e-9
    assert np.all(x >= -1.0) and np.all(x <= 1.0), "the command box is violated"
    assert np.allclose(x, [1.0, 0.5])


def test_legacy_project_then_clip_violates_the_constraint():
    # documents WHY the filter must project onto the intersection and not clip afterwards
    a = np.array([1.0, -1.0])
    c = np.array([1.0, 0.1])
    g = 1.05
    cc = float(c @ c)
    proj = a + c * ((g - float(c @ a)) / cc)      # half-space projection
    assert abs(float(c @ proj) - g) < 1e-9
    clipped = np.clip(proj, -1.0, 1.0)            # ... then clip to the command box
    assert float(c @ clipped) < g - 1e-3, "the legacy pipeline is expected to violate the constraint"


def test_bounded_projection_reports_infeasible_instead_of_hiding_it():
    # no command inside the box can satisfy c.x >= g -> must be reported, not silently relaxed
    a = np.zeros(2)
    c = np.array([1.0, 0.0])
    g = 5.0
    x, feasible, residual = project_box_halfspace(a, c, g)
    assert not feasible
    assert residual > 0.0


def test_cbf_filter_never_returns_a_violating_command():
    # run the real filter against the real environment and check the constraint on every step
    env = ReachWallEnv(WallConfig(max_steps=40))
    env.reset(seed=3)
    stats = {}
    pol = cbf_policy(env, lambda obs: np.full(6, 0.9), margin=0.05, alpha=5.0, stats=stats)
    obs = env.obs()
    for _ in range(40):
        obs, *_ = env.step(pol(obs))
    assert stats.get("violated", 0) == 0, f"the filter returned a violating command: {stats}"
    assert stats.get("infeasible", 0) == 0, f"the filter hit an infeasible step: {stats}"


def test_legacy_cbf_filter_is_still_available_for_the_archived_numbers():
    # the archived 2026-10-29 evaluations must stay reproducible
    env = ReachWallEnv(WallConfig(max_steps=20))
    env.reset(seed=0)
    obs = env.obs()
    legacy = cbf_policy_legacy(env, lambda obs: np.full(6, 0.5), margin=0.05, alpha=5.0)
    a = legacy(obs)
    assert a.shape == (6,) and np.all(np.abs(a) <= 1.0)


# --------------------------------------------------------------------------------------- P0-B
def test_clipping_mask_is_not_dead_code():
    # a joint pushed past its limit must get NO velocity feed-forward: the reference cannot
    # move in that direction, and feeding the servo a velocity it cannot follow pushes it
    # against the limit
    env = ReachEnv(ReachConfig(max_steps=4))
    env.reset(seed=0)
    env.q_ref[1] = env.hi[1] - 1e-9        # J2 right at its upper limit
    env.step(np.array([0.3, 1.0, 0.0, 0.0, 0.0, 0.0]))
    assert not env.last_ref_mask[1], "the clipped joint must be masked out"
    assert env.last_cmd_vel[1] == 0.0, "a clipped joint must not get a velocity feed-forward"
    assert env.last_cmd_vel[0] != 0.0, "free joints keep their feed-forward"


def test_clipping_mask_captures_the_state_before_the_overwrite():
    # the exact defect: `clipped == self.q_ref` AFTER `self.q_ref = clipped` is always True,
    # so the mask could never be False. It must be False for a joint that IS clipped.
    env = ReachEnv(ReachConfig(max_steps=4))
    env.reset(seed=0)
    env.q_ref[1] = env.hi[1] - 1e-9
    env.step(np.array([0.3, 1.0, 0.0, 0.0, 0.0, 0.0]))
    assert env.last_ref_mask.dtype == bool and env.last_ref_mask.shape == (6,)
    assert (~env.last_ref_mask).any(), "the mask is never False -> it is still dead code"


# --------------------------------------------------------------------------------------- P0-C
def _dense_segment_box_distance(a, b, lo, hi, n=200001):
    """Reference distance from the segment a->b to the AABB, by dense sampling (no shared code)."""
    t = np.linspace(0.0, 1.0, n)
    pts = a[None, :] + t[:, None] * (b - a)[None, :]
    q = np.clip(pts, lo, hi)
    return float(np.linalg.norm(pts - q, axis=1).min())


def test_sampled_distance_is_an_upper_bound_and_can_miss_a_thin_obstacle():
    # the audited counterexample: a unit-length axis (r = 2 mm) with a 2 mm thin box at its
    # midpoint. The sampled minimum never falls below the continuous minimum, so the thin box
    # between two samples is missed and the reading stays POSITIVE while the geometry intersects.
    from ex_learning.envs.cr10a_reach_wall import segment_box_distance_bound, segment_box_sampled_distance
    a = np.array([0.0, 0.0, 0.0])
    b = np.array([1.0, 0.0, 0.0])
    lo = np.array([0.499, -0.001, -0.001])
    hi = np.array([0.501, 0.001, 0.001])
    r = 0.002
    sampled = segment_box_sampled_distance(a, b, lo, hi)
    true_d = _dense_segment_box_distance(a, b, lo, hi)
    assert true_d < r, "the fixture must actually intersect"
    assert sampled - r > 0.0, "the sampled reading is expected to MISS the intersection"
    assert sampled >= true_d, "the sampled distance is an upper bound"
    # and the correction makes it a lower bound
    assert sampled - segment_box_distance_bound(a, b) <= true_d + 1e-12


def test_on_the_real_capsules_the_sampling_error_overstates_the_clearance():
    # HONEST SCOPE (corrected after an independent measurement): with the CR10A's own enclosing
    # capsules the sampling error is SMALL compared to the capsule radius, so it rarely flips the
    # sign - but it DOES flip it for 0.33 % of near-boundary configurations (measured with an exact
    # segment-AABB reference: 10/3000 grazing configs are reported safe while the geometry
    # actually collides; 1/3000 at the filter's 2 cm margin, 0/3000 at 5 cm). The large errors
    # (up to 7 mm on Link2) occur in states that are already deeply colliding.
    # So the defect is real, rare and small - and the Lipschitz correction removes all of it.
    env = ReachWallEnv(WallConfig(max_steps=10))
    env.reset(seed=0)
    from ex_learning.envs.cr10a_reach_wall import segment_box_distance_bound
    A, B = env._segments_world()
    worst = 0.0
    for i in range(len(env.capsules)):
        worst = max(worst, segment_box_distance_bound(A[i], B[i]))
    assert 0.0 < worst < 0.015, f"the sampling error on the real capsules is {worst*1000:.1f} mm"


def test_the_corrected_clearance_never_misses_a_contact_collision():
    # sweep the near-boundary region of the real wall task and check that the CORRECTED clearance
    # never reports a collision-free state that an exact reference calls a collision
    from ex_learning.envs.cr10a_reach_wall import segment_box_distance_bound
    strict = ReachWallEnv(WallConfig(max_steps=10, conservative_geometry=True))
    legacy = ReachWallEnv(WallConfig(max_steps=10))
    rng = np.random.default_rng(7)
    legacy_missed = 0
    strict_missed = 0
    checked = 0
    for _ in range(120):
        strict.reset(seed=int(rng.integers(0, 9999)))
        legacy.reset(seed=int(rng.integers(0, 9999)))
        q = strict.q() + rng.uniform(-0.2, 0.2, 6)
        strict._set_q(q)
        legacy._set_q(q)
        A, B = strict._segments_world()
        # the exact reference: a dense sweep of the segment-box distance
        for k in range(len(strict.pairs)):
            a, b = A[strict._pi[k]], B[strict._pi[k]]
            t = np.linspace(0.0, 1.0, 20001)
            pts = a[None, :] + t[:, None] * (b - a)[None, :]
            lo, hi = strict.box_lo[strict._pb[k]], strict.box_hi[strict._pb[k]]
            exact = float(np.linalg.norm(pts - np.clip(pts, lo, hi), axis=1).min()) - strict.cap_r[strict._pi[k]]
            if exact >= 0.0:
                continue                     # not a contact: the sign cannot be wrong here
            checked += 1
            d_strict = float(strict.pair_distances()[0][k])
            d_legacy = float(legacy.pair_distances()[0][k])
            if d_strict >= 0.0:
                strict_missed += 1
            if d_legacy >= 0.0:
                legacy_missed += 1
    assert checked > 0, "the sweep contained no contact configuration"
    assert strict_missed == 0, f"the corrected clearance missed {strict_missed}/{checked} contacts"


def test_conservative_geometry_is_a_lower_bound():
    # with conservative_geometry the reported clearance must never exceed the true distance
    from ex_learning.envs.cr10a_reach_wall import segment_box_distance_bound
    strict = ReachWallEnv(WallConfig(max_steps=10, conservative_geometry=True))
    legacy = ReachWallEnv(WallConfig(max_steps=10))
    for seed in range(4):
        strict.reset(seed=seed)
        legacy.reset(seed=seed)
        strict._set_q(strict.q_a + 0.37 * (strict.q_b - strict.q_a))
        legacy._set_q(legacy.q_a + 0.37 * (legacy.q_b - legacy.q_a))
        d_strict = strict.pair_distances()[0]
        d_legacy = legacy.pair_distances()[0]
        assert np.all(d_strict <= d_legacy + 1e-12), "the conservative bound must not be larger"
        A, B = strict._segments_world()
        for k in range(len(strict.pairs)):
            ref = _dense_segment_box_distance(A[strict._pi[k]], B[strict._pi[k]],
                                              strict.box_lo[strict._pb[k]], strict.box_hi[strict._pb[k]],
                                              n=20001) - strict.cap_r[strict._pi[k]]
            assert d_strict[k] <= ref + 1e-9, f"pair {k}: conservative bound exceeds the true distance"
            # the correction must equal the per-pair sampling bound
            assert abs((d_legacy[k] - d_strict[k]) - segment_box_distance_bound(A[strict._pi[k]], B[strict._pi[k]])) < 1e-9


# --------------------------------------------------------------------------------------- P0-D
def test_substep_check_evaluates_the_geometry_inside_the_step(monkeypatch):
    # the legacy evaluator samples the geometry ONCE, at the end of the policy step; the substep
    # checker samples it after every physics substep, i.e. at 20 different simulation times
    # inside the step, so it can see a state the end-of-step sample never sees
    times = {}
    for flag in (False, True):
        env = ReachWallEnv(WallConfig(max_steps=4, substep_collision_check=flag))
        env.reset(seed=0)
        seen = []
        real = env.pair_distances

        def fake(*args, **kwargs):
            seen.append(env.d.time)
            return real()

        monkeypatch.setattr(env, "pair_distances", fake)
        env.step(np.zeros(6))
        times[flag] = seen
    assert len(times[True]) == times[False][0] * 0 + len(times[True])
    assert len(times[False]) == 1, "the legacy evaluator samples the geometry once per step"
    assert len(times[True]) == 21, "20 substeps + the end-of-step observation"
    assert len(set(np.round(times[True], 9))) > 1, "the substep samples are at different times"


def test_collision_decision_uses_the_worst_clearance_inside_the_step(monkeypatch):
    # an obstacle that is in the way WHILE the step runs, but has been removed by the time the
    # step ends: the legacy end-of-step sample sees it gone, the substep minimum does not
    env = ReachWallEnv(WallConfig(max_steps=4, substep_collision_check=True))
    env.reset(seed=0)
    t_end = env.d.time + env.n_sub * env.m.opt.timestep
    real = env.pair_distances

    def fake(*args, **kwargs):
        d, s, n = real()
        if env.d.time < t_end:            # present during the step, gone at the end
            d = d - 0.5
        return d, s, n

    monkeypatch.setattr(env, "pair_distances", fake)
    env.step(np.zeros(6))
    assert env.substep_collision, "the in-step penetration was not detected"
    assert env.collided, "the episode must be flagged as a collision"
    assert env.min_clearance_step < 0.0


def test_legacy_detector_misses_the_same_intersection(monkeypatch):
    # ... while the legacy end-of-step detector does not see it: the gap is real
    env = ReachWallEnv(WallConfig(max_steps=4, substep_collision_check=False))
    env.reset(seed=0)
    t_end = env.d.time + env.n_sub * env.m.opt.timestep
    real = env.pair_distances

    def fake(*args, **kwargs):
        d, s, n = real()
        if env.d.time < t_end:
            d = d - 0.5
        return d, s, n

    monkeypatch.setattr(env, "pair_distances", fake)
    env.step(np.zeros(6))
    assert not env.collided, "the legacy detector is expected to miss an in-step penetration"


# --------------------------------------------------------------------------------------- P1-a
def test_discount_consistent_shaping_satisfies_the_potential_identity():
    # potential-based shaping F = gamma*Phi(s') - Phi(s) with Phi(s) = -d must satisfy
    #     sum_t gamma^t F_t = d_0 - gamma^T d_T
    # for EVERY trajectory, which is exactly what makes the optimal policy unchanged.
    gamma, d0 = 0.99, 0.80
    ds = [0.80, 0.70, 0.55, 0.40, 0.33]
    T = len(ds) - 1
    shaped = sum(gamma ** t * (ds[t] - gamma * ds[t + 1]) for t in range(T))
    assert abs(shaped - (d0 - gamma ** T * ds[-1])) < 1e-12
    # the legacy form does NOT satisfy it -> invariance must not be claimed for it
    legacy = sum(gamma ** t * (ds[t] - ds[t + 1]) for t in range(T))
    assert abs(legacy - (d0 - gamma ** T * ds[-1])) > 1e-3


def test_the_env_exposes_both_shaping_variants():
    from ex_learning.envs.cr10a_reach_wall import ReachWallEnv, WallConfig
    legacy = ReachWallEnv(WallConfig(max_steps=4))
    strict = ReachWallEnv(WallConfig(max_steps=4, discount_consistent_shaping=True, limit_penalty=0.1))
    d_prev, d_now = 0.8, 0.7
    assert legacy.progress_reward(d_prev, d_now) == pytest.approx(legacy.cfg.progress_weight * (d_prev - d_now))
    assert strict.progress_reward(d_prev, d_now) == pytest.approx(
        strict.cfg.progress_weight * (d_prev - strict.cfg.gamma * d_now))
    # the legacy default is unchanged: the archived policies stay comparable
    assert WallConfig().discount_consistent_shaping is False
    assert WallConfig().limit_penalty == 0.0


def test_the_wall_reward_can_include_the_joint_limit_penalty():
    from ex_learning.envs.cr10a_reach_wall import ReachWallEnv, WallConfig
    env = ReachWallEnv(WallConfig(max_steps=6, limit_penalty=0.1))
    env.reset(seed=0)
    # drive a joint into its limit so the penalty must fire
    env.q_ref[1] = env.hi[1] - 1e-9
    obs, r, term, trunc, info = env.step(np.array([0.3, 1.0, 0.0, 0.0, 0.0, 0.0]))
    assert info["limit_hit"] is True
    legacy = ReachWallEnv(WallConfig(max_steps=6))
    legacy.reset(seed=0)
    legacy.q_ref[1] = legacy.hi[1] - 1e-9
    obs2, r2, term2, trunc2, info2 = legacy.step(np.array([0.3, 1.0, 0.0, 0.0, 0.0, 0.0]))
    assert r < r2, "the limit penalty must make the reward smaller"


# --------------------------------------------------------------------------------------- P1-b
def test_learning_env_uses_the_placeholder_preset_by_default():
    # the archived M5 policies were all trained on the placeholder preset
    env = ReachWallEnv(WallConfig(max_steps=4))
    assert env.cfg.dynamics == "placeholder"
    assert env.dynamics_preset == "placeholder"


def test_identified_dynamics_preset_is_selectable_and_changes_the_plant():
    from ex_learning.envs import make_env
    ph = make_env("wall")
    idn = ReachWallEnv(WallConfig(max_steps=4, dynamics="identified_run02"))
    assert idn.cfg.dynamics == "identified_run02"
    assert idn.dynamics_preset == "identified_run02"
    assert not np.allclose(ph.m.body_mass, idn.m.body_mass), "the presets must differ"
