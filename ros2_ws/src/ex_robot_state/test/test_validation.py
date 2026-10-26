import math

import pytest

from ex_robot_state.core.validation import JointStateValidator, Level, ValidatorConfig, wrap_to_pi

N = ["joint1", "joint2"]


@pytest.fixture
def v(toy_model):
    return JointStateValidator(toy_model.actuated_joints("tcp"), ValidatorConfig(velocity_limit_override=None))


def codes(r):
    return {i.code for i in r.issues}


def test_ok_and_canonical_order(v):
    r = v.validate(["joint2", "joint1"], [0.2, 0.1], 0.0, 1.0)
    assert r.accepted and r.positions == [0.1, 0.2]
    assert "REORDERED" in codes(r) and r.level == Level.OK


@pytest.mark.parametrize("names,pos,code", [
    (["joint1"], [0.1, 0.2], "LENGTH_MISMATCH"),
    (["joint1", "joint1"], [0.1, 0.2], "DUPLICATE_NAMES"),
    (["j1", "j2"], [0.1, 0.2], "MISSING_JOINTS"),
    (N, [math.nan, 0.0], "NON_FINITE"),
    (N, [math.inf, 0.0], "NON_FINITE"),
    (N, [45.0, 0.0], "UNIT_SUSPECT_DEGREES"),
    (N, [3.2, 0.0], "OUT_OF_LIMITS"),
])
def test_rejections(v, names, pos, code):
    r = v.validate(names, pos, 0.0)
    assert not r.accepted and code in codes(r)


def test_extra_joint_is_warning(v):
    r = v.validate(N + ["gripper"], [0.0, 0.0, 0.5], 0.0)
    assert r.accepted and r.level == Level.WARN and "EXTRA_JOINTS" in codes(r)


def test_near_limit_warns(v):
    r = v.validate(N, [2.95, 0.0], 0.0)
    assert r.accepted and "NEAR_LIMIT" in codes(r)


def test_continuous_joint_not_limit_checked(v):
    assert v.validate(N, [0.0, 6.0], 0.0).accepted


def test_jump_detected_and_reference_kept(v):
    assert v.validate(N, [0.0, 0.0], 0.000).accepted
    r = v.validate(N, [0.5, 0.0], 0.008)  # 0.5 rad in 8 ms >> 2 rad/s * 1.5
    assert not r.accepted and "JUMP" in codes(r)
    # rejected sample must not become the new reference
    assert v.validate(N, [0.01, 0.0], 0.016).accepted


def test_velocity_override(toy_model):
    v = JointStateValidator(toy_model.actuated_joints("tcp"), ValidatorConfig(velocity_limit_override=100.0))
    v.validate(N, [0.0, 0.0], 0.0)
    assert v.validate(N, [0.5, 0.0], 0.008).accepted


def test_continuous_wraparound_is_not_a_jump(v):
    v.validate(N, [0.0, math.pi - 0.01], 0.0)
    assert v.validate(N, [0.0, -math.pi + 0.01], 0.008).accepted


def test_resync_after_gap(v):
    v.validate(N, [0.0, 0.0], 0.0)
    r = v.validate(N, [1.0, 0.0], 5.0)
    assert r.accepted and "RESYNC" in codes(r)


def test_stamp_checks(v):
    v.validate(N, [0.0, 0.0], 0.0, stamp=10.0)
    assert "STAMP_REGRESSED" in codes(v.validate(N, [0.0, 0.0], 0.01, stamp=9.0))
    assert "ZERO_STAMP" in codes(v.validate(N, [0.0, 0.0], 0.02, stamp=0.0))


def test_wrap():
    assert wrap_to_pi(3 * math.pi) == pytest.approx(-math.pi)
    assert wrap_to_pi(0.5) == pytest.approx(0.5)
