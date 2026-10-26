import math

import numpy as np
import pytest

from ex_robot_state.core.urdf_model import UrdfModel


def test_toy_chain_and_fk(toy_model):
    m = toy_model
    assert m.root_link == "world"
    assert [j.name for j in m.actuated_joints("tcp")] == ["joint1", "joint2"]
    T = m.fk({"joint1": 0.0, "joint2": 0.0}, "tcp")
    assert np.allclose(T[:3, 3], [1.0, 0.0, 0.8])
    # joint1 = +90deg about z: arm swings from +x to +y
    T = m.fk({"joint1": math.pi / 2, "joint2": 0.0}, "tcp")
    assert np.allclose(T[:3, 3], [0.0, 1.0, 0.8], atol=1e-12)
    # joint2 = +90deg about y: the 0.2 m tool (along +z) tips to +x
    T = m.fk({"joint1": 0.0, "joint2": math.pi / 2}, "tcp")
    assert np.allclose(T[:3, 3], [1.2, 0.0, 0.6], atol=1e-12)


def test_limits_and_types(toy_model):
    j1, j2 = toy_model.joints["joint1"], toy_model.joints["joint2"]
    assert (j1.lower, j1.upper, j1.velocity) == (-3.0, 3.0, 2.0)
    assert j2.is_continuous and math.isinf(j2.lower)


def test_missing_joint_raises(toy_model):
    with pytest.raises(KeyError):
        toy_model.fk({"joint1": 0.0}, "tcp")


def test_all_link_poses_consistent_with_fk(toy_model):
    q = {"joint1": 0.3, "joint2": -1.1}
    assert np.allclose(toy_model.all_link_poses(q)["tcp"], toy_model.fk(q, "tcp"))


# ---------------------------------------------------------------- real CR10
def test_cr10_structure(cr10_model):
    m = cr10_model
    assert m.root_link == "world"
    names = [j.name for j in m.actuated_joints("tcp")]
    assert names == [f"joint{i}" for i in range(1, 7)]
    # CR10A datasheet overlay (scripts/vendor_dobot_cr10.py): J3 +-160 deg, others +-360 deg
    lim = {j.name: (j.lower, j.upper) for j in m.actuated_joints("tcp")}
    assert lim["joint3"] == pytest.approx((-math.radians(160), math.radians(160)), abs=1e-4)
    for n in ("joint1", "joint2", "joint4", "joint5", "joint6"):
        assert lim[n] == pytest.approx((-2 * math.pi, 2 * math.pi), abs=1e-4)
    for f in ("world", "base_link", "Link6", "flange", "tcp"):
        assert f in {m.root_link, *(j.child for j in m.joints.values())}


def test_cr10_joint1_rotates_about_world_z(cr10_model):
    """Physical sanity: J1 must rotate the whole arm about the vertical axis."""
    q0 = {f"joint{i}": v for i, v in enumerate([0.0, 0.3, 1.2, -0.4, -1.4, 0.5], 1)}
    p0 = cr10_model.fk(q0, "flange")[:3, 3]
    q1 = dict(q0, joint1=0.6)
    p1 = cr10_model.fk(q1, "flange")[:3, 3]
    assert p1[2] == pytest.approx(p0[2], abs=1e-12)
    assert math.hypot(*p1[:2]) == pytest.approx(math.hypot(*p0[:2]), abs=1e-12)
    ang = math.atan2(p1[1], p1[0]) - math.atan2(p0[1], p0[0])
    assert math.remainder(ang - 0.6, 2 * math.pi) == pytest.approx(0.0, abs=1e-9)


def test_cr10_link_lengths(cr10_model):
    """Distances between joint origins are joint-angle invariant -> check the
    official CR10 numbers (mm): base->J2 height 176.5, upper arm 607, forearm 568."""
    q = {f"joint{i}": v for i, v in enumerate([0.4, -0.7, 1.1, 0.2, 0.9, -0.3], 1)}
    P = {k: v[:3, 3] for k, v in cr10_model.all_link_poses(q).items()}
    assert np.linalg.norm(P["Link2"] - P["base_link"]) * 1000 == pytest.approx(176.5, abs=1e-6)
    assert np.linalg.norm(P["Link3"] - P["Link2"]) * 1000 == pytest.approx(607.0, abs=1e-6)
    assert np.linalg.norm(P["Link4"] - P["Link3"]) * 1000 == pytest.approx(math.hypot(568, 191), abs=1e-6)
