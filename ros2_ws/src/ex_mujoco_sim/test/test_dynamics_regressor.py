"""ex_robot_state.core.dynamics (linear-in-parameters RNEA) against MuJoCo's own inverse dynamics."""
import os
import sys
from pathlib import Path

import numpy as np
import pytest

PKG = Path(__file__).resolve().parents[1]
SRC = PKG.parent
sys.path.insert(0, str(PKG))
sys.path.insert(0, str(SRC / "ex_robot_state"))

mujoco = pytest.importorskip("mujoco")
xacro = pytest.importorskip("xacro")

from ex_mujoco_sim.model_builder import JOINTS, DynamicsAssumptions, build_mjcf  # noqa: E402
from ex_robot_state.core.dynamics import (ChainDynamics, add_point_mass, base_columns,  # noqa: E402
                                          random_states, to_base)


def _resolve(uri):
    return SRC / "ex_description" / uri.split("package://ex_description/", 1)[1]


@pytest.fixture(scope="module")
def urdf():
    text = (SRC / "ex_description/urdf/ex_cr10.urdf.xacro").read_text().replace("$(find ex_description)", str(SRC / "ex_description"))
    tmp = Path(os.environ.get("PYTEST_TMP", "/tmp")) / "ex_cr10_dyn.xacro"
    tmp.write_text(text)
    return xacro.process_file(str(tmp), mappings={"calibrated": "true"}).toxml()


LO = np.array([-6.0, -6.0, -2.7, -6.0, -6.0, -6.0])  # inside the joint limits: no limit constraint forces
HI = -LO


def _mj(urdf, dyn):
    m = mujoco.MjModel.from_xml_string(build_mjcf(urdf, _resolve, dyn))
    adr = [m.jnt_dofadr[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, n)] for n in JOINTS]
    return m, np.array(adr)


def _mj_inverse(m, adr, q, qd, qdd):
    d = mujoco.MjData(m)
    out = np.empty_like(q)
    for k in range(len(q)):
        d.qpos[adr], d.qvel[adr], d.qacc[adr] = q[k], qd[k], qdd[k]
        mujoco.mj_inverse(m, d)
        out[k] = d.qfrc_inverse[adr]
    return out


def test_rigid_body_matches_mujoco(urdf):
    dyn = DynamicsAssumptions(damping=[0.0] * 6, armature=[0.0] * 6, frictionloss=[0.0] * 6)
    m, adr = _mj(urdf, dyn)
    cd = ChainDynamics(urdf)
    q, qd, qdd = random_states(6, 200, np.random.default_rng(0), LO, HI)
    tau_mj = _mj_inverse(m, adr, q, qd, qdd)
    tau = cd.regressor(q, qd, qdd, extras=()) @ cd.theta_urdf
    assert np.max(np.abs(tau - tau_mj)) < 1e-6 * max(1.0, np.max(np.abs(tau_mj))) * 10


def test_armature_and_viscous_columns_match_mujoco(urdf):
    dyn = DynamicsAssumptions(frictionloss=[0.0] * 6)  # damping + armature on, Coulomb off (constraint in MJ)
    m, adr = _mj(urdf, dyn)
    cd = ChainDynamics(urdf)
    q, qd, qdd = random_states(6, 100, np.random.default_rng(1), LO, HI)
    theta = np.concatenate([cd.theta_urdf, dyn.armature, dyn.damping])
    tau = cd.regressor(q, qd, qdd, extras=("armature", "viscous")) @ theta
    assert np.allclose(tau, _mj_inverse(m, adr, q, qd, qdd), atol=1e-4)


def test_gravity_only_static_torque(urdf):
    cd = ChainDynamics(urdf)
    q = np.zeros((1, 6))
    tau = cd.regressor(q, q, q, extras=()) @ cd.theta_urdf
    assert np.all(np.isfinite(tau)) and abs(tau[0, 0]) < 1e-3  # J1 ~vertical (calibrated tilt ~2e-6 rad): ~no gravity torque


def test_point_mass_tool_equals_heavier_link(urdf):
    cd = ChainDynamics(urdf)
    th = cd.theta_urdf.copy()
    th[50:60] = add_point_mass(th[50:60], 0.4, [0, 0, 0.05])
    q, qd, qdd = random_states(6, 20, np.random.default_rng(2))
    Y = cd.regressor(q, qd, qdd, extras=())
    d = (Y @ th - Y @ cd.theta_urdf)
    assert np.abs(d).max() > 1e-3  # the tool changes the torques


def test_base_parameters_are_identifiable_and_reproduce_torque(urdf):
    cd = ChainDynamics(urdf)
    rng = np.random.default_rng(3)
    q, qd, qdd = random_states(6, 400, rng)
    Y = cd.regressor(q, qd, qdd, extras=("armature", "viscous", "coulomb")).reshape(-1, cd.n_rigid + 18)
    ind, dep = base_columns(Y)
    assert len(ind) == 54  # 36 rigid base params of a 6R chain + 18 joint terms (armature/viscous/Coulomb)
    theta = np.concatenate([cd.theta_urdf, np.ones(18)])
    tb = to_base(theta, ind, dep)
    tau = Y @ theta
    rel = lambda x: np.linalg.norm(x - tau) / np.linalg.norm(tau)  # noqa: E731
    assert rel(Y[:, ind] @ tb) < 1e-4  # dropped weak direction (calibration tilt) costs ~4e-5
    # least squares recovers the base parameters from noise-free data
    est = np.linalg.lstsq(Y[:, ind], tau, rcond=None)[0]
    assert rel(Y[:, ind] @ est) < 1e-4
