import math
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
from ex_robot_state.core.urdf_model import UrdfModel  # noqa: E402


def _resolve(uri):
    rel = uri.split("package://ex_description/", 1)[1]
    return SRC / "ex_description" / rel


@pytest.fixture(scope="module")
def urdf():
    text = (SRC / "ex_description/urdf/ex_cr10.urdf.xacro").read_text().replace("$(find ex_description)", str(SRC / "ex_description"))
    tmp = Path(os.environ.get("PYTEST_TMP", "/tmp")) / "ex_cr10_mj.xacro"
    tmp.write_text(text)
    return xacro.process_file(str(tmp), mappings={"calibrated": "true"}).toxml()


@pytest.fixture(scope="module")
def mj(urdf):
    return mujoco.MjModel.from_xml_string(build_mjcf(urdf, _resolve))


def test_joints_and_actuators(mj):
    names = [mujoco.mj_id2name(mj, mujoco.mjtObj.mjOBJ_JOINT, i) for i in range(mj.njnt)]
    assert names == JOINTS and mj.nu == 6 and mj.nq == 6
    assert mj.opt.timestep == pytest.approx(0.001)
    lim = DynamicsAssumptions().torque_limit
    assert np.allclose(mj.actuator_ctrlrange[:, 1], lim)


def test_kinematics_identical_to_urdf(mj, urdf):
    """MuJoCo forward kinematics must reproduce the URDF FK (same robot, single source of truth)."""
    m = UrdfModel.from_string(urdf)
    d = mujoco.MjData(mj)
    sid = mujoco.mj_name2id(mj, mujoco.mjtObj.mjOBJ_SITE, "tcp_site")
    rng = np.random.default_rng(0)
    for q in rng.uniform(-2.5, 2.5, (50, 6)):
        d.qpos[:] = q
        mujoco.mj_kinematics(mj, d)
        T = m.fk({n: v for n, v in zip(JOINTS, q)}, "tcp", "world")
        # ~1.5 um: mj_saveLastXML writes body quaternions with finite precision
        assert np.allclose(d.site_xpos[sid], T[:3, 3], atol=1e-5)
        assert np.allclose(d.site_xmat[sid].reshape(3, 3), T[:3, :3], atol=1e-5)


def test_masses_match_urdf_inertials(mj, urdf):
    import xml.etree.ElementTree as ET
    total_urdf = sum(float(m.get("value")) for m in ET.fromstring(urdf).iter("mass"))
    assert mj.body_mass.sum() == pytest.approx(total_urdf, rel=1e-6)


def test_gravity_torque_shape(mj):
    """Static gravity torque at an outstretched pose: J2/J3 carry the load, J1 (vertical axis) none."""
    d = mujoco.MjData(mj)
    d.qpos[:] = [0, math.radians(-90), 0, 0, 0, 0]  # arm horizontal
    mujoco.mj_forward(mj, d)
    tau = d.qfrc_bias.copy()
    assert abs(tau[0]) < 0.01  # calibrated model: J1 axis tilted by ~1e-4 deg -> tiny gravity torque
    assert abs(tau[1]) > 50 and abs(tau[1]) > abs(tau[2]) > abs(tau[4])


def test_identified_preset_scales_masses_and_friction(urdf):
    from ex_mujoco_sim.model_builder import IDENTIFIED_RUN02_MASS_SCALE, dynamics_preset, scale_link_masses
    dyn0, k0 = dynamics_preset("placeholder")
    dyn1, k1 = dynamics_preset("identified_run02")
    assert k0 == 1.0 and k1 == IDENTIFIED_RUN02_MASS_SCALE and dyn1.frictionloss[1] > dyn0.frictionloss[1]
    m0 = mujoco.MjModel.from_xml_string(build_mjcf(urdf, _resolve, dyn0))
    m1 = mujoco.MjModel.from_xml_string(build_mjcf(urdf, _resolve, dyn1))
    scale_link_masses(m1, k1)
    assert abs(m1.body_mass.sum() / m0.body_mass.sum() - k1) < 1e-9
    assert np.allclose(m1.dof_frictionloss, dyn1.frictionloss)
    with pytest.raises(ValueError):
        dynamics_preset("made_up")
