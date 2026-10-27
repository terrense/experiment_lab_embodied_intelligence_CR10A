"""URDF -> MuJoCo MJCF for the CR10A, built at runtime from the SAME robot_description
MoveIt uses (single source of truth: kinematics can never diverge).

Kinematics and link inertias come from the URDF (official Dobot CAD values, optionally the
on-site calibrated origins). Everything MuJoCo needs beyond that is an explicit, documented
ASSUMPTION (`DynamicsAssumptions`) until identified from real data:
  joint damping, armature (reflected rotor inertia), friction loss, actuator torque limits.

Robot geometry is visual-only in physics (contype=conaffinity=0): collision avoidance is the
planner's job here; MuJoCo provides the dynamics of the moving arm.
"""

from __future__ import annotations

import os
import tempfile
import xml.etree.ElementTree as ET
from dataclasses import dataclass, field
from pathlib import Path
from typing import Callable, Dict, List

JOINTS = [f"joint{i}" for i in range(1, 7)]


@dataclass
class DynamicsAssumptions:
    """Per-joint values, J1..J6. UNVERIFIED placeholders - see docs/M4_MUJOCO.md."""
    damping: List[float] = field(default_factory=lambda: [8.0, 8.0, 5.0, 1.0, 1.0, 0.5])         # N m s/rad
    armature: List[float] = field(default_factory=lambda: [0.40, 0.40, 0.20, 0.05, 0.05, 0.02])  # kg m^2
    frictionloss: List[float] = field(default_factory=lambda: [3.0, 3.0, 2.0, 0.8, 0.8, 0.4])   # N m
    torque_limit: List[float] = field(default_factory=lambda: [300.0, 300.0, 150.0, 60.0, 60.0, 30.0])  # N m
    timestep: float = 0.001


def _rewrite_meshes(root: ET.Element, resolve: Callable[[str], Path]) -> None:
    for m in root.iter("mesh"):
        path = resolve(m.get("filename"))
        if path is None or not Path(path).exists():
            raise FileNotFoundError(m.get("filename"))
        m.set("filename", str(Path(path).resolve()))


def build_mjcf(urdf_xml: str, resolve: Callable[[str], Path], dyn: DynamicsAssumptions | None = None) -> str:
    import mujoco

    dyn = dyn or DynamicsAssumptions()
    root = ET.fromstring(urdf_xml)
    _rewrite_meshes(root, resolve)
    # drop ros2_control / gazebo extensions, they mean nothing to MuJoCo
    for tag in ("ros2_control", "gazebo"):
        for el in root.findall(tag):
            root.remove(el)
    ext = ET.SubElement(root, "mujoco")
    ET.SubElement(ext, "compiler", {"discardvisual": "false", "fusestatic": "false",
                                    "balanceinertia": "true", "strippath": "false"})
    model = mujoco.MjModel.from_xml_string(ET.tostring(root, encoding="unicode"))
    fd, tmp = tempfile.mkstemp(suffix=".xml")
    os.close(fd)
    try:
        mujoco.mj_saveLastXML(tmp, model)
        mj = ET.fromstring(Path(tmp).read_text())
    finally:
        os.unlink(tmp)

    mj.set("model", "ex_cr10a")
    for old in mj.findall("option"):
        mj.remove(old)
    opt = ET.Element("option", {"timestep": f"{dyn.timestep}", "integrator": "implicitfast", "gravity": "0 0 -9.81"})
    mj.insert(0, opt)

    world = mj.find("worldbody")
    for g in mj.iter("geom"):
        g.set("contype", "0")
        g.set("conaffinity", "0")
    world.insert(0, ET.Element("light", {"pos": "0 0 3", "dir": "0 0 -1", "directional": "true"}))
    world.insert(0, ET.Element("geom", {"name": "floor", "type": "plane", "size": "2 2 0.05", "pos": "0 0 -0.001",
                                        "rgba": "0.85 0.85 0.82 1", "contype": "0", "conaffinity": "0"}))

    joints: Dict[str, ET.Element] = {j.get("name"): j for j in mj.iter("joint")}
    missing = [n for n in JOINTS if n not in joints]
    if missing:
        raise ValueError(f"MJCF lost joints {missing}")
    for i, n in enumerate(JOINTS):
        j = joints[n]
        j.set("damping", f"{dyn.damping[i]}")
        j.set("armature", f"{dyn.armature[i]}")
        j.set("frictionloss", f"{dyn.frictionloss[i]}")

    for body in mj.iter("body"):
        if body.get("name") in ("flange", "tcp"):
            ET.SubElement(body, "site", {"name": f"{body.get('name')}_site", "size": "0.01", "rgba": "1 0.5 0 1"})

    act = mj.find("actuator")
    if act is None:
        act = ET.SubElement(mj, "actuator")
    for i, n in enumerate(JOINTS):
        lim = dyn.torque_limit[i]
        ET.SubElement(act, "motor", {"name": f"tau_{n}", "joint": n, "ctrllimited": "true",
                                     "ctrlrange": f"{-lim} {lim}", "gear": "1"})
    ET.indent(mj, space="  ")
    return ET.tostring(mj, encoding="unicode")
