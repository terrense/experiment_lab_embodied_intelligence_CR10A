"""Minimal URDF kinematic model: joint specs + forward kinematics.

Deliberately ROS-free (only numpy + stdlib XML) so it can be unit-tested anywhere
and used to cross-check what robot_state_publisher puts on /tf, or to compare the
URDF's FK against the TCP pose reported by the real controller (the strongest
offline-checkable evidence that a URDF matches a physical arm).
"""

from __future__ import annotations

import math
import xml.etree.ElementTree as ET
from dataclasses import dataclass, field
from pathlib import Path
from typing import Dict, List, Mapping, Optional, Sequence

import numpy as np

from .se3 import rpy_to_matrix, axis_angle_to_matrix, make_transform


@dataclass(frozen=True)
class JointSpec:
    name: str
    type: str  # revolute | continuous | prismatic | fixed
    parent: str
    child: str
    origin: np.ndarray = field(repr=False)  # 4x4 parent->joint frame at q=0
    axis: np.ndarray = field(repr=False)  # unit vector in joint frame
    lower: float = -math.inf
    upper: float = math.inf
    velocity: float = math.inf

    @property
    def is_actuated(self) -> bool:
        return self.type in ("revolute", "continuous", "prismatic")

    @property
    def is_continuous(self) -> bool:
        return self.type == "continuous"

    def local_transform(self, q: float) -> np.ndarray:
        if self.type in ("revolute", "continuous"):
            motion = make_transform(axis_angle_to_matrix(self.axis, q), np.zeros(3))
        elif self.type == "prismatic":
            motion = make_transform(np.eye(3), self.axis * q)
        else:
            motion = np.eye(4)
        return self.origin @ motion


def _parse_floats(text: Optional[str], n: int, default: float = 0.0) -> np.ndarray:
    if not text:
        return np.full(n, default, dtype=float)
    vals = [float(v) for v in text.split()]
    if len(vals) != n:
        raise ValueError(f"expected {n} floats, got {text!r}")
    return np.array(vals, dtype=float)


class UrdfModel:
    """Tree of joints parsed from a (xacro-expanded) URDF string."""

    def __init__(self, joints: Sequence[JointSpec], name: str = "robot") -> None:
        self.name = name
        self.joints: Dict[str, JointSpec] = {j.name: j for j in joints}
        self._by_child: Dict[str, JointSpec] = {j.child: j for j in joints}
        children = {j.child for j in joints}
        roots = {j.parent for j in joints} - children
        if len(roots) != 1:
            raise ValueError(f"URDF must have exactly one root link, found {sorted(roots)}")
        self.root_link: str = roots.pop()

    # ------------------------------------------------------------------ parsing
    @classmethod
    def from_string(cls, xml: str) -> "UrdfModel":
        root = ET.fromstring(xml)
        if root.tag != "robot":
            raise ValueError("not a URDF <robot> document")
        joints: List[JointSpec] = []
        for j in root.findall("joint"):
            origin_el = j.find("origin")
            xyz = _parse_floats(origin_el.get("xyz") if origin_el is not None else None, 3)
            rpy = _parse_floats(origin_el.get("rpy") if origin_el is not None else None, 3)
            axis_el = j.find("axis")
            axis = _parse_floats(axis_el.get("xyz") if axis_el is not None else None, 3)
            if not np.any(axis):
                axis = np.array([1.0, 0.0, 0.0])  # URDF default
            axis = axis / np.linalg.norm(axis)
            limit = j.find("limit")
            jtype = j.get("type", "fixed")
            lower, upper, vel = -math.inf, math.inf, math.inf
            if limit is not None and jtype in ("revolute", "prismatic"):
                lower = float(limit.get("lower", "0"))
                upper = float(limit.get("upper", "0"))
            if limit is not None and limit.get("velocity") is not None:
                vel = float(limit.get("velocity"))
            joints.append(
                JointSpec(
                    name=j.get("name"),
                    type=jtype,
                    parent=j.find("parent").get("link"),
                    child=j.find("child").get("link"),
                    origin=make_transform(rpy_to_matrix(*rpy), xyz),
                    axis=axis,
                    lower=lower,
                    upper=upper,
                    velocity=vel,
                )
            )
        return cls(joints, name=root.get("name", "robot"))

    @classmethod
    def from_file(cls, path: str | Path) -> "UrdfModel":
        return cls.from_string(Path(path).read_text(encoding="utf-8"))

    # ---------------------------------------------------------------- queries
    def chain(self, tip_link: str, base_link: Optional[str] = None) -> List[JointSpec]:
        """Joints from base_link (default: root) down to tip_link, in order."""
        base_link = base_link or self.root_link
        out: List[JointSpec] = []
        link = tip_link
        while link != base_link:
            j = self._by_child.get(link)
            if j is None:
                raise KeyError(f"no kinematic path from {base_link!r} to {tip_link!r}")
            out.append(j)
            link = j.parent
        return list(reversed(out))

    def actuated_joints(self, tip_link: str, base_link: Optional[str] = None) -> List[JointSpec]:
        return [j for j in self.chain(tip_link, base_link) if j.is_actuated]

    def fk(
        self, positions: Mapping[str, float], tip_link: str, base_link: Optional[str] = None
    ) -> np.ndarray:
        """4x4 pose of tip_link expressed in base_link. Missing joints raise KeyError."""
        T = np.eye(4)
        for j in self.chain(tip_link, base_link):
            q = positions[j.name] if j.is_actuated else 0.0
            T = T @ j.local_transform(q)
        return T

    def all_link_poses(self, positions: Mapping[str, float]) -> Dict[str, np.ndarray]:
        """Pose of every link relative to the root (what /tf should encode)."""
        poses: Dict[str, np.ndarray] = {self.root_link: np.eye(4)}
        pending = list(self.joints.values())
        while pending:
            progressed = False
            for j in list(pending):
                if j.parent in poses:
                    q = positions.get(j.name, 0.0) if j.is_actuated else 0.0
                    poses[j.child] = poses[j.parent] @ j.local_transform(q)
                    pending.remove(j)
                    progressed = True
            if not progressed:
                raise ValueError("disconnected URDF tree")
        return poses
