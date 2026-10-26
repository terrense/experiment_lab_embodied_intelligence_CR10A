"""URDF FK vs controller-reported TCP: the key CR10A compatibility measurement.

Position error is convention-free and is the primary metric. For orientation
the packet's conventions are undocumented, so every plausible hypothesis is
scored and the best one is reported (evidence, not assumption).
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Sequence

import numpy as np

from .se3 import quat_to_matrix, rot_x, rot_y, rot_z, rotation_angle_between


def euler_hypotheses(rx: float, ry: float, rz: float) -> Dict[str, np.ndarray]:
    a, b, c = (math.radians(v) for v in (rx, ry, rz))
    return {
        "euler_xyz_extrinsic(Rz*Ry*Rx)": rot_z(c) @ rot_y(b) @ rot_x(a),
        "euler_xyz_intrinsic(Rx*Ry*Rz)": rot_x(a) @ rot_y(b) @ rot_z(c),
    }


def quat_hypotheses(q: Sequence[float]) -> Dict[str, np.ndarray]:
    if math.sqrt(sum(v * v for v in q)) < 0.5:
        return {}
    a, b, c, d = q
    return {"quat_wxyz": quat_to_matrix(b, c, d, a), "quat_xyzw": quat_to_matrix(a, b, c, d)}


RAW_LEN = 19  # layout defined in nodes/dobot_feedback_observer.py


def split_raw(data: Sequence[float]) -> Optional[dict]:
    """Unpack one /ex/dobot/tool_vector_raw message (None if malformed)."""
    if len(data) < RAW_LEN:
        return None
    return {"tool_vector": list(data[0:6]), "quat": list(data[6:10]), "q_deg": list(data[10:16]),
            "ctrl_ts_ms": data[16], "tool_id": int(data[17]), "user_id": int(data[18])}


@dataclass
class CrosscheckSample:
    pos_err_mm: float
    rot_err_deg: Dict[str, float]


@dataclass
class CrosscheckStats:
    samples: List[CrosscheckSample] = field(default_factory=list)

    def add(self, T_fk: np.ndarray, tool_vector: Sequence[float], quat: Sequence[float]) -> CrosscheckSample:
        x, y, z, rx, ry, rz = tool_vector
        p_ctrl = np.array([x, y, z]) / 1000.0
        pos_err = float(np.linalg.norm(T_fk[:3, 3] - p_ctrl)) * 1000.0
        hyps = {**euler_hypotheses(rx, ry, rz), **quat_hypotheses(quat)}
        rot = {k: math.degrees(rotation_angle_between(T_fk[:3, :3], R)) for k, R in hyps.items()}
        s = CrosscheckSample(pos_err, rot)
        self.samples.append(s)
        return s

    def summary(self) -> dict:
        if not self.samples:
            return {"n": 0}
        pe = np.array([s.pos_err_mm for s in self.samples])
        keys = set().union(*(s.rot_err_deg.keys() for s in self.samples))
        rot = {k: float(np.mean([s.rot_err_deg[k] for s in self.samples if k in s.rot_err_deg])) for k in keys}
        best = min(rot, key=rot.get) if rot else None
        return {
            "n": len(self.samples),
            "pos_err_mm_mean": float(pe.mean()), "pos_err_mm_rms": float(np.sqrt((pe ** 2).mean())),
            "pos_err_mm_max": float(pe.max()),
            "rot_err_deg_mean_by_hypothesis": rot, "best_orientation_hypothesis": best,
        }
