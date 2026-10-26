#!/usr/bin/env python3
"""Evidence for docs/HARDWARE_COMPATIBILITY.md: are the official CR10 and CR10AF
URDFs the same kinematic robot?

Compares (a) joint names/limits, (b) joint-angle-invariant distances between
consecutive joint axes' origins, (c) zero-pose flange position, (d) FK of the
flange over random joint vectors. Output: JSON to stdout (and --out).

usage: python3 scripts/compare_dobot_models.py [--src third_party/dobot_main] [--out data/reports/model_compare.json]
"""

from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "ros2_ws" / "src" / "ex_robot_state"))
from ex_robot_state.core.urdf_model import UrdfModel  # noqa: E402

MODELS = {"cr10": "cr10_robot.xacro", "cr10af": "cr10af_robot.xacro"}


def last_arm_link(m: UrdfModel) -> str:
    j6 = m.joints["joint6"]
    return j6.child


def analyse(m: UrdfModel) -> dict:
    tip = last_arm_link(m)
    chain = m.actuated_joints(tip, "base_link")
    q0 = {j.name: 0.0 for j in chain}
    poses = m.all_link_poses(q0)
    origins = [poses["base_link"][:3, 3]] + [poses[j.child][:3, 3] for j in chain]
    seg = [float(np.linalg.norm(b - a)) * 1000 for a, b in zip(origins, origins[1:])]
    return {
        "joints": [j.name for j in chain],
        "limits_deg": {j.name: [round(math.degrees(j.lower), 2), round(math.degrees(j.upper), 2)] for j in chain},
        "joint_origin_segment_lengths_mm": [round(s, 3) for s in seg],
        "zero_pose_flange_xyz_mm": [round(v * 1000, 3) for v in m.fk(q0, tip, "base_link")[:3, 3]],
        "tip_link": tip,
    }


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--src", default=str(ROOT / "third_party" / "dobot_main"))
    ap.add_argument("--out", default="")
    a = ap.parse_args()
    urdf_dir = Path(a.src) / "cra_description" / "urdf"
    models = {k: UrdfModel.from_file(urdf_dir / f) for k, f in MODELS.items()}
    res = {k: analyse(m) for k, m in models.items()}

    rng = np.random.default_rng(0)
    d = []
    for _ in range(500):
        q = rng.uniform(-math.pi, math.pi, 6)
        p = [models[k].fk({f"joint{i + 1}": q[i] for i in range(6)}, res[k]["tip_link"], "base_link")[:3, 3]
             for k in ("cr10", "cr10af")]
        d.append(np.linalg.norm(p[0] - p[1]) * 1000)
    res["same_q_flange_position_diff_mm"] = {"mean": round(float(np.mean(d)), 2), "max": round(float(np.max(d)), 2),
                                             "n": len(d)}
    reach = {k: round(max(np.linalg.norm(models[k].fk({f"joint{i + 1}": v for i, v in enumerate(rng.uniform(-math.pi, math.pi, 6))},
                                                          res[k]["tip_link"], "base_link")[:3, 3]) for _ in range(3000)) * 1000, 1)
             for k in models}
    res["sampled_max_reach_mm"] = reach
    out = json.dumps(res, indent=2)
    print(out)
    if a.out:
        Path(a.out).parent.mkdir(parents=True, exist_ok=True)
        Path(a.out).write_text(out)


if __name__ == "__main__":
    main()
