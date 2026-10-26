#!/usr/bin/env python3
"""Offline model identification from ONE read-only LIVE_OBSERVE recording.

Reads /ex/dobot/tool_vector_raw from a rosbag2 (each message holds QActual and
the controller TCP of the SAME feedback packet - exact pairing; an earlier
version paired /joint_states_robot with the TCP topic by arrival order and was
off by one packet ~10% of the time) and scores every candidate URDF:

    position error |FK_urdf(q) - p_controller|   (convention-free)
    orientation error under each rotation-convention hypothesis

The URDF whose error is ~ the controller's calibration residual (a few mm)
is the right kinematic model; a constant offset (e.g. 41.9 mm between the
official CR10 and CR10AF flange points) shows up immediately.

usage (inside the ROS env):
  python3 evaluation/crosscheck_bag.py data/bags/<bag> [--out report.json]
      [--urdf name=path ...]   default: official cr10 + cr10af from third_party
"""

from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "ros2_ws" / "src" / "ex_robot_state"))

from ex_robot_state.core.crosscheck import CrosscheckStats, split_raw  # noqa: E402
from ex_robot_state.core.urdf_model import UrdfModel  # noqa: E402

DEFAULT_URDFS = {
    "official_cr10": ROOT / "third_party/dobot_main/cra_description/urdf/cr10_robot.xacro",
    "official_cr10af": ROOT / "third_party/dobot_main/cra_description/urdf/cr10af_robot.xacro",
}
TOOL = "/ex/dobot/tool_vector_raw"


def read_samples(bag: str):
    """Every /ex/dobot/tool_vector_raw message carries TCP *and* QActual from
    the same packet, so no cross-topic pairing is needed."""
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from std_msgs.msg import Float64MultiArray

    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=bag, storage_id=""), rosbag2_py.ConverterOptions("", ""))
    reader.set_filter(rosbag2_py.StorageFilter(topics=[TOOL]))
    out, skipped = [], 0
    while reader.has_next():
        _, data, _ = reader.read_next()
        r = split_raw(deserialize_message(data, Float64MultiArray).data)
        if r is None:
            continue
        if r["tool_id"] != 0 or r["user_id"] != 0:
            skipped += 1
            continue
        out.append(r)
    return out, skipped


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("bag")
    ap.add_argument("--urdf", action="append", default=[], help="name=path (repeatable)")
    ap.add_argument("--base", default="base_link")
    ap.add_argument("--tip", default="Link6", help="flange link in the candidate URDFs")
    ap.add_argument("--out", default="")
    a = ap.parse_args()
    cands = dict(DEFAULT_URDFS)
    for spec in a.urdf:
        k, v = spec.split("=", 1)
        cands[k] = Path(v)
    models = {k: UrdfModel.from_file(p) for k, p in cands.items()}
    samples, skipped = read_samples(a.bag)
    if not samples:
        raise SystemExit("no tool-0/user-0 samples - was the bag recorded in LIVE_OBSERVE?")
    report = {"bag": a.bag, "samples": len(samples), "skipped_non_default_frames": skipped, "candidates": {}}
    names = [f"joint{i}" for i in range(1, 7)]
    for k, m in models.items():
        st = CrosscheckStats()
        for r in samples:
            q = {n: math.radians(v) for n, v in zip(names, r["q_deg"])}
            st.add(m.fk(q, a.tip, a.base), r["tool_vector"], r["quat"])
        report["candidates"][k] = st.summary()
    best = min(report["candidates"], key=lambda k: report["candidates"][k]["pos_err_mm_rms"])
    report["best_position_match"] = best
    print(json.dumps(report, indent=2))
    if a.out:
        Path(a.out).write_text(json.dumps(report, indent=2))


if __name__ == "__main__":
    main()
