"""Diagnose non-repeatable /check_state_validity results (observed: window/sampled_1,2 were
valid while sampling, invalid when re-checked a moment later).

  ros2 run ex_planning_lab validity_probe --ros-args -p scene:=window -p repeats:=20

For every sampled query state: call /check_state_validity `repeats` times (with and without
re-applying the scene in between) and report the valid/invalid counts plus the state's
mesh-surface clearance estimate, to separate (a) scene-update timing from (b) contact-edge
numerics from (c) something else.
"""

from __future__ import annotations

import json
import time
from pathlib import Path

import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from moveit_msgs.srv import GetStateValidity

from ..core.capsules import capsule_box_distance, points_box_distance
from ..core.metrics import clearance_interval
from ..core.scene import Scene
from .benchmark import Benchmark


class ValidityProbe(Benchmark):
    def __init__(self) -> None:
        super().__init__()
        self.declare_parameter("scene", "window")
        self.declare_parameter("repeats", 20)

    def contacts(self, q):
        req = GetStateValidity.Request(robot_state=self._robot_state(q), group_name=self.cfg["group"])
        res = self._call(self.valid_cli, req)
        return bool(res.valid), sorted({f"{c.contact_body_1}-{c.contact_body_2}" for c in res.contacts})

    def run(self) -> None:
        sname = self.get_parameter("scene").value
        n = int(self.get_parameter("repeats").value)
        scene = Scene.from_yaml(Path(get_package_share_directory("ex_planning_lab")) / "scenes" / f"{sname}.yaml")
        self.apply_scene(scene, [])
        time.sleep(0.5)
        queries = self.build_queries(scene)
        report = []
        for q in queries:
            for which in ("start", "goal"):
                x = q[which]
                seq = [self.contacts(x) for _ in range(n)]
                self.apply_scene(scene, [b.name for b in scene.boxes])  # re-apply, then check again
                time.sleep(0.3)
                seq2 = [self.contacts(x) for _ in range(n)]
                lb, est, pair = clearance_interval(np.array([x]), self.link_poses, self.capsules, self.points,
                                                   scene.boxes, capsule_box_distance, points_box_distance, top_k=1)
                rec = {"query": q["name"], "state": which,
                       "valid_count": sum(v for v, _ in seq), "valid_count_after_reapply": sum(v for v, _ in seq2),
                       "repeats": n, "contacts_seen": sorted({c for _, cs in seq + seq2 for c in cs}),
                       "clearance_est_mm": round(est * 1000, 2), "clearance_lb_mm": round(lb * 1000, 2),
                       "closest_pair": "-".join(pair) if pair else ""}
                report.append(rec)
                print(json.dumps(rec))
        out = Path(self.cfg["out_dir"])
        out.mkdir(parents=True, exist_ok=True)
        (out / f"validity_probe_{sname}.json").write_text(json.dumps(report, indent=2))


def main() -> None:
    rclpy.init()
    node = ValidityProbe()
    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
