"""Automated end-to-end check of JointState -> robot_state_publisher -> TF.

Run alongside a running pipeline; it observes for ``duration_s`` and exits
0 (all PASS) or 1 (any FAIL). Writes a JSON report (param report_path).

  ros2 run ex_robot_state pipeline_check --ros-args -p duration_s:=30.0 \
      -p robot_description:="$(xacro ...)" -p expect_mock_frequencies:=true

Checks
  rate            /joint_states rate >= min_rate_hz
  names           JointState.name == URDF arm joints (order + spelling)
  single_pub      exactly one publisher on /joint_states
  tf_frames       every link in the URDF resolvable from world_frame
  tf_vs_fk        /tf(tcp) vs FK(URDF, same joint sample) < fk_tol_m
  motion          (optional) every joint moved > min_motion_deg
  mapping         (optional) each joint's dominant frequency == its mock frequency
  adapter_ok      adapter diagnostics never reported ERROR during the window
"""

from __future__ import annotations

import json
import math
import sys
import time
from pathlib import Path
from typing import Dict, List

import numpy as np
import rclpy
from diagnostic_msgs.msg import DiagnosticArray
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import JointState
from tf2_ros import Buffer, TransformException, TransformListener

from ..core.mock_motion import DEFAULT_FREQS_HZ
from ..core.signal_checks import frequency_amplitudes
from .common import JOINT_STATES_OUT_QOS, arm_joints, load_model
from .joint_state_adapter import DIAG_NAME as ADAPTER_DIAG
from .robot_state_monitor import tf_to_matrix


class PipelineCheck(Node):
    def __init__(self) -> None:
        super().__init__("pipeline_check")
        self.model = load_model(self)
        self.joints = arm_joints(self, self.model)
        p = self.declare_parameter
        p("duration_s", 20.0)
        p("min_rate_hz", 50.0)
        p("world_frame", "world")
        p("tcp_frame", "tcp")
        p("fk_tol_m", 1e-3)
        p("expect_motion", True)
        p("min_motion_deg", 2.0)
        p("expect_mock_frequencies", False)
        p("report_path", "")
        gp = lambda n: self.get_parameter(n).value  # noqa: E731
        self.duration = float(gp("duration_s"))
        self.world, self.tcp = gp("world_frame"), gp("tcp_frame")
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.t_recv: List[float] = []
        self.t_stamp: List[float] = []
        self.q: List[List[float]] = []
        self.names_seen: set = set()
        self.fk_errs: List[float] = []
        self.max_pubs = 0
        self.min_pubs = 99
        self.adapter_levels: List[int] = []
        self.create_subscription(JointState, "/joint_states", self._on_js, JOINT_STATES_OUT_QOS)
        self.create_subscription(DiagnosticArray, "/diagnostics", self._on_diag, 50)
        self.create_timer(0.25, self._sample_tf)
        self.t0 = time.monotonic()

    def _on_js(self, m: JointState) -> None:
        self.t_recv.append(time.monotonic())
        self.t_stamp.append(m.header.stamp.sec + m.header.stamp.nanosec * 1e-9)
        self.names_seen.add(tuple(m.name))
        d = dict(zip(m.name, m.position))
        self.q.append([d.get(j.name, math.nan) for j in self.joints])
        self._last = m

    def _on_diag(self, m: DiagnosticArray) -> None:
        for s in m.status:
            if s.name == ADAPTER_DIAG and time.monotonic() - self.t0 > 2.0:  # skip startup
                self.adapter_levels.append(s.level[0] if isinstance(s.level, bytes) else int(s.level))

    def _sample_tf(self) -> None:
        if time.monotonic() - self.t0 > 2.0:  # let DDS discovery settle first
            n = self.count_publishers("/joint_states")
            self.max_pubs, self.min_pubs = max(self.max_pubs, n), min(self.min_pubs, n)
        m = getattr(self, "_last", None)
        if m is None:
            return
        try:
            tf = self.tf_buffer.lookup_transform(self.world, self.tcp, Time.from_msg(m.header.stamp))
        except TransformException:
            return
        T = self.model.fk(dict(zip(m.name, m.position)), self.tcp, self.world)
        self.fk_errs.append(float(np.linalg.norm(tf_to_matrix(tf)[:3, 3] - T[:3, 3])))

    def done(self) -> bool:
        return time.monotonic() - self.t0 >= self.duration

    def report(self) -> Dict[str, dict]:
        gp = lambda n: self.get_parameter(n).value  # noqa: E731
        R: Dict[str, dict] = {}
        n = len(self.t_recv)
        rate = (n - 1) / (self.t_recv[-1] - self.t_recv[0]) if n > 1 else 0.0
        dts = np.diff(self.t_recv) if n > 1 else np.array([math.nan])
        R["rate"] = {"pass": rate >= float(gp("min_rate_hz")), "rate_hz": round(rate, 2), "samples": n,
                     "max_gap_ms": round(float(np.max(dts)) * 1000, 2) if n > 1 else None,
                     "p99_gap_ms": round(float(np.percentile(dts, 99)) * 1000, 2) if n > 1 else None}
        urdf_names = tuple(j.name for j in self.joints)
        R["names"] = {"pass": self.names_seen == {urdf_names}, "seen": [list(s) for s in self.names_seen],
                      "urdf": list(urdf_names)}
        R["single_pub"] = {"pass": self.max_pubs == 1 and self.min_pubs == 1,
                           "min": self.min_pubs, "max": self.max_pubs}
        links = sorted({j.child for j in self.model.joints.values()})
        missing = [l for l in links if not self.tf_buffer.can_transform(self.world, l, Time())]
        R["tf_frames"] = {"pass": not missing, "checked": len(links), "missing": missing}
        R["tf_vs_fk"] = {"pass": bool(self.fk_errs) and max(self.fk_errs) < float(gp("fk_tol_m")),
                         "samples": len(self.fk_errs),
                         "max_err_mm": round(max(self.fk_errs) * 1000, 6) if self.fk_errs else None}
        Q = np.array(self.q) if self.q else np.zeros((0, len(self.joints)))
        if gp("expect_motion"):
            span = np.degrees(Q.max(0) - Q.min(0)).tolist() if len(Q) else []
            R["motion"] = {"pass": bool(span) and min(span) > float(gp("min_motion_deg")),
                           "range_deg": [round(s, 2) for s in span]}
        if gp("expect_mock_frequencies") and len(Q) > 10:
            t = np.array(self.t_stamp) - self.t_stamp[0]
            dom, amps = [], []
            for k in range(Q.shape[1]):
                a = frequency_amplitudes(t, Q[:, k], DEFAULT_FREQS_HZ)
                dom.append(int(np.argmax(a)))
                amps.append([round(math.degrees(v), 2) for v in a])
            R["mapping"] = {"pass": dom == list(range(Q.shape[1])), "dominant_freq_index": dom,
                            "expected": list(range(Q.shape[1])), "amplitudes_deg": amps}
        R["adapter_ok"] = {"pass": bool(self.adapter_levels) and max(self.adapter_levels) < 2,  # 2 = ERROR
                           "reports": len(self.adapter_levels),
                           "worst_level": max(self.adapter_levels) if self.adapter_levels else None}
        return R


def main() -> None:
    rclpy.init()
    node = PipelineCheck()
    try:
        while rclpy.ok() and not node.done():
            rclpy.spin_once(node, timeout_sec=0.05)
        rep = node.report()
        ok = all(v["pass"] for v in rep.values())
        for k, v in rep.items():
            print(f"[{'PASS' if v['pass'] else 'FAIL'}] {k:11s} {json.dumps({x: y for x, y in v.items() if x != 'pass'})}")
        print("RESULT:", "PASS" if ok else "FAIL")
        path = node.get_parameter("report_path").value
        if path:
            Path(path).parent.mkdir(parents=True, exist_ok=True)
            Path(path).write_text(json.dumps({"result": "PASS" if ok else "FAIL", "wall_time": time.time(),
                                              "duration_s": node.duration, "checks": rep}, indent=2))
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
    sys.exit(0 if ok else 1)
