"""Compare URDF FK(base_link -> flange) with the controller's ToolVectorActual.

Input : /ex/dobot/tool_vector_raw  (one message per feedback packet, see
        dobot_feedback_observer for the layout: TCP, quaternion, QActual deg,
        controller timestamp, tool id, user id) - joints and TCP come from the
        SAME packet, so pairing is exact.
Output: /diagnostics "ex/kinematics_crosscheck", CSV (param csv_path),
        JSON summary on shutdown (param summary_path).

Only meaningful while the controller uses tool 0 (flange) and user 0 (base);
samples with other ids are counted and skipped. With the mock controller the
TCP is computed from this same URDF, so ~0 error only proves the plumbing,
NOT the model.
"""

from __future__ import annotations

import json
import math
import time
from pathlib import Path

import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

from ..core.crosscheck import CrosscheckStats, split_raw
from .common import load_model


class KinematicsCrosscheck(Node):
    def __init__(self) -> None:
        super().__init__("kinematics_crosscheck")
        self.model = load_model(self)
        p = self.declare_parameter
        p("base_link", "base_link")
        p("flange_link", "flange")
        p("joint_names", ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"])
        p("pos_warn_mm", 2.0)
        p("pos_error_mm", 10.0)
        p("csv_path", "")
        p("summary_path", "")
        gp = lambda n: self.get_parameter(n).value  # noqa: E731
        self.base, self.flange = gp("base_link"), gp("flange_link")
        self.names = list(gp("joint_names"))
        self.warn_mm, self.err_mm = float(gp("pos_warn_mm")), float(gp("pos_error_mm"))
        self.summary_path = gp("summary_path")
        self.stats = CrosscheckStats()
        self.skipped_frames = 0
        self.csv = None
        if gp("csv_path"):
            Path(gp("csv_path")).parent.mkdir(parents=True, exist_ok=True)
            self.csv = open(gp("csv_path"), "w", encoding="utf-8")
            self.csv.write("ctrl_ts_ms,q1,q2,q3,q4,q5,q6,fk_x,fk_y,fk_z,ctrl_x,ctrl_y,ctrl_z,pos_err_mm\n")
        self.create_subscription(Float64MultiArray, "/ex/dobot/tool_vector_raw", self._on_raw, 50)
        self.diag_pub = self.create_publisher(DiagnosticArray, "/diagnostics", 10)
        self.create_timer(1.0, self._diag)

    def _on_raw(self, msg: Float64MultiArray) -> None:
        r = split_raw(msg.data)
        if r is None:
            return
        if r["tool_id"] != 0 or r["user_id"] != 0:
            self.skipped_frames += 1
            return
        q = {n: math.radians(v) for n, v in zip(self.names, r["q_deg"])}
        T = self.model.fk(q, self.flange, self.base)
        s = self.stats.add(T, r["tool_vector"], r["quat"])
        if self.csv:
            fk = T[:3, 3] * 1000
            self.csv.write(",".join(f"{v:.6f}" for v in (r["ctrl_ts_ms"], *r["q_deg"], *fk,
                                                        *r["tool_vector"][:3], s.pos_err_mm)) + "\n")

    def _diag(self) -> None:
        sm = self.stats.summary()
        st = DiagnosticStatus()
        st.name, st.hardware_id = "ex/kinematics_crosscheck", "urdf_vs_controller"
        if sm["n"] == 0:
            st.level, st.message = DiagnosticStatus.WARN, "no samples yet"
        else:
            e = sm["pos_err_mm_rms"]
            st.level = (DiagnosticStatus.ERROR if e > self.err_mm else
                        DiagnosticStatus.WARN if e > self.warn_mm else DiagnosticStatus.OK)
            st.message = f"URDF vs controller TCP: rms {e:.2f} mm, max {sm['pos_err_mm_max']:.2f} mm (n={sm['n']})"
        sm["skipped_non_default_frames"] = self.skipped_frames
        st.values = [KeyValue(key=k, value=json.dumps(v)) for k, v in sm.items()]
        arr = DiagnosticArray()
        arr.header.stamp = self.get_clock().now().to_msg()
        arr.status = [st]
        self.diag_pub.publish(arr)

    def destroy_node(self) -> bool:
        if self.csv:
            self.csv.close()
        if self.summary_path:
            sm = self.stats.summary()
            sm["skipped_non_default_frames"] = self.skipped_frames
            Path(self.summary_path).write_text(json.dumps(sm, indent=2))
        return super().destroy_node()


def main() -> None:
    rclpy.init()
    node = KinematicsCrosscheck()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
