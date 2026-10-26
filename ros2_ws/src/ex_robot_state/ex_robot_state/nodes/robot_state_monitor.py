"""Read-only observer that turns the robot state into things you can *see*:

* /ex/hud            MarkerArray: status text (mode, freshness, rate, 6 joints in
                     deg + rad, TCP pose, stamp) + joint-axis arrows
* /ex/tcp_pose       PoseStamped of tcp in world (from /tf, i.e. what RViz shows)
* /ex/tcp_path       nav_msgs/Path trail of the TCP
* /diagnostics       TF integrity: missing frames, foreign /tf publishers, and
                     /tf vs independent FK consistency
* JSONL health log   one line per second (``log_path`` param)
"""

from __future__ import annotations

import json
import math
import os
import time
from pathlib import Path as FsPath
from typing import Dict, List, Optional

import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from builtin_interfaces.msg import Time as TimeMsg
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import Point, PoseStamped
from nav_msgs.msg import Path
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import JointState
from std_msgs.msg import ColorRGBA
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker, MarkerArray

from ..core.rate_monitor import Freshness, RateMonitor
from ..core.se3 import make_transform, matrix_to_rpy, quat_to_matrix
from .common import JOINT_STATES_OUT_QOS, arm_joints, load_model
from .joint_state_adapter import DIAG_NAME as ADAPTER_DIAG

GREEN = ColorRGBA(r=0.2, g=0.9, b=0.3, a=1.0)
YELLOW = ColorRGBA(r=1.0, g=0.8, b=0.1, a=1.0)
RED = ColorRGBA(r=1.0, g=0.2, b=0.2, a=1.0)
WHITE = ColorRGBA(r=0.95, g=0.95, b=0.95, a=1.0)
_LVL = {b"\x00": "OK", b"\x01": "WARN", b"\x02": "ERROR", b"\x03": "STALE"}


def _lvl(level) -> int:
    return level[0] if isinstance(level, (bytes, bytearray)) else int(level)


def tf_to_matrix(tf) -> np.ndarray:
    t, q = tf.transform.translation, tf.transform.rotation
    return make_transform(quat_to_matrix(q.x, q.y, q.z, q.w), np.array([t.x, t.y, t.z]))


class RobotStateMonitor(Node):
    def __init__(self) -> None:
        super().__init__("robot_state_monitor")
        self.model = load_model(self)
        self.joints = arm_joints(self, self.model)
        self.declare_parameter("mode", "SIMULATION")
        self.declare_parameter("world_frame", "world")
        self.declare_parameter("tcp_frame", "tcp")
        self.declare_parameter("expected_frames", ["world", "base_link", "Link1", "Link2", "Link3",
                                                   "Link4", "Link5", "Link6", "flange", "tcp"])
        self.declare_parameter("allowed_tf_publishers", ["robot_state_publisher"])
        self.declare_parameter("trail_max_points", 3000)
        self.declare_parameter("trail_min_dist_m", 0.002)
        self.declare_parameter("hud_position", [0.0, -1.1, 1.45])
        self.declare_parameter("log_path", "")
        self.declare_parameter("fk_tolerance_m", 0.001)

        gp = lambda n: self.get_parameter(n).value  # noqa: E731
        self.mode = gp("mode")
        self.world, self.tcp = gp("world_frame"), gp("tcp_frame")
        self.expected_frames: List[str] = list(gp("expected_frames"))
        self.allowed_tf_pubs = set(gp("allowed_tf_publishers"))
        self.trail_max, self.trail_min = int(gp("trail_max_points")), float(gp("trail_min_dist_m"))
        self.hud_pos = list(gp("hud_position"))
        self.fk_tol = float(gp("fk_tolerance_m"))

        self.tf_buffer = Buffer(cache_time=Duration(seconds=10))
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.js_rate = RateMonitor(stale_after=0.2, lost_after=1.0)
        self.last_js: Optional[JointState] = None
        self.adapter_status: Optional[DiagnosticStatus] = None
        self.tf_issues: List[str] = []
        self.fk_error_m = math.nan
        self.tcp_T: Optional[np.ndarray] = None
        self._hud_lines = 0
        self.path = Path()
        self.path.header.frame_id = self.world

        self.create_subscription(JointState, "/joint_states", self._on_js, JOINT_STATES_OUT_QOS)
        self.create_subscription(DiagnosticArray, "/diagnostics", self._on_diag, 10)
        self.hud_pub = self.create_publisher(MarkerArray, "/ex/hud", 10)
        self.pose_pub = self.create_publisher(PoseStamped, "/ex/tcp_pose", 10)
        self.path_pub = self.create_publisher(Path, "/ex/tcp_path", 10)
        self.diag_pub = self.create_publisher(DiagnosticArray, "/diagnostics", 10)

        log_path = gp("log_path")
        self.log_file = None
        if log_path:
            FsPath(log_path).parent.mkdir(parents=True, exist_ok=True)
            self.log_file = open(log_path, "a", encoding="utf-8")
            self.get_logger().info(f"health log -> {log_path}")

        self.create_timer(1 / 20, self._fast_tick)
        self.create_timer(1.0, self._slow_tick)

    # ------------------------------------------------------------- callbacks
    def _on_js(self, msg: JointState) -> None:
        self.js_rate.tick(time.monotonic())
        self.last_js = msg

    def _on_diag(self, msg: DiagnosticArray) -> None:
        for s in msg.status:
            if s.name == ADAPTER_DIAG:
                self.adapter_status = s

    # ------------------------------------------------------------ 20 Hz loop
    def _fast_tick(self) -> None:
        try:
            tf = self.tf_buffer.lookup_transform(self.world, self.tcp, Time())
        except TransformException:
            self.tcp_T = None
        else:
            self.tcp_T = tf_to_matrix(tf)
            ps = PoseStamped()
            ps.header = tf.header
            ps.pose.position.x = tf.transform.translation.x
            ps.pose.position.y = tf.transform.translation.y
            ps.pose.position.z = tf.transform.translation.z
            ps.pose.orientation = tf.transform.rotation
            self.pose_pub.publish(ps)
            p = ps.pose.position
            if not self.path.poses or math.dist(
                (p.x, p.y, p.z), (self.path.poses[-1].pose.position.x, self.path.poses[-1].pose.position.y,
                                  self.path.poses[-1].pose.position.z)) >= self.trail_min:
                self.path.poses.append(ps)
                if len(self.path.poses) > self.trail_max:
                    del self.path.poses[: len(self.path.poses) - self.trail_max]
                self.path.header.stamp = ps.header.stamp
                self.path_pub.publish(self.path)
        self.hud_pub.publish(self._build_hud())

    def _overall(self) -> tuple[int, str]:
        fr = self.js_rate.freshness(time.monotonic())
        level = 0
        if self.adapter_status is not None:
            level = max(level, _lvl(self.adapter_status.level))
        if fr in (Freshness.LOST, Freshness.NEVER_RECEIVED):
            level = 2
        elif fr == Freshness.STALE:
            level = max(level, 1)
        if self.tf_issues:
            level = 2
        return level, fr.value

    def _build_hud(self) -> MarkerArray:
        now_msg = self.get_clock().now().to_msg()
        level, fr = self._overall()
        st = self.js_rate.stats(time.monotonic())
        # RViz's TEXT_VIEW_FACING centres every line and mangles runs of spaces in
        # multi-line strings, so each line is its own marker with single spaces.
        # ...and under WSLg its font renders a space as a very wide gap, so the
        # HUD uses no spaces at all (key=value|key=value).
        age_ms = st.age * 1000 if math.isfinite(st.age) else -1
        lines = [(f"EMBODIED-X|mode={self.mode}", WHITE),
                 (f"STATE={['VALID', 'VALID(WARN)', 'INVALID'][min(level, 2)]}|feed={fr}",
                  [GREEN, YELLOW, RED][min(level, 2)]),
                 (f"/joint_states={st.rate_hz:.1f}Hz|age={age_ms:.1f}ms", WHITE)]
        if self.last_js is not None:
            s = self.last_js.header.stamp
            lines.append((f"stamp={s.sec}.{s.nanosec:09d}", WHITE))
            for n, q in zip(self.last_js.name, self.last_js.position):
                lines.append((f"{n}={math.degrees(q):+.2f}deg|{q:+.4f}rad", WHITE))
        if self.tcp_T is not None:
            x, y, z = self.tcp_T[:3, 3] * 1000
            r, p, yw = (math.degrees(a) for a in matrix_to_rpy(self.tcp_T[:3, :3]))
            lines.append((f"TCP_xyz_mm=({x:+.1f},{y:+.1f},{z:+.1f})", WHITE))
            lines.append((f"TCP_rpy_deg=({r:+.1f},{p:+.1f},{yw:+.1f})", WHITE))
        nospace = lambda t: t.replace(" ", "_")  # noqa: E731
        if self.adapter_status is not None and self.adapter_status.message != "OK":
            lines.append((nospace(f"!{self.adapter_status.message[:70]}"), YELLOW if level < 2 else RED))
        for i in self.tf_issues[:3]:
            lines.append((nospace(f"!TF:{i[:70]}"), RED))

        arr = MarkerArray()
        size = 0.045
        for k, (text, color) in enumerate(lines):
            txt = Marker()
            txt.header.frame_id = self.world
            txt.header.stamp = now_msg
            txt.ns, txt.id, txt.type, txt.action = "hud", k, Marker.TEXT_VIEW_FACING, Marker.ADD
            txt.pose.position.x, txt.pose.position.y = self.hud_pos[0], self.hud_pos[1]
            txt.pose.position.z = self.hud_pos[2] - k * size * 1.5
            txt.pose.orientation.w = 1.0
            txt.scale.z = size
            txt.color = color
            txt.text = text
            arr.markers.append(txt)
        for k in range(len(lines), self._hud_lines):  # clear lines that disappeared
            m = Marker()
            m.header.frame_id, m.ns, m.id, m.action = self.world, "hud", k, Marker.DELETE
            arr.markers.append(m)
        self._hud_lines = len(lines)
        arr.markers.extend(self._axis_markers(now_msg))
        return arr

    def _axis_markers(self, stamp: TimeMsg) -> List[Marker]:
        """Arrow along each actuated joint's rotation axis (world frame)."""
        if self.last_js is None:
            return []
        q = dict(zip(self.last_js.name, self.last_js.position))
        try:
            poses = self.model.all_link_poses(q)
        except ValueError:
            return []
        root = self.model.root_link
        out = []
        for i, j in enumerate(self.joints):
            T = poses[j.child]  # joint frame == child link frame in URDF
            o = T[:3, 3]
            a = T[:3, :3] @ j.axis
            m = Marker()
            m.header.frame_id, m.header.stamp = root, stamp
            m.ns, m.id, m.type, m.action = "joint_axes", i, Marker.ARROW, Marker.ADD
            m.points = [Point(x=float(o[0]), y=float(o[1]), z=float(o[2])),
                        Point(x=float(o[0] + 0.15 * a[0]), y=float(o[1] + 0.15 * a[1]), z=float(o[2] + 0.15 * a[2]))]
            m.scale.x, m.scale.y, m.scale.z = 0.012, 0.025, 0.03
            m.color = ColorRGBA(r=1.0, g=0.4, b=0.9, a=0.9)
            m.pose.orientation.w = 1.0
            out.append(m)
        return out

    # ------------------------------------------------------------- 1 Hz loop
    def _slow_tick(self) -> None:
        issues: List[str] = []
        missing = [f for f in self.expected_frames
                   if f != self.world and not self.tf_buffer.can_transform(self.world, f, Time())]
        if missing:
            issues.append(f"missing/disconnected frames {missing}")
        foreign = sorted({p.node_name for t in ("/tf", "/tf_static")
                          for p in self.get_publishers_info_by_topic(t)} - self.allowed_tf_pubs - {self.get_name()})
        if foreign:
            issues.append(f"unexpected TF publishers {foreign}")
        self.fk_error_m = self._fk_consistency()
        if math.isfinite(self.fk_error_m) and self.fk_error_m > self.fk_tol:
            issues.append(f"/tf vs URDF FK mismatch {self.fk_error_m * 1000:.2f} mm")
        self.tf_issues = issues

        st = DiagnosticStatus()
        st.name, st.hardware_id = "ex/tf_integrity", "tf"
        st.level = DiagnosticStatus.ERROR if issues else DiagnosticStatus.OK
        st.message = "; ".join(issues) or "OK"
        st.values = [KeyValue(key="fk_error_mm", value=f"{self.fk_error_m * 1000:.4f}")]
        arr = DiagnosticArray()
        arr.header.stamp = self.get_clock().now().to_msg()
        arr.status = [st]
        self.diag_pub.publish(arr)
        self._log()

    def _fk_consistency(self) -> float:
        """Distance between /tf tcp and FK(URDF, joint_states) at the same stamp."""
        if self.last_js is None:
            return math.nan
        js = self.last_js
        try:
            tf = self.tf_buffer.lookup_transform(self.world, self.tcp, Time.from_msg(js.header.stamp))
        except TransformException:
            return math.nan
        q = dict(zip(js.name, js.position))
        try:
            T_fk = self.model.fk(q, self.tcp, self.world)
        except KeyError:
            return math.nan
        return float(np.linalg.norm(tf_to_matrix(tf)[:3, 3] - T_fk[:3, 3]))

    def _log(self) -> None:
        if self.log_file is None:
            return
        level, fr = self._overall()
        st = self.js_rate.stats(time.monotonic())
        rec: Dict[str, object] = {
            "wall_time": time.time(), "mode": self.mode, "level": level, "freshness": fr,
            "rate_hz": round(st.rate_hz, 3), "max_dt_ms": round(st.max_dt * 1000, 3) if st.count > 1 else None,
            "tf_issues": self.tf_issues, "fk_error_mm": None if math.isnan(self.fk_error_m) else round(self.fk_error_m * 1000, 4),
            "adapter": None if self.adapter_status is None else
            {"msg": self.adapter_status.message, **{kv.key: kv.value for kv in self.adapter_status.values}},
        }
        if self.last_js is not None:
            rec["q_rad"] = [round(v, 6) for v in self.last_js.position]
        if self.tcp_T is not None:
            rec["tcp_xyz_m"] = [round(float(v), 6) for v in self.tcp_T[:3, 3]]
        self.log_file.write(json.dumps(rec) + "\n")
        self.log_file.flush()

    def destroy_node(self) -> bool:
        if self.log_file:
            self.log_file.close()
        return super().destroy_node()


def main() -> None:
    rclpy.init()
    node = RobotStateMonitor()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
