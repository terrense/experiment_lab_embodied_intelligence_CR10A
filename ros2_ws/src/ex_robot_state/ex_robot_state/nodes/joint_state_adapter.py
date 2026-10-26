"""JointState adapter: /joint_states_robot (raw) -> validated /joint_states.

Responsibilities
* Map driver joint names onto URDF joint names (``source_names`` -> URDF order).
* Optional unit conversion (``input_in_degrees``) — off by default; the
  validator flags degree-looking data regardless.
* Validate every sample (structure, finite, limits, jumps) and forward only
  accepted ones. On ERROR/LOST the last valid state stays on /tf (frozen) and
  the state is marked INVALID in /diagnostics.
* Detect conflicting publishers on /joint_states (e.g. a forgotten
  joint_state_publisher_gui, or a bag replaying /joint_states).
* Never publishes anything to the robot. It has no network code at all.
"""

from __future__ import annotations

import math
import time
from typing import List, Optional

import rclpy
from rclpy.executors import ExternalShutdownException
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from rclpy.node import Node
from sensor_msgs.msg import JointState

from ..core.rate_monitor import Freshness, RateMonitor
from ..core.validation import JointStateValidator, Level, ValidatorConfig
from .common import JOINT_STATES_OUT_QOS, SENSOR_IN_QOS, arm_joints, load_model

DIAG_NAME = "ex/joint_state_adapter"
_LEVEL_TO_DIAG = {Level.OK: DiagnosticStatus.OK, Level.WARN: DiagnosticStatus.WARN, Level.ERROR: DiagnosticStatus.ERROR}


class JointStateAdapter(Node):
    def __init__(self) -> None:
        super().__init__("joint_state_adapter")
        model = load_model(self)
        self.joints = arm_joints(self, model)
        self.urdf_names = [j.name for j in self.joints]

        self.declare_parameter("input_topic", "/joint_states_robot")
        self.declare_parameter("output_topic", "/joint_states")
        self.declare_parameter("source_names", self.urdf_names)  # driver name for each URDF joint
        self.declare_parameter("input_in_degrees", False)
        self.declare_parameter("stamp_source", "receive")  # receive | source
        self.declare_parameter("stale_after_s", 0.1)
        self.declare_parameter("lost_after_s", 1.0)
        self.declare_parameter("expected_rate_hz", 125.0)
        self.declare_parameter("max_joint_velocity_deg_s", 225.0)  # <=0: use URDF limits

        src = list(self.get_parameter("source_names").value)
        if len(src) != len(self.urdf_names):
            raise ValueError("source_names must have one entry per URDF arm joint")
        self.rename = dict(zip(src, self.urdf_names))
        if src != self.urdf_names:
            self.get_logger().warn(f"joint name mapping active: {self.rename}")
        self.in_degrees = bool(self.get_parameter("input_in_degrees").value)
        self.stamp_source = self.get_parameter("stamp_source").value
        self.expected_rate = float(self.get_parameter("expected_rate_hz").value)

        vmax = float(self.get_parameter("max_joint_velocity_deg_s").value)
        self.validator = JointStateValidator(self.joints, ValidatorConfig(
            resync_gap=float(self.get_parameter("lost_after_s").value),
            velocity_limit_override=math.radians(vmax) if vmax > 0 else None))
        self.in_rate = RateMonitor(float(self.get_parameter("stale_after_s").value),
                                   float(self.get_parameter("lost_after_s").value))
        self.out_rate = RateMonitor(float(self.get_parameter("stale_after_s").value),
                                    float(self.get_parameter("lost_after_s").value))

        self.out_topic = self.get_parameter("output_topic").value
        self.pub = self.create_publisher(JointState, self.out_topic, JOINT_STATES_OUT_QOS)
        self.diag_pub = self.create_publisher(DiagnosticArray, "/diagnostics", 10)
        self.sub = self.create_subscription(
            JointState, self.get_parameter("input_topic").value, self._on_raw, SENSOR_IN_QOS)

        self.last_issues: List[str] = []
        self.last_level = Level.OK
        self.window_level = Level.OK
        self.window_issues: List[str] = []
        self.rejected = 0
        self.accepted = 0
        self.last_valid: Optional[List[float]] = None
        self.max_latency = 0.0
        self.last_latency = math.nan
        self._was_lost = False
        self.create_timer(0.5, self._publish_diagnostics)
        self.get_logger().info(
            f"adapter {self.sub.topic_name} -> {self.out_topic}; URDF joints {self.urdf_names}")

    # ------------------------------------------------------------------ data
    def _on_raw(self, msg: JointState) -> None:
        now = time.monotonic()
        if self._was_lost:
            self.get_logger().warn("feedback resumed after LOST; next sample is a RESYNC")
            self._was_lost = False
        self.in_rate.tick(now)
        names = [self.rename.get(n, n) for n in msg.name]
        pos = list(msg.position)
        if self.in_degrees:
            pos = [math.radians(v) for v in pos]
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        res = self.validator.validate(names, pos, now, stamp)

        # Host-clock latency is only meaningful when the source stamps with the
        # same clock (mock, relay on this host). Controller clocks are NOT synced.
        if stamp > 0:
            self.last_latency = self.get_clock().now().nanoseconds * 1e-9 - stamp

        self.last_level = res.level
        self.last_issues = [f"{i.code}: {i.message}" for i in res.issues if i.level > Level.OK]
        # latch the worst level / issues until the next diagnostics publish, so a
        # 1-sample fault (e.g. an 8 ms jump) is never hidden by the next good sample
        self.window_level = max(self.window_level, res.level)
        for i in self.last_issues:
            if i not in self.window_issues and len(self.window_issues) < 10:
                self.window_issues.append(i)
        if not res.accepted:
            self.rejected += 1
            if self.rejected <= 5 or self.rejected % 100 == 0:
                self.get_logger().error(f"rejected sample #{self.rejected}: {self.last_issues}")
            return
        for i in res.issues:
            if i.level == Level.WARN and i.code in ("RESYNC", "EXTRA_JOINTS"):
                self.get_logger().warn(f"{i.code}: {i.message}")

        out = JointState()
        out.header.stamp = msg.header.stamp if (self.stamp_source == "source" and stamp > 0) \
            else self.get_clock().now().to_msg()
        out.name = list(self.urdf_names)
        out.position = res.positions
        if len(msg.velocity) == len(msg.name):
            vel = dict(zip(names, msg.velocity))
            out.velocity = [float(vel[n]) for n in self.urdf_names]
        self.pub.publish(out)
        self.accepted += 1
        self.last_valid = res.positions
        self.out_rate.tick(now)

    # ----------------------------------------------------------- diagnostics
    def _publish_diagnostics(self) -> None:
        now = time.monotonic()
        st_in = self.in_rate.stats(now)
        level = self.window_level
        msgs: List[str] = []

        if st_in.freshness in (Freshness.LOST, Freshness.NEVER_RECEIVED):
            level = Level.ERROR
            msgs.append(f"feedback {st_in.freshness.value} (age {st_in.age:.1f}s) - display FROZEN, state INVALID")
            if st_in.freshness == Freshness.LOST and not self._was_lost:
                self._was_lost = True
                self.validator.reset_reference()
                self.get_logger().error("joint feedback LOST")
        elif st_in.freshness == Freshness.STALE:
            level = max(level, Level.WARN)
            msgs.append(f"feedback STALE (age {st_in.age * 1000:.0f} ms)")
        elif self.expected_rate > 0 and st_in.rate_hz < 0.8 * self.expected_rate:
            level = max(level, Level.WARN)
            msgs.append(f"rate {st_in.rate_hz:.1f} Hz < 80% of expected {self.expected_rate:.0f} Hz")

        pubs = self.get_publishers_info_by_topic(self.out_topic)
        others = [f"{p.node_namespace.rstrip('/')}/{p.node_name}" for p in pubs if p.node_name != self.get_name()]
        if others:
            level = Level.ERROR
            msgs.append(f"CONFLICT: other publishers on {self.out_topic}: {others}")

        msgs.extend(self.window_issues)
        self.window_level, self.window_issues = Level.OK, []
        st = DiagnosticStatus()
        st.name = DIAG_NAME
        st.hardware_id = "joint_feedback"
        st.level = _LEVEL_TO_DIAG[level]
        st.message = "; ".join(msgs) if msgs else "OK"
        kv = {
            "freshness": st_in.freshness.value,
            "valid": str(level < Level.ERROR),
            "rate_hz": f"{st_in.rate_hz:.2f}",
            "mean_dt_ms": f"{st_in.mean_dt * 1000:.2f}",
            "max_dt_ms": f"{st_in.max_dt * 1000:.2f}",
            "age_ms": f"{st_in.age * 1000:.1f}",
            "accepted": str(self.accepted),
            "rejected": str(self.rejected),
            "stamp_latency_ms": f"{self.last_latency * 1000:.2f}",
            "joint_state_publishers": str(len(pubs)),
        }
        if self.last_valid is not None:
            for n, v in zip(self.urdf_names, self.last_valid):
                kv[f"{n}_deg"] = f"{math.degrees(v):+.3f}"
        st.values = [KeyValue(key=k, value=v) for k, v in kv.items()]
        arr = DiagnosticArray()
        arr.header.stamp = self.get_clock().now().to_msg()
        arr.status = [st]
        self.diag_pub.publish(arr)


def main() -> None:
    rclpy.init()
    node = JointStateAdapter()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
