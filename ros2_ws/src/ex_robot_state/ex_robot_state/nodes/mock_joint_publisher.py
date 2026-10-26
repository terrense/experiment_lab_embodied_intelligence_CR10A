"""Mode A (SIMULATION) joint source. Publishes SIMULATED data on the same topic a
real feedback driver would use, so the downstream chain (adapter -> RSP -> TF ->
RViz) is exercised exactly as in LIVE_OBSERVE.

Fault injection (param ``fault``) lets you watch the safety pipeline react:
  none | dropout (stop for 2s every 10s) | jump (+20 deg spike) |
  degrees (send degrees) | nan | wrong_names
"""

from __future__ import annotations

import math
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import JointState

from ..core.mock_motion import MockMotion, MockMotionConfig
from .common import JOINT_STATES_OUT_QOS, arm_joints, load_model

FAULTS = ("none", "dropout", "jump", "degrees", "nan", "wrong_names")


class MockJointPublisher(Node):
    def __init__(self) -> None:
        super().__init__("mock_joint_publisher")
        model = load_model(self)
        joints = arm_joints(self, model)
        self.declare_parameter("rate_hz", 125.0)
        self.declare_parameter("topic", "/joint_states_robot")
        self.declare_parameter("fault", "none")
        self.declare_parameter("fault_period_s", 10.0)
        self.declare_parameter("home_deg", [0.0, 0.0, 90.0, 0.0, -90.0, 0.0])
        self.declare_parameter("amplitude_scale", 1.0)

        home = [math.radians(v) for v in self.get_parameter("home_deg").value]
        scale = float(self.get_parameter("amplitude_scale").value)
        cfg = MockMotionConfig(home=home)
        cfg.amplitudes = [a * scale for a in cfg.amplitudes]
        self.motion = MockMotion(joints, cfg)
        self.fault = self.get_parameter("fault").value
        if self.fault not in FAULTS:
            raise ValueError(f"fault must be one of {FAULTS}")
        self.fault_period = float(self.get_parameter("fault_period_s").value)

        self.pub = self.create_publisher(JointState, self.get_parameter("topic").value, JOINT_STATES_OUT_QOS)
        self.t0 = time.monotonic()
        self.timer = self.create_timer(1.0 / float(self.get_parameter("rate_hz").value), self._tick)
        self.get_logger().info(
            f"SIMULATED joint source -> {self.pub.topic_name} joints={self.motion.names} "
            f"amps_deg={[round(math.degrees(a), 1) for a in self.motion.amplitudes]} fault={self.fault}"
        )

    def _tick(self) -> None:
        t = time.monotonic() - self.t0
        names = list(self.motion.names)
        q = self.motion.positions(t)
        qd = self.motion.velocities(t)
        phase = t % self.fault_period
        if self.fault == "dropout" and phase > self.fault_period - 2.0:
            return
        if self.fault == "jump" and abs(phase - self.fault_period / 2) < 0.02:
            q[1] += math.radians(20)
        if self.fault == "degrees" and phase > self.fault_period / 2:
            q = [math.degrees(v) for v in q]
        if self.fault == "nan" and phase > self.fault_period / 2:
            q[2] = math.nan
        if self.fault == "wrong_names":
            names = [f"j{i + 1}" for i in range(len(names))]
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = names
        msg.position = q
        msg.velocity = qd
        self.pub.publish(msg)


def main() -> None:
    rclpy.init()
    node = MockJointPublisher()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
