"""Shared helpers for ex_robot_state ROS nodes."""

from __future__ import annotations

import time
from typing import List

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, HistoryPolicy

from ..core.urdf_model import JointSpec, UrdfModel

# Raw feedback: accept from both RELIABLE and BEST_EFFORT publishers (a
# BEST_EFFORT subscriber is compatible with either). Shallow queue: we only
# ever want the newest state, never a backlog.
SENSOR_IN_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=5
)
# /joint_states must be RELIABLE: robot_state_publisher subscribes RELIABLE and a
# BEST_EFFORT publisher would be silently incompatible (no data, no error).
JOINT_STATES_OUT_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST, depth=10
)
LATCHED_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)


def load_model(node: Node, wait_s: float = 10.0) -> UrdfModel:
    """URDF from the 'robot_description' parameter, else from the latched
    /robot_description topic that robot_state_publisher publishes."""
    node.declare_parameter("robot_description", "")
    xml = node.get_parameter("robot_description").get_parameter_value().string_value
    if not xml:
        got = []
        sub = node.create_subscription(String, "/robot_description", lambda m: got.append(m.data), LATCHED_QOS)
        deadline = time.monotonic() + wait_s
        while not got and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
        node.destroy_subscription(sub)
        if not got:
            raise RuntimeError(f"{node.get_name()}: no robot_description parameter or /robot_description topic")
        xml = got[0]
    return UrdfModel.from_string(xml)


def arm_joints(node: Node, model: UrdfModel) -> List[JointSpec]:
    node.declare_parameter("base_link", "base_link")
    node.declare_parameter("tip_link", "Link6")
    base = node.get_parameter("base_link").value
    tip = node.get_parameter("tip_link").value
    joints = model.actuated_joints(tip, base)
    if len(joints) != 6:
        node.get_logger().warn(f"expected 6 actuated joints {base}->{tip}, found {len(joints)}")
    return joints
