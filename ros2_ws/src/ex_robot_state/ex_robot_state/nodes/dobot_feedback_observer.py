"""Mode C (LIVE_OBSERVE): feedback-only bridge from a Dobot V4 controller.

Connects ONLY to a feedback port (default 30004, 8 ms) through
ReadOnlyFeedbackSocket, decodes 1440-byte packets and publishes:

  /joint_states_robot        sensor_msgs/JointState, radians, URDF names,
                             header.stamp = HOST receive time
  /ex/dobot/tcp_controller   geometry_msgs/PoseStamped from ToolVectorActual
                             (mm -> m; orientation from ActualQuaternion)
  /diagnostics               connection, robot mode, enable/error flags,
                             tool/user frame ids, host-controller clock offset

It never opens 29999, never sends a byte, exposes no services/actions, and on
disconnect simply stops publishing (downstream marks the state LOST) instead of
inventing zeros like the official relay does.

Two independent gates must both be satisfied to connect at all:
  param enable_live_connection = true  AND
  param operator_ack = "I_AM_ON_SITE_AND_AUTHORIZED"
"""

from __future__ import annotations

import math
import socket
import statistics
import threading
import time
from collections import deque
from typing import Deque, Optional

import rclpy
from rclpy.executors import ExternalShutdownException
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

from ..core.dobot_feedback import FeedbackPacket, FrameAssembler
from ..core.readonly_socket import ReadOnlyFeedbackSocket
from .common import JOINT_STATES_OUT_QOS

ACK = "I_AM_ON_SITE_AND_AUTHORIZED"


class DobotFeedbackObserver(Node):
    def __init__(self) -> None:
        super().__init__("dobot_feedback_observer")
        p = self.declare_parameter
        p("enable_live_connection", False)
        p("operator_ack", "")
        p("robot_ip", "192.168.5.1")
        p("feedback_port", 30004)
        p("half_close_write", False)  # real CR10A closes on FIN (verified 2026-10-26)
        p("joint_names", ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"])
        p("publish_rate_hz", 0.0)
        p("check_test_value", True)
        p("base_frame", "base_link")
        p("reconnect_backoff_s", [1.0, 2.0, 5.0])
        gp = lambda n: self.get_parameter(n).value  # noqa: E731

        self.enabled = bool(gp("enable_live_connection")) and gp("operator_ack") == ACK
        self.ip, self.port = gp("robot_ip"), int(gp("feedback_port"))
        self.names = list(gp("joint_names"))
        rate = float(gp("publish_rate_hz"))
        self.min_period = 1.0 / rate if rate > 0 else 0.0  # <= 0: publish every packet
        self.base_frame = gp("base_frame")
        self.backoff = list(gp("reconnect_backoff_s"))

        self.js_pub = self.create_publisher(JointState, "/joint_states_robot", JOINT_STATES_OUT_QOS)
        self.tcp_pub = self.create_publisher(PoseStamped, "/ex/dobot/tcp_controller", 10)
        self.diag_pub = self.create_publisher(DiagnosticArray, "/diagnostics", 10)
        self.raw_pub = self.create_publisher(Float64MultiArray, "/ex/dobot/tool_vector_raw", 10)

        self.sock = ReadOnlyFeedbackSocket(self.ip, self.port, timeout=1.0, half_close=bool(gp("half_close_write")))
        self.asm = FrameAssembler(check_test_value=bool(gp("check_test_value")))
        self.state = "DISABLED" if not self.enabled else "CONNECTING"
        self.last_pkt: Optional[FeedbackPacket] = None
        self.last_pub = 0.0
        self.connects = 0
        self.disconnects = 0
        self.last_error = ""
        self.clock_offsets: Deque[float] = deque(maxlen=500)  # host_wall - controller_ms, seconds
        self._lock = threading.Lock()
        self._stop = threading.Event()

        self.create_timer(1.0, self._publish_diag)
        if not self.enabled:
            self.get_logger().error(
                "LIVE connection DISABLED: requires enable_live_connection:=true AND "
                f"operator_ack:={ACK}. No socket will be opened.")
            return
        self.get_logger().warn(
            f"LIVE_OBSERVE: read-only feedback from {self.ip}:{self.port} "
            f"(half_close={self.sock.half_close}). This node never sends data.")
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    # ------------------------------------------------------------ io thread
    def _run(self) -> None:
        attempt = 0
        while not self._stop.is_set() and rclpy.ok():
            try:
                self.sock.connect()
                self.connects += 1
                attempt = 0
                self.state = "CONNECTED"
                self.get_logger().info(f"connected to {self.ip}:{self.port}")
                while not self._stop.is_set():
                    try:
                        data = self.sock.recv()
                    except socket.timeout:
                        self.state = "CONNECTED_NO_DATA"
                        continue
                    if not data:
                        raise ConnectionError("peer closed connection")
                    recv_wall = time.time()
                    # publish EVERY packet: TCP/NAT often delivers several 8 ms packets in one
                    # recv(); keeping only the newest dropped ~14% of samples on the real CR10A
                    # (cr10a_live_01: 2140 controller-timestamp gaps > 12 ms).
                    for pkt in self.asm.feed(data):
                        self.state = "STREAMING"
                        self._on_packet(pkt, recv_wall)
            except (OSError, ConnectionError) as e:
                self.last_error = f"{type(e).__name__}: {e}"
                self.sock.close()
                if self.state != "CONNECTING":
                    self.disconnects += 1
                self.state = "CONNECTING"
                delay = self.backoff[min(attempt, len(self.backoff) - 1)]
                attempt += 1
                self.get_logger().warn(f"feedback connection problem ({self.last_error}); retry in {delay}s")
                self._stop.wait(delay)

    def _on_packet(self, pkt: FeedbackPacket, recv_wall: float) -> None:
        with self._lock:
            self.last_pkt = pkt
            if pkt.timestamp_ms > 0:
                self.clock_offsets.append(recv_wall - pkt.timestamp_ms / 1000.0)
        now = time.monotonic()
        if now - self.last_pub < self.min_period:
            return
        self.last_pub = now
        stamp = self.get_clock().now().to_msg()
        js = JointState()
        js.header.stamp = stamp
        js.header.frame_id = self.base_frame
        js.name = list(self.names)
        js.position = pkt.q_actual_rad
        js.velocity = [math.radians(v) for v in pkt.qd_actual]  # assumed deg/s, UNVERIFIED
        self.js_pub.publish(js)

        x, y, z, rx, ry, rz = pkt.tool_vector_actual
        ps = PoseStamped()
        ps.header.stamp, ps.header.frame_id = stamp, self.base_frame
        ps.pose.position.x, ps.pose.position.y, ps.pose.position.z = x / 1000.0, y / 1000.0, z / 1000.0
        q = pkt.actual_quaternion  # order in packet UNVERIFIED: assumed (w, x, y, z)
        n = math.sqrt(sum(v * v for v in q))
        if n > 0.5:
            ps.pose.orientation.w, ps.pose.orientation.x, ps.pose.orientation.y, ps.pose.orientation.z = (v / n for v in q)
        else:
            ps.pose.orientation.w = 1.0
        self.tcp_pub.publish(ps)

        # One message per packet with everything needed for FK cross-checks, so
        # pairing is exact (cross-topic arrival order is not guaranteed). Layout:
        # [0:6] ToolVectorActual x,y,z mm rx,ry,rz deg | [6:10] ActualQuaternion as in packet
        # [10:16] QActual deg | [16] controller TimeStamp ms | [17] tool id | [18] user id
        raw = Float64MultiArray()
        raw.data = [float(v) for v in (*pkt.tool_vector_actual, *pkt.actual_quaternion, *pkt.q_actual_deg,
                                       pkt.timestamp_ms, pkt.tool_coordinate, pkt.user_coordinate)]
        self.raw_pub.publish(raw)

    # ----------------------------------------------------------- diagnostics
    def _publish_diag(self) -> None:
        with self._lock:
            pkt = self.last_pkt
            offs = list(self.clock_offsets)
        st = DiagnosticStatus()
        st.name, st.hardware_id = "ex/dobot_feedback_observer", f"{self.ip}:{self.port}"
        kv = {
            "state": self.state, "enabled": str(self.enabled), "connects": str(self.connects),
            "disconnects": str(self.disconnects), "packets": str(self.asm.packets),
            "resyncs": str(self.asm.resyncs), "discarded_bytes": str(self.asm.discarded_bytes),
            "decode_errors": str(self.asm.decode_errors), "bytes_received": str(self.sock.bytes_received),
            "last_error": self.last_error,
        }
        level = DiagnosticStatus.OK
        msg = self.state
        if not self.enabled:
            level, msg = DiagnosticStatus.WARN, "live connection disabled by configuration"
        elif self.state != "STREAMING":
            level = DiagnosticStatus.ERROR
        if pkt is not None:
            kv.update({
                "robot_mode": f"{pkt.robot_mode} {pkt.robot_mode_name}",
                "enable_status": str(pkt.enable_status), "error_status": str(pkt.error_status),
                "drag_status": str(pkt.drag_status), "collision_state": str(pkt.collision_state),
                "tool_coordinate": str(pkt.tool_coordinate), "user_coordinate": str(pkt.user_coordinate),
                "speed_scaling": f"{pkt.speed_scaling:.3f}", "load_kg": f"{pkt.load_kg:.3f}",
                "controller_timestamp_ms": str(pkt.timestamp_ms),
                "q_actual_deg": " ".join(f"{v:+.3f}" for v in pkt.q_actual_deg),
                "tool_vector_actual": " ".join(f"{v:+.3f}" for v in pkt.tool_vector_actual),
            })
            if pkt.error_status:
                level = max(level, DiagnosticStatus.WARN)
                msg += " | controller reports ERROR status"
        if len(offs) > 10:
            kv["clock_offset_ms_median"] = f"{statistics.median(offs) * 1000:.1f}"
            kv["clock_offset_ms_stdev"] = f"{statistics.pstdev(offs) * 1000:.2f}"
        st.level, st.message = level, msg
        st.values = [KeyValue(key=k, value=v) for k, v in kv.items()]
        arr = DiagnosticArray()
        arr.header.stamp = self.get_clock().now().to_msg()
        arr.status = [st]
        self.diag_pub.publish(arr)

    def destroy_node(self) -> bool:
        self._stop.set()
        self.sock.close()
        return super().destroy_node()


def main() -> None:
    rclpy.init()
    node = DobotFeedbackObserver()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
