"""MuJoCo plant for the CR10A behind ros2_control (SIMULATION ONLY).

  ros2_control (JointTrajectoryController, 100 Hz)
      --/mujoco/joint_commands (JointState.position)-->  this node: servo drive @ 1 kHz + MuJoCo
      <--/mujoco/joint_states (position, velocity, effort)--

The ros2_control side uses topic_based_ros2_control/TopicBasedSystem, so MoveIt, the
controllers and every M2 tool run unchanged - only the plant is swapped (mock -> physics).

Params: robot_description (else /robot_description topic), bandwidth_hz, damping_ratio,
payload_kg (point mass at the flange), publish_rate_hz, viewer, initial_positions.
"""

from __future__ import annotations

import math
import threading
import time
from pathlib import Path

import mujoco
import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import String

from .model_builder import JOINTS, DynamicsAssumptions, build_mjcf
from .servo import CommandInterpolator, gains_from_inertia, servo_torque

LATCHED = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)


def resolve_uri(uri: str) -> Path:
    if uri.startswith("package://"):
        pkg, rel = uri[len("package://"):].split("/", 1)
        return Path(get_package_share_directory(pkg)) / rel
    return Path(uri.replace("file://", ""))


class MujocoSim(Node):
    def __init__(self) -> None:
        super().__init__("mujoco_sim")
        p = self.declare_parameter
        p("robot_description", "")
        p("commands_topic", "/mujoco/joint_commands")
        p("states_topic", "/mujoco/joint_states")
        p("bandwidth_hz", 8.0)
        p("damping_ratio", 0.9)
        p("payload_kg", 0.0)              # TRUE payload on the physical (simulated) arm
        p("payload_configured_kg", -1.0)  # what the controller believes (SetPayload); <0: same as true
        p("publish_rate_hz", 500.0)
        p("viewer", False)
        p("initial_positions", [0.0, 0.0, math.pi / 2, 0.0, -math.pi / 2, 0.0])
        p("save_mjcf", "")
        gp = lambda n: self.get_parameter(n).value  # noqa: E731

        urdf = gp("robot_description") or self._wait_description()
        dyn = DynamicsAssumptions()
        mjcf = build_mjcf(urdf, resolve_uri, dyn)
        if gp("save_mjcf"):
            Path(gp("save_mjcf")).write_text(mjcf)
        self.m = mujoco.MjModel.from_xml_string(mjcf)
        self.d = mujoco.MjData(self.m)
        self.payload = float(gp("payload_kg"))
        cfg = float(gp("payload_configured_kg"))
        self.payload_cfg = self.payload if cfg < 0 else cfg
        # controller's internal model: same arm, CONFIGURED payload. Gains and the gravity/Coriolis
        # feed-forward come from it, so a wrong SetPayload shows up as a model error.
        self.mc = mujoco.MjModel.from_xml_string(mjcf)
        self.dc = mujoco.MjData(self.mc)
        b = mujoco.mj_name2id(self.m, mujoco.mjtObj.mjOBJ_BODY, "flange")
        for model, mass in ((self.m, self.payload), (self.mc, self.payload_cfg)):
            if mass > 0:  # point mass at the flange body origin
                model.body_mass[b] += mass
                model.body_inertia[b] += 1e-4 * mass
        self.qadr = [self.m.jnt_qposadr[mujoco.mj_name2id(self.m, mujoco.mjtObj.mjOBJ_JOINT, n)] for n in JOINTS]
        self.dadr = [self.m.jnt_dofadr[mujoco.mj_name2id(self.m, mujoco.mjtObj.mjOBJ_JOINT, n)] for n in JOINTS]
        self.d.qpos[self.qadr] = gp("initial_positions")
        mujoco.mj_forward(self.m, self.d)
        self.dc.qpos[:] = self.d.qpos
        mujoco.mj_forward(self.mc, self.dc)

        M = np.zeros((self.mc.nv, self.mc.nv))
        mujoco.mj_fullM(self.mc, M, self.dc.qM)
        m_eff = np.diag(M)[self.dadr]
        self.gains = gains_from_inertia(m_eff, float(gp("bandwidth_hz")), float(gp("damping_ratio")))
        self.tau_max = np.array(dyn.torque_limit)
        self.interp = CommandInterpolator(6)
        self._lock = threading.Lock()
        self._pending = None
        self.saturation = np.zeros(6, int)
        self.max_lag = 0.0
        self.window_lag = 0.0  # max lag since the last diagnostics publish (1 s window)
        self.get_logger().info(
            f"MuJoCo {mujoco.__version__} plant: dt {self.m.opt.timestep*1000:.1f} ms, mass {self.m.body_mass.sum():.2f} kg "
            f"(+payload {self.payload} kg, controller believes {self.payload_cfg} kg), M_eff {np.round(m_eff, 3).tolist()}, kp {np.round(self.gains.kp, 0).tolist()}")

        self.state_pub = self.create_publisher(JointState, gp("states_topic"), 10)
        self.diag_pub = self.create_publisher(DiagnosticArray, "/diagnostics", 10)
        self.create_subscription(JointState, gp("commands_topic"), self._on_cmd, 10)
        self.create_timer(1.0, self._diag)
        self.pub_every = max(1, int(round(1.0 / (float(gp("publish_rate_hz")) * self.m.opt.timestep))))
        self.viewer_enabled = bool(gp("viewer"))
        self._stop = threading.Event()
        self.failed = False
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def _wait_description(self) -> str:
        got = []
        sub = self.create_subscription(String, "/robot_description", lambda m: got.append(m.data), LATCHED)
        t0 = time.monotonic()
        while not got and time.monotonic() - t0 < 15:
            rclpy.spin_once(self, timeout_sec=0.1)
        self.destroy_subscription(sub)
        if not got:
            raise RuntimeError("no robot_description")
        return got[0]

    def _on_cmd(self, msg: JointState) -> None:
        d = dict(zip(msg.name, msg.position))
        if all(n in d for n in JOINTS):
            stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            with self._lock:
                self._pending = ([d[n] for n in JOINTS], stamp if stamp > 0 else None)

    def _run(self) -> None:
        # a dead physics thread must not leave a "healthy-looking" node behind
        try:
            self._loop()
        except Exception as e:  # noqa: BLE001
            self.get_logger().fatal(f"physics thread crashed: {type(e).__name__}: {e} - shutting down")
            self.failed = True
            rclpy.try_shutdown()
            raise

    def _loop(self) -> None:
        viewer = None
        if self.viewer_enabled:
            # NOTE: `import mujoco.viewer` here would make `mujoco` a LOCAL name of this
            # function and break mujoco.mj_step below (UnboundLocalError, observed 2026-10-27)
            from mujoco import viewer as mj_viewer
            viewer = mj_viewer.launch_passive(self.m, self.d)
        dt = self.m.opt.timestep
        t0 = time.monotonic()
        step = 0
        while not self._stop.is_set() and rclpy.ok():
            # pace simulation time to wall time (real-time plant)
            lag = (time.monotonic() - t0) - self.d.time
            if lag < 0:
                time.sleep(min(-lag, 0.002))
                continue
            self.max_lag = max(self.max_lag, lag)
            self.window_lag = max(self.window_lag, lag)
            with self._lock:
                cmd, self._pending = self._pending, None
            if cmd is not None:
                self.interp.update(self.d.time, cmd[0], t_src=cmd[1])
            q = self.d.qpos[self.qadr].copy()
            dq = self.d.qvel[self.dadr].copy()
            if self.interp.has_command():
                q_ref, dq_ref = self.interp.reference(self.d.time)
            else:
                q_ref, dq_ref = q, np.zeros(6)  # hold position until the controller speaks
                if step == 0:
                    self.interp.update(0.0, q)
            # feed-forward from the CONTROLLER's model (configured payload), not the true plant
            self.dc.qpos[:] = self.d.qpos
            self.dc.qvel[:] = self.d.qvel
            mujoco.mj_fwdPosition(self.mc, self.dc)
            mujoco.mj_fwdVelocity(self.mc, self.dc)
            bias = self.dc.qfrc_bias[self.dadr]
            tau = servo_torque(q, dq, q_ref, dq_ref, bias, self.gains, self.tau_max)
            self.saturation += (np.abs(tau) >= self.tau_max - 1e-9)
            self.d.ctrl[:] = tau
            mujoco.mj_step(self.m, self.d)
            step += 1
            if step % self.pub_every == 0:
                js = JointState()
                js.header.stamp = self.get_clock().now().to_msg()
                js.name = list(JOINTS)
                js.position = self.d.qpos[self.qadr].tolist()
                js.velocity = self.d.qvel[self.dadr].tolist()
                js.effort = tau.tolist()
                self.state_pub.publish(js)
            if viewer is not None and step % 33 == 0:
                if not viewer.is_running():
                    viewer = None
                else:
                    viewer.sync()

    def _diag(self) -> None:
        st = DiagnosticStatus(name="ex/mujoco_sim", hardware_id="mujoco")
        st.level = DiagnosticStatus.OK if self.max_lag < 0.05 else DiagnosticStatus.WARN
        st.message = f"sim t={self.d.time:.1f}s max lag {self.max_lag*1000:.1f} ms"
        st.values = [KeyValue(key="sim_time_s", value=f"{self.d.time:.3f}"),
                     KeyValue(key="max_lag_ms", value=f"{self.max_lag*1000:.2f}"),
                     KeyValue(key="window_max_lag_ms", value=f"{self.window_lag*1000:.2f}"),
                     KeyValue(key="torque_saturation_steps", value=str(self.saturation.tolist())),
                     KeyValue(key="payload_kg", value=str(self.payload)),
                     KeyValue(key="payload_configured_kg", value=str(self.payload_cfg))]
        arr = DiagnosticArray()
        arr.header.stamp = self.get_clock().now().to_msg()
        arr.status = [st]
        self.diag_pub.publish(arr)
        self.window_lag = 0.0

    def destroy_node(self) -> bool:
        self._stop.set()
        self._thread.join(timeout=2.0)
        return super().destroy_node()


def main() -> None:
    rclpy.init()
    node = MujocoSim()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        failed = node.failed
        node.destroy_node()
        rclpy.try_shutdown()
    if failed:
        raise SystemExit(1)
