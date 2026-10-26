"""Plan -> execute on the MOCK controller -> compare planned vs executed (SIMULATION ONLY).

  ros2 run ex_planning_lab execute_demo --ros-args -p scene:=wall -p query:=sweep_60 \
      -p planner:=RRTConnect -p out_dir:=data/reports/planning_lab/exec_wall

1. apply the scene; plan current -> query start, execute (positioning move)
2. plan start -> goal; publish planned TCP path (orange); execute via /execute_trajectory
   while recording /joint_states
3. align timelines (estimated start latency) and report per-joint RMS / max tracking error,
   executed TCP path (cyan) and both time series (CSV) for plotting.

With mock_components the "plant" has no dynamics, so tracking error mostly reflects
controller sampling and interpolation, NOT a real arm. A real CR10A would add servo lag,
ServoJ buffering and network latency (future: MuJoCo plant, then hardware with approval).
"""

from __future__ import annotations

import csv
import json
import time
from pathlib import Path

import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Point
from moveit_msgs.action import ExecuteTrajectory
from moveit_msgs.msg import RobotTrajectory
from rclpy.action import ActionClient
from sensor_msgs.msg import JointState
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray

from ..core.metrics import cartesian_path_length, estimate_time_offset, tracking_error
from ..core.scene import Scene
from .benchmark import JOINTS, Benchmark


class ExecuteDemo(Benchmark):
    def __init__(self) -> None:
        super().__init__()
        p = self.declare_parameter
        p("scene", "wall")
        p("query", "sweep_60")
        p("planner", "RRTConnect")
        self.exec_cli = ActionClient(self, ExecuteTrajectory, "/execute_trajectory")
        if not self.exec_cli.wait_for_server(timeout_sec=20.0):
            raise RuntimeError("/execute_trajectory not available")
        self.samples = []
        self.recording = False
        self.create_subscription(JointState, "/joint_states", self._on_js, 100)
        self.last_q = None

    def _on_js(self, m: JointState) -> None:
        if set(JOINTS) <= set(m.name):
            q = [m.position[m.name.index(n)] for n in JOINTS]
            self.last_q = np.array(q)
            if self.recording:
                self.samples.append((time.monotonic(), q))

    def _execute(self, joint_traj) -> bool:
        goal = ExecuteTrajectory.Goal(trajectory=RobotTrajectory(joint_trajectory=joint_traj))
        f = self.exec_cli.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, f, timeout_sec=10)
        gh = f.result()
        if gh is None or not gh.accepted:
            return False
        t_accept = time.monotonic()
        r = gh.get_result_async()
        rclpy.spin_until_future_complete(self, r, timeout_sec=120)
        self.t_accept = t_accept
        return r.result() is not None and r.result().result.error_code.val == 1

    def _plan_traj(self, planner, start, goal):
        """Like Benchmark.plan but returns the JointTrajectory message for execution."""
        from moveit_msgs.msg import Constraints, JointConstraint
        from moveit_msgs.srv import GetMotionPlan
        req = GetMotionPlan.Request()
        r = req.motion_plan_request
        r.group_name, r.planner_id = self.cfg["group"], planner
        r.allowed_planning_time, r.num_planning_attempts = float(self.cfg["planning_time"]), 3
        r.max_velocity_scaling_factor = r.max_acceleration_scaling_factor = float(self.cfg["velocity_scaling"])
        r.start_state = self._robot_state(start)
        c = Constraints()
        for n, v in zip(JOINTS, goal):
            c.joint_constraints.append(JointConstraint(joint_name=n, position=float(v), tolerance_above=1e-3,
                                                       tolerance_below=1e-3, weight=1.0))
        r.goal_constraints = [c]
        res = self._call(self.plan_cli, req, timeout=60).motion_plan_response
        return res.error_code.val, res.trajectory.joint_trajectory

    def _marker(self, P, mid, color, ns):
        m = Marker(type=Marker.LINE_STRIP, action=Marker.ADD, ns=ns, id=mid)
        m.header.frame_id = "world"
        m.scale.x = 0.008
        m.color = color
        m.pose.orientation.w = 1.0
        m.points = [Point(x=float(p[0]), y=float(p[1]), z=float(p[2])) for p in P]
        return m

    def run(self) -> None:
        gp = lambda n: self.get_parameter(n).value  # noqa: E731
        out = Path(self.cfg["out_dir"])
        out.mkdir(parents=True, exist_ok=True)
        scene = Scene.from_yaml(Path(get_package_share_directory("ex_planning_lab")) / "scenes" / f"{gp('scene')}.yaml")
        self.apply_scene(scene, [])
        q = [x for x in self.build_queries(scene) if x["name"] == gp("query")]
        if not q:
            raise RuntimeError(f"query {gp('query')} not found in scene {scene.name}")
        q = q[0]
        while self.last_q is None:
            rclpy.spin_once(self, timeout_sec=0.1)

        # 1. positioning move
        code, traj = self._plan_traj(gp("planner"), self.last_q, q["start"])
        if code != 1 or not self._execute(traj):
            raise RuntimeError(f"positioning move failed (code {code})")
        t_end = time.monotonic() + 1.0
        while time.monotonic() < t_end:
            rclpy.spin_once(self, timeout_sec=0.05)

        # 2. the measured move
        code, traj = self._plan_traj(gp("planner"), self.last_q, q["goal"])
        if code != 1:
            raise RuntimeError(f"planning failed (code {code})")
        t_plan = np.array([p.time_from_start.sec + p.time_from_start.nanosec * 1e-9 for p in traj.points])
        Q_plan = np.array([[p.positions[traj.joint_names.index(n)] for n in JOINTS] for p in traj.points])
        P_plan = np.array([self.fk_tcp(x) for x in Q_plan])
        arr = MarkerArray(markers=[Marker(action=Marker.DELETEALL),
                                   self._marker(P_plan, 0, ColorRGBA(r=1.0, g=0.55, b=0.1, a=0.95), "planned_tcp")])
        self.marker_pub.publish(arr)
        self.samples, self.recording = [], True
        ok = self._execute(traj)
        t_end = time.monotonic() + 1.0
        while time.monotonic() < t_end:
            rclpy.spin_once(self, timeout_sec=0.02)
        self.recording = False
        if not ok:
            raise RuntimeError("execution failed")

        t_exec = np.array([s[0] for s in self.samples]) - self.t_accept
        Q_exec = np.array([s[1] for s in self.samples])
        P_exec = np.array([self.fk_tcp(x) for x in Q_exec])
        d = estimate_time_offset(t_plan, Q_plan, t_exec, Q_exec)
        raw = tracking_error(t_plan, Q_plan, t_exec, Q_exec)
        aligned = tracking_error(t_plan, Q_plan, t_exec - d, Q_exec)
        arr.markers.append(self._marker(P_exec, 1, ColorRGBA(r=0.1, g=0.8, b=1.0, a=0.95), "executed_tcp"))
        self.marker_pub.publish(arr)
        goal_err = np.abs(Q_exec[-1] - q["goal"])
        rep = {
            "scene": scene.name, "query": q["name"], "planner": gp("planner"), "plant": "mock_components/GenericSystem (no dynamics)",
            "planned_duration_s": float(t_plan[-1]), "exec_samples": int(len(Q_exec)),
            "joint_states_rate_hz": float((len(t_exec) - 1) / (t_exec[-1] - t_exec[0])) if len(t_exec) > 1 else None,
            "estimated_start_latency_s": d,
            "tracking_rms_deg_raw": np.degrees(raw["rms"]).round(4).tolist(),
            "tracking_rms_deg_aligned": np.degrees(aligned["rms"]).round(4).tolist(),
            "tracking_max_deg_aligned": np.degrees(aligned["max"]).round(4).tolist(),
            "final_goal_error_deg": np.degrees(goal_err).round(4).tolist(),
            "tcp_path_planned_m": cartesian_path_length(P_plan), "tcp_path_executed_m": cartesian_path_length(P_exec),
        }
        (out / "execute_report.json").write_text(json.dumps(rep, indent=2))
        with open(out / "planned.csv", "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(["t"] + JOINTS + ["tcp_x", "tcp_y", "tcp_z"])
            for t, qq, pp in zip(t_plan, Q_plan, P_plan):
                w.writerow([round(t, 4), *np.round(qq, 6), *np.round(pp, 5)])
        with open(out / "executed.csv", "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(["t"] + JOINTS + ["tcp_x", "tcp_y", "tcp_z"])
            for t, qq, pp in zip(t_exec, Q_exec, P_exec):
                w.writerow([round(t, 4), *np.round(qq, 6), *np.round(pp, 5)])
        print(json.dumps(rep, indent=2))


def main() -> None:
    rclpy.init()
    node = ExecuteDemo()
    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
