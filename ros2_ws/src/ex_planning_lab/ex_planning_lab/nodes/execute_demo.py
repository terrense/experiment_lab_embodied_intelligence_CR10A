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
        p("save_trajectory", "")   # write the measured move here (JSON) for paired experiments
        p("load_trajectory", "")   # replay a saved trajectory instead of planning
        p("speed_scale", 1.0)      # re-time: t/s, dq*s, ddq*s^2
        p("tag", "")
        self.last_exec_code = None
        self.mj_sat = None
        self.mj_lag = None
        from diagnostic_msgs.msg import DiagnosticArray
        self.create_subscription(DiagnosticArray, "/diagnostics", self._on_diag, 20)
        self.exec_cli = ActionClient(self, ExecuteTrajectory, "/execute_trajectory")
        if not self.exec_cli.wait_for_server(timeout_sec=20.0):
            raise RuntimeError("/execute_trajectory not available")
        self.samples = []
        self.recording = False
        self.create_subscription(JointState, "/joint_states", self._on_js, 100)
        self.last_q = None

    def _on_diag(self, msg) -> None:
        for st in msg.status:
            if st.name == "ex/mujoco_sim":
                for kv in st.values:
                    if kv.key == "torque_saturation_steps":
                        self.mj_sat = json.loads(kv.value)
                    if kv.key == "window_max_lag_ms" and self.recording:
                        self.mj_lag = max(self.mj_lag or 0.0, float(kv.value))

    @staticmethod
    def traj_to_json(traj) -> dict:
        pts = []
        for p in traj.points:
            idx = [traj.joint_names.index(n) for n in JOINTS]
            pts.append({"t": p.time_from_start.sec + p.time_from_start.nanosec * 1e-9,
                        "q": [p.positions[i] for i in idx],
                        "dq": [p.velocities[i] for i in idx] if p.velocities else [0.0] * 6,
                        "ddq": [p.accelerations[i] for i in idx] if p.accelerations else [0.0] * 6})
        return {"joint_names": JOINTS, "points": pts}

    @staticmethod
    def json_to_traj(d: dict, speed: float):
        from builtin_interfaces.msg import Duration
        from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
        jt = JointTrajectory(joint_names=list(d["joint_names"]))
        for p in d["points"]:
            t = p["t"] / speed
            pt = JointTrajectoryPoint(positions=p["q"], velocities=[v * speed for v in p["dq"]],
                                      accelerations=[a * speed * speed for a in p["ddq"]])
            pt.time_from_start = Duration(sec=int(t), nanosec=int(round((t - int(t)) * 1e9)))
            jt.points.append(pt)
        return jt

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
        self.last_exec_code = r.result().result.error_code.val if r.result() is not None else None
        return self.last_exec_code == 1

    def _plan_traj(self, planner, start, goal, scaling=None):
        """Like Benchmark.plan but returns the JointTrajectory message for execution."""
        from moveit_msgs.msg import Constraints, JointConstraint
        from moveit_msgs.srv import GetMotionPlan
        req = GetMotionPlan.Request()
        r = req.motion_plan_request
        r.group_name, r.planner_id = self.cfg["group"], planner
        r.allowed_planning_time, r.num_planning_attempts = float(self.cfg["planning_time"]), 3
        r.max_velocity_scaling_factor = r.max_acceleration_scaling_factor = float(
            scaling if scaling is not None else self.cfg["velocity_scaling"])
        r.start_state = self._robot_state(start)
        c = Constraints()
        for n, v in zip(JOINTS, goal):
            c.joint_constraints.append(JointConstraint(joint_name=n, position=float(v), tolerance_above=1e-3,
                                                       tolerance_below=1e-3, weight=1.0))
        r.goal_constraints = [c]
        res = self._call(self.plan_cli, req, timeout=60).motion_plan_response
        return res.error_code.val, res.trajectory.joint_trajectory

    def _plant(self) -> str:
        names = {n for n, _ in self.get_node_names_and_namespaces()}
        return "mujoco (ex_mujoco_sim, servo drive + rigid-body dynamics)" if "mujoco_sim" in names             else "mock_components/GenericSystem (no dynamics)"

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
        all_names = [b.name for b in scene.boxes]
        self.apply_scene(scene, [])
        loaded = json.loads(Path(gp("load_trajectory")).read_text()) if gp("load_trajectory") else None
        if loaded is None:
            q = [x for x in self.build_queries(scene) if x["name"] == gp("query")]
            if not q:
                raise RuntimeError(f"query {gp('query')} not found in scene {scene.name}")
            q = q[0]
        else:
            q = {"name": loaded.get("query", "loaded"), "start": np.array(loaded["points"][0]["q"]),
                 "goal": np.array(loaded["points"][-1]["q"])}
        while self.last_q is None:
            rclpy.spin_once(self, timeout_sec=0.1)

        # 1. positioning move with the table only: the mock robot boots at 'home', which can
        #    intersect scene obstacles (e.g. the forearm runs through the 'wall' box) - a start
        #    state in collision makes every plan fail (observed: invalid states [0..3]).
        table_only = Scene(scene.name, "", [b for b in scene.boxes if b.name == "table"], [])
        self.apply_scene(table_only, all_names)
        # The positioning move is not the measured quantity: retry slower if a weak plant (e.g. a
        # 4 Hz servo) violates the JTC tolerances; if it still fails, WRITE a failure report rather
        # than crash - silently missing runs would bias the study towards good results.
        positioned = False
        for scaling in (None, 0.1, 0.05):
            code, traj = self._plan_traj(gp("planner"), self.last_q, q["start"], scaling)
            if code == 1 and self._execute(traj):
                positioned = True
                break
            t_settle = time.monotonic() + 1.5
            while time.monotonic() < t_settle:
                rclpy.spin_once(self, timeout_sec=0.05)
        if not positioned:
            # the controller keeps holding the last set-point: after settling, the remaining error is the
            # STATIC deviation (e.g. gravity sag from a wrong payload model) - the informative number here
            t_settle = time.monotonic() + 2.0
            while time.monotonic() < t_settle:
                rclpy.spin_once(self, timeout_sec=0.05)
            hold_err_tcp = float(np.linalg.norm(self.fk_tcp(self.last_q) - self.fk_tcp(q["start"])) * 1000)
            rep = {"tag": gp("tag"), "scene": scene.name, "plant": self._plant(), "speed_scale": float(gp("speed_scale")),
                   "execution_ok": False, "positioning_failed": True, "moveit_error_code": self.last_exec_code,
                   "static_hold_error_tcp_mm": hold_err_tcp,
                   "static_hold_error_deg": np.degrees(self.last_q - q["start"]).round(4).tolist(),
                   "tcp_tracking_rms_mm": None, "tcp_tracking_max_mm": None, "tracking_rms_deg_aligned": None,
                   "estimated_start_latency_s": None, "torque_saturation_steps_during_move": None}
            (out / "execute_report.json").write_text(json.dumps(rep, indent=2))
            self.get_logger().error("positioning move failed at all speeds; failure report written")
            return
        t_end = time.monotonic() + 1.0
        while time.monotonic() < t_end:
            rclpy.spin_once(self, timeout_sec=0.05)
        self.apply_scene(scene, ["table"])
        if not self.valid(self.last_q):
            raise RuntimeError("robot is in collision at the query start after re-adding obstacles")

        # 2. the measured move (planned now, or a saved trajectory re-timed by speed_scale)
        speed = float(gp("speed_scale"))
        if loaded is None:
            code, traj = self._plan_traj(gp("planner"), self.last_q, q["goal"])
            if code != 1:
                raise RuntimeError(f"planning failed (code {code})")
            if gp("save_trajectory"):
                d = self.traj_to_json(traj)
                d.update({"scene": scene.name, "query": q["name"], "planner": gp("planner")})
                Path(gp("save_trajectory")).write_text(json.dumps(d))
            if speed != 1.0:
                traj = self.json_to_traj(self.traj_to_json(traj), speed)
        else:
            traj = self.json_to_traj(loaded, speed)
        sat0 = list(self.mj_sat) if self.mj_sat else None
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
            self.get_logger().warn(f"execution did not succeed (MoveIt error code {self.last_exec_code})")

        t_exec = np.array([s[0] for s in self.samples]) - self.t_accept
        Q_exec = np.array([s[1] for s in self.samples])
        P_exec = np.array([self.fk_tcp(x) for x in Q_exec])
        d = estimate_time_offset(t_plan, Q_plan, t_exec, Q_exec)
        raw = tracking_error(t_plan, Q_plan, t_exec, Q_exec)
        aligned = tracking_error(t_plan, Q_plan, t_exec - d, Q_exec)
        arr.markers.append(self._marker(P_exec, 1, ColorRGBA(r=0.1, g=0.8, b=1.0, a=0.95), "executed_tcp"))
        self.marker_pub.publish(arr)
        goal_err = np.abs(Q_exec[-1] - q["goal"])
        m = (t_exec - d >= t_plan[0]) & (t_exec - d <= t_plan[-1])
        Qp_al = np.stack([np.interp(t_exec[m] - d, t_plan, Q_plan[:, j]) for j in range(6)], axis=1)
        tcp_err = np.linalg.norm(np.array([self.fk_tcp(x) for x in Qp_al]) - P_exec[m], axis=1) * 1000
        sat = (np.array(self.mj_sat) - np.array(sat0)).tolist() if (self.mj_sat and sat0) else None
        rep = {
            "tag": gp("tag"), "scene": scene.name, "query": q["name"], "planner": gp("planner"), "plant": self._plant(),
            "speed_scale": speed, "loaded_trajectory": bool(loaded), "execution_ok": bool(ok),
            "moveit_error_code": self.last_exec_code, "torque_saturation_steps_during_move": sat,
            "sim_max_lag_ms": self.mj_lag,  # max real-time lag of the physics plant DURING the measured move
            "loadavg_1m": float(open("/proc/loadavg").read().split()[0]),
            "tcp_tracking_rms_mm": float(np.sqrt((tcp_err ** 2).mean())) if len(tcp_err) else None,
            "tcp_tracking_max_mm": float(tcp_err.max()) if len(tcp_err) else None,
            "planned_duration_s": float(t_plan[-1]), "exec_samples": int(len(Q_exec)),
            "joint_states_rate_hz": float((len(t_exec) - 1) / (t_exec[-1] - t_exec[0])) if len(t_exec) > 1 else None,
            "estimated_start_latency_s": d,
            "tracking_rms_deg_raw": np.degrees(raw["rms"]).round(4).tolist(),
            "tracking_rms_deg_aligned": np.degrees(aligned["rms"]).round(4).tolist(),
            "tracking_max_deg_aligned": np.degrees(aligned["max"]).round(4).tolist(),
            "final_goal_error_deg": np.degrees(goal_err).round(4).tolist(),
            "final_tcp_error_mm": float(np.linalg.norm(self.fk_tcp(Q_exec[-1]) - self.fk_tcp(q["goal"])) * 1000),
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
