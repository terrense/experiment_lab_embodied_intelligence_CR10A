"""OMPL planner benchmark on the CR10 model (SIMULATION ONLY - talks to move_group only).

  ros2 run ex_planning_lab benchmark --ros-args \
      -p scenes:="[wall,window,shelf]" -p trials:=5 -p planning_time:=2.0 \
      -p planners:="[RRTConnect,RRT,RRTstar,BiTRRT,PRM,PRMstar,KPIECE,BKPIECE,EST,SBL]" \
      -p out_dir:=data/reports/planning_lab/run01

For every scene: apply boxes to the planning scene, build queries (explicit + reproducibly
sampled, each validated with /check_state_validity), then for every planner x query x trial
call /plan_kinematic_path and record:
  success, planning_time, joint & TCP path length, trajectory duration (TOTG),
  conservative min clearance (capsule model fitted to the meshes).
Outputs results.csv, summary.json, summary.md, paths.json (TCP polylines for plotting).
"""

from __future__ import annotations

import csv
import json
import math
import time
from pathlib import Path
from typing import Dict, List

import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Pose
from moveit_msgs.msg import (CollisionObject, Constraints, JointConstraint, PlanningScene, RobotState)
from moveit_msgs.srv import ApplyPlanningScene, GetMotionPlan, GetStateValidity
from rclpy.node import Node
from sensor_msgs.msg import JointState
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray

from ex_robot_state.nodes.common import LATCHED_QOS, load_model

from ..core.capsules import capsule_box_distance, fit_capsules, link_point_clouds, points_box_distance
from ..core.metrics import cartesian_path_length, clearance_interval, densify, joint_path_length, summarize
from ..core.scene import Box, Scene, sample_in_tcp_region

JOINTS = [f"joint{i}" for i in range(1, 7)]
ARM_LINKS = ["base_link", "Link1", "Link2", "Link3", "Link4", "Link5", "Link6"]
PALETTE = [(0.90, 0.30, 0.30), (0.30, 0.70, 0.95), (0.95, 0.75, 0.20), (0.45, 0.85, 0.45), (0.80, 0.45, 0.95),
           (0.95, 0.55, 0.20), (0.40, 0.90, 0.85), (0.95, 0.45, 0.70), (0.70, 0.70, 0.70), (0.55, 0.55, 0.95)]


def _resolve_pkg(uri: str):
    if uri.startswith("package://"):
        pkg, rel = uri[len("package://"):].split("/", 1)
        return Path(get_package_share_directory(pkg)) / rel
    if uri.startswith("file://"):
        return Path(uri[len("file://"):])
    return Path(uri)


class Benchmark(Node):
    def __init__(self) -> None:
        super().__init__("planning_lab_benchmark")
        p = self.declare_parameter
        p("scenes", ["wall", "window", "shelf"])
        p("planners", ["RRTConnect", "RRT", "RRTstar", "BiTRRT", "PRM", "PRMstar", "KPIECE", "BKPIECE", "EST", "SBL"])
        p("trials", 5)
        p("planning_time", 2.0)
        p("velocity_scaling", 0.3)
        p("out_dir", "data/reports/planning_lab/latest")
        p("group", "arm")
        p("min_query_clearance", 0.03)
        gp = lambda n: self.get_parameter(n).value  # noqa: E731
        self.cfg = {k: gp(k) for k in ("scenes", "planners", "trials", "planning_time", "velocity_scaling", "out_dir", "group")}

        self.model = load_model(self)
        self.tcp_chain = self.model.actuated_joints("tcp", "base_link")
        self.lower = np.array([j.lower for j in self.tcp_chain])
        self.upper = np.array([j.upper for j in self.tcp_chain])
        urdf_xml = self._robot_description_xml()
        child_origin = {}
        for j in self.model.joints.values():
            if j.parent in ARM_LINKS:
                child_origin[j.parent] = j.origin[:3, 3]
        self.capsules = fit_capsules(urdf_xml, _resolve_pkg, ARM_LINKS, child_origin)
        self.get_logger().info("capsule radii (m): " + ", ".join(f"{c.link}={c.radius:.3f}" for c in self.capsules))
        self.spacing = 0.005
        self.points = link_point_clouds(urdf_xml, _resolve_pkg, ARM_LINKS, self.spacing)
        # surface-sampled distance exceeds the true mesh distance by at most `spacing`
        self.est_error_bound = self.spacing
        self.get_logger().info(f"mesh surface samples per link: { {k: len(v) for k, v in self.points.items()} }; "
                               f"estimate error bound {self.est_error_bound * 1000:.1f} mm")

        self.plan_cli = self.create_client(GetMotionPlan, "/plan_kinematic_path")
        self.valid_cli = self.create_client(GetStateValidity, "/check_state_validity")
        self.scene_cli = self.create_client(ApplyPlanningScene, "/apply_planning_scene")
        self.marker_pub = self.create_publisher(MarkerArray, "/ex/planning_lab/markers", LATCHED_QOS)
        for c in (self.plan_cli, self.valid_cli, self.scene_cli):
            if not c.wait_for_service(timeout_sec=30.0):
                raise RuntimeError(f"service {c.srv_name} not available - is sim_moveit.launch.py running?")
        # Several move_group instances answer the same service names with DIFFERENT planning
        # scenes -> non-repeatable results. Refuse to benchmark unless exactly one exists.
        time.sleep(1.0)
        n_mg = sum(1 for name, ns in self.get_node_names_and_namespaces() if name == "move_group")
        if n_mg != 1:
            raise RuntimeError(f"found {n_mg} move_group nodes; run `bash scripts/m2_session.sh start` (exactly one required)")

    # ------------------------------------------------------------------ utils
    def _robot_description_xml(self) -> str:
        from std_msgs.msg import String
        got: List[str] = []
        sub = self.create_subscription(String, "/robot_description", lambda m: got.append(m.data), LATCHED_QOS)
        t0 = time.monotonic()
        while not got and time.monotonic() - t0 < 10:
            rclpy.spin_once(self, timeout_sec=0.1)
        self.destroy_subscription(sub)
        if not got:
            raise RuntimeError("no /robot_description")
        return got[0]

    def _call(self, cli, req, timeout=30.0):
        f = cli.call_async(req)
        rclpy.spin_until_future_complete(self, f, timeout_sec=timeout)
        if f.result() is None:
            raise RuntimeError(f"{cli.srv_name} timed out")
        return f.result()

    def _q(self, q) -> Dict[str, float]:
        return dict(zip(JOINTS, (float(v) for v in q)))

    def fk_tcp(self, q) -> np.ndarray:
        return self.model.fk(self._q(q), "tcp", "world")[:3, 3]

    def link_poses(self, q) -> Dict[str, np.ndarray]:
        return self.model.all_link_poses(self._q(q))

    def _robot_state(self, q) -> RobotState:
        rs = RobotState()
        rs.joint_state = JointState(name=JOINTS, position=[float(v) for v in q])
        rs.is_diff = False
        return rs

    # ------------------------------------------------------------------ scene
    def apply_scene(self, scene: Scene, previous: List[str]) -> None:
        ps = PlanningScene(is_diff=True)
        for name in previous:
            co = CollisionObject(id=name, operation=CollisionObject.REMOVE)
            co.header.frame_id = "world"
            ps.world.collision_objects.append(co)
        for b in scene.boxes:
            co = CollisionObject(id=b.name, operation=CollisionObject.ADD)
            co.header.frame_id = "world"
            co.primitives = [SolidPrimitive(type=SolidPrimitive.BOX, dimensions=[float(v) for v in b.size])]
            pose = Pose()
            pose.position.x, pose.position.y, pose.position.z = (float(v) for v in b.center)
            pose.orientation.w = 1.0
            co.primitive_poses = [pose]
            ps.world.collision_objects.append(co)
        if not self._call(self.scene_cli, ApplyPlanningScene.Request(scene=ps)).success:
            raise RuntimeError("apply_planning_scene failed")

    def valid(self, q) -> bool:
        req = GetStateValidity.Request(robot_state=self._robot_state(q), group_name=self.cfg["group"])
        return bool(self._call(self.valid_cli, req).valid)

    def state_clearance(self, q, scene: Scene) -> float:
        """Mesh-surface clearance estimate of a single state (m)."""
        _, est, _ = clearance_interval(np.array([q]), self.link_poses, self.capsules, self.points, scene.boxes,
                                       capsule_box_distance, points_box_distance, top_k=1)
        return est

    def build_queries(self, scene: Scene):
        out = []
        for q in scene.queries:
            out.append({"name": q.name, "start": np.array(q.start), "goal": np.array(q.goal), "source": "explicit"})
        spec = scene.sample_spec
        if spec:
            rng = np.random.default_rng(int(spec.get("seed", 0)))
            sb = Box("s", np.array(spec["start_tcp_box"]["lo"]) / 2 + np.array(spec["start_tcp_box"]["hi"]) / 2,
                     np.array(spec["start_tcp_box"]["hi"]) - np.array(spec["start_tcp_box"]["lo"]))
            gb = Box("g", np.array(spec["goal_tcp_box"]["lo"]) / 2 + np.array(spec["goal_tcp_box"]["hi"]) / 2,
                     np.array(spec["goal_tcp_box"]["hi"]) - np.array(spec["goal_tcp_box"]["lo"]))
            # restrict sampling to a sane band (wrist joints within +-180 deg) for reproducibility
            lo = np.maximum(self.lower, -math.pi)
            hi = np.minimum(self.upper, math.pi)
            for k in range(int(spec.get("n", 3))):
                pair = []
                for region in (sb, gb):
                    q = None
                    for _ in range(400):
                        cand = sample_in_tcp_region(rng, self.fk_tcp, lo, hi, region, max_tries=5000)
                        # states grazing an obstacle measure collision-checker numerics, not planners
                        if (cand is not None and self.valid(cand)
                                and self.state_clearance(cand, scene) >= float(self.get_parameter("min_query_clearance").value)):
                            q = cand
                            break
                    pair.append(q)
                if any(x is None for x in pair):
                    self.get_logger().warn(f"{scene.name}: could not sample valid query {k}")
                    continue
                out.append({"name": f"sampled_{k}", "start": pair[0], "goal": pair[1], "source": f"sampled(seed={spec.get('seed', 0)})"})
        return out

    # ------------------------------------------------------------------ plan
    def plan(self, planner: str, start, goal):
        req = GetMotionPlan.Request()
        r = req.motion_plan_request
        r.group_name = self.cfg["group"]
        r.planner_id = planner
        r.allowed_planning_time = float(self.cfg["planning_time"])
        r.num_planning_attempts = 1
        r.max_velocity_scaling_factor = float(self.cfg["velocity_scaling"])
        r.max_acceleration_scaling_factor = float(self.cfg["velocity_scaling"])
        r.start_state = self._robot_state(start)
        c = Constraints()
        for n, v in zip(JOINTS, goal):
            c.joint_constraints.append(JointConstraint(joint_name=n, position=float(v), tolerance_above=1e-3,
                                                       tolerance_below=1e-3, weight=1.0))
        r.goal_constraints = [c]
        t0 = time.monotonic()
        res = self._call(self.plan_cli, req, timeout=r.allowed_planning_time + 30.0).motion_plan_response
        wall = time.monotonic() - t0
        traj = res.trajectory.joint_trajectory
        Q = np.array([[p.positions[traj.joint_names.index(n)] for n in JOINTS] for p in traj.points]) if traj.points else None
        dur = (traj.points[-1].time_from_start.sec + traj.points[-1].time_from_start.nanosec * 1e-9) if traj.points else None
        return res.error_code.val, res.planning_time, wall, Q, dur

    # ------------------------------------------------------------------ run
    def run(self) -> None:
        out = Path(self.cfg["out_dir"])
        out.mkdir(parents=True, exist_ok=True)
        share = Path(get_package_share_directory("ex_planning_lab")) / "scenes"
        rows, paths, query_log = [], {}, []
        previous: List[str] = []
        t_start = time.time()
        for sname in self.cfg["scenes"]:
            scene = Scene.from_yaml(share / f"{sname}.yaml")
            self.apply_scene(scene, previous)
            previous = [b.name for b in scene.boxes]
            time.sleep(0.5)
            queries = self.build_queries(scene)
            for q in queries:
                sv, gv = self.valid(q["start"]), self.valid(q["goal"])
                query_log.append({"scene": sname, "query": q["name"], "source": q["source"],
                                  "start_deg": np.degrees(q["start"]).round(2).tolist(),
                                  "goal_deg": np.degrees(q["goal"]).round(2).tolist(),
                                  "start_tcp": self.fk_tcp(q["start"]).round(3).tolist(),
                                  "goal_tcp": self.fk_tcp(q["goal"]).round(3).tolist(),
                                  "start_valid": sv, "goal_valid": gv,
                                  "start_clearance_m": round(self.state_clearance(q["start"], scene), 4),
                                  "goal_clearance_m": round(self.state_clearance(q["goal"], scene), 4)})
                if not (sv and gv):
                    self.get_logger().error(f"{sname}/{q['name']}: start valid={sv} goal valid={gv} -> skipped")
                    continue
                straight = float(np.linalg.norm(q["goal"] - q["start"]))
                for planner in self.cfg["planners"]:
                    for trial in range(int(self.cfg["trials"])):
                        code, ptime, wall, Q, dur = self.plan(planner, q["start"], q["goal"])
                        row = {"scene": sname, "query": q["name"], "planner": planner, "trial": trial,
                               "success": int(code == 1), "error_code": code, "planning_time_s": round(ptime, 4),
                               "wall_time_s": round(wall, 4), "straight_joint_dist_rad": round(straight, 4)}
                        if code == 1 and Q is not None:
                            P = np.array([self.fk_tcp(x) for x in Q])
                            Qd = densify(Q, 0.02)
                            lb, est, pair = clearance_interval(Qd, self.link_poses, self.capsules, self.points,
                                                               scene.boxes, capsule_box_distance, points_box_distance)
                            row.update({"waypoints": len(Q), "joint_path_rad": round(joint_path_length(Q), 4),
                                        "tcp_path_m": round(cartesian_path_length(P), 4),
                                        "duration_s": round(dur, 4), "clearance_lb_m": round(lb, 4),
                                        "clearance_est_m": round(est, 4),
                                        "clearance_pair": "-".join(pair) if pair else ""})
                            key = f"{sname}/{q['name']}/{planner}"
                            if key not in paths:
                                paths[key] = P.round(4).tolist()
                        rows.append(row)
                    ok = [r for r in rows if r["scene"] == sname and r["query"] == q["name"] and r["planner"] == planner]
                    self.get_logger().info(f"{sname}/{q['name']:10s} {planner:10s} success {sum(r['success'] for r in ok)}/{len(ok)}")
            self.publish_markers(scene, paths, sname)
        self.write(out, rows, paths, query_log, time.time() - t_start)

    def publish_markers(self, scene: Scene, paths, sname: str) -> None:
        arr = MarkerArray()
        arr.markers.append(Marker(action=Marker.DELETEALL))
        for k, (key, P) in enumerate(p for p in paths.items() if p[0].startswith(sname + "/")):
            planner = key.split("/")[-1]
            ci = self.cfg["planners"].index(planner) % len(PALETTE)
            m = Marker(type=Marker.LINE_STRIP, action=Marker.ADD, ns=key.rsplit("/", 1)[0], id=k)
            m.header.frame_id = "world"
            m.scale.x = 0.006
            m.color = ColorRGBA(r=PALETTE[ci][0], g=PALETTE[ci][1], b=PALETTE[ci][2], a=0.9)
            m.pose.orientation.w = 1.0
            from geometry_msgs.msg import Point
            m.points = [Point(x=p[0], y=p[1], z=p[2]) for p in P]
            arr.markers.append(m)
        self.marker_pub.publish(arr)

    def write(self, out: Path, rows, paths, query_log, elapsed: float) -> None:
        fields = ["scene", "query", "planner", "trial", "success", "error_code", "planning_time_s", "wall_time_s",
                  "waypoints", "joint_path_rad", "straight_joint_dist_rad", "tcp_path_m", "duration_s", "clearance_lb_m",
                  "clearance_est_m",
                  "clearance_pair"]
        with open(out / "results.csv", "w", newline="") as f:
            w = csv.DictWriter(f, fieldnames=fields)
            w.writeheader()
            for r in rows:
                w.writerow({k: r.get(k, "") for k in fields})
        summary = {"config": self.cfg, "elapsed_s": round(elapsed, 1), "queries": query_log,
                   "capsules": {c.link: round(c.radius, 4) for c in self.capsules},
                   "clearance_estimate_error_bound_m": round(self.est_error_bound, 4), "by_scene_planner": {}}
        md = ["| scene | planner | success | plan time median [p90] (s) | joint path median (rad) | TCP path median (m) "
              "| duration median (s) | min clearance median: est [capsule LB] (m) |", "|---|---|---|---|---|---|---|---|"]
        for sname in self.cfg["scenes"]:
            for planner in self.cfg["planners"]:
                rs = [r for r in rows if r["scene"] == sname and r["planner"] == planner]
                if not rs:
                    continue
                ok = [r for r in rs if r["success"]]
                s = {"attempts": len(rs), "successes": len(ok), "success_rate": len(ok) / len(rs),
                     "planning_time_s": summarize([r["planning_time_s"] for r in ok]),
                     "joint_path_rad": summarize([r["joint_path_rad"] for r in ok]),
                     "tcp_path_m": summarize([r["tcp_path_m"] for r in ok]),
                     "duration_s": summarize([r["duration_s"] for r in ok]),
                     "clearance_lb_m": summarize([r["clearance_lb_m"] for r in ok]),
                     "clearance_est_m": summarize([r["clearance_est_m"] for r in ok])}
                summary["by_scene_planner"][f"{sname}/{planner}"] = s
                g = lambda k, f="{:.3f}": f.format(s[k]["median"]) if s[k].get("n") else "-"  # noqa: E731
                pt = f"{s['planning_time_s']['median']:.3f} [{s['planning_time_s']['p90']:.3f}]" if ok else "-"
                md.append(f"| {sname} | {planner} | {len(ok)}/{len(rs)} | {pt} | {g('joint_path_rad', '{:.2f}')} | "
                          f"{g('tcp_path_m', '{:.2f}')} | {g('duration_s', '{:.2f}')} | {g('clearance_est_m', '{:.3f}')} [{g('clearance_lb_m', '{:.3f}')}] |")
        (out / "summary.json").write_text(json.dumps(summary, indent=2))
        (out / "summary.md").write_text("\n".join(md) + "\n")
        (out / "paths.json").write_text(json.dumps(paths))
        self.get_logger().info(f"wrote {out}/results.csv, summary.json, summary.md ({len(rows)} plans, {elapsed:.0f} s)")
        print("\n".join(md))


def main() -> None:
    rclpy.init()
    node = Benchmark()
    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
