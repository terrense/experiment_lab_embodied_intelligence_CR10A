"""Excitation-trajectory runner for dynamics identification on the REAL CR10A.

  DRY RUN (default, no command is sent; reads feedback only):
    python3 -m ex_dobot_motion.runner --robot-ip 192.168.201.1
  EXECUTE (only on site, operator at the robot with the e-stop in reach):
    python3 -m ex_dobot_motion.runner --robot-ip 192.168.201.1 --execute --confirm I_AM_AT_THE_ROBOT_WITH_ESTOP

Sequence: read-only preconditions -> plan around the CURRENT pose -> validate every waypoint and
the joint-space segments between them -> [execute: RequestControl, EnableRobot(payload),
SpeedFactor, AccJ, streamed MovJ(joint) with a bounded look-ahead] while every 8 ms feedback packet
is recorded and checked -> back to the start pose -> DisableRobot.
Abort (Stop + DisableRobot) on: controller error / collision flag, unexpected robot mode, TCP or
joints outside the validated envelope (+margin), feedback loss, Ctrl-C.
"""

from __future__ import annotations

import argparse
import gzip
import json
import math
import sys
import time
from pathlib import Path
from typing import Dict, List, Optional

import numpy as np

from ex_robot_state.core.dobot_feedback import FrameAssembler
from ex_robot_state.core.readonly_socket import ReadOnlyFeedbackSocket
from ex_robot_state.core.urdf_model import UrdfModel

from .dashboard import DashboardClient, DashboardError, movj_joint
from .excitation import DEFAULT_AMPLITUDES_DEG, SafetyEnvelope, make_plan, validate_plan

CONFIRM = "I_AM_AT_THE_ROBOT_WITH_ESTOP"
JOINTS = [f"joint{i}" for i in range(1, 7)]
LOOKAHEAD = 6           # MovJ commands queued ahead of the one executing
ABORT_MARGIN_M = 0.02   # TCP envelope slack before aborting
ABORT_MARGIN_DEG = 2.0


class Feedback:
    """Read-only feedback reader that also records every packet."""

    def __init__(self, ip: str, port: int) -> None:
        self.sock = ReadOnlyFeedbackSocket(ip, port, timeout=1.0, half_close=False)  # controller drops on FIN
        self.asm = FrameAssembler()
        self.rows: List[List[float]] = []
        self.last = None

    def connect(self) -> None:
        self.sock.connect()

    def poll(self, timeout_s: float = 1.0):
        t0 = time.monotonic()
        while time.monotonic() - t0 < timeout_s:
            got = False
            for p in self.asm.feed(self.sock.recv()):
                got = True
                self.last = p
                self.rows.append([time.time(), p.timestamp_ms, p.robot_mode, p.current_command_id, p.error_status,
                                  p.collision_state, *p.q_actual_deg, *p.qd_actual, *p.i_actual, *p.m_actual,
                                  *p.q_target_deg, *p.tool_vector_actual])
            if got:
                return self.last
        raise TimeoutError("no feedback packet")

    def close(self) -> None:
        self.sock.close()


COLUMNS = (["host_time", "ctrl_ms", "mode", "cmd_id", "error", "collision"]
           + [f"q{i}_deg" for i in range(1, 7)] + [f"qd{i}" for i in range(1, 7)]
           + [f"i{i}" for i in range(1, 7)] + [f"m{i}" for i in range(1, 7)]
           + [f"qt{i}_deg" for i in range(1, 7)] + ["x_mm", "y_mm", "z_mm", "rx", "ry", "rz"])


def main(argv: Optional[List[str]] = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--robot-ip", required=True)
    ap.add_argument("--dashboard-port", type=int, default=29999)
    ap.add_argument("--feedback-port", type=int, default=30004)
    ap.add_argument("--urdf", default="", help="expanded URDF (default: calibrated ex_cr10 from the install space)")
    ap.add_argument("--amplitude-scale", type=float, default=1.0, help="scales the per-joint amplitudes (<= 1 recommended)")
    ap.add_argument("--waypoints", type=int, default=60)
    ap.add_argument("--speed", type=int, default=20, help="MovJ v%% (<= 30)")
    ap.add_argument("--acc", type=int, default=20, help="MovJ a%% and AccJ (<= 30)")
    ap.add_argument("--speed-factor", type=int, default=20, help="global SpeedFactor (<= 30)")
    ap.add_argument("--blend", type=int, default=50, help="MovJ cp (0-100)")
    ap.add_argument("--payload-kg", type=float, default=0.0)
    ap.add_argument("--out", default="")
    ap.add_argument("--execute", action="store_true")
    ap.add_argument("--confirm", default="")
    a = ap.parse_args(argv)

    out = Path(a.out or f"data/live/excitation_{time.strftime('%Y%m%d_%H%M%S')}")
    out.mkdir(parents=True, exist_ok=True)
    model = UrdfModel.from_string(Path(a.urdf).read_text() if a.urdf else _installed_urdf())
    chain = model.actuated_joints("tcp", "base_link")
    lower = [math.degrees(j.lower) for j in chain]
    upper = [math.degrees(j.upper) for j in chain]

    def link_positions(q_deg) -> Dict[str, np.ndarray]:
        P = model.all_link_poses({n: math.radians(v) for n, v in zip(JOINTS, q_deg)})
        return {k: T[:3, 3] for k, T in P.items()}

    fb = Feedback(a.robot_ip, a.feedback_port)
    fb.connect()
    p0 = fb.poll(3.0)
    print(f"[pre] robot_mode {p0.robot_mode} ({p0.robot_mode_name}) error {p0.error_status} collision {p0.collision_state} "
          f"tool {p0.tool_coordinate} user {p0.user_coordinate}")
    print(f"[pre] q0 deg {np.round(p0.q_actual_deg, 2).tolist()}")
    if p0.error_status or p0.collision_state:
        print("[pre] REFUSED: controller reports error/collision - clear it on the pendant first")
        return 2
    if p0.robot_mode not in (3, 4, 5):
        print(f"[pre] REFUSED: robot must be disabled or enabled-idle (got mode {p0.robot_mode})")
        return 2

    env = SafetyEnvelope()
    amps = [x * a.amplitude_scale for x in DEFAULT_AMPLITUDES_DEG]
    plan = make_plan(p0.q_actual_deg, amps, a.waypoints)
    problems = validate_plan(plan, link_positions, lower, upper, env)
    P = np.array([link_positions(w)["tcp"] for w in plan.waypoints_deg])
    plan.report = {"tcp_min_z_m": float(P[:, 2].min()), "tcp_bbox_min": P.min(0).tolist(), "tcp_bbox_max": P.max(0).tolist(),
                   "n_waypoints": len(plan.waypoints_deg), "amplitude_deg": amps,
                   "speed": a.speed, "acc": a.acc, "speed_factor": a.speed_factor, "problems": problems}
    (out / "plan.json").write_text(json.dumps({"q0_deg": plan.q0_deg, "waypoints_deg": plan.waypoints_deg,
                                               "report": plan.report}, indent=1))
    print(f"[plan] {len(plan.waypoints_deg)} waypoints, amplitudes {amps} deg, TCP z >= {plan.report['tcp_min_z_m']:.3f} m, "
          f"validation: {'OK' if not problems else f'{len(problems)} PROBLEMS'}")
    for pr in problems[:10]:
        print("   ", pr)
    if problems:
        print("[plan] REFUSED: plan violates the safety envelope (move the arm to a more open pose and retry)")
        return 3
    if not a.execute:
        print(f"[dry-run] nothing was sent. plan saved to {out}/plan.json")
        fb.close()
        return 0
    if a.confirm != CONFIRM:
        print(f"[exec] REFUSED: --confirm {CONFIRM} required (operator at the robot, e-stop in reach)")
        return 4

    dash = DashboardClient(a.robot_ip, a.dashboard_port)
    rc = 0
    sent_ids: List[int] = []
    try:
        dash.connect()
        if p0.robot_mode in (3, 4):
            dash.must("RequestControl()")
            dash.must(f"EnableRobot({a.payload_kg})")
            t0 = time.monotonic()
            while fb.poll().robot_mode != 5:
                if time.monotonic() - t0 > 15:
                    raise DashboardError("robot did not reach ENABLE within 15 s")
        dash.must(f"SpeedFactor({a.speed_factor})")
        dash.must(f"AccJ({a.acc})")
        tcp0 = link_positions(p0.q_actual_deg)["tcp"]
        q0 = np.asarray(p0.q_actual_deg)
        todo = list(plan.waypoints_deg)
        started = time.monotonic()
        while True:
            p = fb.poll()
            # ---- safety checks on EVERY packet
            if p.error_status or p.collision_state:
                raise DashboardError(f"controller error {p.error_status} / collision {p.collision_state}")
            if p.robot_mode not in (5, 7):
                raise DashboardError(f"unexpected robot mode {p.robot_mode} ({p.robot_mode_name})")
            q = np.asarray(p.q_actual_deg)
            if np.max(np.abs(q - q0)) > env.max_joint_excursion_deg + ABORT_MARGIN_DEG:
                raise DashboardError(f"joint excursion {np.max(np.abs(q - q0)):.1f} deg beyond envelope")
            tcp = link_positions(q)["tcp"]
            if tcp[2] < env.min_tcp_z_m - ABORT_MARGIN_M or np.linalg.norm(tcp - tcp0) > env.max_tcp_excursion_m + ABORT_MARGIN_M:
                raise DashboardError(f"TCP left the validated envelope: {np.round(tcp, 3).tolist()}")
            # ---- streaming with bounded look-ahead
            while todo and (not sent_ids or p.current_command_id >= sent_ids[-1] - LOOKAHEAD):
                cp = 0 if len(todo) == 1 else a.blend
                r = dash.must(movj_joint(todo.pop(0), a.speed, a.acc, cp))
                sent_ids.append(int(r.values[0]))
            if not todo and sent_ids and p.current_command_id >= sent_ids[-1] and p.robot_mode == 5:
                print(f"[exec] done: {len(sent_ids)} moves in {time.monotonic() - started:.1f} s")
                break
            if time.monotonic() - started > 600:
                raise DashboardError("timeout (600 s)")
    except KeyboardInterrupt:
        print("[exec] Ctrl-C -> Stop")
        rc = 5
    except (DashboardError, TimeoutError, OSError) as e:
        print(f"[exec] ABORT: {e}")
        rc = 6
    finally:
        for cmd in ("Stop()", "DisableRobot()"):
            try:
                if dash._sock is not None:
                    dash.send(cmd)
            except Exception as e:  # noqa: BLE001
                print(f"[exec] {cmd} failed: {e}")
        dash.close()
        try:
            t_end = time.monotonic() + 1.0
            while time.monotonic() < t_end:
                fb.poll(0.5)
        except Exception:  # noqa: BLE001
            pass
        fb.close()
        with gzip.open(out / "feedback.csv.gz", "wt") as f:
            f.write(",".join(COLUMNS) + "\n")
            for row in fb.rows:
                f.write(",".join(f"{v:.6g}" for v in row) + "\n")
        (out / "dashboard_log.json").write_text(json.dumps(dash.log, indent=1))
        (out / "summary.json").write_text(json.dumps({"exit_code": rc, "moves_sent": len(sent_ids),
                                                      "packets": len(fb.rows), "args": vars(a)}, indent=1))
        print(f"[exec] recorded {len(fb.rows)} packets -> {out}")
    return rc


def _installed_urdf() -> str:
    import xacro
    from ament_index_python.packages import get_package_share_directory
    path = Path(get_package_share_directory("ex_description")) / "urdf" / "ex_cr10.urdf.xacro"
    return xacro.process_file(str(path), mappings={"calibrated": "true"}).toxml()


if __name__ == "__main__":
    sys.exit(main())
