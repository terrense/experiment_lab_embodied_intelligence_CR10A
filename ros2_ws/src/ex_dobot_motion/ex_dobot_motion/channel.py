"""The command channel of the REAL CR10A - the on-site capture (SIMULATION-ONLY counterpart:
`evaluation/measure_execution.py`).

Why this exists
---------------
The paper's premise is a *command channel*: the command is integrated into a position reference
and tracked by a delayed, bandwidth-limited servo, so the arm keeps moving after the request
stops. That channel has only ever been measured **on the simulator** (14.9 mrad of coast at a
1 rad/s command, an identified second-order model with omega = 37.7 rad/s and zeta = 0.90). The
"execution-aware" argument only transfers to hardware if the REAL channel behaves like that, and
the amount it differs is exactly the quantity the safety margin has to absorb.

The measurement
---------------
1. A single small **step** of the wrist joint (J5) from the current pose. The wrist is chosen
   because it barely moves the TCP, so the safety envelope is trivially satisfied and a mistake
   cannot reach anything.
2. The whole response is recorded from the read-only feedback at ~125 Hz: the commanded target
   (`q_target`), the measured position (`q_actual`) and the measured velocity (`qd_actual`).
3. When the commanded target stops changing, the **residual motion** is measured: how far the arm
   still travels, how fast it was moving at that instant, and how long it takes to settle. That
   is the hardware counterpart of the simulator's coast.
4. The arm is returned to the pose it started from.

The analysis is NOT here: the capture is written to disk and `evaluation/measure_execution_hardware.py`
fits the model offline, so the analysis can be redone without the robot and so the motion package
stays minimal. The two produce the SAME JSON schema, which is the point: the simulator's channel
and the real one become directly comparable numbers.

Safety (the same properties as the excitation runner, unit tested):
  * **dry run by default** - nothing is sent, the plan is written and inspected;
  * the command whitelist, MovJ only, `v` and `a` inside the caps (dashboard.py checks this
    before a byte reaches the socket);
  * the plan is validated against the safety envelope (joint limits, TCP height, TCP excursion)
    BEFORE anything is sent;
  * an abort (Stop + DisableRobot) on a controller error, a collision flag, an unexpected robot
    mode, feedback loss, or the arm leaving the envelope;
  * **the operator must be at the robot with the e-stop in reach**: the `--execute` mode demands
    the explicit confirmation string.

  DRY RUN (default, no command is sent; reads feedback only):
    python3 -m ex_dobot_motion.channel --robot-ip 192.168.201.1
  EXECUTE (only on site, operator at the robot with the e-stop in reach):
    python3 -m ex_dobot_motion.channel --robot-ip 192.168.201.1 --execute \
        --confirm I_AM_AT_THE_ROBOT_WITH_ESTOP
"""

from __future__ import annotations

import argparse
import json
import math
import sys
import time
from pathlib import Path
from typing import Dict, List, Optional, Sequence

import numpy as np

from ex_robot_state.core.urdf_model import UrdfModel

from .dashboard import DashboardClient, DashboardError, movj_joint
from .excitation import SafetyEnvelope, validate_plan
from .runner import COLUMNS, Feedback, _installed_urdf, JOINTS

CONFIRM = "I_AM_AT_THE_ROBOT_WITH_ESTOP"

#: the joint that is stepped. J5 is the wrist pitch: it moves the TCP by a few millimetres, so a
#: mistake cannot reach anything, and the arm is far from its joint limits in almost any pose.
STEP_JOINT = 5

#: how long the response is recorded after the target stops changing
SETTLE_S = 2.0


def make_step_plan(q0_deg: Sequence[float], joint: int = STEP_JOINT, step_deg: float = 4.0
                   ) -> Dict:
    """The plan: move ONE joint by `step_deg` and come back.

    A pure function, so it can be unit tested without a robot.
    """
    q0 = np.asarray(q0_deg, float)
    if not 1 <= joint <= 6:
        raise ValueError("the joint index is 1..6")
    if not 0.5 <= step_deg <= 10.0:
        raise ValueError("the step must be between 0.5 and 10 degrees")
    fwd = q0.copy()
    fwd[joint - 1] += step_deg
    back = q0.copy()
    return {"joint": joint, "step_deg": step_deg, "q0_deg": list(q0),
            "waypoints_deg": [list(fwd), list(back)]}


def validate_step_plan(plan: Dict, link_positions, lower_deg: Sequence[float],
                       upper_deg: Sequence[float], env: SafetyEnvelope) -> List[str]:
    """Check the step against the safety envelope. The same rules the excitation runner uses."""
    from .excitation import ExcitationPlan
    ex = ExcitationPlan(plan["q0_deg"], plan["waypoints_deg"], [0.0] * 6, {})
    return validate_plan(ex, link_positions, lower_deg, upper_deg, env)


def main(argv: Optional[List[str]] = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--robot-ip", required=True)
    ap.add_argument("--dashboard-port", type=int, default=29999)
    ap.add_argument("--feedback-port", type=int, default=30004)
    ap.add_argument("--urdf", default="", help="expanded URDF (default: calibrated ex_cr10)")
    ap.add_argument("--joint", type=int, default=STEP_JOINT, help="the joint to step (1..6)")
    ap.add_argument("--step-deg", type=float, default=4.0, help="the step size [deg]")
    ap.add_argument("--speed", type=int, default=15, help="MovJ v%% (<= 30)")
    ap.add_argument("--acc", type=int, default=15, help="MovJ a%% and AccJ (<= 30)")
    ap.add_argument("--speed-factor", type=int, default=15, help="global SpeedFactor (<= 30)")
    ap.add_argument("--settle-s", type=float, default=SETTLE_S, help="the recording time after the move")
    ap.add_argument("--payload-kg", type=float, default=0.0)
    ap.add_argument("--out", default="")
    ap.add_argument("--execute", action="store_true")
    ap.add_argument("--confirm", default="")
    a = ap.parse_args(argv)

    out = Path(a.out or f"data/live/channel_{time.strftime('%Y%m%d_%H%M%S')}")
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
    print(f"[pre] mode {p0.robot_mode} ({p0.robot_mode_name}) error {p0.error_status} "
          f"collision {p0.collision_state}")
    print(f"[pre] q0 deg {np.round(p0.q_actual_deg, 2).tolist()}")
    if p0.error_status or p0.collision_state:
        print("[pre] REFUSED: the controller reports an error or a collision - clear it on the pendant")
        return 2
    if p0.robot_mode not in (3, 4, 5):
        print(f"[pre] REFUSED: the robot must be disabled or enabled-idle (got mode {p0.robot_mode})")
        return 2

    env = SafetyEnvelope()
    plan = make_step_plan(p0.q_actual_deg, a.joint, a.step_deg)
    problems = validate_step_plan(plan, link_positions, lower, upper, env)
    plan["report"] = {"problems": problems, "speed": a.speed, "acc": a.acc,
                       "speed_factor": a.speed_factor, "settle_s": a.settle_s,
                       "payload_kg": a.payload_kg}
    (out / "plan.json").write_text(json.dumps(plan, indent=1))
    print(f"[plan] step J{a.joint} by {a.step_deg} deg and back; "
          f"validation: {'OK' if not problems else f'{len(problems)} PROBLEMS'}")
    for pr in problems[:10]:
        print("   ", pr)
    if problems:
        print("[plan] REFUSED: the step violates the safety envelope")
        return 3
    if not a.execute:
        print(f"[dry-run] nothing was sent. the plan is in {out}/plan.json")
        fb.close()
        return 0
    if a.confirm != CONFIRM:
        print(f"[exec] REFUSED: --confirm {CONFIRM} required (operator at the robot, e-stop in reach)")
        return 4

    dash = DashboardClient(a.robot_ip, a.dashboard_port)
    rc = 0
    sent: List[int] = []
    try:
        dash.connect()
        if p0.robot_mode in (3, 4):
            dash.must("RequestControl()")
            dash.must(f"EnableRobot({a.payload_kg})")
            t0 = time.monotonic()
            while fb.poll().robot_mode != 5:
                if time.monotonic() - t0 > 15:
                    raise DashboardError("the robot did not reach ENABLE within 15 s")
        dash.must(f"SpeedFactor({a.speed_factor})")
        dash.must(f"AccJ({a.acc})")
        q0 = np.asarray(p0.q_actual_deg, float)
        tcp0 = link_positions(q0)["tcp"]
        todo = list(plan["waypoints_deg"])
        started = time.monotonic()
        arrived = None
        while True:
            p = fb.poll()
            # ---- the safety checks, on every packet
            if p.error_status or p.collision_state:
                raise DashboardError(f"controller error {p.error_status} / collision {p.collision_state}")
            if p.robot_mode not in (5, 7):
                raise DashboardError(f"unexpected robot mode {p.robot_mode} ({p.robot_mode_name})")
            q = np.asarray(p.q_actual_deg)
            if np.max(np.abs(q - q0)) > env.max_joint_excursion_deg:
                raise DashboardError(f"joint excursion {np.max(np.abs(q - q0)):.1f} deg beyond the envelope")
            tcp = link_positions(q)["tcp"]
            if tcp[2] < env.min_tcp_z_m or np.linalg.norm(tcp - tcp0) > env.max_tcp_excursion_m:
                raise DashboardError(f"the TCP left the validated envelope: {np.round(tcp, 3).tolist()}")
            # ---- stream the two waypoints
            while todo and (not sent or p.current_command_id >= sent[-1] - 2):
                r = dash.must(movj_joint(todo.pop(0), a.speed, a.acc, 0))
                sent.append(int(r.values[0]))
                if not todo:
                    arrived = time.monotonic()
            if not todo and sent and p.current_command_id >= sent[-1]:
                # the arm has been told everything; keep recording so the settling is captured
                if time.monotonic() - arrived > a.settle_s:
                    print(f"[exec] done: {len(sent)} moves, {a.settle_s:.1f} s of settling recorded")
                    break
            if time.monotonic() - started > 120:
                raise DashboardError("timeout (120 s)")
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
        try:
            dash.close()
        except Exception:
            pass
        fb.close()

    # ---- write the capture: one CSV with the same columns the excitation runner uses
    import csv
    csv_path = out / "channel.csv"
    with csv_path.open("w", newline="") as f:
        w = csv.writer(f)
        w.writerow(COLUMNS)
        for row in fb.rows:
            w.writerow(row)
    meta = {"columns": COLUMNS, "rows": len(fb.rows), "plan": plan, "joint": a.joint,
            "step_deg": a.step_deg, "speed": a.speed, "acc": a.acc,
            "speed_factor": a.speed_factor, "settle_s": a.settle_s,
            "note": "the capture of the real arm's command channel. The analysis is "
                    "evaluation/measure_execution_hardware.py; it writes the same JSON schema "
                    "as the simulator's execution_channel.json, so the two are comparable."}
    (out / "capture.json").write_text(json.dumps(meta, indent=1))
    print(f"[exec] wrote {csv_path} ({len(fb.rows)} packets) and {out}/capture.json")
    return rc


if __name__ == "__main__":
    raise SystemExit(main())
