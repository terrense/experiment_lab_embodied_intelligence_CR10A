import json
import os
import socket
import sys
import threading
from pathlib import Path

import numpy as np
import pytest

PKG = Path(__file__).resolve().parents[1]
SRC = PKG.parent
sys.path.insert(0, str(PKG))
sys.path.insert(0, str(SRC / "ex_robot_state"))

from ex_dobot_motion.dashboard import (CommandRejected, check_command, movj_joint, parse_reply)  # noqa: E402
from ex_dobot_motion.excitation import SafetyEnvelope, make_plan, validate_plan  # noqa: E402

GOOD_Q0 = [0.0, -20.0, 100.0, -80.0, -90.0, 0.0]  # arm out front, wrist down, well above the table


# ------------------------------------------------------------------ whitelist
@pytest.mark.parametrize("cmd", ["PowerOn()", "RunScript(x)", "ClearError()", "MovL(pose={1,2,3,4,5,6})",
                                 "ServoJ(1,2,3,4,5,6)", "SetPayload(5)", "BrakeControl(1,1)", "DO(1,1)",
                                 "MovJ(pose={-500,100,200,150,0,90},a=10,v=10)", "MovJ(joint={1,2,3,4,5,6},a=10,v=80)",
                                 "MovJ(joint={1,2,3,4,5,6},a=10)", "SpeedFactor(80)", "AccJ(100)", "EmergencyStop(1)",
                                 "StartDrag()", "MovJ(joint={1,2,3,4,5,6},a=10,v=10);DO(1,1)"])
def test_whitelist_rejects(cmd):
    with pytest.raises(CommandRejected):
        check_command(cmd)


@pytest.mark.parametrize("cmd", ["RequestControl()", "EnableRobot(0.5)", "DisableRobot()", "SpeedFactor(20)",
                                 "AccJ(20)", "Stop()", "RobotMode()", "GetAngle()",
                                 "MovJ(joint={1,-2.5,3,4,5,6},a=20,v=20,cp=50)"])
def test_whitelist_accepts(cmd):
    check_command(cmd)


def test_movj_format_and_reply_parsing():
    assert movj_joint([1, 2.5, -3, 0, 90, -180], 20, 10, 50) == "MovJ(joint={1,2.5,-3,0,90,-180},a=10,v=20,cp=50)"
    r = parse_reply("0,{12},MovJ(joint={1,2,3,4,5,6},a=10,v=20,cp=50);")
    assert r.error_id == 0 and r.values == ["12"] and r.name == "MovJ"
    r = parse_reply("-2,{},EnableRobot();")
    assert r.error_id == -2 and r.values == []


# ------------------------------------------------------------------ planning
@pytest.fixture(scope="module")
def model(cr10_urdf):
    from ex_robot_state.core.urdf_model import UrdfModel
    return UrdfModel.from_string(cr10_urdf)


def _lp(model):
    def f(q_deg):
        P = model.all_link_poses({f"joint{i + 1}": np.radians(v) for i, v in enumerate(q_deg)})
        return {k: T[:3, 3] for k, T in P.items()}
    return f


def test_plan_shape():
    p = make_plan(GOOD_Q0, n_waypoints=60)
    W = np.array(p.waypoints_deg)
    assert W.shape == (61, 6) and np.allclose(W[-1], GOOD_Q0)
    assert np.all(np.max(np.abs(W - GOOD_Q0), axis=0) <= np.array([8, 10, 10, 15, 15, 15]) + 1e-9)
    # every joint actually moves (identifiability)
    assert np.all(W.max(0) - W.min(0) > 10.0)


def test_validation_passes_open_pose_and_fails_low_pose(model):
    lo, hi = [-360, -360, -160, -360, -360, -360], [360, 360, 160, 360, 360, 360]
    assert validate_plan(make_plan(GOOD_Q0), _lp(model), lo, hi, SafetyEnvelope()) == []
    # the old uniform +-15 deg plan is rejected by the envelope (J1/J2 swing the TCP > 0.35 m)
    assert any("tcp moved" in p for p in validate_plan(make_plan(GOOD_Q0, 15.0), _lp(model), lo, hi, SafetyEnvelope()))
    low = [0.0, 60.0, 80.0, 0.0, -90.0, 0.0]  # arm pitched down towards the table
    assert validate_plan(make_plan(low), _lp(model), lo, hi, SafetyEnvelope())
    near_limit = [0.0, -20.0, 150.0, -80.0, -90.0, 0.0]  # J3 near its 160 deg limit
    assert any("joint limit" in p for p in validate_plan(make_plan(near_limit), _lp(model), lo, hi, SafetyEnvelope()))


# ------------------------------------------------------------------ end to end (mock controller)
def _free(port):
    with socket.socket() as s:
        try:
            s.bind(("127.0.0.1", port))
            return True
        except OSError:
            return False


@pytest.fixture
def mock_robot(tmp_path):
    from ex_robot_state.tools.mock_dashboard import MockRobot
    from ex_robot_state.tools.mock_dobot_controller import MockController
    fport = next((p for p in (30006, 30005, 30004) if _free(p)), None)
    if fport is None:
        pytest.skip("no feedback port free")
    robot = MockRobot(GOOD_Q0, port=0)
    ctrl = MockController(fport, 8.0, None, "whole", 0, 0.0)
    ctrl.robot = robot
    threading.Thread(target=ctrl.serve_forever, daemon=True).start()
    yield robot, ctrl, fport
    ctrl.stop()
    robot.stop()


def _run(tmp_path, robot, fport, cr10_urdf, extra=()):
    from ex_dobot_motion.runner import main
    urdf = tmp_path / "cr10.urdf"
    urdf.write_text(cr10_urdf)
    out = tmp_path / "out"
    rc = main(["--robot-ip", "127.0.0.1", "--dashboard-port", str(robot.port), "--feedback-port", str(fport),
               "--urdf", str(urdf), "--waypoints", "20", "--speed", "30", "--speed-factor", "30",
               "--out", str(out), *extra])
    return rc, out


def test_dry_run_sends_nothing(tmp_path, mock_robot, cr10_urdf):
    robot, ctrl, fport = mock_robot
    rc, out = _run(tmp_path, robot, fport, cr10_urdf)
    assert rc == 0 and robot.commands == [] and (out / "plan.json").exists()


def test_execute_requires_confirmation(tmp_path, mock_robot, cr10_urdf):
    robot, ctrl, fport = mock_robot
    rc, _ = _run(tmp_path, robot, fport, cr10_urdf, ["--execute"])
    assert rc == 4 and robot.commands == []


def test_execute_end_to_end(tmp_path, mock_robot, cr10_urdf):
    robot, ctrl, fport = mock_robot
    rc, out = _run(tmp_path, robot, fport, cr10_urdf, ["--execute", "--confirm", "I_AM_AT_THE_ROBOT_WITH_ESTOP"])
    assert rc == 0
    assert robot.rejected == []
    for c in robot.commands:
        check_command(c)  # only whitelisted commands ever reached the controller
    assert robot.commands[0] == "RequestControl()" and robot.commands[-1] == "DisableRobot()"
    assert sum(c.startswith("MovJ") for c in robot.commands) == 21
    q, _, mode, _, _ = robot.state()
    assert np.allclose(q, GOOD_Q0, atol=1e-6) and mode == 4  # back at start, disabled
    s = json.loads((out / "summary.json").read_text())
    assert s["packets"] > 100 and (out / "feedback.csv.gz").exists()


def test_controller_error_aborts_and_disables(tmp_path, mock_robot, cr10_urdf):
    robot, ctrl, fport = mock_robot
    robot.fail_after_moves = 3
    rc, _ = _run(tmp_path, robot, fport, cr10_urdf, ["--execute", "--confirm", "I_AM_AT_THE_ROBOT_WITH_ESTOP"])
    assert rc == 6
    assert robot.commands[-2:] == ["Stop()", "DisableRobot()"]
    assert sum(c.startswith("MovJ") for c in robot.commands) <= 3 + 1 + 6  # stopped streaming promptly
