"""EMBODIED-X single entry point.

  ros2 launch ex_bringup embodied_x.launch.py                       # mode:=simulation
  ros2 launch ex_bringup embodied_x.launch.py mode:=replay bag:=data/bags/sim_demo
  ros2 launch ex_bringup embodied_x.launch.py mode:=live_observe \
      robot_ip:=192.168.5.1 live_ack:=I_AM_ON_SITE_AND_AUTHORIZED   # + config enable flag

Data path in every mode (only the source differs):
  <source> -> /joint_states_robot -> joint_state_adapter -> /joint_states
           -> robot_state_publisher -> /tf -> RViz (+ robot_state_monitor HUD)

Nothing launched here can command the robot: no MoveIt, no controllers, no
official dobot_bringup (which opens the 29999 command port).
"""

from __future__ import annotations

import os
import time
from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, LogInfo, OpaqueFunction, Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

MODES = ("simulation", "replay", "live_observe")


def _robot_description(ctx) -> str:
    import xacro

    lc = lambda n: LaunchConfiguration(n).perform(ctx)  # noqa: E731
    path = os.path.join(get_package_share_directory("ex_description"), "urdf", "ex_cr10.urdf.xacro")
    return xacro.process_file(path, mappings={
        "mount_xyz": lc("mount_xyz"), "mount_rpy": lc("mount_rpy"),
        "tool_xyz": lc("tool_xyz"), "tool_rpy": lc("tool_rpy")}).toxml()


def _setup(ctx, *args, **kwargs):
    lc = lambda n: LaunchConfiguration(n).perform(ctx)  # noqa: E731
    mode = lc("mode")
    if mode not in MODES:
        raise RuntimeError(f"mode must be one of {MODES}, got {mode!r}")
    share = get_package_share_directory("ex_bringup")
    mode_cfg = yaml.safe_load(Path(share, "config", "modes", f"{mode}.yaml").read_text())
    desc = _robot_description(ctx)
    common = {"robot_description": desc}
    log_dir = Path(os.path.expanduser(lc("log_dir")))
    log_path = str(log_dir / f"{mode}_{time.strftime('%Y%m%d_%H%M%S')}.jsonl")

    actions = [
        LogInfo(msg=f"[embodied_x] mode={mode.upper()}  health log: {log_path}"),
        Node(package="robot_state_publisher", executable="robot_state_publisher", output="log",
             parameters=[{"robot_description": desc, "publish_frequency": 50.0}]),
        Node(package="ex_robot_state", executable="joint_state_adapter", output="screen",
             parameters=[common, mode_cfg.get("adapter", {})]),
        Node(package="ex_robot_state", executable="robot_state_monitor", output="screen",
             parameters=[common, {"mode": mode.upper(), "log_path": log_path}, mode_cfg.get("monitor", {})]),
    ]

    if mode == "simulation":
        actions.append(Node(package="ex_robot_state", executable="mock_joint_publisher", output="screen",
                            parameters=[common, {"fault": lc("fault")}, mode_cfg.get("mock", {})]))
    elif mode == "replay":
        bag = lc("bag")
        if not bag:
            raise RuntimeError("mode:=replay requires bag:=<path to rosbag2 directory>")
        # Replay ONLY the raw source topic. Replaying /joint_states or /tf as well
        # would create a second publisher next to the live adapter / RSP.
        cmd = ["ros2", "bag", "play", bag, "--topics", "/joint_states_robot", "--rate", lc("rate")]
        if lc("loop") == "true":
            cmd.append("--loop")
        actions.append(ExecuteProcess(cmd=cmd, output="screen",
                                      on_exit=None if lc("loop") == "true" else [LogInfo(msg="[embodied_x] bag finished")]))
    elif mode == "live_observe":
        live = mode_cfg.get("observer", {})
        ack = lc("live_ack")
        if not live.get("enable_live_connection", False) or ack != "I_AM_ON_SITE_AND_AUTHORIZED":
            return [LogInfo(msg=(
                "[embodied_x] LIVE_OBSERVE REFUSED: set enable_live_connection: true in "
                "ex_bringup/config/modes/live_observe.yaml AND pass live_ack:=I_AM_ON_SITE_AND_AUTHORIZED. "
                "Read docs/LIVE_ROBOT_CHECKLIST.md first.")), Shutdown(reason="live gate closed")]
        actions.append(Node(package="ex_robot_state", executable="dobot_feedback_observer", output="screen",
                            parameters=[live, {"operator_ack": ack, "robot_ip": lc("robot_ip"),
                                               "feedback_port": int(lc("feedback_port"))}]))
        actions.append(Node(package="ex_robot_state", executable="kinematics_crosscheck", output="screen",
                            parameters=[common, {"csv_path": str(log_dir / f"crosscheck_{time.strftime('%Y%m%d_%H%M%S')}.csv"),
                                                 "summary_path": str(log_dir / "crosscheck_summary_latest.json")}]))

    if lc("rviz") == "true":
        actions.append(Node(package="rviz2", executable="rviz2", output="log",
                            arguments=["-d", os.path.join(share, "rviz", "embodied_x.rviz")]))
    return actions


def generate_launch_description() -> LaunchDescription:
    args = [
        ("mode", "simulation", "simulation | replay | live_observe"),
        ("rviz", "true", "start RViz"),
        ("fault", "none", "simulation fault injection: none|dropout|jump|degrees|nan|wrong_names"),
        ("bag", "", "replay: rosbag2 directory"),
        ("loop", "false", "replay: loop the bag"),
        ("rate", "1.0", "replay: playback rate"),
        ("robot_ip", "192.168.5.1", "live_observe: controller IP"),
        ("feedback_port", "30004", "live_observe: 30004 (8 ms) | 30005 (200 ms) | 30006"),
        ("live_ack", "", "live_observe: must equal I_AM_ON_SITE_AND_AUTHORIZED"),
        ("log_dir", "~/.ros/embodied_x", "health logs / crosscheck CSV"),
        ("mount_xyz", "0 0 0", "world->base_link translation"),
        ("mount_rpy", "0 0 0", "world->base_link rotation"),
        ("tool_xyz", "0 0 0", "flange->tcp translation"),
        ("tool_rpy", "0 0 0", "flange->tcp rotation"),
    ]
    return LaunchDescription([DeclareLaunchArgument(n, default_value=d, description=h) for n, d, h in args]
                             + [OpaqueFunction(function=_setup)])
