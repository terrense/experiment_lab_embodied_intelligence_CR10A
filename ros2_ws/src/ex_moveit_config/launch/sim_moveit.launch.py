"""MoveIt 2 + ros2_control for the CR10 model. SIMULATION ONLY.

  ros2 launch ex_moveit_config sim_moveit.launch.py [plant:=mock|mujoco] [rviz:=true] [monitor:=true]
        [calibrated:=true] [mujoco_viewer:=false] [payload_kg:=0.0] [bandwidth_hz:=8.0]
        [dynamics:=placeholder|identified_run02]

Graph:
  plant=mock    ros2_control_node(mock_components) - ideal position following (M2)
  plant=mujoco  ros2_control_node(TopicBasedSystem) <-/mujoco/joint_*-> mujoco_sim (physics, M4)
  both:         arm_controller (JTC) <- FollowJointTrajectory <- move_group (OMPL, KDL, TOTG)
                joint_state_broadcaster -> /joint_states -> robot_state_publisher -> /tf

Do NOT run together with ex_bringup (M1): both would publish /joint_states.
Use scripts/m2_session.sh to start/stop (guarantees a single move_group).
Nothing here talks to a Dobot controller.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def _setup(ctx, *args, **kwargs):
    lc = lambda n: LaunchConfiguration(n).perform(ctx)  # noqa: E731
    plant = lc("plant")
    if plant not in ("mock", "mujoco"):
        raise RuntimeError(f"plant must be mock|mujoco, got {plant!r}")
    share = get_package_share_directory("ex_moveit_config")
    mc = (
        MoveItConfigsBuilder("ex_cr10", package_name="ex_moveit_config")
        .robot_description(file_path="urdf/ex_cr10_moveit.urdf.xacro",
                           mappings={"calibrated": lc("calibrated"), "plant": plant})
        .robot_description_semantic(file_path="config/ex_cr10.srdf")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .joint_limits(file_path="config/joint_limits.yaml")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_scene_monitor(publish_robot_description=True, publish_robot_description_semantic=True)
        .planning_pipelines(pipelines=["ompl"])
        .to_moveit_configs()
    )
    actions = [
        Node(package="robot_state_publisher", executable="robot_state_publisher", output="log",
             parameters=[mc.robot_description, {"publish_frequency": 50.0}]),
        Node(package="controller_manager", executable="ros2_control_node", output="log",
             parameters=[mc.robot_description, os.path.join(share, "config", "ros2_controllers.yaml")]),
        *[Node(package="controller_manager", executable="spawner", output="log",
               arguments=[name, "--controller-manager", "/controller_manager"])
          for name in ("joint_state_broadcaster", "arm_controller")],
        Node(package="moveit_ros_move_group", executable="move_group", output="screen", parameters=[mc.to_dict()]),
    ]
    if plant == "mujoco":
        actions.append(Node(package="ex_mujoco_sim", executable="mujoco_sim", output="screen",
                            parameters=[mc.robot_description,
                                        {"viewer": lc("mujoco_viewer") == "true",
                                         "payload_kg": float(lc("payload_kg")),
                                         "payload_configured_kg": float(lc("payload_configured_kg")),
                                         "bandwidth_hz": float(lc("bandwidth_hz")),
                                         "dynamics": lc("dynamics")}]))
    if lc("rviz") == "true":
        actions.append(Node(package="rviz2", executable="rviz2", output="log",
                            arguments=["-d", os.path.join(share, "rviz", "moveit.rviz")],
                            parameters=[mc.robot_description, mc.robot_description_semantic,
                                        mc.robot_description_kinematics, mc.planning_pipelines, mc.joint_limits]))
    if lc("monitor") == "true":
        actions.append(Node(package="ex_robot_state", executable="robot_state_monitor", output="log",
                            parameters=[mc.robot_description, {"mode": f"MOVEIT_SIM({plant})"}]))
    return actions


def generate_launch_description() -> LaunchDescription:
    args = [("plant", "mock"), ("rviz", "true"), ("monitor", "true"), ("calibrated", "true"),
            ("mujoco_viewer", "false"), ("payload_kg", "0.0"), ("payload_configured_kg", "-1.0"), ("bandwidth_hz", "8.0"),
            ("dynamics", "placeholder")]
    return LaunchDescription([DeclareLaunchArgument(n, default_value=v) for n, v in args]
                             + [OpaqueFunction(function=_setup)])
