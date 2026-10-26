"""MoveIt 2 + ros2_control (mock hardware) for the CR10 model. SIMULATION ONLY.

  ros2 launch ex_moveit_config sim_moveit.launch.py [rviz:=true] [monitor:=true]

Graph:
  ros2_control_node (mock_components) --/joint_states--> robot_state_publisher --> /tf
        ^ arm_controller (JointTrajectoryController, FollowJointTrajectory action)
        |
  move_group (OMPL, KDL IK, TOTG) <-- RViz MotionPlanning / ex_planning_lab scripts

Do NOT run together with ex_bringup (M1): both would publish /joint_states.
Nothing here talks to a Dobot controller.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description() -> LaunchDescription:
    share = get_package_share_directory("ex_moveit_config")
    mc = (
        MoveItConfigsBuilder("ex_cr10", package_name="ex_moveit_config")
        .robot_description(file_path="urdf/ex_cr10_moveit.urdf.xacro")
        .robot_description_semantic(file_path="config/ex_cr10.srdf")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .joint_limits(file_path="config/joint_limits.yaml")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_scene_monitor(publish_robot_description=True, publish_robot_description_semantic=True)
        .planning_pipelines(pipelines=["ompl"])
        .to_moveit_configs()
    )

    rsp = Node(package="robot_state_publisher", executable="robot_state_publisher", output="log",
               parameters=[mc.robot_description, {"publish_frequency": 50.0}])
    control = Node(package="controller_manager", executable="ros2_control_node", output="log",
                   parameters=[mc.robot_description, os.path.join(share, "config", "ros2_controllers.yaml")])
    spawners = [Node(package="controller_manager", executable="spawner", output="log",
                     arguments=[name, "--controller-manager", "/controller_manager"])
                for name in ("joint_state_broadcaster", "arm_controller")]
    move_group = Node(package="moveit_ros_move_group", executable="move_group", output="screen",
                      parameters=[mc.to_dict()])
    rviz = Node(package="rviz2", executable="rviz2", output="log", condition=IfCondition(LaunchConfiguration("rviz")),
                arguments=["-d", os.path.join(share, "rviz", "moveit.rviz")],
                parameters=[mc.robot_description, mc.robot_description_semantic, mc.robot_description_kinematics,
                            mc.planning_pipelines, mc.joint_limits])
    monitor = Node(package="ex_robot_state", executable="robot_state_monitor", output="log",
                   condition=IfCondition(LaunchConfiguration("monitor")),
                   parameters=[mc.robot_description, {"mode": "MOVEIT_SIM"}])

    return LaunchDescription([
        DeclareLaunchArgument("rviz", default_value="true"),
        DeclareLaunchArgument("monitor", default_value="true", description="EMBODIED-X HUD + TCP trail"),
        rsp, control, *spawners, move_group, rviz, monitor,
    ])
