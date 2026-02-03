# Copyright (c) 2025
# Dual UR MoveIt launch - two move_group nodes (robot1, robot2) + RViz with MotionPlanning

import os
import subprocess
import yaml

from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from ament_index_python.packages import get_package_share_directory


def load_yaml(package_name, file_path):
    pkg_path = get_package_share_directory(package_name)
    with open(os.path.join(pkg_path, file_path)) as f:
        return yaml.safe_load(f)


def load_srdf_xacro(package_name, srdf_path, mappings=None):
    """Run xacro on SRDF and return XML string."""
    pkg_path = get_package_share_directory(package_name)
    srdf_abs = os.path.join(pkg_path, srdf_path)
    cmd = ["xacro", srdf_abs]
    if mappings:
        for k, v in mappings.items():
            cmd.extend([f"{k}:={v}"])
    result = subprocess.run(cmd, capture_output=True, text=True)
    if result.returncode != 0:
        raise RuntimeError(f"xacro failed: {result.stderr}")
    return result.stdout


def launch_setup(context, *args, **kwargs):
    ur_type = LaunchConfiguration("ur_type")
    description_file = LaunchConfiguration("description_file")
    safety_limits = LaunchConfiguration("safety_limits", default="true")
    safety_pos_margin = LaunchConfiguration("safety_pos_margin", default="0.15")
    safety_k_position = LaunchConfiguration("safety_k_position", default="20")
    launch_rviz = LaunchConfiguration("launch_rviz", default="true")
    use_sim_time = LaunchConfiguration("use_sim_time", default="true")
    warehouse_sqlite_path = LaunchConfiguration(
        "warehouse_sqlite_path", default=os.path.expanduser("~/.ros/warehouse_ros.sqlite")
    )

    pkg_dual = get_package_share_directory("ur_dual_moveit_config")
    pkg_sim = get_package_share_directory("ur_simulation_gz")
    controllers_robot1 = os.path.join(pkg_sim, "config", "ur_controllers_robot1.yaml")
    controllers_robot2 = os.path.join(pkg_sim, "config", "ur_controllers_robot2.yaml")

    ur_type_val = ur_type.perform(context)
    desc_file_val = description_file.perform(context)

    # Robot descriptions (same as ur_sim_two_robots)
    robot1_desc_cmd = [
        "xacro", desc_file_val,
        "safety_limits:=" + safety_limits.perform(context),
        "safety_pos_margin:=" + safety_pos_margin.perform(context),
        "safety_k_position:=" + safety_k_position.perform(context),
        "name:=ur1", "ur_type:=" + ur_type_val,
        "tf_prefix:=robot1_", "simulation_controllers:=" + controllers_robot1,
        "ros_namespace:=robot1", "origin_xyz:=0 0 0"
    ]
    robot2_desc_cmd = [
        "xacro", desc_file_val,
        "safety_limits:=" + safety_limits.perform(context),
        "safety_pos_margin:=" + safety_pos_margin.perform(context),
        "safety_k_position:=" + safety_k_position.perform(context),
        "name:=ur2", "ur_type:=" + ur_type_val,
        "tf_prefix:=robot2_", "simulation_controllers:=" + controllers_robot2,
        "ros_namespace:=robot2", "origin_xyz:=1.5 0 0"
    ]

    robot1_desc_result = subprocess.run(robot1_desc_cmd, capture_output=True, text=True)
    robot2_desc_result = subprocess.run(robot2_desc_cmd, capture_output=True, text=True)
    if robot1_desc_result.returncode != 0 or robot2_desc_result.returncode != 0:
        raise RuntimeError(f"xacro failed: {robot1_desc_result.stderr or robot2_desc_result.stderr}")

    robot1_description = robot1_desc_result.stdout
    robot2_description = robot2_desc_result.stdout

    # SRDF
    srdf_robot1 = load_srdf_xacro("ur_dual_moveit_config", "srdf/ur_robot1.srdf.xacro")
    srdf_robot2 = load_srdf_xacro("ur_dual_moveit_config", "srdf/ur_robot2.srdf.xacro")

    # Configs
    joint_limits_1 = load_yaml("ur_dual_moveit_config", "config/robot1/joint_limits.yaml")
    kinematics_1 = load_yaml("ur_dual_moveit_config", "config/robot1/kinematics.yaml")
    controllers_1 = load_yaml("ur_dual_moveit_config", "config/robot1/moveit_controllers.yaml")
    ompl_1 = load_yaml("ur_dual_moveit_config", "config/robot1/ompl_planning.yaml")
    pilz_cartesian_1 = load_yaml("ur_dual_moveit_config", "config/robot1/pilz_cartesian_limits.yaml")
    pilz_planning_1 = load_yaml("ur_dual_moveit_config", "config/robot1/pilz_industrial_motion_planner_planning.yaml")

    joint_limits_2 = load_yaml("ur_dual_moveit_config", "config/robot2/joint_limits.yaml")
    kinematics_2 = load_yaml("ur_dual_moveit_config", "config/robot2/kinematics.yaml")
    controllers_2 = load_yaml("ur_dual_moveit_config", "config/robot2/moveit_controllers.yaml")
    ompl_2 = load_yaml("ur_dual_moveit_config", "config/robot2/ompl_planning.yaml")
    pilz_cartesian_2 = load_yaml("ur_dual_moveit_config", "config/robot2/pilz_cartesian_limits.yaml")
    pilz_planning_2 = load_yaml("ur_dual_moveit_config", "config/robot2/pilz_industrial_motion_planner_planning.yaml")

    warehouse_config = {
        "warehouse_plugin": "warehouse_ros_sqlite::DatabaseConnection",
        "warehouse_host": warehouse_sqlite_path.perform(context),
    }

    def make_moveit_params(robot_desc, srdf, joint_limits, kinematics, ompl, pilz_cart, pilz_plan, controllers):
        return {
            "robot_description": robot_desc,
            "robot_description_semantic": srdf,
            "robot_description_kinematics": kinematics,
            "planning_pipelines": ["ompl", "pilz_industrial_motion_planner"],
            "default_planning_pipeline": "ompl",
            "ompl": ompl,
            "pilz_industrial_motion_planner": pilz_plan,
            "robot_description_planning": {
                "joint_limits": joint_limits["joint_limits"],
                "cartesian_limits": pilz_cart["cartesian_limits"],
            },
            "moveit_manage_controllers": True,
            "moveit_controller_manager": "moveit_simple_controller_manager/MoveItSimpleControllerManager",
            "moveit_simple_controller_manager": controllers["moveit_simple_controller_manager"],
            "trajectory_execution": controllers.get("trajectory_execution", {}),
            "publish_planning_scene": True,
            "publish_geometry_updates": True,
            "publish_state_updates": True,
            "publish_transforms_updates": True,
            "publish_robot_description": True,
            "publish_robot_description_semantic": True,
            "publish_robot_description_kinematics": True,
            "use_sim_time": use_sim_time.perform(context).lower() == "true",
        }

    params_robot1 = make_moveit_params(
        robot1_description, srdf_robot1, joint_limits_1, kinematics_1,
        ompl_1, pilz_cartesian_1, pilz_planning_1, controllers_1
    )
    params_robot2 = make_moveit_params(
        robot2_description, srdf_robot2, joint_limits_2, kinematics_2,
        ompl_2, pilz_cartesian_2, pilz_planning_2, controllers_2
    )

    move_group_robot1 = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        namespace="robot1",
        name="move_group",
        output="screen",
        parameters=[params_robot1, warehouse_config],
    )

    move_group_robot2 = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        namespace="robot2",
        name="move_group",
        output="screen",
        parameters=[params_robot2, warehouse_config],
    )

    # Single RViz with both robots (moveit_dual.rviz has two MotionPlanning displays: robot1, robot2).
    # Requires moveit_ros_planning with node-name sanitization fix for parameter paths like
    # /robot1/move_group.robot_description (see rdf_loader synchronized_string_parameter.cpp).
    rviz_config_dual = os.path.join(pkg_dual, "config", "moveit_dual.rviz")
    use_sim = use_sim_time.perform(context).lower() == "true"

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2_moveit",
        output="log",
        arguments=["-d", rviz_config_dual],
        parameters=[{"use_sim_time": use_sim}],
        condition=IfCondition(launch_rviz),
    )

    return [move_group_robot1, move_group_robot2, rviz_node]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("ur_type", default_value="ur5e"),
        DeclareLaunchArgument(
            "description_file",
            default_value=PathJoinSubstitution([
                FindPackageShare("ur_simulation_gz"), "urdf", "ur_gz.urdf.xacro"
            ]),
        ),
        DeclareLaunchArgument("safety_limits", default_value="true"),
        DeclareLaunchArgument("safety_pos_margin", default_value="0.15"),
        DeclareLaunchArgument("safety_k_position", default_value="20"),
        DeclareLaunchArgument("launch_rviz", default_value="true"),
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument(
            "warehouse_sqlite_path",
            default_value=os.path.expanduser("~/.ros/warehouse_ros.sqlite"),
        ),
        OpaqueFunction(function=launch_setup),
    ])
