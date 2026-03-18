# Copyright (c) 2024 FZI Forschungszentrum Informatik
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#
#    * Neither the name of the {copyright_holder} nor the names of its
#      contributors may be used to endorse or promote products derived from
#      this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

#
# Author: Felix Exner
import os
import yaml

from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
)

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from moveit_configs_utils import MoveItConfigsBuilder

from ament_index_python.packages import get_package_share_directory


def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path) as file:
            return yaml.safe_load(file)
    except OSError:  # parent of IOError, OSError *and* WindowsError where available
        return None


def launch_setup(context, *args, **kwargs):
    launch_rviz = LaunchConfiguration("launch_rviz")
    ur_type = LaunchConfiguration("ur_type")
    warehouse_sqlite_path = LaunchConfiguration("warehouse_sqlite_path")
    launch_servo = LaunchConfiguration("launch_servo")
    use_sim_time = LaunchConfiguration("use_sim_time")
    publish_robot_description_semantic = LaunchConfiguration("publish_robot_description_semantic")
    robot_namespace = LaunchConfiguration("robot_namespace")
    robot_description_topic = LaunchConfiguration("robot_description_topic")
    robot_model_name = LaunchConfiguration("robot_model_name")
    rviz_config_file = LaunchConfiguration("rviz_config_file")
    launch_rviz_value = launch_rviz.perform(context)
    ur_type_value = ur_type.perform(context)
    launch_servo_value = launch_servo.perform(context)
    use_sim_time_value = use_sim_time.perform(context).lower() == "true"
    publish_robot_description_semantic_value = (
        publish_robot_description_semantic.perform(context).lower() == "true"
    )
    robot_description_topic_value = robot_description_topic.perform(context)
    robot_model_name_value = robot_model_name.perform(context)
    rviz_config_file_value = rviz_config_file.perform(context)
    robot_namespace_value = robot_namespace.perform(context).strip("/")

    if robot_namespace_value:
        namespaced_warehouse_path = os.path.expanduser(
            f"~/.ros/warehouse_{robot_namespace_value}.sqlite"
        )
    else:
        namespaced_warehouse_path = warehouse_sqlite_path

    moveit_config = (
        MoveItConfigsBuilder(robot_name=robot_model_name_value, package_name="ur_moveit_config")
        .robot_description_semantic(
            Path("srdf") / "ur.srdf.xacro",
            {"name": robot_model_name_value},
        )
        .to_moveit_configs()
    )

    warehouse_ros_config = {
        "warehouse_plugin": "warehouse_ros_sqlite::DatabaseConnection",
        "warehouse_host": namespaced_warehouse_path,
    }

    common_remappings = [
        ("/joint_states", "joint_states"),
        ("/monitored_planning_scene", "monitored_planning_scene"),
        ("/display_planned_path", "display_planned_path"),
        ("/tf", "tf"),
        ("/tf_static", "tf_static"),
        ("/robot_description", robot_description_topic_value),
    ]

    wait_robot_description = Node(
        package="ur_robot_driver",
        executable="wait_for_robot_description",
        output="screen",
        namespace=robot_namespace_value,
        remappings=common_remappings,
    )

    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        namespace=robot_namespace_value,
        remappings=common_remappings,
        parameters=[
            moveit_config.to_dict(),
            warehouse_ros_config,
            {
                "use_sim_time": use_sim_time_value,
                "publish_robot_description_semantic": publish_robot_description_semantic_value,
            },
        ],
    )

    servo_yaml = load_yaml("ur_moveit_config", "config/ur_servo.yaml")
    servo_params = {"moveit_servo": servo_yaml}
    servo_node = Node(
        package="moveit_servo",
        condition=IfCondition(launch_servo_value),
        executable="servo_node",
        namespace=robot_namespace_value,
        remappings=common_remappings,
        parameters=[
            moveit_config.to_dict(),
            servo_params,
        ],
        output="screen",
    )
    rviz_node = Node(
        package="rviz2",
        condition=IfCondition(launch_rviz_value),
        executable="rviz2",
        name="rviz2_moveit",
        output="log",
        namespace=robot_namespace_value,
        arguments=["-d", rviz_config_file_value],
        remappings=common_remappings,
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
            warehouse_ros_config,
            {
                "use_sim_time": use_sim_time_value,
            },
        ],
    )

    return [
        wait_robot_description,
        RegisterEventHandler(
            OnProcessExit(
                target_action=wait_robot_description,
                on_exit=[move_group_node, rviz_node, servo_node],
            )
        ),
    ]


def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument("launch_rviz", default_value="true", description="Launch RViz?"),
        DeclareLaunchArgument(
            "ur_type",
            description="Typo/series of used UR robot.",
            choices=[
                "ur3",
                "ur5",
                "ur10",
                "ur3e",
                "ur5e",
                "ur7e",
                "ur10e",
                "ur12e",
                "ur16e",
                "ur8long",
                "ur15",
                "ur18",
                "ur20",
                "ur30",
            ],
        ),
        DeclareLaunchArgument(
            "warehouse_sqlite_path",
            default_value=os.path.expanduser("~/.ros/warehouse_ros.sqlite"),
            description="Path where the warehouse database should be stored",
        ),
        DeclareLaunchArgument("launch_servo", default_value="false", description="Launch Servo?"),
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            description="Using or not time from simulation",
        ),
        DeclareLaunchArgument(
            "publish_robot_description_semantic",
            default_value="true",
            description="MoveGroup publishes robot description semantic",
        ),
        DeclareLaunchArgument(
            "robot_namespace",
            default_value="",
            description="Namespace for MoveIt stack instance (e.g. ur1).",
        ),
        DeclareLaunchArgument(
            "robot_description_topic",
            default_value="robot_description",
            description="Robot description topic name to wait for / remap.",
        ),
        DeclareLaunchArgument(
            "robot_model_name",
            default_value="ur",
            description="Robot model name used by MoveIt URDF/SRDF.",
        ),
        DeclareLaunchArgument(
            "rviz_config_file",
            default_value=PathJoinSubstitution(
                [FindPackageShare("ur_moveit_config"), "config", "moveit.rviz"]
            ),
            description="Rviz config file (absolute path) to use when launching rviz.",
        ),
    ]

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])
