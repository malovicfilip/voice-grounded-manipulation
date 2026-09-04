"""Run one independently bounded, validator-gated pick-and-place task."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    arguments = [
        DeclareLaunchArgument("request_id"),
        DeclareLaunchArgument("object_id"),
        DeclareLaunchArgument("target_id"),
        DeclareLaunchArgument("object_x"),
        DeclareLaunchArgument("object_y"),
        DeclareLaunchArgument("object_z"),
    ]
    isaac_moveit_share = get_package_share_directory("isaac_moveit")
    moveit_config = (
        MoveItConfigsBuilder("moveit_resources_panda")
        .robot_description(
            file_path=os.path.join(
                isaac_moveit_share, "config", "panda_isaac.urdf.xacro"
            ),
            mappings={"ros2_control_hardware_type": "isaac"},
        )
        .robot_description_semantic(file_path="config/panda.srdf")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .to_moveit_configs()
    )
    parameters = {
        name: ParameterValue(LaunchConfiguration(name), value_type=str)
        for name in ("request_id", "object_id", "target_id")
    }
    parameters.update(
        {
            name: ParameterValue(LaunchConfiguration(name), value_type=float)
            for name in ("object_x", "object_y", "object_z")
        }
    )
    return LaunchDescription(
        arguments
        + [
            Node(
                package="vgm_moveit_demo",
                executable="safe_pick_and_place",
                output="screen",
                parameters=[
                    moveit_config.robot_description,
                    moveit_config.robot_description_semantic,
                    moveit_config.robot_description_kinematics,
                    parameters,
                ],
            )
        ]
    )
