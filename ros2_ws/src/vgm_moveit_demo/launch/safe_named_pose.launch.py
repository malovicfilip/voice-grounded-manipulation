"""Run one allowlisted Panda named-pose skill with the required MoveIt model."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder
from vgm_runtime.config import load_json_config, policy_digest


def generate_launch_description():
    target = DeclareLaunchArgument(
        "target",
        default_value="ready",
        description="Allowlisted high-level Panda pose name",
    )
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

    return LaunchDescription(
        [
            target,
            Node(
                package="vgm_moveit_demo",
                executable="safe_named_pose",
                output="screen",
                parameters=[
                    moveit_config.robot_description,
                    moveit_config.robot_description_semantic,
                    moveit_config.robot_description_kinematics,
                    {"target": LaunchConfiguration("target")},
                    {"safety_policy_digest": policy_digest(load_json_config("safety_policy.json"))},
                ],
            ),
        ]
    )
