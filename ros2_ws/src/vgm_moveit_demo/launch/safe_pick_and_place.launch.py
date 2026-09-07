"""Run one independently bounded, validator-gated pick-and-place task."""

import os
import json

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from moveit_configs_utils import MoveItConfigsBuilder
from vgm_runtime.moveit_parameters import execution_parameters


def generate_launch_description():
    arguments = [
        DeclareLaunchArgument("request_id"),
        DeclareLaunchArgument("object_id", default_value=""),
        DeclareLaunchArgument("target_id", default_value=""),
        DeclareLaunchArgument("object_x"),
        DeclareLaunchArgument("object_y"),
        DeclareLaunchArgument("object_z"),
        DeclareLaunchArgument("skill", default_value="pick_and_place"),
        DeclareLaunchArgument("pose_name", default_value=""),
        DeclareLaunchArgument("scene_file"),
        DeclareLaunchArgument("fault_before_primitive", default_value=""),
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
        for name in ("request_id", "object_id", "target_id", "skill", "pose_name", "fault_before_primitive")
    }
    parameters.update(
        {
            name: ParameterValue(LaunchConfiguration(name), value_type=float)
            for name in ("object_x", "object_y", "object_z")
        }
    )
    def launch_executor(context):
        observed_parameters = {}
        scene_file = LaunchConfiguration("scene_file").perform(context)
        if scene_file:
            with open(scene_file, encoding="utf-8") as stream:
                scene = json.load(stream)
            observed_parameters = execution_parameters(scene, LaunchConfiguration("target_id").perform(context))
        return [Node(
                package="vgm_moveit_demo",
                executable="safe_pick_and_place",
                output="screen",
                parameters=[
                    moveit_config.robot_description,
                    moveit_config.robot_description_semantic,
                    moveit_config.robot_description_kinematics,
                    parameters,
                    observed_parameters,
                    # Isaac publishes /clock and simulation-stamped joint states.
                    # Wall-clock deadlines remain independent in the executor.
                    {"use_sim_time": True},
                ],
            )]
    return LaunchDescription(arguments + [OpaqueFunction(function=launch_executor)])
