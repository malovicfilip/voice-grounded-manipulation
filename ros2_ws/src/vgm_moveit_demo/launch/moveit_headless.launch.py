"""Launch NVIDIA's Panda MoveIt stack without RViz for a headless GPU VM."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    hardware_type = DeclareLaunchArgument(
        "ros2_control_hardware_type",
        default_value="isaac",
        description="Supported values: isaac or mock_components",
    )
    use_sim_time = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
        description="Use Isaac Sim's /clock when true",
    )
    base_x = DeclareLaunchArgument("base_x", default_value="0.0")
    base_y = DeclareLaunchArgument("base_y", default_value="-0.64")
    base_z = DeclareLaunchArgument("base_z", default_value="0.0")
    base_yaw = DeclareLaunchArgument("base_yaw", default_value="0.0")

    isaac_moveit_share = get_package_share_directory("isaac_moveit")
    panda_config_share = get_package_share_directory(
        "moveit_resources_panda_moveit_config"
    )
    moveit_config = (
        MoveItConfigsBuilder("moveit_resources_panda")
        .robot_description(
            file_path=os.path.join(
                isaac_moveit_share, "config", "panda_isaac.urdf.xacro"
            ),
            mappings={
                "ros2_control_hardware_type": LaunchConfiguration(
                    "ros2_control_hardware_type"
                )
            },
        )
        .robot_description_semantic(file_path="config/panda.srdf")
        .trajectory_execution(file_path="config/gripper_moveit_controllers.yaml")
        .planning_pipelines(pipelines=["ompl", "pilz_industrial_motion_planner"])
        .to_moveit_configs()
    )

    common_time = {"use_sim_time": LaunchConfiguration("use_sim_time")}
    return LaunchDescription(
        [
            hardware_type,
            use_sim_time,
            base_x,
            base_y,
            base_z,
            base_yaw,
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="static_transform_publisher_world_to_robot",
                arguments=[
                    LaunchConfiguration("base_x"),
                    LaunchConfiguration("base_y"),
                    LaunchConfiguration("base_z"),
                    LaunchConfiguration("base_yaw"),
                    "0.0",
                    "0.0",
                    "world",
                    "panda_link0",
                ],
                parameters=[common_time],
            ),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                parameters=[moveit_config.robot_description, common_time],
                output="both",
            ),
            Node(
                package="moveit_ros_move_group",
                executable="move_group",
                parameters=[moveit_config.to_dict(), common_time],
                output="screen",
            ),
            Node(
                package="controller_manager",
                executable="ros2_control_node",
                parameters=[
                    os.path.join(panda_config_share, "config", "ros2_controllers.yaml"),
                    common_time,
                ],
                remappings=[
                    ("/controller_manager/robot_description", "/robot_description")
                ],
                output="screen",
            ),
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=["joint_state_broadcaster", "-c", "/controller_manager"],
            ),
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=["panda_arm_controller", "-c", "/controller_manager"],
            ),
            Node(
                package="isaac_moveit",
                executable="gripper_to_isaac",
                parameters=[common_time],
                output="screen",
            ),
        ]
    )
