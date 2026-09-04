"""Offline safety and reproducibility tests for the MoveIt demo."""

import os
import subprocess
import tomllib
import unittest
from pathlib import Path


REPOSITORY_ROOT = Path(__file__).resolve().parents[1]
ROS_WORKSPACE = REPOSITORY_ROOT / "ros2_ws"
PACKAGE_ROOT = ROS_WORKSPACE / "src" / "vgm_moveit_demo"
SKILL_SOURCE = PACKAGE_ROOT / "src" / "safe_named_pose.cpp"
DEMO_LAUNCHER = REPOSITORY_ROOT / "isaac_sim" / "scripts" / "run_moveit_demo.sh"
ISAAC_SCENE = REPOSITORY_ROOT / "isaac_sim" / "scripts" / "run_moveit_scene.py"
INTEGRATED_SCENE = (
    REPOSITORY_ROOT / "isaac_sim" / "scripts" / "run_integrated_scene.py"
)
PICK_PLACE_SOURCE = PACKAGE_ROOT / "src" / "safe_pick_and_place.cpp"
VOICE_DEMO = (
    REPOSITORY_ROOT / "isaac_sim" / "scripts" / "run_voice_manipulation_demo.sh"
)


class MoveItDemoContractTest(unittest.TestCase):
    """Verify the demo remains behind the intended high-level boundary."""

    def test_unapproved_target_is_rejected_before_setup(self):
        completed = subprocess.run(
            ["bash", str(DEMO_LAUNCHER), "unsafe_pose"],
            check=False,
            capture_output=True,
            text=True,
            env={**os.environ, "ACCEPT_EULA": "Y"},
        )

        self.assertEqual(completed.returncode, 2)
        self.assertIn("Rejected target 'unsafe_pose'", completed.stderr)
        self.assertNotIn("Missing required setup path", completed.stderr)

    def test_skill_node_delegates_to_moveit_without_publishers(self):
        source = SKILL_SOURCE.read_text(encoding="utf-8")
        self.assertIn("MoveGroupInterface", source)
        self.assertIn('move_group.plan(plan)', source)
        self.assertIn('move_group.execute(plan)', source)
        self.assertIn('{ "ready", "extended", "transport" }', source)
        self.assertNotIn("create_publisher", source)
        self.assertNotIn("sensor_msgs", source)
        self.assertNotIn("trajectory_msgs", source)

    def test_demo_caps_motion_and_pins_simulator(self):
        skill_source = SKILL_SOURCE.read_text(encoding="utf-8")
        launcher_source = DEMO_LAUNCHER.read_text(encoding="utf-8")
        self.assertIn("kVelocityScale = 0.20", skill_source)
        self.assertIn("kAccelerationScale = 0.20", skill_source)
        self.assertIn("nvcr.io/nvidia/isaac-sim:6.0.1", launcher_source)
        self.assertNotIn(" --publish ", launcher_source)
        self.assertNotIn(" -p ", launcher_source)
        self.assertIn(
            '[[ -s "${HOST_OUTPUT}/isaac_capture.json" ]]', launcher_source
        )

    def test_simulator_only_applies_validated_ros_commands(self):
        source = ISAAC_SCENE.read_text(encoding="utf-8")
        self.assertIn('"isaac_joint_commands"', source)
        self.assertIn('"isaac_joint_states"', source)
        self.assertNotIn("set_joint_positions", source)
        self.assertNotIn("set_joint_velocities", source)
        self.assertNotIn("set_joint_efforts", source)

    def test_pixi_environment_is_jazzy_without_duplicate_isaac(self):
        manifest = tomllib.loads(
            (ROS_WORKSPACE / "pixi.toml").read_text(encoding="utf-8")
        )
        dependencies = manifest["dependencies"]
        self.assertIn("ros-jazzy-moveit", dependencies)
        self.assertIn("ros-jazzy-rmw-fastrtps-cpp", dependencies)
        self.assertNotIn("isaacsim", dependencies)
        self.assertNotIn("torch", dependencies)
        self.assertEqual(
            manifest["activation"]["env"]["RMW_IMPLEMENTATION"],
            "rmw_fastrtps_cpp",
        )
        self.assertIn("faster-whisper", manifest["pypi-dependencies"])

    def test_pick_and_place_executor_is_allowlisted_and_moveit_only(self):
        source = PICK_PLACE_SOURCE.read_text(encoding="utf-8")
        self.assertIn("MoveGroupInterface", source)
        self.assertIn("PlanningSceneInterface", source)
        self.assertIn("kVelocityScale = 0.20", source)
        self.assertIn("kAccelerationScale = 0.20", source)
        self.assertIn("kExecutionDeadlineSeconds = 120.0", source)
        self.assertIn("kAttachedObjectClearance = 0.005", source)
        self.assertIn('"blue_target"', source)
        self.assertIn('"yellow_target"', source)
        self.assertNotIn("create_publisher", source)
        self.assertNotIn("trajectory_msgs", source)
        self.assertNotIn("joint_trajectory", source)
        self.assertIn("timeout 130", VOICE_DEMO.read_text(encoding="utf-8"))
        self.assertLess(
            source.index("arm_.detachObject(object_id)"),
            source.index("CollisionObject::REMOVE"),
        )
        self.assertLess(
            source.index("CollisionObject::REMOVE"),
            source.index('"place_retreat"'),
        )

    def test_integrated_demo_revalidates_rgbd_before_moveit(self):
        scene_source = INTEGRATED_SCENE.read_text(encoding="utf-8")
        launcher_source = VOICE_DEMO.read_text(encoding="utf-8")
        self.assertIn('get_data("rgb")', scene_source)
        self.assertIn('get_data("distance_to_image_plane")', scene_source)
        self.assertIn("capture_verification.request", scene_source)
        self.assertIn("execution_gate", launcher_source)
        self.assertLess(
            launcher_source.index("execution_gate"),
            launcher_source.index("safe_pick_and_place.launch.py"),
        )
        self.assertIn("nvcr.io/nvidia/isaac-sim:6.0.1", launcher_source)
        self.assertNotIn(" --publish ", launcher_source)
        self.assertNotIn(" -p ", launcher_source)

    def test_controller_activation_waits_for_joint_state_broadcaster(self):
        source = (
            PACKAGE_ROOT / "launch" / "moveit_headless.launch.py"
        ).read_text(encoding="utf-8")
        self.assertIn("OnProcessExit", source)
        self.assertIn("target_action=joint_state_spawner", source)
        self.assertIn("on_exit=[arm_spawner]", source)


if __name__ == "__main__":
    unittest.main()
