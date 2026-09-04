"""Offline tests for the Phase 1 Isaac Sim scene builder."""

import json
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path


REPOSITORY_ROOT = Path(__file__).resolve().parents[1]
CONFIG_PATH = REPOSITORY_ROOT / 'config' / 'phase_1_scene.json'
BUILDER_PATH = (
    REPOSITORY_ROOT / 'isaac_sim' / 'scripts' / 'build_phase_1_scene.py'
)


class Phase1SceneBuilderTest(unittest.TestCase):
    """Exercise builder behavior that does not need Isaac Sim or a GPU."""

    def test_validate_only_reports_expected_scene(self):
        """Validate the real contract without importing Isaac Sim."""
        completed = subprocess.run(
            [sys.executable, str(BUILDER_PATH), '--validate-only'],
            check=True,
            capture_output=True,
            text=True,
        )

        summary = json.loads(completed.stdout)
        self.assertEqual(summary['status'], 'valid')
        self.assertEqual(summary['scene_id'], 'phase_1_franka_rgbd')
        self.assertEqual(summary['cube_count'], 6)
        self.assertEqual(summary['camera_streams'], ['rgb', 'depth'])

    def test_validate_only_rejects_direct_control_fields(self):
        """Reject low-level control data before Isaac Sim starts."""
        with CONFIG_PATH.open(encoding='utf-8') as config_file:
            unsafe_config = json.load(config_file)
        unsafe_config['robot']['joint_targets'] = [0.0] * 7

        with tempfile.TemporaryDirectory() as temporary_directory:
            config_path = Path(temporary_directory) / 'unsafe.json'
            with config_path.open('w', encoding='utf-8') as config_file:
                json.dump(unsafe_config, config_file)
            completed = subprocess.run(
                [
                    sys.executable,
                    str(BUILDER_PATH),
                    '--config',
                    str(config_path),
                    '--validate-only',
                ],
                check=False,
                capture_output=True,
                text=True,
            )

        self.assertNotEqual(completed.returncode, 0)
        self.assertIn('direct-control fields are forbidden', completed.stderr)

    def test_builder_contains_no_robot_control_calls(self):
        """Keep the scene builder outside the motion-control boundary."""
        source = BUILDER_PATH.read_text(encoding='utf-8')
        forbidden_call_fragments = (
            'ArticulationController',
            '.apply_action(',
            '.set_joint_positions(',
            '.set_joint_velocities(',
            '.set_joint_efforts(',
            '.publish(',
        )

        for fragment in forbidden_call_fragments:
            self.assertNotIn(fragment, source)

    def test_builder_preserves_errors_during_isaac_shutdown(self):
        """Keep Python failures visible with a nonzero process status."""
        source = BUILDER_PATH.read_text(encoding='utf-8')
        self.assertIn('traceback.print_exc()', source)
        self.assertIn('close(skip_cleanup=True, exit_code=exit_code)', source)

    def test_rtx_camera_authors_required_sensor_schema(self):
        """Let RtxCamera create the prim and apply OmniSensorAPI."""
        source = BUILDER_PATH.read_text(encoding='utf-8')
        camera_source = source.split('def _create_camera', maxsplit=1)[1]
        camera_source = camera_source.split('def _build_scene', maxsplit=1)[0]
        self.assertLess(
            camera_source.index('rtx_camera = RtxCamera('),
            camera_source.index('camera = UsdGeom.Camera('),
        )


if __name__ == '__main__':
    unittest.main()
