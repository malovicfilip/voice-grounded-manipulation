"""Offline tests for the headless Phase 1 RGB-D demo."""

import json
import subprocess
import sys
import unittest
from pathlib import Path


REPOSITORY_ROOT = Path(__file__).resolve().parents[1]
CAPTURE_PATH = (
    REPOSITORY_ROOT
    / 'isaac_sim'
    / 'scripts'
    / 'capture_phase_1_demo.py'
)
LAUNCHER_PATH = (
    REPOSITORY_ROOT
    / 'isaac_sim'
    / 'scripts'
    / 'run_phase_1_demo.sh'
)


class Phase1DemoTest(unittest.TestCase):
    """Check demo behavior that does not require Isaac Sim or a GPU."""

    def test_validate_only_reports_rgbd_capture(self):
        """Preflight the real scene contract without importing Isaac Sim."""
        completed = subprocess.run(
            [sys.executable, str(CAPTURE_PATH), '--validate-only'],
            check=True,
            capture_output=True,
            text=True,
        )

        summary = json.loads(completed.stdout)
        self.assertEqual(summary['status'], 'ready')
        self.assertEqual(summary['capture_frames'], 1)
        self.assertEqual(summary['camera_streams'], ['rgb', 'depth'])

    def test_validate_only_rejects_unbounded_frame_count(self):
        """Keep the short demo from creating accidental bulk output."""
        completed = subprocess.run(
            [
                sys.executable,
                str(CAPTURE_PATH),
                '--frames',
                '11',
                '--validate-only',
            ],
            check=False,
            capture_output=True,
            text=True,
        )

        self.assertNotEqual(completed.returncode, 0)
        self.assertIn('frames must be between 1 and 10', completed.stderr)

    def test_capture_retains_rgb_and_metric_depth(self):
        """Require a viewable RGB image and raw metric-depth output."""
        source = CAPTURE_PATH.read_text(encoding='utf-8')
        self.assertIn("camera_sensor.get_data('rgb')", source)
        self.assertIn("'distance_to_image_plane'", source)
        self.assertIn("f'depth_preview_{capture_index:04d}.png'", source)
        self.assertIn('np.save(depth_path, depth)', source)
        self.assertIn('_author_scene(config)', source)
        self.assertNotIn('open_stage', source)
        self.assertNotIn('rep.create.render_product', source)

    def test_demo_contains_no_robot_control_calls(self):
        """Keep capture and launch code outside the control boundary."""
        source = CAPTURE_PATH.read_text(encoding='utf-8')
        source += LAUNCHER_PATH.read_text(encoding='utf-8')
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

    def test_launcher_requires_explicit_eula_acceptance(self):
        """Do not launch the NVIDIA image without an explicit opt-in."""
        source = LAUNCHER_PATH.read_text(encoding='utf-8')
        self.assertIn('${ACCEPT_EULA:-}', source)
        self.assertNotIn('PRIVACY_CONSENT', source)


if __name__ == '__main__':
    unittest.main()
