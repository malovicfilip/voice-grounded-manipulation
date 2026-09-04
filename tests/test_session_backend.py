"""Recovery must retain the same validation boundary as ordinary execution."""

from pathlib import Path
import sys
import tempfile
import time
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "ros2_ws/src/vgm_runtime"))
from vgm_runtime.session_backend import SimulatorSession
from vgm_runtime.types import GroundedScene, ObjectObservation


class SessionRecoveryTests(unittest.TestCase):
    def setUp(self):
        directory = tempfile.TemporaryDirectory()
        self.addCleanup(directory.cleanup)
        self.directory = Path(directory.name)
        (self.directory / "session.ready").touch()
        self.session = SimulatorSession(self.directory)

    def scene(self, held="red_cube"):
        now = time.time()
        return GroundedScene("0123456789abcdef", now, {
            "green_cube": ObjectObservation("green_cube", (.1, -.18, .775), .98, now, pixel_count=100),
        }, held)

    def test_recovery_only_delegates_allowlisted_place_with_measured_holding(self):
        scene = self.scene()
        with patch.object(self.session, "capture", return_value=scene), \
             patch.object(self.session, "execute", return_value={"status": "succeeded"}) as execute:
            self.session.recover_place("blue_target")
        proposal, baseline = execute.call_args.args
        self.assertEqual(proposal["skill"], "place")
        self.assertEqual(proposal["object_id"], scene.held_object_id)
        self.assertEqual(proposal["target_id"], "blue_target")
        self.assertEqual(baseline, {"scene": scene.to_mapping()})
        self.assertEqual(execute.call_args.kwargs, {"recovery": True})

    def test_recovery_refuses_unknown_target_or_empty_gripper(self):
        with patch.object(self.session, "capture", return_value=self.scene(None)), \
             patch.object(self.session, "execute") as execute:
            with self.assertRaises(ValueError):
                self.session.recover_place("outside_table")
            with self.assertRaises(RuntimeError):
                self.session.recover_place("blue_target")
            execute.assert_not_called()

    def test_recovery_requires_stop_latch_and_place_skill_before_capture(self):
        with patch.object(self.session, "capture") as capture:
            with self.assertRaises(RuntimeError):
                self.session.execute({"skill": "place"}, {}, recovery=True)
            (self.directory / "stopped").touch()
            with self.assertRaises(RuntimeError):
                self.session.execute({"skill": "pick"}, {}, recovery=True)
            capture.assert_not_called()
        self.assertTrue((self.directory / "stopped").exists())

    def test_stopped_session_rejects_ordinary_motion_before_capture(self):
        (self.directory / "stopped").touch()
        with patch.object(self.session, "capture") as capture:
            with self.assertRaises(RuntimeError):
                self.session.execute({"skill": "place"}, {})
            capture.assert_not_called()

    def test_invalid_recovery_proposal_keeps_stop_latched_and_never_launches(self):
        (self.directory / "stopped").touch()
        with patch.object(self.session, "capture", return_value=self.scene()), \
             patch("vgm_runtime.session_backend.subprocess.Popen") as launch:
            with self.assertRaises(ValueError):
                self.session.execute({"skill": "place", "joint_targets": [0]}, {}, recovery=True)
            launch.assert_not_called()
        self.assertTrue((self.directory / "stopped").exists())

    def test_nonmotion_recovery_rejects_active_executor(self):
        (self.directory / "execution.active").touch()
        with patch.object(self.session, "capture") as capture:
            with self.assertRaises(RuntimeError):
                self.session.recover()
            capture.assert_not_called()


if __name__ == "__main__":
    unittest.main()
