"""Cross-host timestamp checks must use the camera host's clock domain."""

import json
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "ros2_ws/src/vgm_runtime"))
from vgm_runtime.audit import AuditLogger
from vgm_runtime.ssh_backend import SSHBackend
from vgm_runtime.tasks import TaskSession
from vgm_runtime.types import GroundedScene, ObjectObservation
from vgm_runtime.validator import SkillValidationError, SkillValidator


class SSHClockTests(unittest.TestCase):
    def response(self, server_time):
        return subprocess.CompletedProcess([], 0, json.dumps(
            {"ok": True, "result": {"status": "fixture"}, "server_time_s": server_time}), "")

    def test_clock_is_anchored_to_remote_time_and_advances_monotonically(self):
        backend = SSHBackend("fixture")
        with patch("vgm_runtime.ssh_backend.subprocess.run", return_value=self.response(1000.54)), \
             patch("vgm_runtime.ssh_backend.time.monotonic", return_value=200):
            backend.request("robot_state")
            self.assertEqual(backend.clock(), 1000.54)
        with patch("vgm_runtime.ssh_backend.time.monotonic", return_value=205):
            self.assertEqual(backend.clock(), 1005.54)

    def test_remote_clock_metadata_is_required_and_finite(self):
        for value in (None, True, "1000", float("nan"), float("inf")):
            backend = SSHBackend("fixture")
            with patch("vgm_runtime.ssh_backend.subprocess.run", return_value=self.response(value)):
                with self.assertRaises(RuntimeError):
                    backend.request("capture")

    def test_unobserved_clock_cannot_authorize_a_task(self):
        with self.assertRaises(RuntimeError):
            SSHBackend("fixture").clock()

    def test_task_session_uses_backend_clock_without_rewriting_sensor_time(self):
        backend = SSHBackend("fixture")
        with tempfile.TemporaryDirectory() as directory:
            session = TaskSession(None, backend, AuditLogger(Path(directory) / "audit.jsonl"))
            self.assertEqual(session.clock, backend.clock)
        with patch("vgm_runtime.ssh_backend.subprocess.run", return_value=self.response(1000.54)), \
             patch("vgm_runtime.ssh_backend.time.monotonic", return_value=200):
            backend.request("capture")
            self.assertEqual(session.clock(), 1000.54)
            scene = GroundedScene("0123456789abcdef", 1000.5, {
                "red_cube": ObjectObservation("red_cube", (-.1, -.18, .775), .98, 1000.5, pixel_count=100),
            })
            proposal = {"schema_version": 1, "request_id": "clock_fixture", "skill": "inspect",
                        "object_id": "red_cube", "target_id": None, "pose_name": None,
                        "reason": None, "scene_revision": scene.revision}
            with self.assertRaises(SkillValidationError):
                SkillValidator(clock=lambda: 1000.0).validate(proposal, scene)
            SkillValidator(clock=session.clock).validate(proposal, scene)
            self.assertEqual(scene.captured_at_s, 1000.5)


if __name__ == "__main__":
    unittest.main()
