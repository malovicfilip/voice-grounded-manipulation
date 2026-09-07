"""Acceptance assertions must track real validation, not accept transport errors."""
import importlib.util
from pathlib import Path
import unittest
from unittest.mock import patch

from test_skill_runtime import grounded_scene, NOW
from vgm_runtime.validator import SkillValidator, SkillValidationError

spec = importlib.util.spec_from_file_location(
    "acceptance_runner", Path(__file__).resolve().parents[1] /
    "isaac_sim/scripts/validate_all_phases.py")
runner = importlib.util.module_from_spec(spec)
spec.loader.exec_module(runner)


class Backend:
    def capture(self):
        return grounded_scene()

    def request(self, operation):
        return {"positions": [0.0] * 7}

    def execute(self, proposal, scene):
        try:
            SkillValidator(clock=lambda: NOW).validate(proposal, scene)
        except SkillValidationError as error:
            raise RuntimeError(str(error)) from error
        raise AssertionError("invalid fixture reached execution")


class AcceptanceRunnerTests(unittest.TestCase):
    def test_stop_fixture_cancels_on_failure(self):
        class Cancellable:
            stopped = False
            def stop(self):
                self.stopped = True
        backend = Cancellable()
        with patch.object(runner, "_live_stop", side_effect=AssertionError("fixture failed")):
            with self.assertRaisesRegex(AssertionError, "fixture failed"):
                runner.live_stop(backend)
        self.assertTrue(backend.stopped)

    def test_boundary_matches_actual_validator(self):
        result = runner.boundary(Backend())
        self.assertEqual(len(result["rejected"]), 4)
        self.assertEqual(result["joint_delta_rad"], 0)

    def test_transport_failure_is_not_safety_refusal(self):
        class Offline(Backend):
            def execute(self, proposal, scene):
                raise RuntimeError("SSH connection failed")
        with self.assertRaisesRegex(AssertionError, "unrelated reason"):
            runner.boundary(Offline())
