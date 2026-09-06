"""Regression invariants for 3D grounding, skill semantics and cancellation."""

import copy
from dataclasses import replace
import importlib.util
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest
from unittest.mock import patch

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "ros2_ws/src/vgm_runtime"))
from vgm_runtime.config import load_json_config, policy_digest
from vgm_runtime.coordinator import TaskCoordinator
from vgm_runtime.execution_gate import rebind_after_capture
from vgm_runtime.moveit_parameters import execution_parameters
from vgm_runtime.outcome import OutcomeValidationError, validate_placement_outcome
from vgm_runtime.perception import CameraIntrinsics, ColorDepthGrounder
from vgm_runtime.pipeline import IntentPipeline
from vgm_runtime.serialization import scene_from_mapping
from vgm_runtime.types import GroundedScene, ObjectObservation
from vgm_runtime.validator import SkillValidationError, SkillValidator
from test_skill_runtime import grounded_scene, proposal, NOW, REVISION
from test_tasks import Backend, Model, step
from vgm_runtime.audit import AuditLogger
from vgm_runtime.tasks import TaskSession


STOP_PHRASES = ("stop", "stop it", "please stop", "stop the robot", "cancel", "abort",
                "halt", "emergency stop", "STOP!!!", "Please, stop it now.")


class SafetyUpgradeTests(unittest.TestCase):
    def setUp(self):
        self.scene = grounded_scene()
        self.policy = load_json_config("safety_policy.json")

    def validator(self):
        return SkillValidator(clock=lambda: NOW)

    def test_state_preconditions_are_in_validator_and_revalidation(self):
        held = replace(self.scene, held_object_id="red_cube")
        for value in (proposal("open_gripper"), proposal("close_gripper"),
                      proposal("pick", object_id="blue_cube", scene_revision=REVISION),
                      proposal("pick_and_place", object_id="blue_cube", target_id="blue_target", scene_revision=REVISION),
                      proposal("move_named_pose", pose_name="transport")):
            with self.subTest(skill=value["skill"]), self.assertRaises(SkillValidationError):
                self.validator().validate(value, held)
        for value in (proposal("close_gripper"), proposal("place", object_id="red_cube", target_id="blue_target", scene_revision=REVISION)):
            with self.assertRaises(SkillValidationError):
                self.validator().validate(value, self.scene)
        accepted = self.validator().validate(proposal("open_gripper"), self.scene)
        with self.assertRaises(SkillValidationError):
            self.validator().revalidate(accepted, held)
        with self.assertRaises(SkillValidationError):
            self.validator().validate(proposal("open_gripper"), None)

    def test_schema_itself_controls_syntax_before_robot_state(self):
        schema = load_json_config("robot_skill.schema.json")
        schema["properties"]["request_id"]["maxLength"] = 3
        validator = SkillValidator(schema=schema, clock=lambda: NOW)
        with patch.object(validator, "_validate_robot_state") as state:
            with self.assertRaises(SkillValidationError):
                validator.validate(proposal("open_gripper"), self.scene)
            state.assert_not_called()
        # A coordinate payload is rejected as raw syntax, including nested data.
        for change in ({"coordinates": [0, 0, 0]}, {"object_id": {"x": 0}}, {"trajectory": []}):
            with self.assertRaises(SkillValidationError):
                self.validator().validate(proposal("pick", **change), self.scene)

    def test_measured_target_drives_plan_and_cpp_parameters(self):
        target = replace(self.scene.targets["blue_target"], position_m=(.05, .25, .8))
        scene = replace(self.scene, targets={"blue_target": target})
        value = proposal("pick_and_place", object_id="red_cube", target_id="blue_target", scene_revision=REVISION)
        skill = self.validator().validate(value, scene)
        plan = TaskCoordinator(self.policy).create_plan(skill, scene)
        self.assertEqual(plan.primitives[6].position_m, (.05, .25, 1.05))
        params = execution_parameters(scene.to_mapping(), "blue_target", clock=lambda: NOW)
        self.assertEqual([params["target_" + axis] for axis in "xyz"], [.05, .25, .8])
        self.assertEqual(params["safety_policy_digest"], policy_digest(self.policy))
        self.assertEqual(scene_from_mapping(scene.to_mapping()), scene)

    def test_absent_stale_low_confidence_nonworld_and_outside_targets_fail_closed(self):
        original = self.scene.targets["blue_target"]
        value = proposal("pick_and_place", object_id="red_cube", target_id="blue_target", scene_revision=REVISION)
        targets = [{}, {"blue_target": replace(original, confidence=.1)},
                   {"blue_target": replace(original, observed_at_s=NOW-11)},
                   {"blue_target": replace(original, frame_id="camera")},
                   {"blue_target": replace(original, position_m=(.9, 0, .8))}]
        for observations in targets:
            with self.assertRaises(SkillValidationError):
                self.validator().validate(value, replace(self.scene, targets=observations))
        legacy = self.scene.to_mapping()
        del legacy["targets"]
        self.assertFalse(scene_from_mapping(legacy).targets)

    def test_target_motion_cannot_be_hidden_by_rebinding_revision(self):
        target = replace(self.scene.targets["blue_target"], position_m=(.05, .3, .751))
        changed = replace(self.scene, targets={**self.scene.targets, "blue_target": target})
        value = proposal("pick_and_place", object_id="red_cube", target_id="blue_target", scene_revision=REVISION)
        with self.assertRaises(SkillValidationError) as error:
            rebind_after_capture(value, self.scene, changed, self.validator())
        self.assertEqual(error.exception.code, "target_moved")

    def test_two_frame_3d_resting_verification(self):
        obj = replace(self.scene.objects["red_cube"], position_m=(0., .3, .776))
        final = replace(self.scene, objects={"red_cube": obj})
        previous = replace(final, captured_at_s=NOW-1,
                           objects={"red_cube": replace(obj, observed_at_s=NOW-1)})
        execution = {"validated_skill": {"skill": "place", "object_id": "red_cube", "target_id": "blue_target"}}
        result = validate_placement_outcome(execution, final, self.policy, previous_scene=previous, clock=lambda: NOW)
        self.assertEqual(result["z_error_m"], 0.)
        cases = [(replace(final, held_object_id="red_cube"), previous, "object_still_attached"),
                 (replace(final, objects={"red_cube": replace(obj, position_m=(0., .3, .9))}), previous, "placement_z_error"),
                 (final, None, "rest_evidence_required"),
                 (final, final, "rest_evidence_required"),
                 (final, replace(previous, objects={"red_cube": replace(previous.objects["red_cube"], position_m=(0., .32, .776))}), "object_not_resting")]
        for end, start, code in cases:
            with self.subTest(code=code), self.assertRaises(OutcomeValidationError) as error:
                validate_placement_outcome(execution, end, self.policy, previous_scene=start, clock=lambda: NOW)
            self.assertEqual(error.exception.code, code)

    def test_stop_variants_skip_capture_model_confirmation_and_state_gate(self):
        with tempfile.TemporaryDirectory() as directory:
            for phrase in STOP_PHRASES:
                backend, model = Backend(), Model([step("inspect", "red_cube")])
                session = TaskSession(model, backend, AuditLogger(Path(directory)/"audit.jsonl"))
                session.state = "running"
                with patch.object(backend, "capture", side_effect=AssertionError("must not capture")):
                    self.assertEqual(session.prepare(phrase)["status"], "stopped")
                self.assertEqual(model.calls, 0)
                self.assertEqual(backend.stops, 1)
                validator = self.validator()
                with patch.object(model, "propose", side_effect=AssertionError("must not call LLM")):
                    decision = IntentPipeline(model, validator, TaskCoordinator()).decide(phrase, self.scene)
                    self.assertEqual(decision.plan.skill, "stop")

    def test_clarifications_are_structured_without_recursive_prompt_wrapping(self):
        with tempfile.TemporaryDirectory() as directory:
            model = Model([step("refuse", reason="Which cube?")])
            session = TaskSession(model, Backend(), AuditLogger(Path(directory)/"audit.jsonl"))
            session.prepare("move a cube")
            session.prepare("red")
            model.steps = [step("refuse", reason="Which target?")]
            session.prepare("the small one")
            payload = json.loads(model.transcripts[-1])
            self.assertEqual(payload["original_request"], "move a cube")
            self.assertEqual([turn["answer"] for turn in payload["clarification_turns"]], ["red", "the small one"])
            self.assertNotIn("Original request:", model.transcripts[-1])
            session.stop()
            self.assertIsNone(session._clarification)


class Perception3DTests(unittest.TestCase):
    def test_raised_top_and_vertical_face_centers_retain_measured_z(self):
        grid = np.linspace(-.025, .025, 21)
        camera = np.array([.35, -1.1, 1.35])
        for z in (.8, .9, 1.):
            top = np.array([[x, y, z] for x in grid for y in grid])
            self.assertAlmostEqual(ColorDepthGrounder._cube_surface_center(top, camera, .05)[2], z-.025)
            side = np.array([[x, -.025, z+h] for x in grid for h in grid])
            self.assertAlmostEqual(ColorDepthGrounder._cube_surface_center(side, camera, .05)[2], z)

    def target_scene(self, x=.12, z=.85, *, hole=False, invalid_depth=False):
        config = load_json_config("phase_1_scene.json")
        rgb = np.zeros((161,161,3), dtype=float)
        rows, cols = np.indices(rgb.shape[:2])
        r = np.hypot((cols-80)/1000, (rows-80)/1000)
        mask = (r <= .06) & ((r >= .035) if hole else True)
        rgb[mask] = config["targets"][0]["color_rgb"]
        depth = np.full(rgb.shape[:2], np.nan)
        depth[mask] = np.nan if invalid_depth else 1.
        transform = np.eye(4)
        transform[:3,3] = [x, .22, z-1.]
        return ColorDepthGrounder(config).ground(rgb, depth, CameraIntrinsics(1000,1000,80,80), transform)

    def test_target_is_measured_in_xyz_and_survives_center_occlusion(self):
        for hole in (False, True):
            scene = self.target_scene(hole=hole)
            np.testing.assert_allclose(scene.targets["blue_target"].position_m, [.12,.22,.85], atol=.002)
        self.assertNotEqual(self.target_scene().revision, self.target_scene(x=.16).revision)
        self.assertNotEqual(self.target_scene().revision, self.target_scene(z=.9).revision)
        self.assertFalse(self.target_scene(invalid_depth=True).targets)


class CanonicalPolicyTests(unittest.TestCase):
    def test_perception_import_does_not_require_command_runtime_dependencies(self):
        script = '''
import importlib.abc
import sys
sys.path.insert(0, sys.argv[1])
class NoSchema(importlib.abc.MetaPathFinder):
    def find_spec(self, fullname, path=None, target=None):
        if fullname == "jsonschema" or fullname.startswith("jsonschema."):
            raise ModuleNotFoundError("jsonschema deliberately unavailable")
sys.meta_path.insert(0, NoSchema())
from vgm_runtime.perception import ColorDepthGrounder
from vgm_runtime.config import load_json_config
assert load_json_config("safety_policy.json")["policy_version"] == 1
assert "vgm_runtime.validator" not in sys.modules
try:
    from vgm_runtime import SkillValidator
except ModuleNotFoundError:
    pass
else:
    raise AssertionError("validator must fail closed without its schema dependency")
'''
        subprocess.run([sys.executable, "-c", script, str(ROOT / "ros2_ws/src/vgm_runtime")],
                       check=True, capture_output=True, text=True)

    def test_header_generation_changes_with_policy_and_compiles(self):
        script = ROOT / "ros2_ws/src/vgm_moveit_demo/scripts/generate_safety_header.py"
        spec = importlib.util.spec_from_file_location("generator", script)
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        policy = load_json_config("safety_policy.json")
        original = module.generate(policy)
        changed = copy.deepcopy(policy)
        changed["maximum_velocity_scale"] = .1
        changed["workspace_m"]["x"][1] = .5
        modified = module.generate(changed)
        self.assertIn("kVelocityScale = 0.1", modified)
        self.assertIn(policy_digest(changed), modified)
        self.assertNotEqual(original, modified)
        # Compiler reads generated text from stdin; no ROS or dependency install.
        subprocess.run(["g++", "-std=c++17", "-x", "c++", "-fsyntax-only", "-"],
                       input=modified, text=True, check=True, capture_output=True)
        self.assertFalse(any("position_m" in v for v in policy["targets"].values()))


if __name__ == "__main__":
    unittest.main()
