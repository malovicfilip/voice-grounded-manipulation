"""Offline acceptance tests for the language-to-skill safety boundary."""

from __future__ import annotations

import json
import sys
import tempfile
import unittest
from dataclasses import replace
from pathlib import Path


REPOSITORY_ROOT = Path(__file__).resolve().parents[1]
RUNTIME_SOURCE = REPOSITORY_ROOT / "ros2_ws" / "src" / "vgm_runtime"
sys.path.insert(0, str(RUNTIME_SOURCE))

from vgm_runtime.coordinator import TaskCoordinator  # noqa: E402
from vgm_runtime.execution_gate import gate_decision  # noqa: E402
from vgm_runtime.pipeline import IntentPipeline  # noqa: E402
from vgm_runtime.serialization import plan_to_mapping  # noqa: E402
from vgm_runtime.rule_intent import RuleBasedIntentModel  # noqa: E402
from vgm_runtime.types import GroundedScene, ObjectObservation  # noqa: E402
from vgm_runtime.validator import SkillValidationError, SkillValidator  # noqa: E402
from scene_fixtures import targets


NOW = 1_800_000_000.0
REVISION = "0123456789abcdef"


def grounded_scene(
    *, confidence: float = 0.98, captured_at_s: float = NOW, x: float = -0.1
) -> GroundedScene:
    return GroundedScene(
        revision=REVISION,
        captured_at_s=captured_at_s,
        targets=targets(captured_at_s),
        objects={
            "red_cube": ObjectObservation(
                "red_cube",
                (x, -0.18, 0.775),
                confidence,
                captured_at_s,
                pixel_count=120,
            ),
            "blue_cube": ObjectObservation(
                "blue_cube",
                (0.3, -0.18, 0.775),
                0.97,
                captured_at_s,
                pixel_count=115,
            ),
        },
    )


def proposal(skill: str, **changes):
    value = {
        "schema_version": 1,
        "request_id": "req_test_1",
        "skill": skill,
        "object_id": None,
        "target_id": None,
        "pose_name": None,
        "reason": None,
        "scene_revision": None,
    }
    value.update(changes)
    return value


class SkillValidatorTest(unittest.TestCase):
    def validator(self) -> SkillValidator:
        return SkillValidator(clock=lambda: NOW)

    def assert_rejected(self, value, code, scene=None):
        with self.assertRaises(SkillValidationError) as raised:
            self.validator().validate(value, scene)
        self.assertEqual(raised.exception.code, code)

    def test_accepts_every_allowlisted_shape(self):
        cases = [
            (proposal("move_named_pose", pose_name="ready"), grounded_scene()),
            (proposal("open_gripper"), grounded_scene()),
            (
                proposal("pick", object_id="red_cube", scene_revision=REVISION),
                grounded_scene(),
            ),
            (
                proposal(
                    "place",
                    object_id="red_cube",
                    target_id="blue_target",
                    scene_revision=REVISION,
                ),
                replace(grounded_scene(), held_object_id="red_cube"),
            ),
            (
                proposal(
                    "pick_and_place",
                    object_id="red_cube",
                    target_id="yellow_target",
                    scene_revision=REVISION,
                ),
                grounded_scene(),
            ),
            (proposal("stop", reason="user requested stop"), None),
            (proposal("refuse", reason="ambiguous object"), None),
        ]
        for value, scene in cases:
            with self.subTest(skill=value["skill"]):
                result = self.validator().validate(value, scene)
                self.assertEqual(result.proposal.skill, value["skill"])

    def test_rejects_extra_missing_and_wrong_typed_fields(self):
        extra = proposal("open_gripper", debug=True)
        self.assert_rejected(extra, "schema_fields")
        missing = proposal("open_gripper")
        del missing["reason"]
        self.assert_rejected(missing, "schema_fields")
        self.assert_rejected(
            proposal("open_gripper", schema_version=True), "schema_version"
        )

    def test_rejects_direct_control_fields_before_generic_schema_error(self):
        for field in (
            "joint_targets",
            "desired_joint_positions",
            "velocity",
            "motor_commands",
            "trajectory",
            "torques",
        ):
            with self.subTest(field=field):
                value = proposal("open_gripper")
                value[field] = [0.0]
                self.assert_rejected(value, "forbidden_control_field")

    def test_rejects_invalid_skill_parameter_combinations(self):
        self.assert_rejected(
            proposal("pick", object_id="red_cube"), "scene_revision_required"
        )
        self.assert_rejected(
            proposal("open_gripper", object_id="red_cube"), "invalid_parameters"
        )
        self.assert_rejected(
            proposal("move_named_pose", pose_name="unsafe"), "pose_name"
        )

    def test_rejects_stale_ungrounded_low_confidence_and_outside_objects(self):
        value = proposal("pick", object_id="red_cube", scene_revision=REVISION)
        self.assert_rejected(
            value, "stale_scene", grounded_scene(captured_at_s=NOW - 11.0)
        )
        self.assert_rejected(
            value, "low_confidence", grounded_scene(confidence=0.4)
        )
        self.assert_rejected(
            value, "outside_workspace", grounded_scene(x=0.9)
        )
        unknown = proposal(
            "pick", object_id="green_cube", scene_revision=REVISION
        )
        self.assert_rejected(unknown, "object_not_grounded", grounded_scene())

    def test_rejects_stale_revision_and_unknown_target(self):
        stale = proposal(
            "pick", object_id="red_cube", scene_revision="ffffffffffffffff"
        )
        self.assert_rejected(stale, "stale_revision", grounded_scene())
        target = proposal(
            "place",
            object_id="red_cube",
            target_id="unknown_target",
            scene_revision=REVISION,
        )
        self.assert_rejected(target, "target_not_grounded", replace(grounded_scene(), held_object_id="red_cube"))

    def test_rejects_replay_but_stop_remains_idempotent(self):
        validator = self.validator()
        value = proposal("open_gripper")
        validator.validate(value, grounded_scene())
        with self.assertRaises(SkillValidationError) as raised:
            validator.validate(value, grounded_scene())
        self.assertEqual(raised.exception.code, "replayed_request")
        stop = proposal("stop", reason="stop")
        validator.validate(stop, None)
        validator.validate(stop, None)

    def test_revalidation_rejects_a_changed_scene_revision(self):
        validator = self.validator()
        scene = grounded_scene()
        validated = validator.validate(
            proposal(
                "pick_and_place",
                object_id="red_cube",
                target_id="blue_target",
                scene_revision=REVISION,
            ),
            scene,
        )
        changed = GroundedScene(
            revision="fedcba9876543210",
            captured_at_s=NOW,
            objects=scene.objects,
        )
        with self.assertRaises(SkillValidationError) as raised:
            validator.revalidate(validated, changed)
        self.assertEqual(raised.exception.code, "stale_revision")


class CoordinatorAndPipelineTest(unittest.TestCase):
    def test_pick_and_place_expands_only_after_validation(self):
        validator = SkillValidator(clock=lambda: NOW)
        scene = grounded_scene()
        validated = validator.validate(
            proposal(
                "pick_and_place",
                object_id="red_cube",
                target_id="blue_target",
                scene_revision=REVISION,
            ),
            scene,
        )
        plan = TaskCoordinator(validator.policy).create_plan(validated, scene)
        kinds = [primitive.kind for primitive in plan.primitives]
        self.assertEqual(
            kinds,
            [
                "open_gripper",
                "move_cartesian",
                "move_cartesian",
                "close_gripper",
                "attach_object",
                "move_cartesian",
                "move_cartesian",
                "move_cartesian",
                "open_gripper",
                "detach_object",
                "move_cartesian",
                "move_named_pose",
            ],
        )
        self.assertNotIn("joint", json.dumps(kinds))

    def test_rule_pipeline_accepts_grounded_pick_and_place(self):
        validator = SkillValidator(clock=lambda: NOW)
        pipeline = IntentPipeline(
            RuleBasedIntentModel(request_id_factory=lambda: "req_rule_1"),
            validator,
            TaskCoordinator(validator.policy),
        )
        result = pipeline.decide(
            "Pick the red cube and place it on the blue target",
            grounded_scene(),
        )
        self.assertTrue(result.accepted)
        self.assertEqual(result.validated_skill.proposal.skill, "pick_and_place")

    def test_rule_pipeline_refuses_ambiguous_command(self):
        validator = SkillValidator(clock=lambda: NOW)
        pipeline = IntentPipeline(
            RuleBasedIntentModel(request_id_factory=lambda: "req_rule_2"),
            validator,
            TaskCoordinator(validator.policy),
        )
        result = pipeline.decide(
            "Pick the red cube and blue cube",
            grounded_scene(),
        )
        self.assertFalse(result.accepted)
        self.assertEqual(result.code, "model_refusal")

    def test_pipeline_fails_closed_when_model_raises(self):
        class BrokenModel:
            def propose(self, transcript, scene, policy):
                raise TimeoutError("simulated")

        validator = SkillValidator(clock=lambda: NOW)
        result = IntentPipeline(
            BrokenModel(), validator, TaskCoordinator(validator.policy)
        ).decide("pick something", grounded_scene())
        self.assertFalse(result.accepted)
        self.assertEqual(result.code, "intent_model_failure")
        self.assertIsNone(result.plan)

    def test_execution_gate_revalidates_and_rebuilds_plan(self):
        scene = grounded_scene()
        validator = SkillValidator(clock=lambda: NOW)
        accepted = validator.validate(
            proposal(
                "pick_and_place",
                object_id="red_cube",
                target_id="blue_target",
                scene_revision=REVISION,
            ),
            scene,
        )
        initial_plan = TaskCoordinator(validator.policy).create_plan(accepted, scene)
        decision = {
            "accepted": True,
            "code": "accepted",
            "message": "validated",
            "validated_skill": accepted.to_mapping(),
            "plan": plan_to_mapping(initial_plan),
        }
        gated = gate_decision(
            decision, scene, SkillValidator(clock=lambda: NOW + 0.5)
        )
        self.assertEqual(gated["plan"], plan_to_mapping(initial_plan))
        self.assertEqual(
            gated["validated_skill"]["validated_at_s"], NOW + 0.5
        )

    def test_execution_gate_rejects_commanded_object_drift(self):
        scene = grounded_scene()
        validator = SkillValidator(clock=lambda: NOW)
        accepted = validator.validate(
            proposal(
                "pick_and_place",
                object_id="red_cube",
                target_id="blue_target",
                scene_revision=REVISION,
            ),
            scene,
        )
        initial_plan = TaskCoordinator(validator.policy).create_plan(accepted, scene)
        decision = {
            "accepted": True,
            "validated_skill": accepted.to_mapping(),
            "plan": plan_to_mapping(initial_plan),
        }
        moved = grounded_scene(x=-0.08)
        with self.assertRaises(SkillValidationError) as raised:
            gate_decision(
                decision, moved, SkillValidator(clock=lambda: NOW + 0.5)
            )
        self.assertEqual(raised.exception.code, "object_moved")


if __name__ == "__main__":
    unittest.main()
