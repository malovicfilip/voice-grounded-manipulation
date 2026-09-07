"""Final RGB-D placement acceptance tests."""

from __future__ import annotations

import json
import sys
import unittest
from pathlib import Path


REPOSITORY_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPOSITORY_ROOT / "ros2_ws" / "src" / "vgm_runtime"))

from vgm_runtime.outcome import (  # noqa: E402
    OutcomeValidationError,
    validate_placement_outcome,
)
from vgm_runtime.types import GroundedScene, ObjectObservation  # noqa: E402
from vgm_runtime.config import load_json_config
from dataclasses import replace
from scene_fixtures import targets


POLICY = load_json_config("safety_policy.json")
EXECUTION = {
    "validated_skill": {
        "skill": "pick_and_place",
        "object_id": "red_cube",
        "target_id": "blue_target",
    }
}


def final_scene(position=(0.018, 0.291, 0.775), confidence=0.81):
    return GroundedScene(
        "0123456789abcdef",
        2.0,
        {
            "red_cube": ObjectObservation(
                "red_cube", position, confidence, 2.0, pixel_count=381
            )
        },
        targets=targets(2.0),
    )


class OutcomeValidationTest(unittest.TestCase):
    def test_accepts_observed_cube_inside_target_tolerance(self):
        final = final_scene()
        previous = replace(final, captured_at_s=1., objects={key: replace(obj, observed_at_s=1.) for key, obj in final.objects.items()})
        result = validate_placement_outcome(EXECUTION, final, POLICY, previous_scene=previous, clock=lambda: 2.)
        self.assertEqual(result["status"], "accepted")
        self.assertLess(result["error_m"], 0.03)


    def test_accepts_target_occluded_by_placed_cube_using_fresh_preexecution_pose(self):
        final = final_scene(position=(0.004, 0.300, 0.776))
        final = replace(final, targets={})
        previous = replace(
            final, captured_at_s=1.0,
            objects={"red_cube": replace(final.objects["red_cube"], observed_at_s=1.0)},
        )
        execution = {
            **EXECUTION,
            "target_position_m": (0.0, 0.3, 0.751),
        }
        result = validate_placement_outcome(
            execution, final, POLICY, previous_scene=previous, clock=lambda: 2.0
        )
        self.assertEqual(result["status"], "accepted")
        self.assertEqual(result["target_final_visibility"], "occluded_by_placed_object")
        self.assertEqual(result["target_reference"], "pre_execution_observation")
        self.assertLess(result["target_occlusion_xy_error_m"], 0.01)

    def test_missing_target_fallback_requires_geometric_occlusion_evidence(self):
        final = final_scene(position=(0.05, 0.300, 0.776))
        final = replace(final, targets={})
        previous = replace(
            final, captured_at_s=1.0,
            objects={"red_cube": replace(final.objects["red_cube"], observed_at_s=1.0)},
        )
        execution = {
            **EXECUTION,
            "target_position_m": (0.0, 0.3, 0.751),
        }
        with self.assertRaises(OutcomeValidationError) as raised:
            validate_placement_outcome(
                execution, final, POLICY, previous_scene=previous, clock=lambda: 2.0
            )
        self.assertEqual(raised.exception.code, "target_occlusion_unverified")

    def test_missing_target_without_measured_execution_reference_is_rejected(self):
        final = replace(final_scene(position=(0.004, 0.300, 0.776)), targets={})
        previous = replace(
            final, captured_at_s=1.0,
            objects={"red_cube": replace(final.objects["red_cube"], observed_at_s=1.0)},
        )
        with self.assertRaises(OutcomeValidationError) as raised:
            validate_placement_outcome(
                EXECUTION, final, POLICY, previous_scene=previous, clock=lambda: 2.0
            )
        self.assertEqual(raised.exception.code, "target_not_observed")

    def test_rejects_missing_low_confidence_and_misplaced_cube(self):
        cases = [
            (GroundedScene("0123456789abcdef", 1.0, {}), "object_not_observed"),
            (final_scene(confidence=0.4), "low_outcome_confidence"),
            (final_scene(position=(0.2, 0.0, 0.775)), "placement_error"),
        ]
        for scene, code in cases:
            with self.subTest(code=code), self.assertRaises(OutcomeValidationError) as raised:
                validate_placement_outcome(EXECUTION, scene, POLICY, clock=lambda: 2.)
            self.assertEqual(raised.exception.code, code)


if __name__ == "__main__":
    unittest.main()
