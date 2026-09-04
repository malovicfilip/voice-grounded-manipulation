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


POLICY = json.loads(
    (REPOSITORY_ROOT / "config" / "safety_policy.json").read_text()
)
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
        1.0,
        {
            "red_cube": ObjectObservation(
                "red_cube", position, confidence, 1.0, pixel_count=381
            )
        },
    )


class OutcomeValidationTest(unittest.TestCase):
    def test_accepts_observed_cube_inside_target_tolerance(self):
        result = validate_placement_outcome(EXECUTION, final_scene(), POLICY)
        self.assertEqual(result["status"], "accepted")
        self.assertLess(result["error_m"], 0.03)

    def test_rejects_missing_low_confidence_and_misplaced_cube(self):
        cases = [
            (GroundedScene("0123456789abcdef", 1.0, {}), "object_not_observed"),
            (final_scene(confidence=0.4), "low_outcome_confidence"),
            (final_scene(position=(0.2, 0.0, 0.775)), "placement_error"),
        ]
        for scene, code in cases:
            with self.subTest(code=code), self.assertRaises(OutcomeValidationError) as raised:
                validate_placement_outcome(EXECUTION, scene, POLICY)
            self.assertEqual(raised.exception.code, code)


if __name__ == "__main__":
    unittest.main()
