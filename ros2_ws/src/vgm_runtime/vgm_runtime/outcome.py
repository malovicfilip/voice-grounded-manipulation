"""Deterministic verification of a completed placement from final RGB-D."""

from __future__ import annotations

import math
from collections.abc import Mapping
from typing import Any

from .types import GroundedScene


class OutcomeValidationError(ValueError):
    def __init__(self, code: str, message: str) -> None:
        super().__init__(message)
        self.code = code


def validate_placement_outcome(
    execution: Mapping[str, Any],
    final_scene: GroundedScene,
    policy: Mapping[str, Any],
) -> dict[str, Any]:
    skill = execution.get("validated_skill")
    if not isinstance(skill, Mapping) or skill.get("skill") != "pick_and_place":
        raise OutcomeValidationError(
            "unsupported_outcome", "outcome validation requires pick_and_place"
        )
    object_id = skill.get("object_id")
    target_id = skill.get("target_id")
    target = policy["targets"].get(target_id)
    if not isinstance(object_id, str) or target is None:
        raise OutcomeValidationError(
            "invalid_outcome_ids", "outcome object or target is invalid"
        )
    observation = final_scene.objects.get(object_id)
    if observation is None:
        raise OutcomeValidationError(
            "object_not_observed", "placed object is absent from final RGB-D"
        )
    if observation.confidence < policy["minimum_object_confidence"]:
        raise OutcomeValidationError(
            "low_outcome_confidence", "placed object confidence is too low"
        )
    target_position = target["position_m"]
    error_m = math.dist(observation.position_m[:2], target_position[:2])
    if error_m > policy["maximum_placement_error_m"]:
        raise OutcomeValidationError(
            "placement_error", "placed object is outside target tolerance"
        )
    return {
        "confidence": observation.confidence,
        "error_m": error_m,
        "object_id": object_id,
        "status": "accepted",
        "target_id": target_id,
    }
