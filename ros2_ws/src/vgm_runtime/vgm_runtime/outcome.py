"""Deterministic 3D placement acceptance using two fresh RGB-D observations."""

from __future__ import annotations

import math
import time
from collections.abc import Mapping
from typing import Any

from .types import GroundedScene
from .validator import SkillValidator, SkillValidationError


class OutcomeValidationError(ValueError):
    def __init__(self, code: str, message: str) -> None:
        super().__init__(message)
        self.code = code


def validate_placement_outcome(
    execution: Mapping[str, Any], final_scene: GroundedScene, policy: Mapping[str, Any],
    *, previous_scene: GroundedScene | None = None, clock=time.time,
) -> dict[str, Any]:
    skill = execution.get("validated_skill")
    if not isinstance(skill, Mapping) or skill.get("skill") not in {"pick_and_place", "place"}:
        raise OutcomeValidationError("unsupported_outcome", "outcome requires a placement")
    object_id, target_id = skill.get("object_id"), skill.get("target_id")
    if object_id not in policy["objects"] or target_id not in policy["targets"]:
        raise OutcomeValidationError("invalid_outcome_ids", "outcome object or target is invalid")
    observation = final_scene.objects.get(object_id)
    if observation is None:
        raise OutcomeValidationError("object_not_observed", "placed object is absent from final RGB-D")
    if final_scene.held_object_id is not None:
        raise OutcomeValidationError("object_still_attached", "placement requires an empty attachment state")
    target = final_scene.targets.get(target_id)
    if target is None:
        raise OutcomeValidationError("target_not_observed", "final target observation is required")
    validator = SkillValidator(policy=policy, clock=clock)
    try:
        validator._validate_scene_age(final_scene)
        validator._validate_observation(observation, final_scene, "object")
        validator._validate_observation(target, final_scene, "target")
    except SkillValidationError as error:
        raise OutcomeValidationError("low_outcome_confidence" if error.code == "low_confidence" else error.code,
                                     str(error)) from error
    expected = execution.get("target_position_m")
    if expected is not None:
        if len(expected) != 3 or not all(type(v) in (int, float) and math.isfinite(v) for v in expected):
            raise OutcomeValidationError("invalid_target_pose", "invalid deterministic target pose")
        if math.dist(expected, target.position_m) > policy["maximum_object_drift_m"]:
            raise OutcomeValidationError("target_moved", "target moved during placement")
    xy_error = math.dist(observation.position_m[:2], target.position_m[:2])
    z_error = abs(observation.position_m[2] - (target.position_m[2] + policy["objects"][object_id]["size_m"] / 2))
    if xy_error > policy["maximum_placement_error_m"]:
        raise OutcomeValidationError("placement_error", "XY error exceeds placement tolerance")
    if z_error > policy["maximum_placement_z_error_m"]:
        raise OutcomeValidationError("placement_z_error", "object is not resting at target support height")
    if previous_scene is None or object_id not in previous_scene.objects:
        raise OutcomeValidationError("rest_evidence_required", "two separated RGB-D observations are required")
    if previous_scene.held_object_id is not None:
        raise OutcomeValidationError("object_still_attached", "rest interval must begin detached")
    previous = previous_scene.objects[object_id]
    try:
        validator._validate_scene_age(previous_scene)
        validator._validate_observation(previous, previous_scene, "object")
    except SkillValidationError as error:
        raise OutcomeValidationError(error.code, str(error)) from error
    interval = observation.observed_at_s - previous.observed_at_s
    if not math.isfinite(interval) or interval < policy["minimum_rest_observation_s"]:
        raise OutcomeValidationError("rest_evidence_required", "rest observation interval is too short")
    speed = math.dist(previous.position_m, observation.position_m) / interval
    if not math.isfinite(speed) or speed > policy["maximum_rest_speed_m_s"]:
        raise OutcomeValidationError("object_not_resting", "object moved during the rest observation interval")
    return {"confidence": observation.confidence, "error_m": xy_error,
            "xy_error_m": xy_error, "z_error_m": z_error, "speed_m_s": speed,
            "rest_interval_s": interval, "detached": True,
            "object_id": object_id, "target_id": target_id, "status": "accepted"}
