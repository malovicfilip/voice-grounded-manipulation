"""Deterministic 3D placement acceptance using two fresh RGB-D observations."""

from __future__ import annotations

import math
import time
from collections.abc import Mapping, Sequence
from typing import Any

from .types import GroundedScene
from .validator import SkillValidator, SkillValidationError


class OutcomeValidationError(ValueError):
    def __init__(self, code: str, message: str) -> None:
        super().__init__(message)
        self.code = code


def _finite_pose(value: Any) -> tuple[float, float, float] | None:
    if (
        not isinstance(value, Sequence)
        or isinstance(value, (str, bytes, bytearray))
        or len(value) != 3
        or not all(type(item) in (int, float) and math.isfinite(item) for item in value)
    ):
        return None
    return tuple(float(item) for item in value)


def _validate_expected_target_pose(
    value: Any,
    validator: SkillValidator,
) -> tuple[float, float, float] | None:
    if value is None:
        return None
    pose = _finite_pose(value)
    if pose is None:
        raise OutcomeValidationError("invalid_target_pose", "invalid deterministic target pose")
    try:
        # This pose was measured from the execution-gate RGB-D scene. Recheck
        # the numeric/workspace invariant here without pretending it is a new
        # observation or minting any new motion authority.
        validator._validate_workspace_position(pose, "target")
    except SkillValidationError as error:
        raise OutcomeValidationError(error.code, str(error)) from error
    return pose


def validate_placement_outcome(
    execution: Mapping[str, Any], final_scene: GroundedScene, policy: Mapping[str, Any],
    *, previous_scene: GroundedScene | None = None, clock=time.time,
) -> dict[str, Any]:
    """Accept a placement only from fresh, detached, two-frame 3D evidence.

    The final colored target marker is normally re-observed and checked for
    drift. A narrow fallback exists for the expected case where the placed cube
    physically occludes the center of that marker: the fresh pre-execution
    target measurement may remain the geometric reference, but only when both
    post-place object observations overlap that reference closely enough to
    explain the occlusion and the object is verified detached and resting.
    Authored/spawn coordinates are never used by this function.
    """
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

    validator = SkillValidator(policy=policy, clock=clock)
    try:
        validator.validate_scene(final_scene)
        validator.validate_observation(observation, final_scene, "object")
    except SkillValidationError as error:
        raise OutcomeValidationError(
            "low_outcome_confidence" if error.code == "low_confidence" else error.code,
            str(error),
        ) from error

    expected = _validate_expected_target_pose(execution.get("target_position_m"), validator)
    target = final_scene.targets.get(target_id)
    target_reference: tuple[float, float, float]
    target_reference_source: str
    target_final_visibility: str

    if target is not None:
        try:
            validator.validate_observation(target, final_scene, "target")
        except SkillValidationError as error:
            raise OutcomeValidationError(
                "low_outcome_confidence" if error.code == "low_confidence" else error.code,
                str(error),
            ) from error
        if expected is not None and math.dist(expected, target.position_m) > policy["maximum_object_drift_m"]:
            raise OutcomeValidationError("target_moved", "target moved during placement")
        target_reference = target.position_m
        target_reference_source = "final_observation"
        target_final_visibility = "visible"
    else:
        if expected is None:
            raise OutcomeValidationError("target_not_observed", "final target observation is required")
        target_reference = expected
        target_reference_source = "pre_execution_observation"
        target_final_visibility = "occluded_by_placed_object"

        # If the first resting frame can still see the target, use that newer
        # measurement and prove it remained within the normal drift bound.
        if previous_scene is not None and target_id in previous_scene.targets:
            previous_target = previous_scene.targets[target_id]
            try:
                validator.validate_observation(previous_target, previous_scene, "target")
            except SkillValidationError as error:
                raise OutcomeValidationError(
                    "low_outcome_confidence" if error.code == "low_confidence" else error.code,
                    str(error),
                ) from error
            if math.dist(expected, previous_target.position_m) > policy["maximum_object_drift_m"]:
                raise OutcomeValidationError("target_moved", "target moved during placement")
            target_reference = previous_target.position_m
            target_reference_source = "rest_start_observation"

    xy_error = math.dist(observation.position_m[:2], target_reference[:2])
    z_error = abs(
        observation.position_m[2]
        - (target_reference[2] + policy["objects"][object_id]["size_m"] / 2)
    )
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
        validator.validate_observation(previous, previous_scene, "object")
    except SkillValidationError as error:
        raise OutcomeValidationError(error.code, str(error)) from error

    interval = observation.observed_at_s - previous.observed_at_s
    if not math.isfinite(interval) or interval < policy["minimum_rest_observation_s"]:
        raise OutcomeValidationError("rest_evidence_required", "rest observation interval is too short")
    speed = math.dist(previous.position_m, observation.position_m) / interval
    if not math.isfinite(speed) or speed > policy["maximum_rest_speed_m_s"]:
        raise OutcomeValidationError("object_not_resting", "object moved during the rest observation interval")

    occlusion_xy_error = None
    if target is None:
        # Missing-target acceptance is deliberately stricter than ordinary
        # placement tolerance. The cube must overlap the measured target center
        # in both rest frames closely enough for the cube itself to plausibly
        # explain why the colored marker disappeared from RGB-D segmentation.
        occlusion_limit = (
            float(policy["objects"][object_id]["size_m"]) / 2
            + float(policy["maximum_object_drift_m"])
        )
        current_overlap = math.dist(observation.position_m[:2], target_reference[:2])
        previous_overlap = math.dist(previous.position_m[:2], target_reference[:2])
        occlusion_xy_error = max(current_overlap, previous_overlap)
        if occlusion_xy_error > occlusion_limit:
            raise OutcomeValidationError(
                "target_occlusion_unverified",
                "missing target cannot be attributed to occlusion by the placed object",
            )

    return {
        "confidence": observation.confidence,
        "error_m": xy_error,
        "xy_error_m": xy_error,
        "z_error_m": z_error,
        "speed_m_s": speed,
        "rest_interval_s": interval,
        "detached": True,
        "object_id": object_id,
        "target_id": target_id,
        "status": "accepted",
        # Internal geometric evidence is intentionally retained in result.json
        # for deterministic completion checks and audit. semantic_result()
        # strips these coordinates before anything is sent back to the LLM.
        "object_position_m": list(observation.position_m),
        "target_reference_position_m": list(target_reference),
        "target_reference": target_reference_source,
        "target_final_visibility": target_final_visibility,
        "target_occlusion_xy_error_m": occlusion_xy_error,
    }
