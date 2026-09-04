"""Final independent revalidation gate for a previously accepted decision."""

from __future__ import annotations

from collections.abc import Mapping
import math
from typing import Any

from .coordinator import TaskCoordinator
from .serialization import plan_to_mapping
from .types import GroundedScene, SkillProposal, ValidatedSkill
from .validator import SkillValidationError, SkillValidator


_VALIDATED_METADATA = {
    "policy_version",
    "validated_at_s",
    "validation_status",
}


def gate_decision(
    decision: Mapping[str, Any],
    latest_scene: GroundedScene,
    validator: SkillValidator,
) -> dict[str, Any]:
    """Return a fresh deterministic plan or refuse before MoveIt can run."""
    if not isinstance(decision, Mapping) or decision.get("accepted") is not True:
        raise SkillValidationError("decision_not_accepted", "decision is not accepted")
    raw_validated = decision.get("validated_skill")
    if not isinstance(raw_validated, Mapping):
        raise SkillValidationError("validated_skill_missing", "validated skill is missing")
    expected_fields = set(SkillProposal.field_names()) | _VALIDATED_METADATA
    if set(raw_validated) != expected_fields:
        raise SkillValidationError(
            "validated_skill_fields", "validated skill fields are invalid"
        )
    if raw_validated.get("validation_status") != "accepted":
        raise SkillValidationError(
            "validation_status", "validated skill status is not accepted"
        )
    if type(raw_validated.get("policy_version")) is not int:
        raise SkillValidationError("policy_version", "policy version is invalid")
    if not isinstance(raw_validated.get("validated_at_s"), (int, float)):
        raise SkillValidationError("validated_at", "validation timestamp is invalid")

    proposal = SkillProposal.from_mapping(raw_validated)
    validated = ValidatedSkill(
        proposal=proposal,
        policy_version=raw_validated["policy_version"],
        validated_at_s=float(raw_validated["validated_at_s"]),
    )
    refreshed = validator.revalidate(validated, latest_scene)
    if proposal.skill in {"pick", "pick_and_place"}:
        _reject_object_drift(
            decision.get("plan"), proposal.object_id, latest_scene, validator
        )
    plan = TaskCoordinator(validator.policy).create_plan(refreshed, latest_scene)
    return {
        "validated_skill": refreshed.to_mapping(),
        "plan": plan_to_mapping(plan),
    }


def _reject_object_drift(
    original_plan: Any,
    object_id: str | None,
    latest_scene: GroundedScene,
    validator: SkillValidator,
) -> None:
    """Compare the original pickup origin with the newest RGB-D observation."""
    if not isinstance(original_plan, Mapping):
        raise SkillValidationError("plan_missing", "original deterministic plan is missing")
    primitives = original_plan.get("primitives")
    if not isinstance(primitives, list):
        raise SkillValidationError("plan_invalid", "original plan primitives are invalid")
    first_motion = next(
        (
            primitive
            for primitive in primitives
            if isinstance(primitive, Mapping)
            and primitive.get("kind") == "move_cartesian"
        ),
        None,
    )
    if first_motion is None:
        raise SkillValidationError("plan_invalid", "original pickup motion is missing")
    position = first_motion.get("position_m")
    if not isinstance(position, list) or len(position) != 3:
        raise SkillValidationError("plan_invalid", "original pickup position is invalid")
    try:
        original = (
            float(position[0]),
            float(position[1]),
            float(position[2]) - float(validator.policy["pick"]["approach_height_m"]),
        )
    except (TypeError, ValueError) as error:
        raise SkillValidationError(
            "plan_invalid", "original pickup position is not numeric"
        ) from error
    latest = latest_scene.objects.get(object_id or "")
    if latest is None:
        raise SkillValidationError("object_not_grounded", "object is not grounded")
    drift = math.dist(original, latest.position_m)
    if not math.isfinite(drift) or drift > validator.policy["maximum_object_drift_m"]:
        raise SkillValidationError(
            "object_moved", "object moved after the original intent decision"
        )
