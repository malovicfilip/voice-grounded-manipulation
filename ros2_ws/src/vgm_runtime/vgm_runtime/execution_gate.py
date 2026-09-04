"""Final independent revalidation gate for a previously accepted decision."""

from __future__ import annotations

from collections.abc import Mapping
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
    plan = TaskCoordinator(validator.policy).create_plan(refreshed, latest_scene)
    return {
        "validated_skill": refreshed.to_mapping(),
        "plan": plan_to_mapping(plan),
    }
