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


def rebind_after_capture(
    proposal: Mapping[str, Any], original: GroundedScene,
    latest: GroundedScene, validator: SkillValidator,
) -> dict[str, Any]:
    """Refresh a revision only after independently comparing measured geometry.

    Hash bins can differ for submillimeter noise at a rounding boundary. Missing
    objects, changed holding state, stale observations, or movement beyond the
    existing one-centimeter limit still reject the entire request before motion.
    """
    baseline_age = validator.clock() - original.captured_at_s
    if not math.isfinite(baseline_age) or not 0 <= baseline_age <= 120.0:
        raise SkillValidationError("stale_baseline", "comparison observation is expired")
    # The original capture is historical comparison evidence. Validate it at
    # its capture time; only the new observation may authorize motion now.
    SkillValidator(policy=validator.policy, schema=validator.schema,
                   clock=lambda: original.captured_at_s).validate(proposal, original)
    if proposal.get("scene_revision") != original.revision:
        raise SkillValidationError("stale_revision", "proposal does not match its original observation")
    if original.held_object_id != latest.held_object_id or set(original.objects) != set(latest.objects):
        raise SkillValidationError("scene_changed", "object identities or holding state changed")
    for object_id, observation in original.objects.items():
        drift = math.dist(observation.position_m, latest.objects[object_id].position_m)
        if not math.isfinite(drift) or drift > validator.policy["maximum_object_drift_m"]:
            raise SkillValidationError("object_moved", "scene geometry moved between captures")
    if set(original.targets) != set(latest.targets):
        raise SkillValidationError("target_not_grounded", "target identities changed between captures")
    for target_id, observation in original.targets.items():
        drift = math.dist(observation.position_m, latest.targets[target_id].position_m)
        if not math.isfinite(drift) or drift > validator.policy["maximum_object_drift_m"]:
            raise SkillValidationError("target_moved", "target moved between captures")
    refreshed = {**proposal, "scene_revision": latest.revision}
    SkillValidator(policy=validator.policy, schema=validator.schema, clock=validator.clock).validate(refreshed, latest)
    return refreshed


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
    if proposal.skill in {"place", "pick_and_place"}:
        primitives = decision.get("plan", {}).get("primitives", [])
        motions = [p.get("position_m") for p in primitives
                   if isinstance(p, Mapping) and p.get("kind") == "move_cartesian"]
        expected_count = 6 if proposal.skill == "pick_and_place" else 3
        if len(motions) != expected_count or not isinstance(motions[-3], list) or len(motions[-3]) != 3:
            raise SkillValidationError("plan_invalid", "original placement motion is missing")
        position = motions[-3]
        if not all(type(v) in (int, float) and math.isfinite(v) for v in position):
            raise SkillValidationError("plan_invalid", "invalid original placement motion")
        original_target = (position[0], position[1], position[2] - validator.policy["place"]["approach_height_m"])
        if math.dist(original_target, latest_scene.targets[proposal.target_id].position_m) > validator.policy["maximum_object_drift_m"]:
            raise SkillValidationError("target_moved", "target moved after intent validation")
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
