"""Strict syntactic, semantic, grounding, and policy validation."""

from __future__ import annotations

import math
import re
import time
from collections.abc import Mapping
from typing import Any

from .config import load_json_config
from .types import GroundedScene, SkillProposal, ValidatedSkill


_IDENTIFIER = re.compile(r"^[a-z][a-z0-9_]{0,63}$")
_REQUEST_ID = re.compile(r"^[a-zA-Z0-9][a-zA-Z0-9_-]{0,63}$")
_SCENE_REVISION = re.compile(r"^[a-f0-9]{16}$")
_DIRECT_CONTROL_TOKENS = frozenset(
    {"joint", "joints", "velocity", "velocities", "motor", "motors", "effort", "efforts", "torque", "torques", "trajectory", "trajectories"}
)


class SkillValidationError(ValueError):
    """A proposal was refused before any motion-planning request."""

    def __init__(self, code: str, message: str) -> None:
        super().__init__(message)
        self.code = code


def _mapping_keys(value: Any) -> set[str]:
    keys: set[str] = set()
    if isinstance(value, Mapping):
        for key, child in value.items():
            keys.add(str(key))
            keys.update(_mapping_keys(child))
    elif isinstance(value, list):
        for child in value:
            keys.update(_mapping_keys(child))
    return keys


class SkillValidator:
    """The only component allowed to mint :class:`ValidatedSkill` values."""

    def __init__(
        self,
        policy: Mapping[str, Any] | None = None,
        schema: Mapping[str, Any] | None = None,
        *,
        clock=time.time,
    ) -> None:
        self.policy = dict(policy or load_json_config("safety_policy.json"))
        self.schema = dict(schema or load_json_config("robot_skill.schema.json"))
        self.clock = clock
        self._accepted_request_ids: set[str] = set()
        self._verify_configuration()

    def _verify_configuration(self) -> None:
        if self.policy.get("policy_version") != 1:
            raise ValueError("policy_version must be 1")
        schema_skills = set(
            self.schema["properties"]["skill"]["enum"]
        )
        policy_skills = set(self.policy["allowed_skills"])
        if schema_skills != policy_skills:
            raise ValueError("schema and policy skill allowlists differ")
        if self.policy["maximum_velocity_scale"] > 0.2:
            raise ValueError("velocity scale may not exceed 0.2")
        if self.policy["maximum_acceleration_scale"] > 0.2:
            raise ValueError("acceleration scale may not exceed 0.2")
        drift = self.policy.get("maximum_object_drift_m")
        if not isinstance(drift, (int, float)) or not 0.0 < drift <= 0.01:
            raise ValueError("maximum object drift must be in (0, 0.01]")

    def validate(
        self,
        raw_proposal: Mapping[str, Any],
        scene: GroundedScene | None,
    ) -> ValidatedSkill:
        """Validate a raw model proposal or raise before planning is possible."""
        if not isinstance(raw_proposal, Mapping):
            raise SkillValidationError("invalid_type", "proposal must be an object")

        self._reject_direct_control_fields(raw_proposal)
        supplied_fields = set(raw_proposal)
        expected_fields = SkillProposal.field_names()
        if supplied_fields != expected_fields:
            missing = sorted(expected_fields - supplied_fields)
            extra = sorted(supplied_fields - expected_fields)
            raise SkillValidationError(
                "schema_fields",
                f"proposal fields differ; missing={missing}, extra={extra}",
            )

        proposal = SkillProposal.from_mapping(raw_proposal)
        self._validate_scalar_types(proposal)
        self._validate_field_combination(proposal)

        if proposal.request_id in self._accepted_request_ids and proposal.skill != "stop":
            raise SkillValidationError("replayed_request", "request_id was already accepted")

        if proposal.skill in {"pick", "place", "pick_and_place"}:
            self._validate_grounding(proposal, scene)

        validated_at = float(self.clock())
        if proposal.skill != "stop":
            self._accepted_request_ids.add(proposal.request_id)
        return ValidatedSkill(
            proposal=proposal,
            policy_version=self.policy["policy_version"],
            validated_at_s=validated_at,
        )

    def revalidate(
        self,
        validated: ValidatedSkill,
        latest_scene: GroundedScene | None,
    ) -> ValidatedSkill:
        """Recheck a minted skill against the latest scene before execution."""
        if not isinstance(validated, ValidatedSkill):
            raise SkillValidationError(
                "unvalidated_skill", "execution requires a validator-minted skill"
            )
        if validated.policy_version != self.policy["policy_version"]:
            raise SkillValidationError(
                "policy_changed", "the safety policy changed after validation"
            )
        proposal = validated.proposal
        self._validate_scalar_types(proposal)
        self._validate_field_combination(proposal)
        if proposal.skill in {"pick", "place", "pick_and_place"}:
            self._validate_grounding(proposal, latest_scene)
        return ValidatedSkill(
            proposal=proposal,
            policy_version=validated.policy_version,
            validated_at_s=float(self.clock()),
        )

    def _reject_direct_control_fields(self, value: Mapping[str, Any]) -> None:
        configured = {str(field).lower() for field in self.policy["forbidden_fields"]}
        found: set[str] = set()
        for key in _mapping_keys(value):
            lowered = key.lower()
            tokens = set(re.split(r"[^a-z]+", lowered))
            if lowered in configured or tokens & _DIRECT_CONTROL_TOKENS:
                found.add(key)
        if found:
            raise SkillValidationError(
                "forbidden_control_field",
                f"direct-control fields are forbidden: {', '.join(sorted(found))}",
            )

    def _validate_scalar_types(self, proposal: SkillProposal) -> None:
        if type(proposal.schema_version) is not int or proposal.schema_version != 1:
            raise SkillValidationError("schema_version", "schema_version must be integer 1")
        if not isinstance(proposal.request_id, str) or not _REQUEST_ID.fullmatch(
            proposal.request_id
        ):
            raise SkillValidationError("request_id", "request_id is invalid")
        if not isinstance(proposal.skill, str) or proposal.skill not in self.policy["allowed_skills"]:
            raise SkillValidationError("skill_not_allowed", "skill is not allowlisted")

        for field_name in ("object_id", "target_id"):
            value = getattr(proposal, field_name)
            if value is not None and (
                not isinstance(value, str) or not _IDENTIFIER.fullmatch(value)
            ):
                raise SkillValidationError(field_name, f"{field_name} is invalid")

        if proposal.pose_name is not None and (
            not isinstance(proposal.pose_name, str)
            or proposal.pose_name not in self.policy["allowed_named_poses"]
        ):
            raise SkillValidationError("pose_name", "pose_name is not allowlisted")
        if proposal.reason is not None and (
            not isinstance(proposal.reason, str)
            or not proposal.reason.strip()
            or len(proposal.reason) > 240
        ):
            raise SkillValidationError("reason", "reason must be 1 to 240 characters")
        if proposal.scene_revision is not None and (
            not isinstance(proposal.scene_revision, str)
            or not _SCENE_REVISION.fullmatch(proposal.scene_revision)
        ):
            raise SkillValidationError("scene_revision", "scene_revision is invalid")

    def _validate_field_combination(self, proposal: SkillProposal) -> None:
        empty = {
            "object_id": proposal.object_id is None,
            "target_id": proposal.target_id is None,
            "pose_name": proposal.pose_name is None,
            "reason": proposal.reason is None,
        }
        skill = proposal.skill
        valid = False
        if skill == "move_named_pose":
            valid = empty["object_id"] and empty["target_id"] and not empty["pose_name"] and empty["reason"]
        elif skill in {"open_gripper", "close_gripper"}:
            valid = all(empty.values())
        elif skill == "pick":
            valid = not empty["object_id"] and empty["target_id"] and empty["pose_name"] and empty["reason"]
        elif skill in {"place", "pick_and_place"}:
            valid = not empty["object_id"] and not empty["target_id"] and empty["pose_name"] and empty["reason"]
        elif skill in {"stop", "refuse"}:
            valid = empty["object_id"] and empty["target_id"] and empty["pose_name"] and not empty["reason"]
        if not valid:
            raise SkillValidationError(
                "invalid_parameters", f"fields are invalid for skill {skill}"
            )
        if skill in {"pick", "place", "pick_and_place"} and proposal.scene_revision is None:
            raise SkillValidationError(
                "scene_revision_required", "grounded skills require a scene revision"
            )

    def _validate_grounding(
        self,
        proposal: SkillProposal,
        scene: GroundedScene | None,
    ) -> None:
        if scene is None:
            raise SkillValidationError("scene_required", "grounded scene is required")
        if proposal.scene_revision != scene.revision:
            raise SkillValidationError("stale_revision", "scene revision does not match")

        age = float(self.clock()) - scene.captured_at_s
        if age < 0.0 or age > self.policy["maximum_scene_age_s"]:
            raise SkillValidationError("stale_scene", "grounded scene is stale")

        observation = scene.objects.get(proposal.object_id or "")
        if observation is None:
            raise SkillValidationError("object_not_grounded", "object is not grounded")
        if observation.confidence < self.policy["minimum_object_confidence"]:
            raise SkillValidationError("low_confidence", "object confidence is too low")
        if scene.captured_at_s - observation.observed_at_s > self.policy["maximum_scene_age_s"]:
            raise SkillValidationError("stale_object", "object observation is stale")
        self._validate_workspace_position(observation.position_m, "object")

        if proposal.skill in {"place", "pick_and_place"}:
            target = self.policy["targets"].get(proposal.target_id)
            if target is None:
                raise SkillValidationError("target_not_grounded", "target is not allowlisted")
            self._validate_workspace_position(tuple(target["position_m"]), "target")

    def _validate_workspace_position(
        self, position: tuple[float, float, float], label: str
    ) -> None:
        if len(position) != 3 or not all(
            isinstance(value, (int, float)) and math.isfinite(value)
            for value in position
        ):
            raise SkillValidationError("invalid_position", f"{label} position is invalid")
        workspace = self.policy["workspace_m"]
        for axis, value in zip(("x", "y", "z"), position, strict=True):
            lower, upper = workspace[axis]
            if value < lower or value > upper:
                raise SkillValidationError(
                    "outside_workspace", f"{label} is outside the {axis} workspace bound"
                )
