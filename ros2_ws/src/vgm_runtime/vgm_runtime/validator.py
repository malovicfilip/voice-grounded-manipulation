"""Strict syntactic, semantic, grounding, and policy validation."""

from __future__ import annotations

import math
import re
import time
from jsonschema import Draft202012Validator
from collections.abc import Mapping
from typing import Any

from .config import load_json_config
from .types import GroundedScene, SkillProposal, ValidatedSkill


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
        Draft202012Validator.check_schema(self.schema)
        self._schema_validator = Draft202012Validator(self.schema)

    def _verify_configuration(self) -> None:
        if self.policy.get("policy_version") != 1:
            raise ValueError("policy_version must be 1")
        schema_skills = set(
            self.schema["properties"]["skill"]["enum"]
        )
        policy_skills = set(self.policy["allowed_skills"])
        if schema_skills != policy_skills:
            raise ValueError("schema and policy skill allowlists differ")
        for key in ("maximum_velocity_scale", "maximum_acceleration_scale", "maximum_scene_age_s",
                    "minimum_object_confidence", "maximum_placement_z_error_m", "minimum_rest_observation_s",
                    "maximum_rest_speed_m_s", "planning_time_s", "attached_object_clearance_m"):
            value = self.policy[key]
            if type(value) not in (int, float) or not math.isfinite(value) or value <= 0:
                raise ValueError(f"{key} must be finite and positive")
        if not self.policy["objects"] or not self.policy["targets"]:
            raise ValueError("object and target allowlists must not be empty")
        if self.policy["maximum_velocity_scale"] > 0.2:
            raise ValueError("velocity scale may not exceed 0.2")
        if self.policy["maximum_acceleration_scale"] > 0.2:
            raise ValueError("acceleration scale may not exceed 0.2")
        drift = self.policy.get("maximum_object_drift_m")
        if not isinstance(drift, (int, float)) or not 0.0 < drift <= 0.01:
            raise ValueError("maximum object drift must be in (0, 0.01]")
        execution_time = self.policy.get("maximum_execution_time_s")
        if (
            not isinstance(execution_time, (int, float))
            or not 0.0 < execution_time <= 120.0
        ):
            raise ValueError("maximum execution time must be in (0, 120]")
        placement_error = self.policy.get("maximum_placement_error_m")
        if (
            not isinstance(placement_error, (int, float))
            or not 0.0 < placement_error <= 0.06
        ):
            raise ValueError("maximum placement error must be in (0, 0.06]")

    def validate(
        self,
        raw_proposal: Mapping[str, Any],
        scene: GroundedScene | None,
    ) -> ValidatedSkill:
        """Validate a raw model proposal or raise before planning is possible."""
        self.validate_syntax(raw_proposal)
        proposal = SkillProposal.from_mapping(raw_proposal)
        self._validate_robot_state(proposal, scene)

        if proposal.request_id in self._accepted_request_ids and proposal.skill != "stop":
            raise SkillValidationError("replayed_request", "request_id was already accepted")

        if proposal.skill in {"pick", "place", "pick_and_place", "inspect"}:
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
        self.validate_syntax(proposal.to_mapping())
        self._validate_robot_state(proposal, latest_scene)
        if proposal.skill in {"pick", "place", "pick_and_place", "inspect"}:
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

    def validate_syntax(self, raw) -> None:
        # The checked-in schema is executed FIRST, even for nested control fields.
        error = next(self._schema_validator.iter_errors(raw), None)
        if error is None:
            return
        self._reject_direct_control_fields(raw)
        path = list(error.absolute_path)
        if not isinstance(raw, Mapping):
            code = "invalid_type"
        elif error.validator in {"required", "additionalProperties"}:
            code = "schema_fields"
        elif path and path[-1] == "scene_revision" and raw.get("scene_revision") is None:
            code = "scene_revision_required"
        elif "then" in error.absolute_schema_path:
            code = "invalid_parameters"
        else:
            code = str(path[-1]) if path else "schema_invalid"
        # Do not echo untrusted model values in safety errors.
        raise SkillValidationError(code, f"JSON Schema rejected {'.'.join(map(str, path)) or 'proposal'}")

    def _validate_robot_state(self, proposal, scene):
        skill = proposal.skill
        if skill in {"stop", "refuse"}:
            return
        if scene is None:
            raise SkillValidationError("scene_required", "fresh robot state is required")
        self._validate_scene_age(scene)
        held = scene.held_object_id
        if skill == "place" and held != proposal.object_id:
            raise SkillValidationError("not_holding_object", "place requires the same held object")
        if held is not None and skill != "place":
            raise SkillValidationError("object_already_held", "place the held object before another action")
        # A free-standing close has no validated grasp geometry. Only the
        # coordinator's pick sequence may close around an observed object.
        if skill == "close_gripper":
            raise SkillValidationError("grasp_context_required", "close_gripper requires a validated pick sequence")
        if proposal.pose_name is not None and proposal.pose_name not in self.policy["allowed_named_poses"]:
            raise SkillValidationError("pose_name", "named pose is not allowed by policy")

    def validate_scene(self, scene: GroundedScene | None) -> None:
        """Validate scene freshness without minting an executable skill.

        This is intentionally public so higher-level supervisors can use the
        exact same freshness rule when deciding which capabilities are safe to
        expose. It does not authorize motion.
        """
        if scene is None:
            raise SkillValidationError("scene_required", "fresh robot state is required")
        self._validate_scene_age(scene)

    def validate_observation(self, observation, scene: GroundedScene, label: str) -> None:
        """Validate one grounded observation using the canonical safety rules."""
        if label not in {"object", "target"}:
            raise ValueError("observation label must be object or target")
        self._validate_scene_age(scene)
        self._validate_observation(observation, scene, label)

    def _validate_scene_age(self, scene):
        age = float(self.clock()) - scene.captured_at_s
        if not math.isfinite(age) or not 0.0 <= age <= self.policy["maximum_scene_age_s"]:
            raise SkillValidationError("stale_scene", "grounded scene or robot state is stale")

    def _validate_observation(self, observation, scene, label):
        if observation.frame_id != "world":
            raise SkillValidationError("invalid_frame", f"{label} must be in the world frame")
        if not math.isfinite(observation.confidence) or not (
            self.policy["minimum_object_confidence"] <= observation.confidence <= 1.0
        ):
            raise SkillValidationError("low_confidence", f"{label} confidence is too low")
        age = scene.captured_at_s - observation.observed_at_s
        if not math.isfinite(age) or not 0.0 <= age <= self.policy["maximum_scene_age_s"]:
            raise SkillValidationError("stale_" + label, f"{label} observation is stale")
        self._validate_workspace_position(observation.position_m, label)

    def _validate_grounding(self, proposal, scene):
        if scene is None:
            raise SkillValidationError("scene_required", "grounded scene is required")
        if proposal.scene_revision != scene.revision:
            raise SkillValidationError("stale_revision", "scene revision does not match")
        if proposal.object_id not in self.policy["objects"]:
            raise SkillValidationError("object_not_grounded", "object is not allowlisted")
        if not (proposal.skill == "place" and scene.held_object_id == proposal.object_id):
            observation = scene.objects.get(proposal.object_id)
            if observation is None:
                raise SkillValidationError("object_not_grounded", "object is not grounded")
            self._validate_observation(observation, scene, "object")
        if proposal.skill in {"place", "pick_and_place"}:
            target = scene.targets.get(proposal.target_id)
            if proposal.target_id not in self.policy["targets"] or target is None:
                raise SkillValidationError("target_not_grounded", "fresh perceived target is required")
            self._validate_observation(target, scene, "target")

    def _validate_workspace_position(
        self, position: tuple[float, float, float], label: str
    ) -> None:
        if len(position) != 3 or not all(
            type(value) in (int, float) and math.isfinite(value)
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
