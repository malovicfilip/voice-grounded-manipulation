"""Closed-loop LLM task control through a deterministic capability gateway.

The agent never receives Cartesian poses, joint state, trajectories, or motion
limits. It receives a semantic observation and can request one allowlisted
capability at a time. Every executable capability is converted back into the
existing robot-skill schema and passes the same validator/backend execution
path used by confirmed planned tasks.
"""

from __future__ import annotations

import hashlib
import json
import math
import re
import threading
import time
from dataclasses import dataclass
from typing import Any, Callable, Mapping, Protocol

from jsonschema import Draft202012Validator, ValidationError

from .audit import AuditLogger
from .config import load_json_config
from .openai_intent import OpenAIIntentModel
from .stop_intent import is_stop_request
from .tasks import TaskBackend, check_preconditions
from .types import GroundedScene
from .validator import SkillValidationError, SkillValidator


EXECUTABLE_CAPABILITIES = frozenset(
    {"inspect", "pick", "place", "pick_and_place", "move_named_pose"}
)
TERMINAL_CAPABILITIES = frozenset({"finish", "request_human_help", "stop"})
NO_MOTION_CAPABILITIES = frozenset({"observe_workspace"}) | TERMINAL_CAPABILITIES


class AgentValidationError(ValueError):
    """A mission or agent action was rejected before execution."""

    def __init__(self, code: str, message: str) -> None:
        super().__init__(message)
        self.code = code


def mission_schema() -> dict[str, Any]:
    """Schema for the operator-reviewed semantic authority envelope."""
    return {
        "type": "object",
        "additionalProperties": False,
        "required": [
            "schema_version", "request_id", "scene_revision", "goal_summary",
            "object_ids", "target_ids", "pose_names", "max_actions", "success_conditions",
        ],
        "properties": {
            "schema_version": {"type": "integer", "const": 1},
            "request_id": {"type": "string", "pattern": "^[a-zA-Z0-9][a-zA-Z0-9_-]{0,63}$"},
            "scene_revision": {"type": "string", "pattern": "^[a-f0-9]{16}$"},
            "goal_summary": {"type": "string", "minLength": 1, "maxLength": 500, "pattern": "\\S"},
            "object_ids": {
                "type": "array", "uniqueItems": True, "maxItems": 6,
                "items": {"type": "string", "pattern": "^[a-z][a-z0-9_]{0,63}$"},
            },
            "target_ids": {
                "type": "array", "uniqueItems": True, "maxItems": 4,
                "items": {"type": "string", "pattern": "^[a-z][a-z0-9_]{0,63}$"},
            },
            "pose_names": {
                "type": "array", "uniqueItems": True, "maxItems": 8,
                "items": {"type": "string", "pattern": "^[a-z][a-z0-9_]{0,63}$"},
            },
            "max_actions": {"type": "integer", "minimum": 1, "maximum": 10},
            "success_conditions": {
                "type": "array",
                "minItems": 1,
                "maxItems": 8,
                "items": {
                    "type": "object",
                    "additionalProperties": False,
                    "required": ["kind", "object_id", "target_id", "pose_name"],
                    "properties": {
                        "kind": {
                            "type": "string",
                            "enum": [
                                "object_on_target", "object_inspected",
                                "workspace_observed", "named_pose_reached",
                            ],
                        },
                        "object_id": {"type": ["string", "null"], "pattern": "^[a-z][a-z0-9_]{0,63}$"},
                        "target_id": {"type": ["string", "null"], "pattern": "^[a-z][a-z0-9_]{0,63}$"},
                        "pose_name": {"type": ["string", "null"], "pattern": "^[a-z][a-z0-9_]{0,63}$"},
                    },
                },
            },
        },
    }


def action_schema() -> dict[str, Any]:
    """Schema for one semantic capability decision."""
    capabilities = [
        "observe_workspace", "inspect", "pick", "place", "pick_and_place",
        "move_named_pose", "finish", "request_human_help", "stop",
    ]
    schema: dict[str, Any] = {
        "type": "object",
        "additionalProperties": False,
        "required": [
            "schema_version", "request_id", "scene_revision", "capability",
            "object_id", "target_id", "pose_name", "reason",
        ],
        "properties": {
            "schema_version": {"type": "integer", "const": 1},
            "request_id": {"type": "string", "pattern": "^[a-zA-Z0-9][a-zA-Z0-9_-]{0,63}$"},
            "scene_revision": {"type": "string", "pattern": "^[a-f0-9]{16}$"},
            "capability": {"type": "string", "enum": capabilities},
            "object_id": {"type": ["string", "null"], "pattern": "^[a-z][a-z0-9_]{0,63}$"},
            "target_id": {"type": ["string", "null"], "pattern": "^[a-z][a-z0-9_]{0,63}$"},
            "pose_name": {"type": ["string", "null"], "pattern": "^[a-z][a-z0-9_]{0,63}$"},
            "reason": {"type": "string", "minLength": 1, "maxLength": 240, "pattern": "\\S"},
        },
    }
    branches = []
    for capability in capabilities:
        required = {
            "object_id": "string" if capability in {"inspect", "pick", "place", "pick_and_place"} else "null",
            "target_id": "string" if capability in {"place", "pick_and_place"} else "null",
            "pose_name": "string" if capability == "move_named_pose" else "null",
        }
        branches.append({
            "if": {"properties": {"capability": {"const": capability}}},
            "then": {"properties": {name: {"type": kind} for name, kind in required.items()}},
        })
    schema["allOf"] = branches
    return schema


class OpenAIMissionModel(OpenAIIntentModel):
    """Create the finite semantic authority envelope that the operator reviews."""

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(
            schema=mission_schema(), response_name="robot_agent_mission", max_output_tokens=900,
            instructions=(
                "Convert the operator request into a semantic robot mission authority envelope. "
                "Copy request_id and scene_revision exactly. Select only IDs present in the supplied "
                "available lists. Include only objects, targets, and named poses genuinely needed for "
                "the requested goal. Encode one or more success_conditions that can be checked by the "
                "deterministic supervisor: object_on_target for transfers, object_inspected for inspection, "
                "workspace_observed for an observation-only goal, or named_pose_reached for a named pose. "
                "Each condition must use only IDs in the mission scope. max_actions is a small upper bound "
                "for closed-loop decisions, not a motion parameter; prefer 4-6 and never exceed 10. "
                "Never emit coordinates, joint data, "
                "velocities, motors, torques, trajectories, or safety limits. goal_summary must describe "
                "what the operator asked for, not invent a broader goal."
            ),
            **kwargs,
        )


class OpenAIAgentActionModel(OpenAIIntentModel):
    """Choose exactly one currently offered semantic capability."""

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(
            schema=action_schema(), response_name="robot_agent_action", max_output_tokens=700,
            instructions=(
                "You are the high-level closed-loop task controller. The transcript field contains a "
                "JSON semantic observation with the confirmed mission, prior result and a list named "
                "allowed_capabilities. Choose exactly one capability from that list and copy the current "
                "scene_revision exactly. Use only IDs explicitly offered for that capability. Prefer "
                "pick_and_place over a separate pick/place when both satisfy the goal. Finish only when "
                "the confirmed goal is semantically satisfied. Request human help when the safe offered "
                "capabilities cannot complete the goal. Never invent or emit coordinates, joints, motor "
                "values, velocities, torques, trajectories, safety limits, or unoffered actions. The "
                "supervisor offers finish only after every operator-confirmed completion condition is "
                "deterministically satisfied."
            ),
            **kwargs,
        )


@dataclass(frozen=True, slots=True)
class CompletionCondition:
    kind: str
    object_id: str | None = None
    target_id: str | None = None
    pose_name: str | None = None

    @classmethod
    def from_mapping(cls, value: Mapping[str, Any]) -> "CompletionCondition":
        return cls(
            kind=str(value["kind"]),
            object_id=value.get("object_id"),
            target_id=value.get("target_id"),
            pose_name=value.get("pose_name"),
        )

    def to_mapping(self) -> dict[str, Any]:
        return {
            "kind": self.kind,
            "object_id": self.object_id,
            "target_id": self.target_id,
            "pose_name": self.pose_name,
        }


@dataclass(frozen=True, slots=True)
class Mission:
    request_id: str
    scene_revision: str
    goal_summary: str
    object_ids: tuple[str, ...]
    target_ids: tuple[str, ...]
    pose_names: tuple[str, ...]
    max_actions: int
    success_conditions: tuple[CompletionCondition, ...] = ()

    @classmethod
    def from_mapping(cls, value: Mapping[str, Any]) -> "Mission":
        return cls(
            request_id=str(value["request_id"]),
            scene_revision=str(value["scene_revision"]),
            goal_summary=str(value["goal_summary"]),
            object_ids=tuple(value["object_ids"]),
            target_ids=tuple(value["target_ids"]),
            pose_names=tuple(value["pose_names"]),
            max_actions=int(value["max_actions"]),
            success_conditions=tuple(
                CompletionCondition.from_mapping(item) for item in value["success_conditions"]
            ),
        )

    def to_mapping(self) -> dict[str, Any]:
        return {
            "request_id": self.request_id,
            "scene_revision": self.scene_revision,
            "goal_summary": self.goal_summary,
            "object_ids": list(self.object_ids),
            "target_ids": list(self.target_ids),
            "pose_names": list(self.pose_names),
            "max_actions": self.max_actions,
            "success_conditions": [item.to_mapping() for item in self.success_conditions],
        }


@dataclass(frozen=True, slots=True)
class PendingMission:
    mission: Mission
    confirmation: str
    expires_at_s: float


class AgentActionModel(Protocol):
    def propose(self, transcript: str, scene: GroundedScene, policy: Mapping[str, Any]) -> dict[str, Any]: ...


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


def _reject_control_fields(value: Mapping[str, Any], safety_policy: Mapping[str, Any]) -> None:
    forbidden = {str(name).lower() for name in safety_policy["forbidden_fields"]}
    direct_tokens = {"joint", "joints", "velocity", "velocities", "motor", "motors", "effort", "efforts", "torque", "torques", "trajectory", "trajectories"}
    found = []
    for key in _mapping_keys(value):
        lowered = key.lower()
        tokens = set(filter(None, re.split(r"[^a-z]+", lowered)))
        if lowered in forbidden or tokens & direct_tokens:
            found.append(key)
    if found:
        raise AgentValidationError("forbidden_control_field", "direct-control fields are forbidden")


def _validate_schema(value: Mapping[str, Any], schema: Mapping[str, Any], safety_policy: Mapping[str, Any]) -> None:
    try:
        Draft202012Validator(schema).validate(value)
    except ValidationError as error:
        _reject_control_fields(value, safety_policy)
        raise AgentValidationError("schema_invalid", "agent JSON failed schema validation") from error


def validate_mission(
    raw: Mapping[str, Any],
    scene: GroundedScene,
    safety_policy: Mapping[str, Any],
    agent_policy: Mapping[str, Any],
    *,
    now_s: float | None = None,
) -> Mission:
    if not isinstance(raw, Mapping):
        raise AgentValidationError("schema_invalid", "mission proposal must be an object")
    _validate_schema(raw, mission_schema(), safety_policy)
    _reject_control_fields(raw, safety_policy)
    if raw["scene_revision"] != scene.revision:
        raise AgentValidationError("stale_revision", "mission proposal changed the scene revision")
    validator = SkillValidator(policy=safety_policy, clock=_validation_clock(now_s))
    if not _scene_is_usable(scene, safety_policy, now_s=now_s, validator=validator):
        raise AgentValidationError("stale_scene", "mission requires a fresh grounded scene")
    if raw["max_actions"] > agent_policy["maximum_actions"]:
        raise AgentValidationError("action_budget", "mission action budget exceeds agent policy")
    if not set(raw["object_ids"]) <= (set(scene.objects) & set(safety_policy["objects"])):
        raise AgentValidationError("object_scope", "mission includes an unavailable object")
    if any(
        not _observation_is_usable(
            scene.objects.get(object_id), scene, safety_policy, "object", now_s=now_s, validator=validator
        )
        for object_id in raw["object_ids"]
    ):
        raise AgentValidationError("object_scope", "mission includes an object without usable fresh grounding")
    if not set(raw["target_ids"]) <= (set(scene.targets) & set(safety_policy["targets"])):
        raise AgentValidationError("target_scope", "mission includes an unavailable target")
    if any(
        not _observation_is_usable(
            scene.targets.get(target_id), scene, safety_policy, "target", now_s=now_s, validator=validator
        )
        for target_id in raw["target_ids"]
    ):
        raise AgentValidationError("target_scope", "mission includes a target without usable fresh grounding")
    if not set(raw["pose_names"]) <= set(safety_policy["allowed_named_poses"]):
        raise AgentValidationError("pose_scope", "mission includes a disallowed named pose")
    for condition in raw["success_conditions"]:
        kind = condition["kind"]
        object_id, target_id, pose_name = condition["object_id"], condition["target_id"], condition["pose_name"]
        expected = {
            "object_on_target": (True, True, False),
            "object_inspected": (True, False, False),
            "workspace_observed": (False, False, False),
            "named_pose_reached": (False, False, True),
        }[kind]
        present = (object_id is not None, target_id is not None, pose_name is not None)
        if present != expected:
            raise AgentValidationError("completion_condition", f"invalid fields for completion condition {kind}")
        if object_id is not None and object_id not in raw["object_ids"]:
            raise AgentValidationError("completion_scope", "completion condition object is outside the confirmed mission")
        if target_id is not None and target_id not in raw["target_ids"]:
            raise AgentValidationError("completion_scope", "completion condition target is outside the confirmed mission")
        if pose_name is not None and pose_name not in raw["pose_names"]:
            raise AgentValidationError("completion_scope", "completion condition pose is outside the confirmed mission")
    return Mission.from_mapping(raw)


def _validation_clock(now_s: float | None) -> Callable[[], float]:
    fixed = float(time.time() if now_s is None else now_s)
    return lambda: fixed


def _scene_is_usable(
    scene: GroundedScene,
    safety_policy: Mapping[str, Any],
    *,
    now_s: float | None = None,
    validator: SkillValidator | None = None,
) -> bool:
    try:
        (validator or SkillValidator(policy=safety_policy, clock=_validation_clock(now_s))).validate_scene(scene)
    except (SkillValidationError, ValueError, TypeError, OverflowError):
        return False
    return True


def _observation_is_usable(
    observation: Any,
    scene: GroundedScene,
    safety_policy: Mapping[str, Any],
    label: str,
    *,
    now_s: float | None = None,
    validator: SkillValidator | None = None,
) -> bool:
    if observation is None:
        return False
    try:
        (validator or SkillValidator(policy=safety_policy, clock=_validation_clock(now_s))).validate_observation(
            observation, scene, label
        )
    except (SkillValidationError, ValueError, TypeError, OverflowError):
        return False
    return True


def _occupancy(
    scene: GroundedScene,
    target_id: str,
    safety_policy: Mapping[str, Any],
    *,
    exclude: str | None = None,
    now_s: float | None = None,
    validator: SkillValidator | None = None,
) -> str | None:
    target = scene.targets.get(target_id)
    validator = validator or SkillValidator(policy=safety_policy, clock=_validation_clock(now_s))
    if not _observation_is_usable(
        target, scene, safety_policy, "target", now_s=now_s, validator=validator
    ):
        return None
    clearance = float(safety_policy["target_occupancy_clearance_m"])
    for object_id in sorted(scene.objects):
        if object_id == exclude:
            continue
        observation = scene.objects[object_id]
        if not _observation_is_usable(
            observation, scene, safety_policy, "object", now_s=now_s, validator=validator
        ):
            continue
        if math.dist(observation.position_m[:2], target.position_m[:2]) < clearance:
            return object_id
    return None


def _completion_marker(kind: str, value: str | None = None) -> str:
    return f"{kind}:{value}" if value is not None else kind


def completion_status(
    mission: Mission,
    scene: GroundedScene,
    safety_policy: Mapping[str, Any],
    completed_markers: set[str] | frozenset[str],
    *,
    verified_placements: Mapping[tuple[str, str], tuple[float, float, float]] | None = None,
    now_s: float | None = None,
    _validator: SkillValidator | None = None,
) -> list[dict[str, Any]]:
    """Evaluate the operator-confirmed completion contract deterministically."""
    validator = _validator or SkillValidator(policy=safety_policy, clock=_validation_clock(now_s))
    status: list[dict[str, Any]] = []
    scene_usable = _scene_is_usable(scene, safety_policy, now_s=now_s, validator=validator)
    for condition in mission.success_conditions:
        satisfied = False
        if condition.kind == "object_on_target":
            object_observation = scene.objects.get(condition.object_id) if condition.object_id else None
            target_observation = scene.targets.get(condition.target_id) if condition.target_id else None
            object_usable = (
                scene_usable
                and scene.held_object_id != condition.object_id
                and _observation_is_usable(
                    object_observation, scene, safety_policy, "object", now_s=now_s, validator=validator
                )
            )
            visible_target_satisfied = (
                object_usable
                and _observation_is_usable(
                    target_observation, scene, safety_policy, "target", now_s=now_s, validator=validator
                )
                and _occupancy(
                    scene, condition.target_id, safety_policy, now_s=now_s, validator=validator
                ) == condition.object_id
            )
            verified_satisfied = False
            if object_usable and condition.object_id and condition.target_id and verified_placements:
                verified_position = verified_placements.get((condition.object_id, condition.target_id))
                if verified_position is not None:
                    verified_satisfied = (
                        len(verified_position) == 3
                        and all(type(value) in (int, float) and math.isfinite(value) for value in verified_position)
                        and math.dist(object_observation.position_m, verified_position)
                        <= float(safety_policy["maximum_object_drift_m"])
                    )
            satisfied = visible_target_satisfied or verified_satisfied
        elif condition.kind == "object_inspected":
            satisfied = scene_usable and _completion_marker("object_inspected", condition.object_id) in completed_markers
        elif condition.kind == "workspace_observed":
            satisfied = scene_usable and _completion_marker("workspace_observed") in completed_markers
        elif condition.kind == "named_pose_reached":
            satisfied = scene_usable and _completion_marker("named_pose_reached", condition.pose_name) in completed_markers
        status.append({**condition.to_mapping(), "satisfied": satisfied})
    return status


def mission_satisfied(
    mission: Mission,
    scene: GroundedScene,
    safety_policy: Mapping[str, Any],
    completed_markers: set[str] | frozenset[str],
    *,
    verified_placements: Mapping[tuple[str, str], tuple[float, float, float]] | None = None,
    now_s: float | None = None,
    _validator: SkillValidator | None = None,
) -> bool:
    statuses = completion_status(
        mission, scene, safety_policy, completed_markers, verified_placements=verified_placements,
        now_s=now_s, _validator=_validator
    )
    return bool(statuses) and all(item["satisfied"] for item in statuses)


def capability_options(
    scene: GroundedScene,
    mission: Mission,
    safety_policy: Mapping[str, Any],
    agent_policy: Mapping[str, Any],
    *,
    remaining_actions: int,
    completed_markers: set[str] | frozenset[str] = frozenset(),
    verified_placements: Mapping[tuple[str, str], tuple[float, float, float]] | None = None,
    now_s: float | None = None,
    _validator: SkillValidator | None = None,
) -> list[dict[str, Any]]:
    """Return the exact deterministic choices exposed to the model.

    The structure is semantic and intentionally omits all Cartesian positions.
    """
    validator = _validator or SkillValidator(policy=safety_policy, clock=_validation_clock(now_s))
    policy_caps = set(agent_policy["allowed_capabilities"])
    options: list[dict[str, Any]] = []

    def add(capability: str, **fields: Any) -> None:
        if capability in policy_caps:
            options.append({"capability": capability, **fields})

    add("stop")
    add("request_human_help")
    if not _scene_is_usable(scene, safety_policy, now_s=now_s):
        return options

    held = scene.held_object_id
    visible_objects = sorted(
        object_id for object_id in mission.object_ids
        if _observation_is_usable(
            scene.objects.get(object_id), scene, safety_policy, "object", now_s=now_s, validator=validator
        )
    )
    visible_targets = sorted(
        target_id for target_id in mission.target_ids
        if _observation_is_usable(
            scene.targets.get(target_id), scene, safety_policy, "target", now_s=now_s, validator=validator
        )
    )

    if held is not None:
        if held in mission.object_ids:
            pairs = [
                {"object_id": held, "target_id": target_id}
                for target_id in visible_targets
                if _occupancy(
                    scene, target_id, safety_policy, exclude=held, now_s=now_s, validator=validator
                ) is None
            ]
            if pairs:
                add("place", pairs=pairs)
        return options

    if mission_satisfied(
        mission, scene, safety_policy, completed_markers, verified_placements=verified_placements,
        now_s=now_s, _validator=validator
    ):
        add("finish")
    add("observe_workspace")
    if visible_objects:
        add("inspect", object_ids=visible_objects)
    if mission.pose_names:
        add("move_named_pose", pose_names=sorted(mission.pose_names))

    pairs = [
        {"object_id": object_id, "target_id": target_id}
        for object_id in visible_objects
        for target_id in visible_targets
        if _occupancy(
            scene, target_id, safety_policy, exclude=object_id, now_s=now_s, validator=validator
        ) is None
    ]
    if pairs:
        add("pick_and_place", pairs=pairs)
        # A standalone pick consumes one decision and must leave at least one
        # more decision for the mandatory place. Do not offer it at the edge.
        if remaining_actions >= 2:
            pickable = sorted({pair["object_id"] for pair in pairs})
            add("pick", object_ids=pickable)
    return options


def semantic_scene(
    scene: GroundedScene,
    mission: Mission,
    safety_policy: Mapping[str, Any],
    *,
    now_s: float | None = None,
    _validator: SkillValidator | None = None,
) -> dict[str, Any]:
    """Produce model-visible state with no robot/world coordinates."""
    validator = _validator or SkillValidator(policy=safety_policy, clock=_validation_clock(now_s))
    scene_usable = _scene_is_usable(scene, safety_policy, now_s=now_s, validator=validator)
    minimum_confidence = float(safety_policy["minimum_object_confidence"])
    objects = []
    for object_id in mission.object_ids:
        observation = scene.objects.get(object_id)
        grounded = _observation_is_usable(
            observation, scene, safety_policy, "object", now_s=now_s, validator=validator
        )
        confidence = "unavailable"
        if observation is not None and isinstance(observation.confidence, (int, float)) and math.isfinite(observation.confidence):
            if observation.confidence >= 0.9:
                confidence = "high"
            elif observation.confidence >= minimum_confidence:
                confidence = "acceptable"
            else:
                confidence = "low"
        objects.append({
            "object_id": object_id,
            "visible": observation is not None or scene.held_object_id == object_id,
            "grounded": grounded or scene.held_object_id == object_id,
            "held": scene.held_object_id == object_id,
            "confidence": confidence,
        })
    targets = []
    for target_id in mission.target_ids:
        observation = scene.targets.get(target_id)
        grounded = _observation_is_usable(
            observation, scene, safety_policy, "target", now_s=now_s, validator=validator
        )
        confidence = "unavailable"
        if observation is not None and isinstance(observation.confidence, (int, float)) and math.isfinite(observation.confidence):
            if observation.confidence >= 0.9:
                confidence = "high"
            elif observation.confidence >= minimum_confidence:
                confidence = "acceptable"
            else:
                confidence = "low"
        targets.append({
            "target_id": target_id,
            "visible": observation is not None,
            "grounded": grounded,
            "occupied_by": _occupancy(
                scene, target_id, safety_policy, now_s=now_s, validator=validator
            ) if grounded else None,
            "confidence": confidence,
        })
    return {
        "scene_revision": scene.revision,
        "scene_fresh": scene_usable,
        "held_object_id": scene.held_object_id,
        "objects": objects,
        "targets": targets,
    }


def semantic_result(result: Mapping[str, Any] | None) -> dict[str, Any] | None:
    """Strip geometry and implementation details before feeding results to the LLM."""
    if result is None:
        return None
    output: dict[str, Any] = {
        "status": str(result.get("status", "unknown")),
    }
    for key in ("code", "message", "skill"):
        if isinstance(result.get(key), str):
            output[key] = result[key]
    observation = result.get("observation")
    if isinstance(observation, Mapping) and isinstance(observation.get("object_id"), str):
        output["observed_object_id"] = observation["object_id"]
        output["observation_available"] = True
    outcome = result.get("outcome")
    if isinstance(outcome, Mapping):
        output["outcome"] = {
            "status": outcome.get("status"),
            "object_id": outcome.get("object_id"),
            "target_id": outcome.get("target_id"),
            "detached": outcome.get("detached"),
            "target_final_visibility": outcome.get("target_final_visibility"),
            "target_reference": outcome.get("target_reference"),
        }
    return output


def semantic_agent_input(
    mission: Mission,
    scene: GroundedScene,
    safety_policy: Mapping[str, Any],
    agent_policy: Mapping[str, Any],
    *,
    action_index: int,
    last_result: Mapping[str, Any] | None,
    completed_markers: set[str] | frozenset[str] = frozenset(),
    verified_placements: Mapping[tuple[str, str], tuple[float, float, float]] | None = None,
    now_s: float | None = None,
) -> dict[str, Any]:
    remaining = mission.max_actions - action_index
    validator = SkillValidator(policy=safety_policy, clock=_validation_clock(now_s))
    return {
        "mission": {
            "goal_summary": mission.goal_summary,
            "object_ids": list(mission.object_ids),
            "target_ids": list(mission.target_ids),
            "pose_names": list(mission.pose_names),
            "max_actions": mission.max_actions,
            "success_conditions": [item.to_mapping() for item in mission.success_conditions],
        },
        "action_index": action_index,
        "remaining_actions": remaining,
        "scene": semantic_scene(
            scene, mission, safety_policy, now_s=now_s, _validator=validator
        ),
        "last_result": semantic_result(last_result),
        "completion_status": completion_status(
            mission, scene, safety_policy, completed_markers, verified_placements=verified_placements,
            now_s=now_s, _validator=validator
        ),
        "allowed_capabilities": capability_options(
            scene, mission, safety_policy, agent_policy, remaining_actions=remaining,
            completed_markers=completed_markers, verified_placements=verified_placements,
            now_s=now_s, _validator=validator,
        ),
    }


def validate_action(
    raw: Mapping[str, Any],
    scene: GroundedScene,
    mission: Mission,
    options: list[dict[str, Any]],
    safety_policy: Mapping[str, Any],
) -> dict[str, Any]:
    if not isinstance(raw, Mapping):
        raise AgentValidationError("schema_invalid", "agent action must be an object")
    _validate_schema(raw, action_schema(), safety_policy)
    _reject_control_fields(raw, safety_policy)
    if raw["scene_revision"] != scene.revision:
        raise AgentValidationError("stale_revision", "agent action does not match the current scene")
    capability = raw["capability"]
    option = next((item for item in options if item["capability"] == capability), None)
    if option is None:
        raise AgentValidationError("capability_not_allowed", "agent selected a capability not offered by the supervisor")
    object_id, target_id, pose_name = raw["object_id"], raw["target_id"], raw["pose_name"]
    if "object_ids" in option and object_id not in option["object_ids"]:
        raise AgentValidationError("object_scope", "agent selected an object outside the current capability scope")
    if "pose_names" in option and pose_name not in option["pose_names"]:
        raise AgentValidationError("pose_scope", "agent selected a pose outside the current capability scope")
    if "pairs" in option and {"object_id": object_id, "target_id": target_id} not in option["pairs"]:
        raise AgentValidationError("target_scope", "agent selected an object/target pair not offered by the supervisor")
    if object_id is not None and object_id not in mission.object_ids:
        raise AgentValidationError("object_scope", "agent selected an object outside the confirmed mission")
    if target_id is not None and target_id not in mission.target_ids:
        raise AgentValidationError("target_scope", "agent selected a target outside the confirmed mission")
    if pose_name is not None and pose_name not in mission.pose_names:
        raise AgentValidationError("pose_scope", "agent selected a pose outside the confirmed mission")
    return dict(raw)


def action_to_skill(action: Mapping[str, Any]) -> dict[str, Any]:
    capability = action["capability"]
    if capability not in EXECUTABLE_CAPABILITIES:
        raise AgentValidationError("not_executable", "capability has no robot-skill mapping")
    return {
        "schema_version": 1,
        "request_id": action["request_id"],
        "skill": capability,
        "object_id": action["object_id"],
        "target_id": action["target_id"],
        "pose_name": action["pose_name"],
        "reason": None,
        "scene_revision": action["scene_revision"],
    }


class AgentSession:
    """Operator-confirmed, bounded closed-loop semantic controller."""

    def __init__(
        self,
        mission_model: AgentActionModel,
        action_model: AgentActionModel,
        backend: TaskBackend,
        audit: AuditLogger,
        *,
        clock: Callable[[], float] | None = None,
        event_callback: Callable[[str, Mapping[str, Any]], None] | None = None,
    ) -> None:
        self.mission_model = mission_model
        self.action_model = action_model
        self.backend = backend
        self.audit = audit
        self.clock = clock if clock is not None else getattr(backend, "clock", time.time)
        self.safety_policy = load_json_config("safety_policy.json")
        self.agent_policy = load_json_config("agent_policy.json")
        self.event_callback = event_callback
        self.pending: PendingMission | None = None
        self.state = "idle"
        self._used_missions: set[str] = set()
        self._used_actions: set[str] = set()
        self._stop = threading.Event()
        self._state_lock = threading.RLock()

    def _emit(self, event: str, payload: Mapping[str, Any]) -> None:
        self.audit.record(event, dict(payload))
        if self.event_callback is not None:
            self.event_callback(event, dict(payload))

    def prepare(self, transcript: str) -> dict[str, Any]:
        if is_stop_request(transcript):
            return self.stop()
        if self.state not in {"idle", "completed", "refused", "needs_human"}:
            raise RuntimeError("recover a stopped/faulted session before preparing a new mission")
        self.pending = None
        limit = int(self.agent_policy["maximum_goal_characters"])
        if not isinstance(transcript, str) or not transcript.strip() or len(transcript) > limit:
            return self._refuse("invalid_transcript", f"Provide a nonempty command up to {limit} characters.")
        try:
            scene = self.backend.capture()
            if scene.held_object_id is not None:
                return self._refuse("mission_requires_empty_hand", "Start an agent mission only with an empty gripper; use explicit placement recovery first.")
            raw = self.mission_model.propose(transcript, scene, self.safety_policy)
            if self._stop.is_set():
                return {"status": "stopped"}
            mission = validate_mission(
                raw, scene, self.safety_policy, self.agent_policy, now_s=self.clock()
            )
            if mission.request_id in self._used_missions:
                raise AgentValidationError("replayed_mission", "mission request ID was already consumed")
            canonical = json.dumps(mission.to_mapping(), sort_keys=True, separators=(",", ":"))
            confirmation = hashlib.sha256(canonical.encode()).hexdigest()[:16]
            ttl = float(self.agent_policy["confirmation_ttl_s"])
            with self._state_lock:
                if self._stop.is_set():
                    return {"status": "stopped"}
                self.pending = PendingMission(mission, confirmation, self.clock() + ttl)
                self.state = "awaiting_confirmation"
            result = {
                "status": self.state,
                "mission": mission.to_mapping(),
                "confirmation": confirmation,
                "transcript": transcript,
                "timeline": [],
            }
            self._emit("agent_mission_prepared", {"mission": mission.to_mapping(), "transcript": transcript})
            return result
        except Exception as error:
            return self._refuse(getattr(error, "code", "invalid_mission"), str(error))

    def confirm(self, token: str) -> dict[str, Any]:
        with self._state_lock:
            pending = self.pending
            if self._stop.is_set() or pending is None or self.state != "awaiting_confirmation" or token != pending.confirmation:
                raise ValueError("confirmation must match the pending mission")
            self.pending = None
            self.state = "running"
        if self.clock() > pending.expires_at_s:
            return self._refuse("confirmation_expired", "Please repeat the command to obtain a fresh mission.")
        mission = pending.mission
        self._used_missions.add(mission.request_id)
        deadline = time.monotonic() + float(self.agent_policy["mission_deadline_s"])
        timeline: list[dict[str, Any]] = []
        last_result: Mapping[str, Any] | None = None
        completed_markers: set[str] = set()
        verified_placements: dict[tuple[str, str], tuple[float, float, float]] = {}
        try:
            for index in range(mission.max_actions):
                if self._stop.is_set():
                    break
                if time.monotonic() > deadline:
                    raise RuntimeError("agent mission deadline exceeded")
                scene = self.backend.capture()
                semantic = semantic_agent_input(
                    mission, scene, self.safety_policy, self.agent_policy,
                    action_index=index, last_result=last_result, completed_markers=completed_markers,
                    verified_placements=verified_placements, now_s=self.clock(),
                )
                self._emit("agent_observation", {"mission_id": mission.request_id, **semantic})
                if self._stop.is_set():
                    break
                raw = self.action_model.propose(
                    json.dumps(semantic, separators=(",", ":"), allow_nan=False),
                    scene,
                    self.safety_policy,
                )
                if self._stop.is_set():
                    break
                options = semantic["allowed_capabilities"]
                action = validate_action(raw, scene, mission, options, self.safety_policy)
                if action["request_id"] in self._used_actions:
                    raise AgentValidationError("replayed_action", "agent action request ID was already consumed")
                self._used_actions.add(action["request_id"])
                decision = {
                    "index": index,
                    "capability": action["capability"],
                    "object_id": action["object_id"],
                    "target_id": action["target_id"],
                    "pose_name": action["pose_name"],
                    "reason": action["reason"],
                }
                timeline.append({**decision, "status": "selected"})
                self._emit("agent_decision", {"mission_id": mission.request_id, **decision})

                capability = action["capability"]
                if capability == "stop":
                    stop_result = self.stop()
                    timeline[-1]["status"] = "stopped"
                    return {"status": "stopped", "mission": mission.to_mapping(), "timeline": timeline, "stop_result": stop_result}
                if capability == "request_human_help":
                    self.state = "needs_human"
                    timeline[-1]["status"] = "needs_human"
                    result = {
                        "status": self.state, "mission": mission.to_mapping(), "timeline": timeline,
                        "message": action["reason"],
                    }
                    self._emit("agent_result", result)
                    return result
                if capability == "finish":
                    if scene.held_object_id is not None:
                        raise AgentValidationError("finish_while_holding", "agent cannot finish while holding an object")
                    if not mission_satisfied(
                        mission, scene, self.safety_policy, completed_markers,
                        verified_placements=verified_placements, now_s=self.clock()
                    ):
                        raise AgentValidationError("goal_not_satisfied", "agent cannot finish before the confirmed completion conditions are satisfied")
                    self.state = "completed"
                    timeline[-1]["status"] = "completed"
                    result = {
                        "status": self.state, "mission": mission.to_mapping(), "timeline": timeline,
                        "message": action["reason"],
                    }
                    self._emit("agent_result", result)
                    return result
                if capability == "observe_workspace":
                    completed_markers.add(_completion_marker("workspace_observed"))
                    last_result = {"status": "succeeded", "skill": "observe_workspace"}
                    timeline[-1]["status"] = "succeeded"
                    self._emit("agent_action_result", {"mission_id": mission.request_id, "index": index, "result": last_result})
                    continue

                proposal = action_to_skill(action)
                validator = SkillValidator(clock=self.clock)
                validator.validate(proposal, scene)
                check_preconditions(proposal, scene.held_object_id, scene, self.safety_policy)
                if self._stop.is_set():
                    break
                result = self.backend.execute(proposal, scene)
                timeline[-1]["status"] = str(result.get("status", "unknown"))
                last_result = result
                self._emit("agent_action_result", {
                    "mission_id": mission.request_id,
                    "index": index,
                    "result": semantic_result(result) or {"status": "unknown"},
                })
                if result.get("status") != "succeeded":
                    raise RuntimeError(f"robot capability failed: {result.get('code', 'backend_failure')}")
                if capability in {"place", "pick_and_place"}:
                    outcome = result.get("outcome")
                    if isinstance(outcome, Mapping) and outcome.get("status") == "accepted":
                        object_position = outcome.get("object_position_m")
                        if (
                            action.get("object_id")
                            and action.get("target_id")
                            and isinstance(object_position, list)
                            and len(object_position) == 3
                            and all(type(value) in (int, float) and math.isfinite(value) for value in object_position)
                        ):
                            verified_placements[(action["object_id"], action["target_id"])] = tuple(object_position)
                if capability == "inspect":
                    completed_markers.add(_completion_marker("object_inspected", action["object_id"]))
                elif capability == "move_named_pose":
                    completed_markers.add(_completion_marker("named_pose_reached", action["pose_name"]))

            if self._stop.is_set():
                self.state = "stopped"
                result = {"status": "stopped", "mission": mission.to_mapping(), "timeline": timeline}
            else:
                # Budget exhaustion is not success. The operator must choose a
                # new goal/recovery path; autonomous authority ends here.
                self.state = "needs_human"
                result = {
                    "status": self.state,
                    "code": "action_budget_exhausted",
                    "message": "Agent action budget ended before it declared the goal complete.",
                    "mission": mission.to_mapping(),
                    "timeline": timeline,
                }
            self._emit("agent_result", result)
            return result
        except Exception as error:
            self.state = "stopped" if self._stop.is_set() else "faulted"
            result = {
                "status": self.state,
                "code": getattr(error, "code", "agent_execution_failure"),
                "message": str(error),
                "mission": mission.to_mapping(),
                "timeline": timeline,
            }
            try:
                result["cancellation"] = self.backend.stop()
            except Exception as stop_error:
                self.state = "faulted"
                result.update(status="faulted", cancellation_confirmed=False, cancellation_error=str(stop_error))
            self._emit("agent_result", result)
            return result

    def stop(self) -> dict[str, Any]:
        with self._state_lock:
            self._stop.set()
            self.pending = None
            self.state = "stopped"
        try:
            result = self.backend.stop()
        except Exception as error:
            self.state = "faulted"
            result = {"status": "stop_unconfirmed", "message": str(error)}
        self._emit("agent_operator_stop", result)
        return {"status": result.get("status", "stop_requested"), "backend": result}

    def recover(self) -> dict[str, Any]:
        if self.state not in {"stopped", "faulted"}:
            raise RuntimeError("recovery requires a stopped or faulted agent")
        result = self.backend.recover()
        if result.get("status") != "recovered":
            return result
        self._stop.clear()
        self.pending = None
        self.state = "idle"
        self._emit("agent_operator_recovery", result)
        return result

    def cancel(self) -> dict[str, Any]:
        with self._state_lock:
            if self.state != "awaiting_confirmation":
                raise RuntimeError("cancel requires a pending mission")
            self.pending = None
            self.state = "idle"
        result = {"status": "cancelled"}
        self._emit("agent_mission_cancelled", result)
        return result

    def _refuse(self, code: str, message: str) -> dict[str, Any]:
        with self._state_lock:
            self.pending = None
            if self._stop.is_set():
                return {"status": self.state if self.state in {"stopped", "faulted"} else "stopped"}
            self.state = "refused"
        result = {"status": "refused", "code": code, "message": message}
        self._emit("agent_refused", result)
        return result
