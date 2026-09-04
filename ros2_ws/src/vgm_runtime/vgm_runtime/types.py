"""Immutable values passed across the deterministic safety boundary."""

from __future__ import annotations

from dataclasses import dataclass, field
from types import MappingProxyType
from typing import Any, Mapping


@dataclass(frozen=True, slots=True)
class ObjectObservation:
    object_id: str
    position_m: tuple[float, float, float]
    confidence: float
    observed_at_s: float
    frame_id: str = "world"
    pixel_count: int = 0

    def to_mapping(self) -> dict[str, Any]:
        return {
            "object_id": self.object_id,
            "position_m": list(self.position_m),
            "confidence": self.confidence,
            "observed_at_s": self.observed_at_s,
            "frame_id": self.frame_id,
            "pixel_count": self.pixel_count,
        }


@dataclass(frozen=True, slots=True)
class GroundedScene:
    revision: str
    captured_at_s: float
    objects: Mapping[str, ObjectObservation] = field(default_factory=dict)
    held_object_id: str | None = None

    def __post_init__(self) -> None:
        object.__setattr__(self, "objects", MappingProxyType(dict(self.objects)))

    def to_mapping(self) -> dict[str, Any]:
        return {
            "revision": self.revision,
            "captured_at_s": self.captured_at_s,
            "held_object_id": self.held_object_id,
            "objects": [
                self.objects[object_id].to_mapping()
                for object_id in sorted(self.objects)
            ],
        }


@dataclass(frozen=True, slots=True)
class SkillProposal:
    schema_version: int
    request_id: str
    skill: str
    object_id: str | None
    target_id: str | None
    pose_name: str | None
    reason: str | None
    scene_revision: str | None

    @classmethod
    def from_mapping(cls, value: Mapping[str, Any]) -> "SkillProposal":
        return cls(**{field_name: value.get(field_name) for field_name in cls.field_names()})

    @classmethod
    def field_names(cls) -> frozenset[str]:
        return frozenset(cls.__dataclass_fields__)

    def to_mapping(self) -> dict[str, Any]:
        return {
            field_name: getattr(self, field_name)
            for field_name in self.__dataclass_fields__
        }


@dataclass(frozen=True, slots=True)
class ValidatedSkill:
    proposal: SkillProposal
    policy_version: int
    validated_at_s: float

    def to_mapping(self) -> dict[str, Any]:
        return {
            **self.proposal.to_mapping(),
            "policy_version": self.policy_version,
            "validated_at_s": self.validated_at_s,
            "validation_status": "accepted",
        }


@dataclass(frozen=True, slots=True)
class MotionPrimitive:
    kind: str
    object_id: str | None = None
    pose_name: str | None = None
    position_m: tuple[float, float, float] | None = None


@dataclass(frozen=True, slots=True)
class TaskPlan:
    request_id: str
    skill: str
    primitives: tuple[MotionPrimitive, ...]


@dataclass(frozen=True, slots=True)
class ExecutionResult:
    request_id: str
    status: str
    completed_primitives: int
    code: str
    message: str
