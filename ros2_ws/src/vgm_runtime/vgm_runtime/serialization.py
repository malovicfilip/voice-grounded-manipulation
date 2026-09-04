"""Strict serialization helpers for grounded scenes and plans."""

from __future__ import annotations

import json
from pathlib import Path
from typing import Any, Mapping

from .types import GroundedScene, ObjectObservation, TaskPlan


def scene_from_mapping(value: Mapping[str, Any]) -> GroundedScene:
    if set(value) != {"revision", "captured_at_s", "objects"}:
        raise ValueError("grounded scene fields are invalid")
    observations: dict[str, ObjectObservation] = {}
    if not isinstance(value["objects"], list):
        raise ValueError("grounded scene objects must be an array")
    expected = {
        "object_id",
        "position_m",
        "confidence",
        "observed_at_s",
        "frame_id",
        "pixel_count",
    }
    for raw in value["objects"]:
        if not isinstance(raw, Mapping) or set(raw) != expected:
            raise ValueError("object observation fields are invalid")
        position = raw["position_m"]
        if not isinstance(position, list) or len(position) != 3:
            raise ValueError("object position must contain three values")
        observation = ObjectObservation(
            object_id=raw["object_id"],
            position_m=tuple(float(component) for component in position),
            confidence=float(raw["confidence"]),
            observed_at_s=float(raw["observed_at_s"]),
            frame_id=raw["frame_id"],
            pixel_count=int(raw["pixel_count"]),
        )
        if observation.object_id in observations:
            raise ValueError("duplicate grounded object ID")
        observations[observation.object_id] = observation
    return GroundedScene(
        revision=value["revision"],
        captured_at_s=float(value["captured_at_s"]),
        objects=observations,
        held_object_id=value.get("held_object_id"),
    )


def load_scene(path: Path) -> GroundedScene:
    with path.open(encoding="utf-8") as scene_file:
        value = json.load(scene_file)
    if not isinstance(value, Mapping):
        raise ValueError("grounded scene must be an object")
    return scene_from_mapping(value)


def plan_to_mapping(plan: TaskPlan) -> dict[str, Any]:
    return {
        "request_id": plan.request_id,
        "skill": plan.skill,
        "primitives": [
            {
                "kind": primitive.kind,
                "object_id": primitive.object_id,
                "pose_name": primitive.pose_name,
                "position_m": (
                    list(primitive.position_m)
                    if primitive.position_m is not None
                    else None
                ),
            }
            for primitive in plan.primitives
        ],
    }
