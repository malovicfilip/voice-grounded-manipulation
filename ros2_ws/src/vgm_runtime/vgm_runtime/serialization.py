"""Strict serialization helpers for grounded scenes and plans."""

from __future__ import annotations

import json
import math
import re
from pathlib import Path
from typing import Any, Mapping

from .types import GroundedScene, ObjectObservation, TargetObservation, TaskPlan


def scene_from_mapping(value: Mapping[str, Any]) -> GroundedScene:
    required = {"revision", "captured_at_s", "objects"}
    if not isinstance(value, Mapping) or not required <= set(value) or set(value) - required - {"held_object_id", "targets"}:
        raise ValueError("grounded scene fields are invalid")
    if not isinstance(value["revision"], str) or not re.fullmatch("[a-f0-9]{16}", value["revision"]):
        raise ValueError("invalid scene revision")
    held = value.get("held_object_id")
    if held is not None and (not isinstance(held, str) or not re.fullmatch("[a-z][a-z0-9_]{0,63}", held)):
        raise ValueError("invalid held object identifier")

    def number(raw):
        if type(raw) not in {int, float} or not math.isfinite(raw):
            raise ValueError("scene numbers must be finite numeric values")
        return float(raw)
    observations: dict[str, ObjectObservation] = {}
    # Reuse the same strict observation decoder; do not invent target poses
    # when loading historical files that predate target observations.
    targets = {}
    if not isinstance(value.get("targets", []), list):
        raise ValueError("grounded targets must be an array")
    for raw in value.get("targets", []):
        if not isinstance(raw, Mapping) or "target_id" not in raw or "object_id" in raw:
            raise ValueError("invalid target observation")
        converted = {("object_id" if key == "target_id" else key): item for key, item in raw.items()}
        decoded = scene_from_mapping({"revision": value["revision"], "captured_at_s": value["captured_at_s"],
                                      "objects": [converted]}).objects[raw["target_id"]]
        if raw["target_id"] in targets:
            raise ValueError("duplicate target ID")
        targets[raw["target_id"]] = TargetObservation(raw["target_id"], decoded.position_m,
            decoded.confidence, decoded.observed_at_s, decoded.frame_id, decoded.pixel_count)
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
        if not isinstance(raw["object_id"], str) or not re.fullmatch("[a-z][a-z0-9_]{0,63}", raw["object_id"]):
            raise ValueError("invalid object identifier")
        if raw["frame_id"] != "world":
            raise ValueError("object observation must be in the world frame")
        if type(raw["pixel_count"]) is not int or raw["pixel_count"] < 0:
            raise ValueError("invalid pixel support")
        confidence = number(raw["confidence"])
        if not 0 <= confidence <= 1:
            raise ValueError("confidence must be in [0, 1]")
        observation = ObjectObservation(
            object_id=raw["object_id"],
            position_m=tuple(number(component) for component in position),
            confidence=confidence,
            observed_at_s=number(raw["observed_at_s"]),
            frame_id=raw["frame_id"],
            pixel_count=int(raw["pixel_count"]),
        )
        if observation.object_id in observations:
            raise ValueError("duplicate grounded object ID")
        observations[observation.object_id] = observation
    return GroundedScene(
        revision=value["revision"],
        captured_at_s=number(value["captured_at_s"]),
        objects=observations,
        held_object_id=value.get("held_object_id"),
        targets=targets,
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
