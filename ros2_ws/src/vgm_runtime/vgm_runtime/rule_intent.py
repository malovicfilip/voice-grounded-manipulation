"""Deterministic parser used for offline tests and API-independent demos."""

from __future__ import annotations

import re
import uuid
from collections.abc import Mapping
from typing import Any

from .types import GroundedScene


class RuleBasedIntentModel:
    def __init__(self, request_id_factory=None) -> None:
        self.request_id_factory = request_id_factory or (
            lambda: f"req_{uuid.uuid4().hex[:16]}"
        )

    def propose(
        self, transcript: str, scene: GroundedScene, policy: Mapping[str, Any]
    ) -> dict[str, Any]:
        text = " ".join(transcript.lower().split())
        request_id = self.request_id_factory()
        base: dict[str, Any] = {
            "schema_version": 1,
            "request_id": request_id,
            "skill": "refuse",
            "object_id": None,
            "target_id": None,
            "pose_name": None,
            "reason": "command is unsupported or ambiguous",
            "scene_revision": None,
        }

        if re.search(r"\b(stop|halt|cancel|freeze)\b", text):
            return {**base, "skill": "stop", "reason": "user requested stop"}

        pose_mentions = [
            pose
            for pose in policy["allowed_named_poses"]
            if re.search(rf"\b{re.escape(pose)}\b", text)
        ]
        if len(pose_mentions) == 1 and re.search(r"\b(move|go|pose)\b", text):
            return {
                **base,
                "skill": "move_named_pose",
                "pose_name": pose_mentions[0],
                "reason": None,
            }

        if re.search(r"\b(open|release)\b.*\b(gripper|hand|fingers)\b", text):
            return {**base, "skill": "open_gripper", "reason": None}
        if re.search(r"\b(close|shut)\b.*\b(gripper|hand|fingers)\b", text):
            return {**base, "skill": "close_gripper", "reason": None}

        object_mentions = [
            object_id
            for object_id in sorted(scene.objects)
            if re.search(
                rf"\b{re.escape(object_id.replace('_', ' '))}\b", text
            )
        ]
        target_mentions = [
            target_id
            for target_id in sorted(policy["targets"])
            if re.search(
                rf"\b{re.escape(target_id.replace('_', ' '))}\b", text
            )
        ]
        if len(object_mentions) != 1:
            return {**base, "reason": "exactly one grounded object is required"}

        has_pick = bool(re.search(r"\b(pick|grab|grasp)\b", text))
        has_place = bool(re.search(r"\b(place|put|drop)\b", text))
        if has_pick and has_place and len(target_mentions) == 1:
            return {
                **base,
                "skill": "pick_and_place",
                "object_id": object_mentions[0],
                "target_id": target_mentions[0],
                "reason": None,
                "scene_revision": scene.revision,
            }
        if has_pick and not has_place:
            return {
                **base,
                "skill": "pick",
                "object_id": object_mentions[0],
                "reason": None,
                "scene_revision": scene.revision,
            }
        if has_place and len(target_mentions) == 1:
            return {
                **base,
                "skill": "place",
                "object_id": object_mentions[0],
                "target_id": target_mentions[0],
                "reason": None,
                "scene_revision": scene.revision,
            }
        return base
