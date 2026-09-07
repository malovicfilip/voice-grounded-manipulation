"""Zero-motion local replay/sandbox backend for the unified demo dashboard."""

from __future__ import annotations

import hashlib
import json
import math
import time
from dataclasses import replace

from .config import load_json_config
from .tasks import check_preconditions
from .types import GroundedScene, ObjectObservation, TargetObservation
from .validator import SkillValidator


class ReplayBackend:
    """Deterministic in-memory scene used only for UI/agent demonstrations.

    It intentionally performs no ROS, SSH, cloud, API, or robot operation.
    """

    def __init__(self) -> None:
        geometry = load_json_config("phase_1_scene.json")
        self.policy = load_json_config("safety_policy.json")
        self.positions = {item["object_id"]: tuple(item["position"]) for item in geometry["cubes"]}
        self.targets = {item["target_id"]: tuple(item["position"]) for item in geometry["targets"]}
        self.held: str | None = None
        self.stopped = False
        self.executed: list[dict] = []
        self._revision_counter = 0

    @staticmethod
    def clock() -> float:
        return time.time()

    def _revision(self) -> str:
        payload = json.dumps({"positions": self.positions, "held": self.held, "n": self._revision_counter}, sort_keys=True)
        return hashlib.sha256(payload.encode()).hexdigest()[:16]

    def capture(self) -> GroundedScene:
        if self.stopped:
            raise RuntimeError("replay session is stopped; recover before continuing")
        self._revision_counter += 1
        now = time.time()
        objects = {
            object_id: ObjectObservation(object_id, position, 0.99, now, pixel_count=500)
            for object_id, position in self.positions.items()
            if object_id != self.held
        }
        targets = {
            target_id: TargetObservation(target_id, position, 0.99, now, pixel_count=800)
            for target_id, position in self.targets.items()
        }
        return GroundedScene(self._revision(), now, objects, self.held, targets)

    def execute(self, proposal, scene):
        if self.stopped:
            raise RuntimeError("replay session is stopped")
        SkillValidator(clock=self.clock).validate(proposal, scene)
        check_preconditions(proposal, scene.held_object_id, scene, self.policy)
        self.executed.append(dict(proposal))
        skill = proposal["skill"]
        object_id = proposal.get("object_id")
        target_id = proposal.get("target_id")
        if skill == "inspect":
            return {"status": "succeeded", "skill": skill, "observation": scene.objects[object_id].to_mapping()}
        if skill == "pick":
            self.held = object_id
        elif skill == "place":
            if self.held != object_id:
                raise RuntimeError("replay place requires the held object")
            target = self.targets[target_id]
            self.positions[object_id] = (target[0], target[1], target[2] + self.policy["objects"][object_id]["size_m"] / 2)
            self.held = None
        elif skill == "pick_and_place":
            target = self.targets[target_id]
            self.positions[object_id] = (target[0], target[1], target[2] + self.policy["objects"][object_id]["size_m"] / 2)
        return {
            "status": "succeeded",
            "skill": skill,
            "request_id": proposal["request_id"],
            **({"outcome": {"status": "accepted", "object_id": object_id, "target_id": target_id, "detached": True}}
               if skill in {"place", "pick_and_place"} else {}),
        }

    def stop(self):
        self.stopped = True
        return {"status": "stopped", "mode": "replay"}

    def recover(self):
        if self.held is not None:
            return {"status": "refused", "message": "replay recovery requires an empty gripper"}
        self.stopped = False
        return {"status": "recovered", "mode": "replay"}

    def visual_state(self):
        # Browser-only normalized table coordinates; never supplied to the LLM.
        def point(position):
            x, y = position[:2]
            return {"u": round((x + 0.55) / 1.10, 4), "v": round((0.35 - y) / 0.70, 4)}
        return {
            "objects": [
                {"object_id": object_id, **point(position), "held": object_id == self.held}
                for object_id, position in sorted(self.positions.items())
            ],
            "targets": [
                {"target_id": target_id, **point(position)}
                for target_id, position in sorted(self.targets.items())
            ],
            "held_object_id": self.held,
            "label": "Local replay sandbox — no robot motion",
        }


class ReplayMissionModel:
    def __init__(self) -> None:
        self.calls = 0

    def propose(self, transcript, scene, policy):
        self.calls += 1
        text = transcript.lower()
        explicit_objects = [object_id for object_id in sorted(scene.objects) if object_id.removesuffix("_cube") in text]
        explicit_targets = [target_id for target_id in sorted(scene.targets) if target_id.replace("_", " ") in text]
        object_ids = explicit_objects or (sorted(scene.objects) if any(word in text for word in ("all", "workspace", "tidy")) else sorted(scene.objects)[:1])
        if explicit_targets:
            target_ids = explicit_targets
        elif any(word in text for word in ("place", "move", "put", "target", "tidy")):
            target_ids = sorted(scene.targets)
        else:
            target_ids = []
        poses = [pose for pose in policy["allowed_named_poses"] if pose in text]
        if any(word in text for word in ("inspect", "look", "check")):
            conditions = [{
                "kind": "object_inspected", "object_id": object_ids[0],
                "target_id": None, "pose_name": None,
            }]
        elif poses:
            conditions = [{
                "kind": "named_pose_reached", "object_id": None,
                "target_id": None, "pose_name": poses[0],
            }]
        elif target_ids and object_ids:
            desired_object = explicit_objects[0] if explicit_objects else object_ids[0]
            desired_target = explicit_targets[0] if explicit_targets else target_ids[0]
            conditions = [{
                "kind": "object_on_target", "object_id": desired_object,
                "target_id": desired_target, "pose_name": None,
            }]
        else:
            conditions = [{
                "kind": "workspace_observed", "object_id": None,
                "target_id": None, "pose_name": None,
            }]
        return {
            "schema_version": 1,
            "request_id": f"replay_mission_{self.calls}",
            "scene_revision": scene.revision,
            "goal_summary": transcript.strip()[:500],
            "object_ids": object_ids,
            "target_ids": target_ids,
            "pose_names": poses,
            "max_actions": 6,
            "success_conditions": conditions,
        }


class ReplayAgentActionModel:
    def __init__(self) -> None:
        self.calls = 0

    def propose(self, transcript, scene, policy):
        self.calls += 1
        state = json.loads(transcript)
        goal = state["mission"]["goal_summary"].lower()
        options = {item["capability"]: item for item in state["allowed_capabilities"]}
        last = state.get("last_result") or {}

        def action(capability, *, object_id=None, target_id=None, pose_name=None, reason):
            return {
                "schema_version": 1,
                "request_id": f"replay_action_{self.calls}",
                "scene_revision": scene.revision,
                "capability": capability,
                "object_id": object_id,
                "target_id": target_id,
                "pose_name": pose_name,
                "reason": reason,
            }

        if "finish" in options:
            return action("finish", reason="All deterministic completion conditions are satisfied.")

        if last.get("status") == "succeeded" and last.get("skill") == "pick" and "place" in options:
            pair = options["place"]["pairs"][0]
            return action("place", **pair, reason="Complete the transfer by placing the held object on an allowed target.")

        if any(word in goal for word in ("inspect", "look", "check")) and "inspect" in options:
            return action("inspect", object_id=options["inspect"]["object_ids"][0], reason="Inspect the requested grounded object before finishing.")
        if "move_named_pose" in options and any(pose in goal for pose in options["move_named_pose"]["pose_names"]):
            pose = next(pose for pose in options["move_named_pose"]["pose_names"] if pose in goal)
            return action("move_named_pose", pose_name=pose, reason="Move to the named pose explicitly requested by the operator.")
        if "pick_and_place" in options:
            pairs = options["pick_and_place"]["pairs"]
            desired_object = next((obj for obj in state["mission"]["object_ids"] if obj.removesuffix("_cube") in goal), None)
            desired_target = next((target for target in state["mission"]["target_ids"] if target.replace("_", " ") in goal), None)
            pair = next((pair for pair in pairs if (desired_object is None or pair["object_id"] == desired_object)
                         and (desired_target is None or pair["target_id"] == desired_target)), pairs[0])
            return action("pick_and_place", **pair, reason="Execute the requested transfer using the currently offered bounded capability.")
        if "observe_workspace" in options:
            return action("observe_workspace", reason="Refresh semantic workspace state before deciding how to proceed.")
        return action("request_human_help", reason="No offered replay capability can safely complete this mission.")


class ReplayTaskModel:
    def __init__(self) -> None:
        self.calls = 0

    def propose(self, transcript, scene, policy):
        self.calls += 1
        text = transcript.lower()
        object_id = next((item for item in sorted(scene.objects) if item.removesuffix("_cube") in text), sorted(scene.objects)[0])
        target_id = next((item for item in sorted(scene.targets) if item.replace("_", " ") in text), sorted(scene.targets)[0])
        if any(word in text for word in ("inspect", "look", "check")):
            steps = [{"skill": "inspect", "object_id": object_id, "target_id": None, "pose_name": None, "reason": None}]
        else:
            steps = [{"skill": "pick_and_place", "object_id": object_id, "target_id": target_id, "pose_name": None, "reason": None}]
        return {
            "schema_version": 1,
            "request_id": f"replay_task_{self.calls}",
            "scene_revision": scene.revision,
            "steps": steps,
        }
