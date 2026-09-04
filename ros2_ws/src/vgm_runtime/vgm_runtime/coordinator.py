"""Expand validated skills into deterministic, policy-bounded task plans."""

from __future__ import annotations

from .config import load_json_config
from .types import GroundedScene, MotionPrimitive, TaskPlan, ValidatedSkill


class TaskCoordinator:
    """Create motion primitives only from a validator-minted skill."""

    def __init__(self, policy=None) -> None:
        self.policy = dict(policy or load_json_config("safety_policy.json"))

    def create_plan(
        self, skill: ValidatedSkill, scene: GroundedScene | None
    ) -> TaskPlan:
        proposal = skill.proposal
        if skill.policy_version != self.policy["policy_version"]:
            raise ValueError("validated skill policy version does not match")

        if proposal.skill == "move_named_pose":
            primitives = (
                MotionPrimitive("move_named_pose", pose_name=proposal.pose_name),
            )
        elif proposal.skill == "open_gripper":
            primitives = (MotionPrimitive("open_gripper"),)
        elif proposal.skill == "close_gripper":
            primitives = (MotionPrimitive("close_gripper"),)
        elif proposal.skill == "pick":
            primitives = self._pick_primitives(proposal.object_id, scene)
        elif proposal.skill == "place":
            primitives = self._place_primitives(
                proposal.object_id, proposal.target_id
            )
        elif proposal.skill == "pick_and_place":
            primitives = (
                *self._pick_primitives(proposal.object_id, scene),
                *self._place_primitives(proposal.object_id, proposal.target_id),
            )
        elif proposal.skill == "stop":
            primitives = (MotionPrimitive("stop"),)
        elif proposal.skill == "refuse":
            primitives = ()
        else:  # The validator should make this branch unreachable.
            raise ValueError(f"unsupported validated skill: {proposal.skill}")

        return TaskPlan(
            request_id=proposal.request_id,
            skill=proposal.skill,
            primitives=tuple(primitives),
        )

    def _pick_primitives(
        self, object_id: str | None, scene: GroundedScene | None
    ) -> tuple[MotionPrimitive, ...]:
        if object_id is None or scene is None or object_id not in scene.objects:
            raise ValueError("validated pick is missing its grounded object")
        x, y, z = scene.objects[object_id].position_m
        pick = self.policy["pick"]
        return (
            MotionPrimitive("open_gripper", object_id=object_id),
            MotionPrimitive(
                "move_cartesian",
                object_id=object_id,
                position_m=(x, y, z + pick["approach_height_m"]),
            ),
            MotionPrimitive(
                "move_cartesian",
                object_id=object_id,
                position_m=(x, y, z + pick["grasp_height_offset_m"]),
            ),
            MotionPrimitive("close_gripper", object_id=object_id),
            MotionPrimitive("attach_object", object_id=object_id),
            MotionPrimitive(
                "move_cartesian",
                object_id=object_id,
                position_m=(x, y, z + pick["retreat_height_m"]),
            ),
        )

    def _place_primitives(
        self, object_id: str | None, target_id: str | None
    ) -> tuple[MotionPrimitive, ...]:
        if object_id is None or target_id not in self.policy["targets"]:
            raise ValueError("validated place is missing its allowlisted target")
        x, y, z = self.policy["targets"][target_id]["position_m"]
        place = self.policy["place"]
        return (
            MotionPrimitive(
                "move_cartesian",
                object_id=object_id,
                position_m=(x, y, z + place["approach_height_m"]),
            ),
            MotionPrimitive(
                "move_cartesian",
                object_id=object_id,
                position_m=(x, y, z + place["release_height_offset_m"]),
            ),
            MotionPrimitive("open_gripper", object_id=object_id),
            MotionPrimitive("detach_object", object_id=object_id),
            MotionPrimitive(
                "move_cartesian",
                object_id=object_id,
                position_m=(x, y, z + place["retreat_height_m"]),
            ),
        )
