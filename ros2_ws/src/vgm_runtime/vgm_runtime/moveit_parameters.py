"""Deterministic launch parameters; never part of the LLM input or output."""

from .config import load_json_config, policy_digest
from .serialization import scene_from_mapping
from .validator import SkillValidator, SkillValidationError


def execution_parameters(raw_scene, target_id="", *, clock=None):
    policy = load_json_config("safety_policy.json")
    raw = dict(raw_scene)
    collision_objects = raw.pop("collision_objects", raw["objects"])
    scene = scene_from_mapping(raw)
    validator = SkillValidator(policy=policy, **({"clock": clock} if clock else {}))
    validator._validate_scene_age(scene)
    parameters = {"safety_policy_digest": policy_digest(policy),
                  "scene_captured_at_s": scene.captured_at_s}
    for item in collision_objects:
        object_id = item["object_id"]
        if object_id not in policy["objects"] or "scene_" + object_id in parameters:
            raise ValueError("invalid collision object ID")
        validator._validate_workspace_position(item["position_m"], "collision object")
        parameters["scene_" + object_id] = [float(v) for v in item["position_m"]]
    if target_id:
        target = scene.targets.get(target_id)
        if target_id not in policy["targets"] or target is None:
            raise SkillValidationError("target_not_grounded", "measured target is required for MoveIt")
        validator._validate_observation(target, scene, "target")
        parameters.update({"target_" + axis: float(v) for axis, v in zip("xyz", target.position_m)})
    return parameters
