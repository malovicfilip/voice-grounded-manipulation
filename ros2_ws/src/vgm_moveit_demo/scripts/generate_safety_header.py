"""Build-only C++ constants generated from the same policy Python loads."""

import argparse
import json
import math
import os
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "vgm_runtime"))
from vgm_runtime.config import load_json_config, policy_digest


def generate(policy):
    def scalar(value):
        if type(value) not in (int, float) or not math.isfinite(value) or value <= 0:
            raise ValueError("safety constants must be finite and positive")
        return repr(float(value))

    def vector(values):
        if len(values) != 3 or not all(type(v) in (int, float) and math.isfinite(v) for v in values):
            raise ValueError("geometry must be finite XYZ")
        return "{" + ", ".join(repr(float(v)) for v in values) + "}"

    def strings(values):
        return "{" + ", ".join(json.dumps(v) for v in sorted(values)) + "}"

    if not 0 < policy["maximum_velocity_scale"] <= .2 or not 0 < policy["maximum_acceleration_scale"] <= .2:
        raise ValueError("policy exceeds motion scaling ceiling")
    lines = ["// GENERATED: edit config/safety_policy.json or its scene geometry, not this file.",
             "#pragma once", "#include <array>", "#include <map>", "#include <set>",
             "#include <string>", "namespace vgm_safety {",
             "inline const std::string kPolicyDigest = " + json.dumps(policy_digest(policy)) + ";"]
    for name, key in {"kVelocityScale": "maximum_velocity_scale", "kAccelerationScale": "maximum_acceleration_scale",
                      "kPlanningTimeSeconds": "planning_time_s", "kExecutionDeadlineSeconds": "maximum_execution_time_s",
                      "kAttachedObjectClearance": "attached_object_clearance_m", "kSceneAgeSeconds": "maximum_scene_age_s",
                      "kMinimumConfidence": "minimum_object_confidence"}.items():
        lines.append(f"inline constexpr double {name} = {scalar(policy[key])};")
    lines.append(f"inline constexpr int kPlanningAttempts = {int(policy['planning_attempts'])};")
    home = policy["post_place_named_pose"]
    if home not in policy["allowed_named_poses"]:
        raise ValueError("post-place pose must be allowlisted")
    lines.append("inline const std::string kPostPlaceNamedPose = " + json.dumps(home) + ";")
    for group in ("pick", "place"):
        for key, value in policy[group].items():
            lines.append(f"inline constexpr double k_{group}_{key} = {scalar(value)};")
    for name, values in {"kAllowedObjects": policy["objects"], "kAllowedTargets": policy["targets"],
                         "kAllowedNamedPoses": policy["allowed_named_poses"],
                         "kAllowedSkills": set(policy["allowed_skills"]) - {"stop", "refuse", "inspect"}}.items():
        lines.append(f"inline const std::set<std::string> {name} = {strings(values)};")
    sizes = ", ".join("{" + json.dumps(key) + ", " + scalar(value["size_m"]) + "}"
                      for key, value in sorted(policy["objects"].items()))
    lines.append("inline const std::map<std::string, double> kObjectSizes = {" + sizes + "};")
    lower, upper = [], []
    for axis in ("x", "y", "z"):
        lo, hi = policy["workspace_m"][axis]
        if not lo < hi:
            raise ValueError("workspace bounds must be ordered")
        lower.append(lo)
        upper.append(hi)
    for name, values in {"kWorkspaceLower": lower, "kWorkspaceUpper": upper,
                         "kTablePosition": policy["table"]["center"], "kTableDimensions": policy["table"]["size"]}.items():
        lines.append(f"inline constexpr std::array<double, 3> {name} = {vector(values)};")
    return "\n".join([*lines, "}  // namespace vgm_safety", ""])


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config-directory", required=True)
    parser.add_argument("--output", required=True, type=Path)
    args = parser.parse_args()
    os.environ["VGM_CONFIG_DIRECTORY"] = args.config_directory
    args.output.write_text(generate(load_json_config("safety_policy.json")), encoding="utf-8")
