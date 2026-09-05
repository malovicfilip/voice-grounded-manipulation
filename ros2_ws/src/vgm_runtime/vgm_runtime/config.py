"""Configuration loading with source-tree and ROS-install fallbacks."""

from __future__ import annotations

import json
import hashlib
import os
from pathlib import Path
from typing import Any


def _source_config_directory() -> Path:
    return Path(__file__).resolve().parents[4] / "config"


def config_directory() -> Path:
    """Return the configured project policy directory."""
    override = os.environ.get("VGM_CONFIG_DIRECTORY")
    if override:
        return Path(override).expanduser().resolve()

    source_directory = _source_config_directory()
    if source_directory.is_dir():
        return source_directory

    try:
        from ament_index_python.packages import get_package_share_directory

        return Path(get_package_share_directory("vgm_runtime")) / "config"
    except (ImportError, LookupError) as error:
        raise RuntimeError("VGM configuration directory is unavailable") from error


def load_json_config(name: str) -> dict[str, Any]:
    path = config_directory() / name
    with path.open(encoding="utf-8") as config_file:
        value = json.load(config_file)
    if not isinstance(value, dict):
        raise ValueError(f"configuration must contain an object: {path}")
    if name == "safety_policy.json":
        geometry = load_json_config(value["scene_geometry_file"])
        # Author geometry once. Spawn poses are NOT runtime target grounding.
        value["objects"] = {item["object_id"]: {"size_m": item["size_m"]}
                            for item in geometry["cubes"]}
        value["targets"] = {item["target_id"]: {"radius_m": item["radius_m"]}
                            for item in geometry["targets"]}
        value["table"] = geometry["table"]["top"]
    return value


def policy_digest(policy: dict[str, Any]) -> str:
    return hashlib.sha256(json.dumps(policy, sort_keys=True, separators=(",", ":"),
                                     allow_nan=False).encode()).hexdigest()
