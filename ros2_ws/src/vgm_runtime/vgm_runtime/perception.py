"""Deterministic colored-object grounding from synchronized RGB-D frames."""

from __future__ import annotations

import hashlib
import json
import math
import time
from dataclasses import dataclass
from typing import Any

import numpy as np

from .types import GroundedScene, ObjectObservation


@dataclass(frozen=True, slots=True)
class CameraIntrinsics:
    fx: float
    fy: float
    cx: float
    cy: float
    frame_id: str = "workspace_camera_optical_frame"

    def __post_init__(self) -> None:
        if min(self.fx, self.fy) <= 0.0:
            raise ValueError("camera focal lengths must be positive")


class ColorDepthGrounder:
    """Ground the configured colored cubes without learned-model ambiguity."""

    def __init__(
        self,
        scene_config: dict[str, Any],
        *,
        color_distance_threshold: float = 0.38,
        minimum_pixels: int = 20,
        clock=time.time,
    ) -> None:
        self.scene_config = scene_config
        self.color_distance_threshold = float(color_distance_threshold)
        self.minimum_pixels = int(minimum_pixels)
        self.clock = clock
        if not 0.0 < self.color_distance_threshold <= math.sqrt(3.0):
            raise ValueError("color distance threshold is invalid")
        if self.minimum_pixels < 1:
            raise ValueError("minimum_pixels must be positive")
        self._object_ids = tuple(cube["object_id"] for cube in scene_config["cubes"])
        surface_height = float(scene_config["table"]["surface_height_m"])
        self._object_center_z = {
            cube["object_id"]: surface_height + float(cube["size_m"]) / 2.0
            for cube in scene_config["cubes"]
        }
        self._reference_colors = np.asarray(
            [cube["color_rgb"] for cube in scene_config["cubes"]], dtype=np.float32
        )

    def ground(
        self,
        rgb: np.ndarray,
        depth_m: np.ndarray,
        intrinsics: CameraIntrinsics,
        camera_to_world: np.ndarray,
        *,
        captured_at_s: float | None = None,
    ) -> GroundedScene:
        """Return observations for colors with sufficient RGB-D support."""
        rgb_array = np.asarray(rgb)
        depth_array = np.asarray(depth_m, dtype=np.float64).squeeze()
        if rgb_array.ndim != 3 or rgb_array.shape[2] not in (3, 4):
            raise ValueError("rgb must have shape (height, width, 3 or 4)")
        if depth_array.shape != rgb_array.shape[:2]:
            raise ValueError("rgb and depth dimensions differ")
        transform = np.asarray(camera_to_world, dtype=np.float64)
        if transform.shape != (4, 4) or not np.isfinite(transform).all():
            raise ValueError("camera_to_world must be a finite 4x4 matrix")

        colors = rgb_array[..., :3].astype(np.float32)
        if np.issubdtype(rgb_array.dtype, np.integer):
            colors /= 255.0
        if np.any(colors < 0.0) or np.any(colors > 1.0):
            raise ValueError("rgb values must be in [0, 1]")

        distances = np.linalg.norm(
            colors[:, :, None, :] - self._reference_colors[None, None, :, :],
            axis=3,
        )
        nearest = np.argmin(distances, axis=2)
        nearest_distance = np.min(distances, axis=2)
        finite_depth = np.isfinite(depth_array) & (depth_array > 0.0)
        timestamp = float(self.clock() if captured_at_s is None else captured_at_s)

        observations: dict[str, ObjectObservation] = {}
        for color_index, object_id in enumerate(self._object_ids):
            mask = (
                (nearest == color_index)
                & (nearest_distance <= self.color_distance_threshold)
                & finite_depth
            )
            rows, columns = np.nonzero(mask)
            if rows.size < self.minimum_pixels:
                continue

            object_depths = depth_array[rows, columns]
            median_depth = float(np.median(object_depths))
            median_row = float(np.median(rows))
            median_column = float(np.median(columns))
            camera_point = np.array(
                [
                    (median_column - intrinsics.cx) * median_depth / intrinsics.fx,
                    (median_row - intrinsics.cy) * median_depth / intrinsics.fy,
                    median_depth,
                    1.0,
                ],
                dtype=np.float64,
            )
            world_point = transform @ camera_point
            if not np.isfinite(world_point[:3]).all() or abs(world_point[3]) < 1e-9:
                continue
            projected = [float(value / world_point[3]) for value in world_point[:3]]
            # RGB-D supplies the horizontal location. The configured tabletop
            # and known cube size remove the visible-surface bias from depth so
            # planning receives the physical cube center, not its top face.
            position = (
                projected[0],
                projected[1],
                self._object_center_z[object_id],
            )

            mean_color_distance = float(np.mean(nearest_distance[mask]))
            median_absolute_deviation = float(
                np.median(np.abs(object_depths - median_depth))
            )
            color_score = max(
                0.0, 1.0 - mean_color_distance / self.color_distance_threshold
            )
            area_score = min(1.0, rows.size / (self.minimum_pixels * 4.0))
            depth_score = max(0.0, 1.0 - median_absolute_deviation / 0.03)
            confidence = float(0.55 * color_score + 0.25 * area_score + 0.20 * depth_score)
            observations[object_id] = ObjectObservation(
                object_id=object_id,
                position_m=position,
                confidence=confidence,
                observed_at_s=timestamp,
                frame_id="world",
                pixel_count=int(rows.size),
            )

        revision_payload = [
            {
                "object_id": object_id,
                # Millimeter quantization keeps a static scene revision stable
                # across harmless renderer/depth jitter while still detecting
                # meaningful object motion before execution.
                "position_m": [round(value, 3) for value in observations[object_id].position_m],
                "confidence": round(observations[object_id].confidence, 4),
            }
            for object_id in sorted(observations)
        ]
        revision = hashlib.sha256(
            json.dumps(revision_payload, sort_keys=True, separators=(",", ":")).encode()
        ).hexdigest()[:16]
        return GroundedScene(
            revision=revision,
            captured_at_s=timestamp,
            objects=observations,
        )
