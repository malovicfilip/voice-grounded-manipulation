"""Deterministic colored-object grounding from synchronized RGB-D frames."""

from __future__ import annotations

import hashlib
import json
import math
import time
from dataclasses import dataclass
from collections.abc import Mapping
from typing import Any

import numpy as np

from .types import GroundedScene, ObjectObservation, TargetObservation


def _connected_components(mask: np.ndarray) -> list[tuple[np.ndarray, np.ndarray]]:
    """Return four-connected true-pixel regions without an OpenCV dependency."""
    height, width = mask.shape
    remaining = set(int(index) for index in np.flatnonzero(mask))
    components: list[tuple[np.ndarray, np.ndarray]] = []
    while remaining:
        seed = remaining.pop()
        stack = [seed]
        indices = [seed]
        while stack:
            index = stack.pop()
            row, column = divmod(index, width)
            for neighbor in (
                index - width if row > 0 else -1,
                index + width if row + 1 < height else -1,
                index - 1 if column > 0 else -1,
                index + 1 if column + 1 < width else -1,
            ):
                if neighbor in remaining:
                    remaining.remove(neighbor)
                    stack.append(neighbor)
                    indices.append(neighbor)
        values = np.asarray(indices, dtype=np.int64)
        components.append((values // width, values % width))
    return components


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
        self._expected_positions = {
            cube["object_id"]: tuple(float(value) for value in cube["position"])
            for cube in scene_config["cubes"]
        }
        self._object_sizes = {
            cube["object_id"]: float(cube["size_m"])
            for cube in scene_config["cubes"]
        }
        self._reference_colors = np.asarray(
            [item["color_rgb"] for item in (*scene_config["cubes"], *scene_config.get("targets", []))], dtype=np.float32
        )

    def ground(
        self,
        rgb: np.ndarray,
        depth_m: np.ndarray,
        intrinsics: CameraIntrinsics,
        camera_to_world: np.ndarray,
        *,
        captured_at_s: float | None = None,
        expected_positions: Mapping[str, tuple[float, float, float]] | None = None,
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
        # USD display colors are linear; RTX RGB may be display-transformed.
        # Keep both color encodings as appearance cues, never pose evidence.
        marker_colors = self._reference_colors[len(self._object_ids):]
        if len(marker_colors):
            srgb = np.where(marker_colors <= .0031308, 12.92 * marker_colors,
                            1.055 * marker_colors ** (1 / 2.4) - .055)
            distances[:, :, len(self._object_ids):] = np.minimum(
                distances[:, :, len(self._object_ids):],
                np.linalg.norm(colors[:, :, None, :] - srgb[None, None, :, :], axis=3))
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
            candidates = []
            expected = (
                expected_positions.get(object_id, self._expected_positions[object_id])
                if expected_positions is not None
                else self._expected_positions[object_id]
            )
            if len(expected) != 3 or not all(
                isinstance(value, (int, float)) and math.isfinite(value)
                for value in expected
            ):
                raise ValueError("expected object positions must be finite XYZ values")
            for rows, columns in _connected_components(mask):
                if rows.size < self.minimum_pixels:
                    continue
                object_depths = depth_array[rows, columns]
                median_depth = float(np.median(object_depths))
                camera_points = np.array(
                    [
                        (columns - intrinsics.cx) * object_depths / intrinsics.fx,
                        (rows - intrinsics.cy) * object_depths / intrinsics.fy,
                        object_depths,
                        np.ones_like(object_depths),
                    ],
                    dtype=np.float64,
                )
                world_points = transform @ camera_points
                if not np.isfinite(world_points).all() or np.any(np.abs(world_points[3]) < 1e-9):
                    continue
                points = (world_points[:3] / world_points[3]).T
                center = self._cube_surface_center(
                    points, transform[:3, 3], self._object_sizes[object_id]
                )
                if center is None:
                    continue
                position = center
                location_error = math.hypot(
                    position[0] - expected[0], position[1] - expected[1]
                )
                if location_error > 0.15:
                    continue

                component_distances = nearest_distance[rows, columns]
                mean_color_distance = float(np.mean(component_distances))
                median_absolute_deviation = float(
                    np.median(np.abs(object_depths - median_depth))
                )
                color_score = max(
                    0.0,
                    1.0 - mean_color_distance / self.color_distance_threshold,
                )
                area_score = min(
                    1.0, rows.size / (self.minimum_pixels * 4.0)
                )
                depth_score = max(
                    0.0, 1.0 - median_absolute_deviation / 0.03
                )
                location_score = max(0.0, 1.0 - location_error / 0.15)
                confidence = float(
                    0.20 * color_score
                    + 0.25 * area_score
                    + 0.20 * depth_score
                    + 0.35 * location_score
                )
                candidates.append(
                    (-confidence, location_error, -rows.size, position, confidence, rows.size)
                )
            if not candidates:
                continue
            # Inside the spatial gate, rank measured RGB/depth/pixel support
            # together with location. Nearest-only ranking can pick a tiny
            # reflection over the real cube for a submillimeter distance gain.
            _, _, _, position, confidence, pixel_count = min(candidates)
            observations[object_id] = ObjectObservation(
                object_id=object_id,
                position_m=position,
                confidence=confidence,
                observed_at_s=timestamp,
                frame_id="world",
                pixel_count=int(pixel_count),
            )

        targets = self._ground_targets(colors, finite_depth,
                                       depth_array, intrinsics, transform, timestamp)
        revision_payload = [
            {
                "object_id": object_id,
                # One-centimeter bins keep a static scene revision stable
                # across measured 1-2 mm RGB-D jitter. Confidence is enforced
                # separately and must not make an unchanged geometry stale.
                "position_bin_1cm": [
                    int(round(value / 0.01))
                    for value in observations[object_id].position_m
                ],
            }
            for object_id in sorted(observations)
        ]
        revision_payload.extend({"target_id": key, "position_bin_1cm":
                                 [int(round(v / 0.01)) for v in targets[key].position_m]}
                                for key in sorted(targets))
        revision = hashlib.sha256(
            json.dumps(revision_payload, sort_keys=True, separators=(",", ":")).encode()
        ).hexdigest()[:16]
        return GroundedScene(
            revision=revision,
            captured_at_s=timestamp,
            objects=observations,
            targets=targets,
        )

    @staticmethod
    def _cube_surface_center(points, camera_origin, size):
        """Fit the configured upright cube's center from visible surface points.

        A front-face median is a surface point, not a grasp center. A supported
        face spans a cube width along one horizontal axis and is nearly planar
        along the other; move half a known cube width away from the camera on
        that planar axis. Full top-face extents use their measured midpoint.
        Partial/ambiguous silhouettes are refused instead of snapped to hints.
        """
        bounds = np.quantile(points[:, :2], [.05, .95], axis=0)
        spans = bounds[1] - bounds[0]
        height_span = float(np.diff(np.quantile(points[:, 2], [.05, .95]))[0])
        if np.any(spans > size * 1.6) or not np.any(spans >= size * .5):
            return None
        center = []
        for axis in range(2):
            face = float(np.median(points[:, axis]))
            face_support = float(np.mean(np.abs(points[:, axis] - face) <= size * .05))
            if face_support >= .6 and height_span >= size * .4:
                # Other visible faces can widen the extrema while a majority
                # still lies on one near-facing vertical plane.
                direction = float(np.sign(face - camera_origin[axis]))
                if direction == 0:
                    return None
                center.append(face + direction * size / 2)
            elif spans[axis] >= size * .5:
                center.append(float((bounds[0, axis] + bounds[1, axis]) / 2))
            else:
                return None
        z_bounds = np.quantile(points[:, 2], [.05, .95])
        if height_span > size * 1.6:
            return None
        if height_span <= size * .15:
            # Horizontal visible face: half-size toward the cube interior.
            face = float(np.median(points[:, 2]))
            direction = float(np.sign(face - camera_origin[2]))
            if direction == 0:
                return None
            center.append(face + direction * size / 2)
        elif height_span >= size * .8:
            center.append(float((z_bounds[0] + z_bounds[1]) / 2))
        else:
            # A clipped side cannot establish height without a supported top.
            top = float(np.quantile(points[:, 2], .95))
            if camera_origin[2] <= top or np.mean(np.abs(points[:, 2] - top) < size * .05) < .15:
                return None
            center.append(top - size / 2)
        return tuple(center)

    def _ground_targets(self, colors, valid_depth, depth, intrinsics, transform, timestamp):
        """Fit a horizontal circular marker from measured RGB-D only.

        Color identifies the ID; radius/planarity distinguish markers from
        cubes and background. Circle fitting tolerates a centrally occluding
        placed cube. Incomplete or ambiguous observations fail closed.
        No authored target XYZ is consulted.
        """
        targets = {}
        for index, spec in enumerate(self.scene_config.get("targets", []), len(self._object_ids)):
            reference = self._reference_colors[index]
            srgb = np.where(reference <= .0031308, 12.92 * reference, 1.055 * reference ** (1 / 2.4) - .055)
            chroma = colors / np.maximum(colors.sum(axis=2, keepdims=True), 1e-9)
            chroma_error = np.minimum(np.linalg.norm(chroma - reference / reference.sum(), axis=2),
                                     np.linalg.norm(chroma - srgb / srgb.sum(), axis=2))
            appearance_error = np.minimum(np.linalg.norm(colors-reference, axis=2), np.linalg.norm(colors-srgb, axis=2))
            mask = (chroma_error <= .14) & (appearance_error <= self.color_distance_threshold) & valid_depth
            candidates = []
            for rows, cols in _connected_components(mask):
                if rows.size < self.minimum_pixels * 4:
                    continue
                d = depth[rows, cols]
                camera = np.array([(cols-intrinsics.cx)*d/intrinsics.fx,
                                   (rows-intrinsics.cy)*d/intrinsics.fy, d, np.ones_like(d)])
                world = transform @ camera
                points = (world[:3] / world[3]).T
                if not np.isfinite(points).all():
                    continue
                z = float(np.median(points[:, 2]))
                if np.quantile(np.abs(points[:, 2] - z), .95) > .004:
                    continue
                # Outer boundary per angular sector; a missing center is OK,
                # a missing side isn't. Fit twice to refine the initial center.
                xy = points[:, :2]
                center = (np.min(xy, axis=0) + np.max(xy, axis=0)) / 2
                boundary = None
                for _ in range(2):
                    delta = xy - center
                    sector = np.floor((np.arctan2(delta[:, 1], delta[:, 0]) + np.pi) * 24 / (2*np.pi)).astype(int) % 24
                    if len(np.unique(sector)) < 22:
                        boundary = None
                        break
                    boundary = np.array([xy[indices[np.argmax(np.linalg.norm(delta[indices], axis=1))]]
                                         for part in np.unique(sector)
                                         for indices in (np.flatnonzero(sector == part),)])
                    matrix = np.column_stack((2*boundary, np.ones(len(boundary))))
                    fit, _, rank, _ = np.linalg.lstsq(matrix, np.sum(boundary**2, axis=1), rcond=None)
                    if rank != 3:
                        boundary = None
                        break
                    center = fit[:2]
                if boundary is None:
                    continue
                radii = np.linalg.norm(boundary-center, axis=1)
                radius = float(np.median(radii))
                if abs(radius-spec["radius_m"]) > spec["radius_m"]*.15 or np.max(np.abs(radii-radius)) > .008:
                    continue
                score = .8 + .2 * max(0., 1. - float(np.mean(appearance_error[rows, cols])) / self.color_distance_threshold)
                candidates.append(TargetObservation(spec["target_id"], (float(center[0]), float(center[1]), z),
                                                     score, timestamp, pixel_count=int(rows.size)))
            if len(candidates) == 1:
                targets[spec["target_id"]] = candidates[0]
        return targets
