"""Synthetic RGB-D tests for deterministic object grounding."""

from __future__ import annotations

import json
import sys
import unittest
from pathlib import Path

import numpy as np


REPOSITORY_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPOSITORY_ROOT / "ros2_ws" / "src" / "vgm_runtime"))

from vgm_runtime.perception import CameraIntrinsics, ColorDepthGrounder  # noqa: E402


class PerceptionGroundingTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.config = json.loads(
            (REPOSITORY_ROOT / "config" / "phase_1_scene.json").read_text()
        )

    def test_exact_color_patch_backprojects_to_world(self):
        rgb = np.zeros((20, 20, 3), dtype=np.uint8)
        rgb[8:12, 8:12] = np.array([204, 13, 13], dtype=np.uint8)
        depth = np.full((20, 20), np.nan, dtype=np.float32)
        depth[8:12, 8:12] = 1.0
        grounder = ColorDepthGrounder(
            self.config, minimum_pixels=4, clock=lambda: 1234.0
        )
        scene = grounder.ground(
            rgb,
            depth,
            CameraIntrinsics(100.0, 100.0, 9.5, 9.5),
            np.array(
                [
                    [1.0, 0.0, 0.0, -0.1],
                    [0.0, 1.0, 0.0, -0.18],
                    [0.0, 0.0, 1.0, 0.0],
                    [0.0, 0.0, 0.0, 1.0],
                ]
            ),
        )
        observation = scene.objects["red_cube"]
        self.assertAlmostEqual(observation.position_m[0], -0.1, places=6)
        self.assertAlmostEqual(observation.position_m[1], -0.18, places=6)
        self.assertAlmostEqual(observation.position_m[2], 0.775, places=6)
        self.assertGreaterEqual(observation.confidence, 0.95)
        self.assertEqual(len(scene.revision), 16)

    def test_rejects_unsynchronized_shapes_and_invalid_transform(self):
        grounder = ColorDepthGrounder(self.config)
        intrinsics = CameraIntrinsics(100.0, 100.0, 1.0, 1.0)
        with self.assertRaises(ValueError):
            grounder.ground(
                np.zeros((2, 2, 3)), np.ones((3, 3)), intrinsics, np.eye(4)
            )
        with self.assertRaises(ValueError):
            grounder.ground(
                np.zeros((2, 2, 3)), np.ones((2, 2)), intrinsics, np.eye(3)
            )

    def test_ignores_pixels_without_finite_positive_depth(self):
        color = np.array(self.config["cubes"][0]["color_rgb"])
        rgb = np.broadcast_to(color, (20, 20, 3)).copy()
        depth = np.zeros((20, 20), dtype=np.float32)
        scene = ColorDepthGrounder(self.config).ground(
            rgb,
            depth,
            CameraIntrinsics(100.0, 100.0, 10.0, 10.0),
            np.eye(4),
            captured_at_s=1.0,
        )
        self.assertEqual(dict(scene.objects), {})

    def test_disconnected_same_color_marker_does_not_shift_cube(self):
        red = np.array([204, 13, 13], dtype=np.uint8)
        rgb = np.zeros((40, 40, 3), dtype=np.uint8)
        rgb[8:13, 8:13] = red
        rgb[25:35, 25:35] = red
        depth = np.full((40, 40), np.nan, dtype=np.float32)
        depth[8:13, 8:13] = 1.0
        depth[25:35, 25:35] = 1.0
        transform = np.eye(4)
        transform[0, 3] = -0.105
        transform[1, 3] = -0.185
        scene = ColorDepthGrounder(self.config, minimum_pixels=4).ground(
            rgb,
            depth,
            CameraIntrinsics(100.0, 100.0, 9.5, 9.5),
            transform,
            captured_at_s=1.0,
        )
        observation = scene.objects["red_cube"]
        self.assertLess(observation.position_m[0], 0.0)
        self.assertLess(observation.position_m[1], 0.0)
        self.assertEqual(observation.pixel_count, 25)

    def test_revision_ignores_sub_bin_jitter_but_detects_motion(self):
        red = np.array([204, 13, 13], dtype=np.uint8)
        rgb = np.zeros((20, 20, 3), dtype=np.uint8)
        rgb[8:12, 8:12] = red
        depth = np.full((20, 20), np.nan, dtype=np.float32)
        depth[8:12, 8:12] = 1.0
        intrinsics = CameraIntrinsics(100.0, 100.0, 9.5, 9.5)
        grounder = ColorDepthGrounder(self.config, minimum_pixels=4)

        def revision_with_x_offset(offset):
            transform = np.eye(4)
            transform[0, 3] = -0.1 + offset
            transform[1, 3] = -0.18
            return grounder.ground(
                rgb,
                depth,
                intrinsics,
                transform,
                captured_at_s=1.0,
            ).revision

        baseline = revision_with_x_offset(0.0)
        self.assertEqual(baseline, revision_with_x_offset(0.001))
        self.assertNotEqual(baseline, revision_with_x_offset(0.02))

    def test_expected_position_can_ground_a_relocated_object(self):
        red = np.array([204, 13, 13], dtype=np.uint8)
        rgb = np.zeros((30, 30, 3), dtype=np.uint8)
        rgb[13:18, 13:18] = red
        depth = np.full((30, 30), np.nan, dtype=np.float32)
        depth[13:18, 13:18] = 1.0
        transform = np.eye(4)
        transform[1, 3] = 0.3
        scene = ColorDepthGrounder(self.config, minimum_pixels=4).ground(
            rgb,
            depth,
            CameraIntrinsics(100.0, 100.0, 15.0, 15.0),
            transform,
            captured_at_s=1.0,
            expected_positions={"red_cube": (0.0, 0.3, 0.775)},
        )
        observation = scene.objects["red_cube"]
        self.assertAlmostEqual(observation.position_m[0], 0.0)
        self.assertAlmostEqual(observation.position_m[1], 0.3)
        self.assertGreaterEqual(observation.confidence, 0.95)


if __name__ == "__main__":
    unittest.main()
