"""Offline acceptance tests for the versioned Phase 1 scene contract."""

import json
import math
import unittest
from pathlib import Path


REPOSITORY_ROOT = Path(__file__).resolve().parents[1]
CONFIG_PATH = REPOSITORY_ROOT / 'config' / 'phase_1_scene.json'


def collect_keys(value):
    """Return every mapping key in a nested JSON-compatible value."""
    if isinstance(value, dict):
        keys = set(value)
        for child in value.values():
            keys.update(collect_keys(child))
        return keys
    if isinstance(value, list):
        keys = set()
        for child in value:
            keys.update(collect_keys(child))
        return keys
    return set()


class Phase1SceneConfigTest(unittest.TestCase):
    """Validate invariants that do not require an Isaac Sim runtime."""

    @classmethod
    def setUpClass(cls):
        """Load the shared scene contract once for the test class."""
        with CONFIG_PATH.open(encoding='utf-8') as config_file:
            cls.config = json.load(config_file)

    def test_contract_identity_and_units(self):
        """Require an explicitly versioned, metric Phase 1 contract."""
        self.assertEqual(self.config['schema_version'], 1)
        self.assertEqual(self.config['scene_id'], 'phase_1_franka_rgbd')
        self.assertEqual(self.config['units'], 'meters')

    def test_franka_asset_and_prim_are_explicit(self):
        """Pin the Franka asset and its stable scene path."""
        robot = self.config['robot']
        asset = self.config['assets'][robot['asset_key']]

        self.assertEqual(robot['object_id'], 'franka_panda')
        self.assertEqual(robot['prim_path'], '/World/Franka')
        self.assertEqual(
            asset['relative_usd_path'],
            '/Isaac/Robots/FrankaRobotics/FrankaPanda/franka.usd',
        )

    def test_six_cubes_have_unique_identity_color_and_path(self):
        """Require six distinguishable manipulation objects."""
        cubes = self.config['cubes']
        expected_count = self.config['constraints']['expected_cube_count']

        self.assertEqual(len(cubes), expected_count)
        self.assertEqual(
            len({cube['object_id'] for cube in cubes}), expected_count
        )
        self.assertEqual(
            len({cube['prim_path'] for cube in cubes}), expected_count
        )
        self.assertEqual(
            len({tuple(cube['color_rgb']) for cube in cubes}), expected_count
        )

        for cube in cubes:
            self.assertTrue(
                cube['prim_path'].startswith('/World/Objects/')
            )
            self.assertGreater(cube['size_m'], 0.0)
            self.assertGreater(cube['mass_kg'], 0.0)
            self.assertTrue(
                all(
                    0.0 <= channel <= 1.0
                    for channel in cube['color_rgb']
                )
            )

    def test_cubes_rest_on_table_with_clearance_and_no_overlap(self):
        """Keep every cube supported, in bounds, and separated."""
        table = self.config['table']
        table_center = table['top']['center']
        table_size = table['top']['size']
        surface_height = table['surface_height_m']
        edge_clearance = self.config['constraints'][
            'table_edge_clearance_m'
        ]
        cubes = self.config['cubes']

        expected_surface_height = table_center[2] + table_size[2] / 2.0
        self.assertAlmostEqual(surface_height, expected_surface_height)

        for cube in cubes:
            x, y, z = cube['position']
            half_size = cube['size_m'] / 2.0
            self.assertAlmostEqual(z - half_size, surface_height)
            self.assertLessEqual(
                abs(x - table_center[0]) + half_size + edge_clearance,
                table_size[0] / 2.0,
            )
            self.assertLessEqual(
                abs(y - table_center[1]) + half_size + edge_clearance,
                table_size[1] / 2.0,
            )

        minimum_separation = self.config['constraints'][
            'minimum_cube_center_separation_m'
        ]
        for index, cube in enumerate(cubes):
            for other_cube in cubes[index + 1:]:
                distance = math.dist(
                    cube['position'], other_cube['position']
                )
                self.assertGreaterEqual(distance, minimum_separation)

    def test_camera_contract_requires_rgb_and_depth(self):
        """Require deterministic RGB and depth camera outputs."""
        camera = self.config['camera']

        self.assertEqual(camera['sensor_id'], 'workspace_rgbd')
        self.assertTrue(camera['streams']['rgb'])
        self.assertTrue(camera['streams']['depth'])
        self.assertEqual(camera['resolution'], [640, 480])
        self.assertLess(
            camera['clipping_range_m'][0], camera['clipping_range_m'][1]
        )
        self.assertNotEqual(camera['position'], camera['look_at'])

    def test_contract_contains_no_direct_control_fields(self):
        """Reject low-level robot-control fields from scene configuration."""
        forbidden_fields = set(
            self.config['constraints']['forbidden_control_fields']
        )
        scene_without_constraint_metadata = {
            key: value
            for key, value in self.config.items()
            if key != 'constraints'
        }

        self.assertTrue(forbidden_fields)
        self.assertTrue(
            forbidden_fields.isdisjoint(
                collect_keys(scene_without_constraint_metadata)
            )
        )


if __name__ == '__main__':
    unittest.main()
