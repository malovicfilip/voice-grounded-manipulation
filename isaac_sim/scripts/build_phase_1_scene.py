#!/usr/bin/env python3
"""Build the deterministic Phase 1 scene from its JSON contract."""

import argparse
import json
import math
from pathlib import Path


REPOSITORY_ROOT = Path(__file__).resolve().parents[2]
DEFAULT_CONFIG = REPOSITORY_ROOT / 'config' / 'phase_1_scene.json'
DEFAULT_OUTPUT = (
    REPOSITORY_ROOT / 'isaac_sim' / '_output' / 'phase_1_scene.usd'
)
DIRECT_CONTROL_FIELDS = frozenset(
    {
        'joint_targets',
        'joint_velocities',
        'motor_commands',
        'efforts',
        'torques',
        'trajectory',
        'trajectories',
    }
)


def _collect_keys(value):
    """Return every mapping key in a nested JSON-compatible value."""
    if isinstance(value, dict):
        keys = set(value)
        for child in value.values():
            keys.update(_collect_keys(child))
        return keys
    if isinstance(value, list):
        keys = set()
        for child in value:
            keys.update(_collect_keys(child))
        return keys
    return set()


def _load_config(config_path):
    """Load a UTF-8 JSON scene contract from disk."""
    with config_path.open(encoding='utf-8') as config_file:
        return json.load(config_file)


def _validate_contract(config):
    """Validate essential build and safety invariants."""
    if config.get('schema_version') != 1:
        raise ValueError('schema_version must be 1')
    if config.get('scene_id') != 'phase_1_franka_rgbd':
        raise ValueError('scene_id must be phase_1_franka_rgbd')
    if config.get('units') != 'meters':
        raise ValueError('scene units must be meters')

    scene_data = {
        key: value
        for key, value in config.items()
        if key != 'constraints'
    }
    found_control_fields = DIRECT_CONTROL_FIELDS & _collect_keys(scene_data)
    if found_control_fields:
        fields = ', '.join(sorted(found_control_fields))
        raise ValueError(f'direct-control fields are forbidden: {fields}')

    constraints = config['constraints']
    forbidden_metadata = set(constraints['forbidden_control_fields'])
    if not DIRECT_CONTROL_FIELDS.issubset(forbidden_metadata):
        raise ValueError(
            'forbidden_control_fields weakens the safety boundary'
        )

    cubes = config['cubes']
    expected_count = constraints['expected_cube_count']
    if expected_count != 6 or len(cubes) != expected_count:
        raise ValueError('the Phase 1 scene must contain exactly six cubes')
    if len({cube['object_id'] for cube in cubes}) != expected_count:
        raise ValueError('cube object_id values must be unique')
    if len({cube['prim_path'] for cube in cubes}) != expected_count:
        raise ValueError('cube prim_path values must be unique')

    if len(config['table']['legs']) != 4:
        raise ValueError('the Phase 1 table must contain four legs')
    streams = config['camera']['streams']
    if not streams.get('rgb') or not streams.get('depth'):
        raise ValueError('the workspace camera must enable RGB and depth')

    return {
        'camera_streams': ['rgb', 'depth'],
        'cube_count': len(cubes),
        'scene_id': config['scene_id'],
        'status': 'valid',
    }


def _set_transform(prim, translation, yaw_degrees=0.0):
    """Set a deterministic translation and optional Z rotation."""
    from pxr import Gf, UsdGeom

    xformable = UsdGeom.Xformable(prim)
    xformable.ClearXformOpOrder()
    xformable.AddTranslateOp().Set(Gf.Vec3d(*translation))
    if yaw_degrees:
        xformable.AddRotateZOp().Set(float(yaw_degrees))


def _create_box(
    stage,
    prim_path,
    center,
    size,
    color,
    *,
    mass_kg=None,
    object_id=None,
):
    """Create a visible collidable box, optionally as a rigid body."""
    from pxr import Gf, Sdf, UsdGeom, UsdPhysics

    box = UsdGeom.Cube.Define(stage, prim_path)
    box.CreateSizeAttr(1.0)
    box.CreateDisplayColorAttr([Gf.Vec3f(*color)])
    xformable = UsdGeom.Xformable(box.GetPrim())
    xformable.AddTranslateOp().Set(Gf.Vec3d(*center))
    xformable.AddScaleOp().Set(Gf.Vec3f(*size))
    UsdPhysics.CollisionAPI.Apply(box.GetPrim())

    if mass_kg is not None:
        UsdPhysics.RigidBodyAPI.Apply(box.GetPrim())
        mass_api = UsdPhysics.MassAPI.Apply(box.GetPrim())
        mass_api.CreateMassAttr(float(mass_kg))
    if object_id is not None:
        attribute = box.GetPrim().CreateAttribute(
            'vgm:objectId', Sdf.ValueTypeNames.String
        )
        attribute.Set(object_id)


def _create_physics(stage, config):
    """Create the stage physics scene from the contract rates."""
    from pxr import Gf, Sdf, UsdPhysics

    physics = config['physics']
    gravity = physics['gravity_m_s2']
    magnitude = math.sqrt(sum(component ** 2 for component in gravity))
    direction = [component / magnitude for component in gravity]

    scene = UsdPhysics.Scene.Define(stage, '/World/PhysicsScene')
    scene.CreateGravityDirectionAttr(Gf.Vec3f(*direction))
    scene.CreateGravityMagnitudeAttr(magnitude)
    scene.GetPrim().CreateAttribute(
        'vgm:physicsHz', Sdf.ValueTypeNames.Int
    ).Set(physics['physics_hz'])
    scene.GetPrim().CreateAttribute(
        'vgm:renderHz', Sdf.ValueTypeNames.Int
    ).Set(physics['render_hz'])


def _create_static_workspace(stage, config):
    """Create the ground, supported table, and scene lighting."""
    from pxr import Gf, UsdGeom, UsdLux

    ground = config['ground_plane']
    ground_size = ground['size']
    ground_center = [0.0, 0.0, ground['height_m'] - ground_size[2] / 2.0]
    _create_box(
        stage,
        ground['prim_path'],
        ground_center,
        ground_size,
        ground['color_rgb'],
    )

    table = config['table']
    UsdGeom.Xform.Define(stage, table['prim_path'])
    top = table['top']
    _create_box(
        stage,
        top['prim_path'],
        top['center'],
        top['size'],
        top['color_rgb'],
    )
    for leg in table['legs']:
        _create_box(
            stage,
            leg['prim_path'],
            leg['center'],
            leg['size'],
            top['color_rgb'],
        )

    lighting = config['lighting']
    dome_light = UsdLux.DomeLight.Define(stage, lighting['prim_path'])
    dome_light.CreateIntensityAttr(float(lighting['intensity']))
    dome_light.CreateColorAttr(Gf.Vec3f(*lighting['color_rgb']))


def _create_robot(stage, config, assets_root):
    """Place the referenced Franka asset without commanding it."""
    from pxr import Sdf, UsdGeom

    robot = config['robot']
    asset = config['assets'][robot['asset_key']]
    asset_path = assets_root.rstrip('/') + asset['relative_usd_path']
    robot_prim = UsdGeom.Xform.Define(stage, robot['prim_path']).GetPrim()
    robot_prim.GetReferences().AddReference(asset_path)
    pose = robot['base_pose']
    _set_transform(robot_prim, pose['position'], pose['yaw_degrees'])
    robot_prim.GetVariantSet('Gripper').SetVariantSelection(
        'AlternateFinger'
    )
    robot_prim.GetVariantSet('Mesh').SetVariantSelection('Quality')
    robot_prim.CreateAttribute(
        'vgm:objectId', Sdf.ValueTypeNames.String
    ).Set(robot['object_id'])


def _create_cubes(stage, config):
    """Create the six dynamic manipulation cubes."""
    from pxr import UsdGeom

    UsdGeom.Xform.Define(stage, '/World/Objects')
    for cube in config['cubes']:
        size = [cube['size_m']] * 3
        _create_box(
            stage,
            cube['prim_path'],
            cube['position'],
            size,
            cube['color_rgb'],
            mass_kg=cube['mass_kg'],
            object_id=cube['object_id'],
        )


def _create_target_markers(stage, config):
    """Create non-colliding visual placement targets on the table."""
    from pxr import Gf, Sdf, UsdGeom

    UsdGeom.Xform.Define(stage, '/World/Targets')
    for target in config.get('targets', []):
        marker = UsdGeom.Cylinder.Define(stage, target['prim_path'])
        marker.CreateAxisAttr(UsdGeom.Tokens.z)
        marker.CreateRadiusAttr(float(target['radius_m']))
        marker.CreateHeightAttr(0.002)
        marker.CreateDisplayColorAttr([Gf.Vec3f(*target['color_rgb'])])
        xformable = UsdGeom.Xformable(marker.GetPrim())
        xformable.AddTranslateOp().Set(Gf.Vec3d(*target['position']))
        marker.GetPrim().CreateAttribute(
            'vgm:targetId', Sdf.ValueTypeNames.String
        ).Set(target['target_id'])


def _create_camera(stage, config):
    """Create an Isaac Sim 6.0 RTX camera with RGB and depth outputs."""
    import isaacsim.core.experimental.utils.transform as transform_utils
    import numpy as np
    from isaacsim.sensors.experimental.rtx import CameraSensor, RtxCamera
    from pxr import Gf, Sdf, UsdGeom

    camera_config = config['camera']
    position = np.asarray(camera_config['position'], dtype=np.float32)
    orientation = transform_utils.look_at_quaternion(
        eye=position,
        target=np.asarray(camera_config['look_at'], dtype=np.float32),
    ).numpy()
    rtx_camera = RtxCamera(
        camera_config['prim_path'],
        tick_rate=float(config['physics']['render_hz']),
        positions=position,
        orientations=orientation,
    )
    camera = UsdGeom.Camera(
        stage.GetPrimAtPath(camera_config['prim_path'])
    )

    near, far = camera_config['clipping_range_m']
    camera.CreateClippingRangeAttr(Gf.Vec2f(near, far))
    camera.CreateFocalLengthAttr(float(camera_config['focal_length_mm']))
    camera.CreateHorizontalApertureAttr(
        float(camera_config['horizontal_aperture_mm'])
    )
    camera.CreateVerticalApertureAttr(
        float(camera_config['vertical_aperture_mm'])
    )
    camera.GetPrim().CreateAttribute(
        'vgm:sensorId', Sdf.ValueTypeNames.String
    ).Set(camera_config['sensor_id'])
    camera.GetPrim().CreateAttribute(
        'vgm:frameId', Sdf.ValueTypeNames.String
    ).Set(camera_config['frame_id'])

    width, height = camera_config['resolution']
    sensor = CameraSensor(
        rtx_camera,
        resolution=(height, width),
        annotators=['rgb', 'distance_to_image_plane'],
    )
    return sensor


def _author_scene(config):
    """Author the validated Phase 1 stage in the active Isaac Sim app."""
    import isaacsim.core.experimental.utils.stage as stage_utils
    from isaacsim.storage.native import get_assets_root_path
    from pxr import UsdGeom

    stage_utils.create_new_stage()
    stage = stage_utils.get_current_stage()
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    world = UsdGeom.Xform.Define(stage, '/World')
    stage.SetDefaultPrim(world.GetPrim())
    stage.SetTimeCodesPerSecond(config['physics']['physics_hz'])

    assets_root = get_assets_root_path()
    if not assets_root:
        raise RuntimeError('Isaac Sim assets root is unavailable')

    _create_physics(stage, config)
    _create_static_workspace(stage, config)
    _create_robot(stage, config, assets_root)
    _create_cubes(stage, config)
    _create_target_markers(stage, config)
    camera_sensor = _create_camera(stage, config)
    return stage, camera_sensor


def _build_scene(config, output_path, headless, summary):
    """Launch Isaac Sim, author the scene, and save the resulting USD."""
    from isaacsim import SimulationApp

    simulation_app = SimulationApp({'headless': headless})
    exit_code = 0
    try:
        import isaacsim.core.experimental.utils.stage as stage_utils

        stage, camera_sensor = _author_scene(config)
        simulation_app.update()

        output_path.parent.mkdir(parents=True, exist_ok=True)
        if not stage_utils.save_stage(str(output_path)):
            raise RuntimeError(f'failed to save scene to {output_path}')

        summary.update({'output': str(output_path), 'status': 'built'})
        print(json.dumps(summary, sort_keys=True), flush=True)
        return camera_sensor
    except BaseException:
        import traceback

        traceback.print_exc()
        exit_code = 1
        raise
    finally:
        simulation_app.close(skip_cleanup=True, exit_code=exit_code)


def _parse_args():
    """Parse standalone scene-builder arguments."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', type=Path, default=DEFAULT_CONFIG)
    parser.add_argument('--output', type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument(
        '--headless',
        action='store_true',
        help='run Isaac Sim without its graphical interface',
    )
    parser.add_argument(
        '--validate-only',
        action='store_true',
        help='validate the JSON contract without importing Isaac Sim',
    )
    return parser, parser.parse_args()


def main():
    """Validate arguments and run the requested builder mode."""
    parser, args = _parse_args()
    try:
        config = _load_config(args.config.resolve())
        summary = _validate_contract(config)
        if args.validate_only:
            print(json.dumps(summary, sort_keys=True))
            return
        output_path = args.output.resolve()
        if output_path.suffix.lower() not in {'.usd', '.usda', '.usdc'}:
            raise ValueError('output must use .usd, .usda, or .usdc')
        _build_scene(config, output_path, args.headless, summary)
    except (KeyError, OSError, TypeError, ValueError) as error:
        parser.error(str(error))


if __name__ == '__main__':
    main()
