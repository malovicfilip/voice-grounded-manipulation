#!/usr/bin/env python3
"""Capture visible RGB and metric-depth evidence from the Phase 1 scene."""

import argparse
import json
import math
import traceback
from pathlib import Path

from build_phase_1_scene import (
    DEFAULT_CONFIG,
    DEFAULT_OUTPUT as DEFAULT_SCENE,
    _author_scene,
    _load_config,
    _validate_contract,
)


DEFAULT_OUTPUT_DIRECTORY = (
    Path(__file__).resolve().parents[1] / '_output' / 'phase_1_demo'
)


def _validate_capture(config, frame_count):
    """Validate the scene contract and capture arguments without Isaac Sim."""
    summary = _validate_contract(config)
    if frame_count < 1 or frame_count > 10:
        raise ValueError('frames must be between 1 and 10')
    return {
        **summary,
        'capture_frames': frame_count,
        'status': 'ready',
    }


def _write_depth_preview(depth_path, preview_path):
    """Create a viewable depth preview while retaining metric float data."""
    import matplotlib.pyplot as plt
    import numpy as np

    depth = np.load(depth_path)
    finite_depth = depth[np.isfinite(depth) & (depth > 0.0)]
    if finite_depth.size == 0:
        raise RuntimeError('captured depth map contains no finite positive data')

    minimum = float(finite_depth.min())
    maximum = float(finite_depth.max())
    display = np.where(np.isfinite(depth), depth, maximum)
    plt.imsave(
        preview_path,
        display,
        cmap='viridis_r',
        vmin=minimum,
        vmax=maximum if maximum > minimum else minimum + 1e-6,
    )
    return minimum, maximum


def _capture(scene_path, output_directory, config, summary):
    """Build and render RGB-D frames without robot actuation."""
    from isaacsim import SimulationApp

    simulation_app = SimulationApp({'headless': True})
    exit_code = 0
    try:
        import carb.settings
        import matplotlib.pyplot as plt
        import numpy as np
        import omni.timeline
        import isaacsim.core.experimental.utils.stage as stage_utils

        if output_directory.exists() and any(output_directory.iterdir()):
            raise ValueError(
                f'demo output directory is not empty: {output_directory}'
            )
        output_directory.mkdir(parents=True, exist_ok=True)

        print('Authoring validated Phase 1 scene', flush=True)
        stage, camera_sensor = _author_scene(config)
        simulation_app.update()
        scene_path.parent.mkdir(parents=True, exist_ok=True)
        if not stage_utils.save_stage(str(scene_path)):
            raise RuntimeError(f'failed to save scene to {scene_path}')
        print('Scene ready; capturing RGB-D frame', flush=True)

        camera = config['camera']
        width, height = camera['resolution']
        carb.settings.get_settings().set('rtx/post/dlss/execMode', 2)
        timeline = omni.timeline.get_timeline_interface()
        timeline.play()

        captures = []
        for capture_index in range(summary['capture_frames']):
            for _ in range(120):
                simulation_app.update()
                rgb_data, _ = camera_sensor.get_data('rgb')
                depth_data, _ = camera_sensor.get_data(
                    'distance_to_image_plane'
                )
                if rgb_data is not None and depth_data is not None:
                    break
            else:
                raise RuntimeError(
                    'RGB-D sensor produced no data after 120 updates'
                )

            rgb = rgb_data.numpy()
            depth = np.squeeze(depth_data.numpy())
            rgb_path = output_directory / f'rgb_{capture_index:04d}.png'
            depth_path = output_directory / f'depth_m_{capture_index:04d}.npy'
            preview_path = (
                output_directory / f'depth_preview_{capture_index:04d}.png'
            )
            plt.imsave(rgb_path, rgb)
            np.save(depth_path, depth)
            depth_minimum, depth_maximum = _write_depth_preview(
                depth_path,
                preview_path,
            )
            if not math.isfinite(depth_minimum) or not math.isfinite(
                depth_maximum
            ):
                raise RuntimeError('captured depth range is not finite')
            captures.append(
                {
                    'depth_preview': preview_path.name,
                    'depth_range_m': [depth_minimum, depth_maximum],
                    'depth_raw': depth_path.name,
                    'rgb': rgb_path.name,
                }
            )
        timeline.stop()

        result = {
            **summary,
            'camera_resolution': [width, height],
            'captures': captures,
            'output_directory': str(output_directory),
            'scene': str(scene_path),
            'status': 'captured',
        }
        manifest_path = output_directory / 'manifest.json'
        with manifest_path.open('w', encoding='utf-8') as manifest_file:
            json.dump(result, manifest_file, indent=2, sort_keys=True)
            manifest_file.write('\n')
        print(json.dumps(result, sort_keys=True), flush=True)
    except BaseException:
        traceback.print_exc()
        exit_code = 1
        raise
    finally:
        simulation_app.close(skip_cleanup=True, exit_code=exit_code)


def _parse_args():
    """Parse capture arguments."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', type=Path, default=DEFAULT_CONFIG)
    parser.add_argument('--scene', type=Path, default=DEFAULT_SCENE)
    parser.add_argument(
        '--output-directory',
        type=Path,
        default=DEFAULT_OUTPUT_DIRECTORY,
    )
    parser.add_argument('--frames', type=int, default=1)
    parser.add_argument(
        '--validate-only',
        action='store_true',
        help='validate inputs without importing Isaac Sim',
    )
    return parser, parser.parse_args()


def main():
    """Validate the request and capture RGB-D evidence."""
    parser, args = _parse_args()
    try:
        config = _load_config(args.config.resolve())
        summary = _validate_capture(config, args.frames)
        if args.validate_only:
            print(json.dumps(summary, sort_keys=True))
            return
        _capture(
            args.scene.resolve(),
            args.output_directory.resolve(),
            config,
            summary,
        )
    except (KeyError, OSError, TypeError, ValueError) as error:
        parser.error(str(error))


if __name__ == '__main__':
    main()
