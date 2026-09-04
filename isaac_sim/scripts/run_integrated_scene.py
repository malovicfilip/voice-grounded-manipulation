#!/usr/bin/env python3
"""Run the Phase 1 RGB-D scene with NVIDIA's ROS 2 MoveIt bridge.

This simulator process never accepts language or generates motion. It only
publishes measured joint state, applies commands emitted by ros2_control, and
captures synchronized RGB-D evidence on explicit file-based requests.
"""

import argparse
import json
import re
import sys
import time
from pathlib import Path

import numpy as np
from isaacsim import SimulationApp


SCRIPT_DIRECTORY = Path(__file__).resolve().parent
REPOSITORY_ROOT = SCRIPT_DIRECTORY.parents[1]
RUNTIME_SOURCE = REPOSITORY_ROOT / "ros2_ws" / "src" / "vgm_runtime"
sys.path.insert(0, str(SCRIPT_DIRECTORY))
sys.path.insert(0, str(RUNTIME_SOURCE))


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path)
    parser.add_argument("--headless", action="store_true")
    parser.add_argument("--max-frames", type=int, default=0)
    parser.add_argument("--output-directory", type=Path, required=True)
    args, _ = parser.parse_known_args()
    if args.max_frames < 0:
        parser.error("--max-frames may not be negative")
    return args


args = _parse_args()
simulation_app = SimulationApp(
    {"renderer": "RealTimePathTracing", "headless": args.headless}
)

import isaacsim.core.experimental.utils.app as app_utils  # noqa: E402
import isaacsim.core.experimental.utils.stage as stage_utils  # noqa: E402
import omni.graph.core as og  # noqa: E402
import usdrt.Sdf  # noqa: E402
from isaacsim.core.rendering_manager import ViewportManager  # noqa: E402
from isaacsim.core.simulation_manager import SimulationManager  # noqa: E402

from build_phase_1_scene import (  # noqa: E402
    DEFAULT_CONFIG,
    _author_scene,
    _load_config,
    _validate_contract,
)
from vgm_runtime.perception import CameraIntrinsics, ColorDepthGrounder  # noqa: E402


def _create_ros_action_graph(robot_prim_path: str) -> None:
    """Wire only the standard Isaac joint-state/ros2_control boundary."""
    og.Controller.edit(
        {"graph_path": "/ActionGraph", "evaluator_name": "execution"},
        {
            og.Controller.Keys.CREATE_NODES: [
                ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                ("ReadSimTime", "isaacsim.core.nodes.IsaacReadSimulationTime"),
                ("ReadJointState", "isaacsim.sensors.physics.IsaacReadJointState"),
                ("Context", "isaacsim.ros2.bridge.ROS2Context"),
                ("PublishJointState", "isaacsim.ros2.bridge.ROS2PublishJointState"),
                ("SubscribeJointState", "isaacsim.ros2.bridge.ROS2SubscribeJointState"),
                (
                    "ArticulationController",
                    "isaacsim.core.nodes.IsaacArticulationController",
                ),
                ("PublishClock", "isaacsim.ros2.bridge.ROS2PublishClock"),
            ],
            og.Controller.Keys.CONNECT: [
                ("OnPlaybackTick.outputs:tick", "ReadJointState.inputs:execIn"),
                ("ReadJointState.outputs:execOut", "PublishJointState.inputs:execIn"),
                ("ReadJointState.outputs:jointNames", "PublishJointState.inputs:jointNames"),
                ("ReadJointState.outputs:jointPositions", "PublishJointState.inputs:jointPositions"),
                ("ReadJointState.outputs:jointVelocities", "PublishJointState.inputs:jointVelocities"),
                ("ReadJointState.outputs:jointEfforts", "PublishJointState.inputs:jointEfforts"),
                ("ReadJointState.outputs:jointDofTypes", "PublishJointState.inputs:jointDofTypes"),
                ("ReadJointState.outputs:stageMetersPerUnit", "PublishJointState.inputs:stageMetersPerUnit"),
                ("ReadJointState.outputs:sensorTime", "PublishJointState.inputs:sensorTime"),
                ("OnPlaybackTick.outputs:tick", "SubscribeJointState.inputs:execIn"),
                ("OnPlaybackTick.outputs:tick", "PublishClock.inputs:execIn"),
                ("OnPlaybackTick.outputs:tick", "ArticulationController.inputs:execIn"),
                ("Context.outputs:context", "PublishJointState.inputs:context"),
                ("Context.outputs:context", "SubscribeJointState.inputs:context"),
                ("Context.outputs:context", "PublishClock.inputs:context"),
                ("ReadSimTime.outputs:simulationTime", "PublishClock.inputs:timeStamp"),
                ("SubscribeJointState.outputs:jointNames", "ArticulationController.inputs:jointNames"),
                ("SubscribeJointState.outputs:positionCommand", "ArticulationController.inputs:positionCommand"),
                ("SubscribeJointState.outputs:velocityCommand", "ArticulationController.inputs:velocityCommand"),
                ("SubscribeJointState.outputs:effortCommand", "ArticulationController.inputs:effortCommand"),
            ],
            og.Controller.Keys.SET_VALUES: [
                ("ArticulationController.inputs:robotPath", robot_prim_path),
                ("ReadJointState.inputs:prim", [usdrt.Sdf.Path(robot_prim_path)]),
                ("PublishJointState.inputs:topicName", "isaac_joint_states"),
                ("SubscribeJointState.inputs:topicName", "isaac_joint_commands"),
            ],
        },
    )


def _camera_calibration(camera_config: dict) -> tuple[CameraIntrinsics, np.ndarray]:
    """Derive pinhole intrinsics and the ROS optical-frame world transform."""
    width, height = camera_config["resolution"]
    focal = float(camera_config["focal_length_mm"])
    intrinsics = CameraIntrinsics(
        fx=focal * width / float(camera_config["horizontal_aperture_mm"]),
        fy=focal * height / float(camera_config["vertical_aperture_mm"]),
        cx=(width - 1.0) / 2.0,
        cy=(height - 1.0) / 2.0,
        frame_id=camera_config["frame_id"],
    )

    eye = np.asarray(camera_config["position"], dtype=np.float64)
    forward = np.asarray(camera_config["look_at"], dtype=np.float64) - eye
    forward /= np.linalg.norm(forward)
    right = np.cross(forward, np.asarray([0.0, 0.0, 1.0]))
    right /= np.linalg.norm(right)
    down = np.cross(forward, right)
    down /= np.linalg.norm(down)
    camera_to_world = np.eye(4, dtype=np.float64)
    camera_to_world[:3, :3] = np.column_stack((right, down, forward))
    camera_to_world[:3, 3] = eye
    return intrinsics, camera_to_world


def _tensor_to_numpy(value) -> np.ndarray | None:
    if value is None:
        return None
    return np.asarray(value.numpy())


def _capture_rgbd(
    prefix: str,
    rgb: np.ndarray,
    depth_m: np.ndarray,
    output_directory: Path,
    grounder: ColorDepthGrounder,
    intrinsics: CameraIntrinsics,
    camera_to_world: np.ndarray,
    expected_positions=None,
) -> None:
    import matplotlib.pyplot as plt

    captured_at_s = time.time()
    scene = grounder.ground(
        rgb,
        depth_m,
        intrinsics,
        camera_to_world,
        captured_at_s=captured_at_s,
        expected_positions=expected_positions,
    )
    plt.imsave(output_directory / f"{prefix}.png", rgb)
    np.save(output_directory / f"{prefix}_depth_m.npy", depth_m)
    calibration = {
        "camera_to_world": camera_to_world.tolist(),
        "frame_id": intrinsics.frame_id,
        "fx": intrinsics.fx,
        "fy": intrinsics.fy,
        "cx": intrinsics.cx,
        "cy": intrinsics.cy,
    }
    (output_directory / f"{prefix}_calibration.json").write_text(
        json.dumps(calibration, indent=2) + "\n", encoding="utf-8"
    )
    (output_directory / f"{prefix}_grounded_scene.json").write_text(
        json.dumps(scene.to_mapping(), indent=2) + "\n", encoding="utf-8"
    )
    (output_directory / f"{prefix}.ready").touch()
    print(
        f"VGM_RGBD_CAPTURE prefix={prefix} revision={scene.revision} "
        f"objects={len(scene.objects)}",
        flush=True,
    )


def main() -> int:
    output_directory = args.output_directory.resolve()
    output_directory.mkdir(parents=True, exist_ok=True)
    config_path = (args.config or DEFAULT_CONFIG).resolve()
    config = _load_config(config_path)
    _validate_contract(config)

    app_utils.enable_extension("isaacsim.ros2.bridge")
    simulation_app.update()
    stage, camera_sensor = _author_scene(config)
    robot_path = config["robot"]["prim_path"]
    _create_ros_action_graph(robot_path)
    ViewportManager.set_camera_view(
        "/OmniverseKit_Persp",
        eye=np.asarray([1.2, 1.2, 1.25]),
        target=np.asarray([0.0, 0.0, 0.75]),
    )
    simulation_app.update()
    stage_utils.save_stage(str(output_directory / "integrated_scene.usd"))

    intrinsics, camera_to_world = _camera_calibration(config["camera"])
    grounder = ColorDepthGrounder(config)
    SimulationManager.setup_simulation(dt=1.0 / 60.0, device="cpu")
    app_utils.play()
    simulation_app.update()
    (output_directory / "bridge.ready").touch()
    print("VGM_ISAAC_INTEGRATED_BRIDGE_READY", flush=True)

    frame_count = 0
    initial_captured = False
    verification_captured = False
    final_settle_frames = 0
    processed_captures = set()
    while simulation_app.is_running():
        simulation_app.update()
        frame_count += 1
        rgb_data, _ = camera_sensor.get_data("rgb")
        depth_data, _ = camera_sensor.get_data("distance_to_image_plane")
        rgb = _tensor_to_numpy(rgb_data)
        depth = _tensor_to_numpy(depth_data)
        if depth is not None:
            depth = np.asarray(depth, dtype=np.float32).squeeze()

        # Reusable sessions request uniquely named captures. These messages
        # contain perception hints only and never robot-control instructions.
        if rgb is not None and depth is not None:
            for request in sorted(output_directory.glob("capture-*.json")):
                if request.name in processed_captures:
                    continue
                if not re.fullmatch(r"capture-[a-f0-9]{16}\.json", request.name):
                    continue
                value = json.loads(request.read_text(encoding="utf-8"))
                if set(value) != {"expected_positions"}:
                    raise ValueError("capture request fields are invalid")
                expected = value["expected_positions"]
                if not isinstance(expected, dict) or set(expected) - set(grounder._object_ids):
                    raise ValueError("capture hints contain unknown objects")
                prefix = request.stem
                _capture_rgbd(prefix, rgb, depth, output_directory, grounder,
                              intrinsics, camera_to_world, expected)
                processed_captures.add(request.name)

        initial_request = output_directory / "capture_initial.request"
        if (
            not initial_captured
            and initial_request.exists()
            and rgb is not None
            and depth is not None
        ):
            _capture_rgbd(
                "initial", rgb, depth, output_directory, grounder,
                intrinsics, camera_to_world,
            )
            initial_captured = True

        verification_request = output_directory / "capture_verification.request"
        if (
            initial_captured
            and not verification_captured
            and verification_request.exists()
            and rgb is not None
            and depth is not None
        ):
            _capture_rgbd(
                "verification", rgb, depth, output_directory, grounder,
                intrinsics, camera_to_world,
            )
            verification_captured = True

        final_request = output_directory / "capture_final.request"
        if initial_captured and final_request.exists():
            final_settle_frames += 1
            if final_settle_frames >= 30 and rgb is not None and depth is not None:
                outcome = json.loads(
                    (output_directory / "expected_outcome.json").read_text(
                        encoding="utf-8"
                    )
                )
                if set(outcome) != {"object_id", "target_id"}:
                    raise ValueError("expected outcome fields are invalid")
                cube = next(
                    item
                    for item in config["cubes"]
                    if item["object_id"] == outcome["object_id"]
                )
                target = next(
                    item
                    for item in config["targets"]
                    if item["target_id"] == outcome["target_id"]
                )
                expected_positions = {
                    cube["object_id"]: (
                        float(target["position"][0]),
                        float(target["position"][1]),
                        float(config["table"]["surface_height_m"])
                        + float(cube["size_m"]) / 2.0,
                    )
                }
                _capture_rgbd(
                    "final", rgb, depth, output_directory, grounder,
                    intrinsics, camera_to_world, expected_positions,
                )
                break

        if args.max_frames and frame_count >= args.max_frames:
            break

    manifest = {
        "frame_count": frame_count,
        "initial_captured": initial_captured,
        "verification_captured": verification_captured,
        "final_captured": (output_directory / "final.ready").exists(),
        "status": "complete" if (output_directory / "final.ready").exists() else "stopped",
    }
    (output_directory / "isaac_capture.json").write_text(
        json.dumps(manifest, indent=2) + "\n", encoding="utf-8"
    )
    app_utils.stop()
    return 0 if manifest["status"] == "complete" else 1


exit_code = 1
try:
    exit_code = main()
except BaseException:
    import traceback

    traceback.print_exc()
finally:
    simulation_app.close(skip_cleanup=True, exit_code=exit_code)
