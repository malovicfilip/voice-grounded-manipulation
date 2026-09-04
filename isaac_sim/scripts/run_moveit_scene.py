#!/usr/bin/env python3
# SPDX-FileCopyrightText: Copyright (c) 2020-2026 NVIDIA CORPORATION & AFFILIATES.
# SPDX-License-Identifier: Apache-2.0
"""Run NVIDIA's Franka ROS 2 action graph for the headless MoveIt demo.

Adapted from Isaac Sim 6.0.1's bundled ROS 2 MoveIt example.

This process only bridges ROS 2 joint-state feedback and commands produced by
the deterministic MoveIt/ros2_control stack. It does not accept language input
or construct joint trajectories.
"""

import argparse
import json
import sys
from pathlib import Path

import numpy as np
from isaacsim import SimulationApp


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--headless", action="store_true")
    parser.add_argument("--max-frames", type=int, default=0)
    parser.add_argument("--output-directory", type=Path)
    args, _ = parser.parse_known_args()
    return args


args = parse_args()
simulation_app = SimulationApp(
    {"renderer": "RealTimePathTracing", "headless": args.headless}
)

import carb  # noqa: E402
import isaacsim.core.experimental.utils.app as app_utils  # noqa: E402
import isaacsim.core.experimental.utils.stage as stage_utils  # noqa: E402
import omni.graph.core as og  # noqa: E402
import usdrt.Sdf  # noqa: E402
from isaacsim.core.experimental.utils.prim import get_prim_at_path  # noqa: E402
from isaacsim.core.rendering_manager import ViewportManager  # noqa: E402
from isaacsim.core.simulation_manager import SimulationManager  # noqa: E402
from isaacsim.storage.native import get_assets_root_path  # noqa: E402
from pxr import Gf, UsdGeom  # noqa: E402


FRANKA_STAGE_PATH = "/Franka"
FRANKA_USD_PATH = "/Isaac/Robots/FrankaRobotics/FrankaPanda/franka.usd"
BACKGROUND_STAGE_PATH = "/background"
BACKGROUND_USD_PATH = "/Isaac/Environments/Simple_Room/simple_room.usd"


app_utils.enable_extension("isaacsim.ros2.bridge")
simulation_app.update()
stage_utils.set_stage_units(meters_per_unit=1.0)

assets_root_path = get_assets_root_path()
if assets_root_path is None:
    carb.log_error("Could not find the Isaac Sim assets folder")
    simulation_app.close()
    sys.exit(1)

ViewportManager.set_camera_view(
    "/OmniverseKit_Persp",
    eye=np.array([1.2, 1.2, 0.8]),
    target=np.array([0.0, 0.0, 0.5]),
)
stage_utils.add_reference_to_stage(
    assets_root_path + BACKGROUND_USD_PATH, BACKGROUND_STAGE_PATH
)
stage_utils.add_reference_to_stage(
    assets_root_path + FRANKA_USD_PATH, FRANKA_STAGE_PATH
)

robot = get_prim_at_path(FRANKA_STAGE_PATH)
xform_api = UsdGeom.XformCommonAPI(robot)
xform_api.SetTranslate(Gf.Vec3d(0.0, -0.64, 0.0))
xform_api.SetRotate((0.0, 0.0, 90.0), UsdGeom.XformCommonAPI.RotationOrderXYZ)
robot.GetVariantSet("Gripper").SetVariantSelection("AlternateFinger")
robot.GetVariantSet("Mesh").SetVariantSelection("Quality")

camera_sensor = None
output_directory = args.output_directory.resolve() if args.output_directory else None
if output_directory:
    import isaacsim.core.experimental.utils.transform as transform_utils
    from isaacsim.sensors.experimental.rtx import CameraSensor, RtxCamera

    output_directory.mkdir(parents=True, exist_ok=True)
    # Start from NVIDIA's sample viewport pose, then widen the lens enough to
    # keep the entire arm visible throughout all three allowlisted poses.
    camera_position = np.asarray([1.2, 1.2, 0.8], dtype=np.float32)
    camera_orientation = transform_utils.look_at_quaternion(
        eye=camera_position,
        target=np.asarray([0.0, 0.0, 0.5], dtype=np.float32),
    ).numpy()
    rtx_camera = RtxCamera(
        "/MoveItDemoCamera",
        tick_rate=30.0,
        positions=camera_position,
        orientations=camera_orientation,
    )
    camera_sensor = CameraSensor(
        rtx_camera,
        resolution=(480, 640),
        annotators=["rgb"],
    )
    UsdGeom.Camera(
        stage_utils.get_current_stage().GetPrimAtPath("/MoveItDemoCamera")
    ).CreateFocalLengthAttr(18.0)
simulation_app.update()

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
            (
                "ReadJointState.outputs:jointNames",
                "PublishJointState.inputs:jointNames",
            ),
            (
                "ReadJointState.outputs:jointPositions",
                "PublishJointState.inputs:jointPositions",
            ),
            (
                "ReadJointState.outputs:jointVelocities",
                "PublishJointState.inputs:jointVelocities",
            ),
            (
                "ReadJointState.outputs:jointEfforts",
                "PublishJointState.inputs:jointEfforts",
            ),
            (
                "ReadJointState.outputs:jointDofTypes",
                "PublishJointState.inputs:jointDofTypes",
            ),
            (
                "ReadJointState.outputs:stageMetersPerUnit",
                "PublishJointState.inputs:stageMetersPerUnit",
            ),
            (
                "ReadJointState.outputs:sensorTime",
                "PublishJointState.inputs:sensorTime",
            ),
            ("OnPlaybackTick.outputs:tick", "SubscribeJointState.inputs:execIn"),
            ("OnPlaybackTick.outputs:tick", "PublishClock.inputs:execIn"),
            ("OnPlaybackTick.outputs:tick", "ArticulationController.inputs:execIn"),
            ("Context.outputs:context", "PublishJointState.inputs:context"),
            ("Context.outputs:context", "SubscribeJointState.inputs:context"),
            ("Context.outputs:context", "PublishClock.inputs:context"),
            (
                "ReadSimTime.outputs:simulationTime",
                "PublishClock.inputs:timeStamp",
            ),
            (
                "SubscribeJointState.outputs:jointNames",
                "ArticulationController.inputs:jointNames",
            ),
            (
                "SubscribeJointState.outputs:positionCommand",
                "ArticulationController.inputs:positionCommand",
            ),
            (
                "SubscribeJointState.outputs:velocityCommand",
                "ArticulationController.inputs:velocityCommand",
            ),
            (
                "SubscribeJointState.outputs:effortCommand",
                "ArticulationController.inputs:effortCommand",
            ),
        ],
        og.Controller.Keys.SET_VALUES: [
            ("ArticulationController.inputs:robotPath", FRANKA_STAGE_PATH),
            ("ReadJointState.inputs:prim", [usdrt.Sdf.Path(FRANKA_STAGE_PATH)]),
            ("PublishJointState.inputs:topicName", "isaac_joint_states"),
            ("SubscribeJointState.inputs:topicName", "isaac_joint_commands"),
        ],
    },
)

simulation_app.update()
SimulationManager.setup_simulation(dt=1.0 / 60.0, device="cpu")
app_utils.play()
simulation_app.update()
print("VGM_ISAAC_MOVEIT_BRIDGE_READY", flush=True)

frame_count = 0
initial_capture_written = False
final_settle_frames = 0
while simulation_app.is_running():
    simulation_app.update()
    frame_count += 1

    if camera_sensor is not None:
        rgb_data, _ = camera_sensor.get_data("rgb")
        if rgb_data is not None and not initial_capture_written:
            import matplotlib.pyplot as plt

            initial_path = output_directory / "initial.png"
            plt.imsave(initial_path, rgb_data.numpy())
            (output_directory / "initial.ready").touch()
            initial_capture_written = True
            print(f"VGM_ISAAC_CAPTURE_INITIAL={initial_path}", flush=True)

        final_request = output_directory / "capture_final.request"
        if initial_capture_written and final_request.exists():
            final_settle_frames += 1
            if final_settle_frames >= 30 and rgb_data is not None:
                import matplotlib.pyplot as plt

                final_path = output_directory / "final.png"
                plt.imsave(final_path, rgb_data.numpy())
                capture_manifest = {
                    "final_rgb": final_path.name,
                    "frame_count": frame_count,
                    "initial_rgb": "initial.png",
                    "status": "captured",
                }
                with (output_directory / "isaac_capture.json").open(
                    "w", encoding="utf-8"
                ) as manifest_file:
                    json.dump(capture_manifest, manifest_file, indent=2)
                    manifest_file.write("\n")
                print(f"VGM_ISAAC_CAPTURE_FINAL={final_path}", flush=True)
                break

    if args.max_frames and frame_count >= args.max_frames:
        break

app_utils.stop()
simulation_app.close()
