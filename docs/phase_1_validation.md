# Phase 1 validation

Phase 1 is accepted only when the following tests pass in simulation. These tests establish the control boundary before perception, voice input, or multi-step autonomy are introduced.

The versioned scene contract is [`config/phase_1_scene.json`](../config/phase_1_scene.json).
It fixes the coordinate system, physics rates, Franka asset, table geometry, six
cube identities and poses, and RGB-D camera requirements before simulator code
is executed. Its simulator-independent checks run locally with:

```bash
python3 -m unittest discover -s tests -v
```

The deterministic builder is
[`isaac_sim/scripts/build_phase_1_scene.py`](../isaac_sim/scripts/build_phase_1_scene.py).
Its validation-only mode deliberately imports no Isaac Sim modules, so the
contract and safety boundary can be checked in WSL without a GPU:

```bash
python3 isaac_sim/scripts/build_phase_1_scene.py --validate-only
```

On a workstation with Isaac Sim 6.0.1, run the same script through Isaac Sim's
Python launcher. The output path is ignored by Git because it is generated:

```bash
/path/to/isaac-sim/python.sh \
  isaac_sim/scripts/build_phase_1_scene.py --headless
```

The builder uses NVIDIA's current
[RTX camera API](https://docs.isaacsim.omniverse.nvidia.com/6.0.1/py/source/extensions/isaacsim.sensors.experimental.rtx/docs/index.html)
with `rgb` and `distance_to_image_plane` annotators. The JSON stores resolution
as `[width, height]`; the API receives `(height, width)` as required by its
OpenCV/NumPy convention.

## Environment and integration

1. **Isaac Sim scene starts:** The project can construct and open the baseline scene from the versioned contract with a Franka Panda, a stable world frame, a table, exactly six uniquely identified colored cubes, and an RGB-D camera.
2. **ROS 2 Jazzy connectivity:** Required simulation, robot-state, transform, and control interfaces are discoverable and exchange messages with expected timestamps.
3. **MoveIt 2 model alignment:** MoveIt 2 receives the Panda joint state and transform tree, and its planning model matches the simulated robot configuration.
4. **Safe simulated execution:** MoveIt 2 can plan and execute an approved, collision-free movement between predefined safe poses. The execution result and final robot state agree within configured tolerances.
5. **Stop behavior:** A stop request or execution fault prevents further commanded motion and reports a clear failure state.

## Skill-validation boundary

6. **Allowed skill accepted:** A schema-valid request for an explicitly allowed high-level skill is accepted only when all required parameters and preconditions are present.
7. **Malformed output rejected:** Missing fields, wrong types, extra unapproved fields, or invalid identifiers are rejected before planning is requested.
8. **Disallowed action rejected:** A request for a skill outside the allowlist is rejected before planning or control interfaces are called.
9. **Direct-control fields rejected:** Any LLM output containing joint targets, velocities, torques, motor commands, or trajectories is rejected. The LLM must never directly control joints, velocities, motors, or trajectories.
10. **Ambiguous or ungrounded request rejected:** A request lacking a resolvable target or required scene state is refused without robot motion.
11. **Unsafe request rejected:** A request violating configured workspace, collision, or policy constraints is refused without robot motion.

## Evidence required

For each test, record the scene/configuration version, command or input fixture, expected result, observed result, relevant ROS 2 and simulation logs, and a pass/fail outcome. A Phase 1 pass requires no unexpected robot motion in any rejection test.

## Current verified evidence

The repository's 59 simulator-independent tests pass. They cover the scene and
RGB-D contracts, exact skill shapes, malformed/extra fields, recursive
direct-control-field rejection, ambiguity, missing/stale/low-confidence
grounding, workspace and target policy, replay protection, scene revalidation,
commanded-object drift, deterministic task expansion, LLM response handling,
audit redaction, stop/fault/timeout supervision, MoveIt-only execution, and
final outcome validation.

The `vgm_moveit_demo` and `vgm_runtime` packages build in the locked
Pixi/RoboStack environment. The following live evidence has been collected on
the Brev NVIDIA L4 instance using Isaac Sim 6.0.1 and ROS 2 Jazzy:

- NVIDIA's Isaac Sim compatibility checker passed with the L4, Vulkan, and the
  installed 595.71.05 driver.
- The Phase 1 scene produced 640 x 480 RGB and metric-depth images from the live
  USD stage.
- `ready` and `extended` named poses planned and executed through MoveIt 2. In
  the `moveit-visual-v3` run, joint 4 stopped at its configured upper limit
  (`-0.0698` rad) instead of the named pose's nominal zero, showing that the
  MoveIt/controller limit remained authoritative.
- The integrated `pick_and_place` run `integrated-rules-v9` completed all 11
  deterministic primitives with `plan=success execution=success`. The red cube
  moved from approximately `[-0.0903, -0.1985, 0.7750]` m to
  `[0.0205, 0.2936, 0.7750]` m. The launcher's final live RGB-D outcome gate
  accepted confidence 0.8082 and a 0.0215 m error from `blue_target`, inside the
  required 0.06 m tolerance. Its manifest records `result: success`, Isaac Sim
  6.0.1, ROS 2 Jazzy, and the deterministic `rules` intent provider.
- A generated spoken WAV was transcribed by the pinned `faster-whisper`
  `small.en` model as “Pick the red cube and place it on the blue target.” with
  confidence 0.8227 (minimum accepted confidence: 0.55).
- A live `gpt-5.6-terra` Responses API smoke test returned an accepted
  `pick_and_place(red_cube, blue_target)` proposal using strict structured
  output, no tools, no coordinates, and `store: false`. The validator expanded
  it to 11 deterministic primitives.

The deterministic rules provider used for the physical integration run is a
repeatable intent-test fixture, not a substitute for the LLM. The combined
audio-to-LLM-to-simulator run requires an API key on the temporary GPU instance;
credentials are not transferred there without explicit authorization.

## Acceptance status

Criteria 1–4 and 6–11 have passing automated and/or live-simulator evidence.
The software execution supervisor also has passing tests showing that stop,
backend faults, and deadline expiration call cancellation and prevent following
primitives. Before final Phase 1 safety sign-off, criterion 5 should additionally
be exercised as a live mid-motion stop/fault injection against the running
Isaac/MoveIt stack, with before/after robot state and logs retained. Until that
test is captured, the integrated manipulation demo is accepted, but the broader
Phase 1 safety campaign remains open.
