# Voice-Grounded Manipulation

Voice-Grounded Manipulation is an embodied-AI robotics project for safely executing spoken manipulation tasks with a simulated Franka Panda robot. The system combines NVIDIA Isaac Sim, ROS 2 Jazzy, MoveIt 2, RGB-D perception, Whisper speech transcription, and an LLM that translates user intent into a small, validated vocabulary of high-level robot skills.

Safety is a core design constraint: the LLM must never directly control joints, velocities, motors, or trajectories. It may only propose structured high-level skills; deterministic validation, task planning, motion planning, collision checking, and robot-control layers retain authority over execution.

## Safety boundary

The LLM is an intent parser, not a motion controller. It receives only a
transcript, a scene revision, and allowlisted object/target identifiers. Its
strict JSON response is validated again against fresh RGB-D observations before
the deterministic coordinator can create a plan. The LLM must never directly
control joints, coordinates, velocities, motors, torques, efforts, or trajectories.

Only MoveIt 2 may plan trajectories. The execution layer additionally enforces
workspace limits, collision objects, 20% velocity/acceleration scaling, a
120-second deadline, and final RGB-D outcome validation. Any missing, stale,
ambiguous, malformed, or unsafe input fails closed without a motion request.

The latest [safety hardening](docs/safety_hardening.md) adds measured 3D targets,
two-frame resting-object verification, retained placement collisions, canonical
Python/C++ safety configuration, direct JSON Schema enforcement, and deterministic
STOP handling. Offline regression tests pass; these revisions still require a
fresh MoveIt build and simulator acceptance run. Phase acceptance labels below
describe the previously demonstrated version, not certification of these changes.

## Roadmap

### Phase 1 — Foundation and validation (accepted in simulation)

- Bring up the Panda scene in Isaac Sim and verify ROS 2 connectivity.
- Confirm MoveIt 2 can plan and execute a small set of safe simulated motions.
- Establish the accepted high-level skill schema and validation boundary.
- Verify that invalid, ambiguous, or unsafe requests are rejected without robot motion.

### Phase 2 — Perception and grounding (accepted for configured cubes)

- Add RGB-D camera inputs and scene/object representations.
- Ground validated skills against observable objects, poses, and workspace constraints.
- Test perception failure handling and confidence-based refusal paths.

### Phase 3 — Voice interaction (accepted with spoken WAV input)

- Integrate Whisper for speech-to-text.
- Connect transcription to constrained intent extraction and skill proposals.
- Add confirmations, clarifications, and audit logs for spoken commands.

### Phase 4 — Task-level autonomy (accepted for bounded simulated tasks)

- Expand the validated skill library for pick, place, inspect, and related tasks.
- Compose multi-step tasks only through validated skill sequences.
- Evaluate robustness, recovery behavior, and safety limits in simulation before any hardware work.

## Repository layout

- `isaac_sim/` — Isaac Sim scenes and supporting scripts.
- `ros2_ws/` — ROS 2 Jazzy workspace source tree.
- `config/` — shared configuration files.
- `docs/` — architecture and validation criteria.
- `tests/` — simulator-independent safety and regression tests; the live acceptance runner is in `isaac_sim/scripts/`.

See [`docs/local_development.md`](docs/local_development.md) for the verified
Ubuntu 24.04 WSL, ROS 2 Jazzy, and workspace setup.

See [`docs/brev_development.md`](docs/brev_development.md) for the NVIDIA Brev
GPU environment, cost guard, SSH/VS Code connection, and persistence workflow.

The versioned runtime contracts are
[`config/robot_skill.schema.json`](config/robot_skill.schema.json),
[`config/safety_policy.json`](config/safety_policy.json), and
[`config/phase_1_scene.json`](config/phase_1_scene.json).

## Phase 1 visual demo

On the configured Brev instance, generate RGB and metric-depth evidence from
the real Isaac Sim scene without commanding the robot:

```bash
cd /home/ubuntu/workspace
ACCEPT_EULA=Y isaac_sim/scripts/run_phase_1_demo.sh
```

The launcher rebuilds the versioned scene, captures one 640 x 480 RGB-D frame,
and prints the unique, git-ignored output directory. Each run contains an RGB
PNG, raw metric depth as NumPy data, a viewable depth PNG, and `manifest.json`.
It does not publish network ports or call any robot-control interface. See the
[Brev demo runbook](docs/brev_development.md#phase-1-visual-demo) for artifact
copy and inspection commands.

## Constrained MoveIt demo

On the configured Brev instance, run an allowlisted named-pose skill through
ROS 2 Jazzy and MoveIt 2 against the live Isaac Sim Franka:

```bash
cd /home/ubuntu/workspace
ACCEPT_EULA=Y isaac_sim/scripts/run_moveit_demo.sh extended
```

The accepted targets are `ready`, `extended`, and `transport`; any other value
is rejected before Isaac Sim or MoveIt starts. The application caps velocity
and acceleration scaling at 20% and asks MoveIt to plan and execute the named
pose. It never publishes joints or trajectories itself. Each successful run
writes initial/final RGB images, initial/final joint-state snapshots, component
logs, and a manifest under the git-ignored `isaac_sim/_output/` directory.

Isaac Sim 6.0.1 can take several minutes to produce its first camera frame on
this headless environment. Wait for the launcher's success result rather than
assuming that a CPU-bound startup has failed. See the
[Brev development runbook](docs/brev_development.md#ros-2-jazzy-and-moveit-2)
for environment setup, build, and artifact details.

## Voice-grounded pick-and-place demo

The integrated launcher captures and grounds a live RGB-D scene, converts a
command to a schema-constrained high-level skill, revalidates the referenced
object immediately before execution, asks MoveIt 2 to perform the bounded
pick-and-place, then accepts the result only if a final RGB-D frame observes the
cube within 6 cm of the allowlisted target.

Use the deterministic intent provider for a repeatable simulator acceptance
run that needs no API credential:

```bash
cd /home/ubuntu/workspace
ACCEPT_EULA=Y isaac_sim/scripts/run_voice_manipulation_demo.sh \
  --transcript "Pick the red cube and place it on the blue target" \
  --provider rules
```

For the real LLM and audio path, use the workstation console below. It keeps
`OPENAI_API_KEY` in WSL, runs `faster-whisper` (`small.en`, CPU/int8) on Brev,
and requires confirmation before motion. Do not commit `.env.local`, API keys,
recordings, or generated run artifacts.

The rules provider is only a deterministic integration-test fixture. It does
not replace the schema-constrained LLM in the intended system.

## Confirmed voice and multi-step tasks

The workstation console keeps the OpenAI API key in WSL and sends constrained
skill requests over SSH to the existing Brev instance. Brev independently validates
each proposal against a fresh RGB-D capture before invoking MoveIt. A reusable
simulator session supports `inspect`, `pick`, `place`, `pick_and_place`, the
allowlisted named poses, and gripper skills with holding preconditions.

After checking the Brev shutdown guard and building the project packages,
start a session on Brev:

```bash
cd /home/ubuntu/workspace
ACCEPT_EULA=Y isaac_sim/scripts/run_voice_manipulation_demo.sh \
  --session --run-id my-session
```

When it prints `Reusable simulator session ready`, start the console in WSL:

```bash
PYTHONPATH=ros2_ws/src/vgm_runtime python3 -m vgm_runtime.task_cli \
  --session my-session
```

Enter a command such as “Inspect the red cube, pick it up, place it on the blue
target, then inspect it again.” The console displays the proposed sequence and
waits for `yes`. Enter `audio /path/to/command.wav` to use Whisper. Ambiguous
requests ask for clarification. Ctrl+C cancels an executing task; `stop` also
works from an idle console. `recover` checks robot state and requires a new
command before anything resumes. A held object requires the separately
confirmed `recover place blue_target` or `recover place yellow_target` operation.

See [all-phase acceptance and operator instructions](docs/all_phases_validation.md)
for the complete campaign, per-phase criteria, audit evidence, and limitations.

## Local verification

The simulator-independent suite needs Python, NumPy, and jsonschema (listed in
`requirements-test.txt`), but no GPU, ROS installation, API call, or cloud instance:

```bash
python3 -m unittest discover -s tests -v
python3 isaac_sim/scripts/build_phase_1_scene.py --validate-only
```

The first command runs the safety, schema, planning-boundary, perception,
Whisper-adapter, audit, and outcome tests. The second validates the scene
contract without importing Isaac Sim or requiring a GPU.

## Current status

All four phases are accepted for the configured six-cube simulation milestone.
The 2026-09-04 (Toronto) campaign passed all six live suites on the existing Brev
L4: invalid-request rejection, mid-motion stop, injected-fault recovery,
clarification dialogue, spoken manipulation, and a confirmed multi-step task.

The real Whisper → `gpt-5.6-terra` → validated skills → MoveIt → Isaac path moved
the red cube to the blue target with 3.98 mm of camera-measured planar placement
error. The subsequent LLM-planned `inspect → pick → place → inspect` sequence
moved it to the yellow target with 5.04 mm of placement error. Both tasks ended
stationary and empty-handed. The API key stayed in WSL; audio was transcribed
on Brev and was not sent to OpenAI.

All 94 simulator-independent tests pass, and both project ROS packages build.
See the [acceptance record and demo runbook](docs/all_phases_validation.md) for
the exact evidence and reproducible commands. This is a working simulation
demo, not hardware safety certification, arbitrary-object perception, or a
statistically established manipulation success rate.
