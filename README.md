# Voice-Grounded Robotic Manipulation

A simulated Franka Panda that turns spoken or typed goals into **operator-authorized,
safety-bounded robot behavior**. The stack combines NVIDIA Isaac Sim, ROS 2 Jazzy,
MoveIt 2, RGB-D perception, Whisper, and an LLM that can act as either a one-shot
task interpreter or a **closed-loop high-level controller**.

Example: **“Put the red cube on the yellow target.”**

In closed-loop agent mode, the operator first reviews a finite semantic mission
scope—object IDs, target IDs, named poses, deterministic completion conditions,
and an action budget. After confirmation, the LLM can repeatedly observe semantic
state and choose one currently offered
capability at a time. It never receives robot/world coordinates, joint commands,
trajectories, or safety limits. Fresh perception, deterministic validation, MoveIt,
STOP, and outcome verification retain physical authority.

[Architecture](docs/architecture.md) · [Agent control](docs/agent_control.md) ·
[Demo dashboard](docs/demo_dashboard.md) · [Safety validation](docs/safety_hardening.md) · [CI](.github/workflows/ci.yml)

## Fastest way to see it

No Isaac Sim, GPU, cloud VM, API key, ROS installation, or robot motion is required
for the local sandbox:

```bash
./demo replay
```

Open `http://localhost:8766/` if the browser does not open automatically. Type:

```text
Put the red cube on the yellow target.
```

Review the proposed mission, click **Confirm mission & start agent**, and watch the
bounded agent decisions in the timeline. Replay mode is an in-memory demonstration
of the control architecture; it is **not recorded Isaac footage and does not claim
physical execution**.

Passive local preflight:

```bash
./demo doctor --mode replay
```

For the actual Isaac Sim session and WebRTC stream, start the existing reusable
simulator/viewer on the GPU machine, then run on your workstation:

```bash
./demo doctor \
  --mode remote \
  --session YOUR_SESSION_ID \
  --host vgm-isaac-dev \
  --remote-root /home/ubuntu/voice-grounded-manipulation \
  --viewer-url http://YOUR_SERVER_IPV4:8210/

./demo run \
  --mode remote \
  --control agent \
  --session YOUR_SESSION_ID \
  --host vgm-isaac-dev \
  --remote-root /home/ubuntu/voice-grounded-manipulation \
  --viewer-url http://YOUR_SERVER_IPV4:8210/
```

`--remote-root` makes the remote repository location explicit instead of assuming
`/home/ubuntu/workspace`. Omit it when the repository actually lives at the legacy
path.

The live stream uses a dedicated presentation-only spectator camera and matte studio
backdrop. Those prims have no collision or rigid-body API and do not alter the
`/World/WorkspaceCamera` RGB-D sensor used for grounding and outcome verification.

The dashboard stays bound to `127.0.0.1` and embeds the existing Isaac viewer in
the same page. The live simulator is deliberately the dominant surface: mission
controls sit directly below it, while execution, world-state, completion, and safety
telemetry live in a compact right rail. It does not make the viewer public, start
cloud compute, or relax streaming firewall rules. See
[Demo dashboard](docs/demo_dashboard.md) and [Interactive viewing](docs/live_view.md).

## What the LLM controls

The LLM controls **task-level decisions**, not actuators.

```text
Operator goal
    │
    ▼
Mission proposal: scope + completion contract + action budget
    │
    ├── operator review / confirmation
    ▼
Fresh semantic observation
    │
    ▼
LLM chooses ONE offered capability
    │
    ▼
Capability gateway
    ├── JSON Schema
    ├── confirmed mission scope
    ├── deterministic completion-condition checks
    ├── current allowed-capability set
    ├── robot/scene preconditions
    └── existing robot-skill schema
    │
    ▼
Fresh execution gate → deterministic geometry → MoveIt 2 → Panda
    │
    ▼
Verified result / fresh semantic observation
    └───────────────────────────────────────────────► LLM
```

The agent may choose among bounded capabilities such as:

- `observe_workspace`
- `inspect`
- `pick`
- `place`
- `pick_and_place`
- `move_named_pose`
- `finish`
- `request_human_help`
- `stop`

The deterministic supervisor computes the exact options available **for the current
state**. `finish` is not a model assertion: it is offered only after every
operator-confirmed completion condition is deterministically satisfied. Placement
completion normally uses a fresh visible target observation. When the placed cube
physically occludes the target marker, the verifier may instead retain the fresh
pre-execution RGB-D target measurement, but only after two detached resting-object
frames geometrically prove that the cube itself explains the occlusion. The accepted
placement pose is then carried into the next fresh semantic observation so target
occlusion cannot leave the agent stuck in `running`. After a standalone `pick`, the
model is not offered another pick, a named-pose motion, or `finish`; it can only place
the held object, request human help, or stop.

The LLM is never allowed to supply:

```text
Cartesian XYZ / poses
joint positions or targets
joint velocities
motor / effort / torque commands
trajectory waypoints
collision-scene edits
workspace or motion limits
STOP implementation
physical success claims
```

Every executable agent decision is converted back into the existing strict
`robot_skill.schema.json` shape before the backend can request motion.

## Two control modes

The unified dashboard supports both modes:

```bash
# Closed-loop agent (default)
./demo run ... --control agent

# Original operator-confirmed fixed task plan
./demo run ... --control task
```

**Agent mode** confirms a semantic authority envelope once—including a machine-checkable
completion contract—then permits multiple fresh high-level decisions within that
envelope and finite budget.

**Task mode** asks the model for an exact ordered task first and binds confirmation
to those steps. It remains useful when you want the most restrictive/reproducible
operator interaction.

Neither mode permits direct LLM motion control.

## Safety boundary

The physical execution chain remains deterministic:

```text
LLM capability request
  → capability/scope validation
  → robot-skill JSON Schema validation
  → fresh scene + robot-state checks
  → execution-time drift revalidation
  → deterministic task coordinator
  → MoveIt collision-aware planning
  → ros2_control / Isaac Panda
  → fresh RGB-D + attachment-state outcome verification
```

Important properties:

- STOP/cancel/abort/halt/freeze phrases bypass the LLM.
- Browser STOP bypasses the normal work queue.
- Scenes, grounded observations, IDs, holding state, and target occupancy are
  validated outside the model. Stale scenes expose only STOP/help; low-confidence,
  stale, invalid-frame, or out-of-workspace observations are never offered as
  executable object/target capabilities.
- The execution backend captures again immediately before motion and rejects
  excessive drift.
- Released objects remain collision obstacles during retreat.
- Successful placement requires fresh two-frame 3D/rest evidence and detached
  robot state. If the placed cube occludes the visual target marker, acceptance uses
  only the fresh execution-gate target measurement (never authored/spawn coordinates)
  and a stricter overlap test in both rest frames.
- A physical execution failure does **not** trigger autonomous LLM retry. The
  agent faults and requests cancellation; recovery remains explicit.
- Mission confirmation expires, action count is bounded, and budget exhaustion
  is reported as `needs_human`, never success.

Software STOP is not a certified hardware emergency stop. This repository is a
simulation/research system, not physical-robot safety certification.

## Model-visible state

Closed-loop decisions receive semantic state similar to:

```json
{
  "mission": {
    "goal_summary": "Put the red cube on the yellow target",
    "object_ids": ["red_cube"],
    "target_ids": ["yellow_target"],
    "pose_names": [],
    "max_actions": 6,
    "success_conditions": [
      {
        "kind": "object_on_target",
        "object_id": "red_cube",
        "target_id": "yellow_target",
        "pose_name": null
      }
    ]
  },
  "remaining_actions": 5,
  "scene": {
    "scene_revision": "…",
    "scene_fresh": true,
    "held_object_id": null,
    "objects": [
      {"object_id": "red_cube", "visible": true, "grounded": true, "held": false, "confidence": "high"}
    ],
    "targets": [
      {"target_id": "yellow_target", "visible": true, "grounded": true, "occupied_by": null, "confidence": "high"}
    ]
  },
  "last_result": null,
  "allowed_capabilities": ["… supervisor-generated structured options …"]
}
```

No `position_m`, joint data, trajectory data, velocity, torque, or motor fields are
included in this model-visible representation. Inspect results are sanitized before
they return to the LLM even though the deterministic backend internally retains
measured geometry.

## Demo modes

| Mode | Command | What it uses |
| --- | --- | --- |
| Replay sandbox | `./demo replay` | Local Python only; no API/ROS/SSH/motion |
| Local live session | `./demo run --mode local --session ID` | Existing Isaac/ROS/MoveIt session on this machine |
| Remote live session | `./demo run --mode remote --session ID --host HOST` | Existing session over authenticated SSH |

Add `--viewer-url http://HOST:8210/` in live modes to place the Isaac WebRTC
viewer inside the same dashboard. Add `--no-open` for terminal-only launch.

`./demo doctor` is passive: it validates configuration and dashboard assets; in
live modes it can check the session, robot stationary state, fresh RGB-D capture,
and viewer HTTP reachability. It never requests robot motion. Use `--skip-capture`
if even passive RGB-D capture is undesirable during a diagnostic run.

## Evidence and current limitations

| Milestone | Evidence and scope |
| --- | --- |
| September 4, 2026 simulation campaign | Six historical live suites passed: invalid requests, cancellation, fault recovery, clarification, spoken manipulation, and a confirmed multi-step task. |
| Historical placement examples | Two runs measured 3.98 mm and 5.04 mm **planar** error. These are individual examples, not an accuracy benchmark. |
| Current offline regression suite | **155 automated tests pass** in this working tree, including the original 125 tests plus 30 closed-loop agent, dashboard, presentation, and post-place outcome regressions. |
| Current static checks | Python compilation, shell syntax, browser JavaScript syntax, JSON parsing, and scene-contract validation pass. |
| Pending live acceptance | The latest release/retreat safety changes still require a fresh full live manipulation campaign. The new closed-loop agent layer also requires live acceptance before being represented as live-validated. |

The [historical acceptance record](docs/all_phases_validation.md) does not certify
newer safety or agent changes. See [current safety hardening](docs/safety_hardening.md).

The configured perception stack recognizes six known colored cubes and two known
target markers. It is not arbitrary household-object recognition or general 6-DoF
pose estimation. The replay dashboard is a control-flow sandbox, not a simulator
accuracy claim.

## Technology

- **Simulation:** NVIDIA Isaac Sim 6.0.1, Franka Panda, RGB-D camera.
- **Planning/runtime:** ROS 2 Jazzy, MoveIt 2, Python and C++.
- **Speech/language:** faster-whisper (`small.en`, CPU/int8), OpenAI Responses API.
- **Control boundary:** JSON Schema, deterministic capability generation, semantic
  mission scope, robot-state/scene validation, execution gate, outcome verification.
- **Environment:** Windows 11 + WSL2 Ubuntu 24.04; NVIDIA Brev L4 GPU VM in the
  documented reference deployment.

The API key stays on the workstation. Whisper can transcribe on the remote simulator
host in the existing workflow. OpenAI receives text and high-level semantic IDs,
not audio, robot coordinates, joints, or trajectories. Credentials and generated
recordings are excluded from Git.

## Local checks

Install the small test dependency set from `requirements-test.txt`, then run:

```bash
python3 -m unittest discover -s tests -v
python3 isaac_sim/scripts/build_phase_1_scene.py --validate-only
./demo doctor --mode replay
node --check isaac_sim/browser_console/app.js
```

These do not require a GPU, cloud instance, API call, or robot motion. They do not
replace live acceptance.

## Repository layout

| Path | Contents |
| --- | --- |
| `config/robot_skill.schema.json` | Strict physical skill proposal schema |
| `config/safety_policy.json` | Canonical geometry/motion safety policy |
| `config/agent_policy.json` | Closed-loop semantic capability/action-budget policy |
| `ros2_ws/src/vgm_runtime/vgm_runtime/agent.py` | Mission scope, deterministic completion contract, semantic state, capability gateway, agent loop |
| `ros2_ws/src/vgm_runtime/vgm_runtime/demo.py` | Dashboard launcher and passive doctor |
| `ros2_ws/src/vgm_runtime/vgm_runtime/replay.py` | Zero-motion local replay/sandbox backend |
| `isaac_sim/browser_console/` | Unified operator dashboard |
| `isaac_sim/` | Simulator, viewer, capture, launch, and recording tools |
| `ros2_ws/src/vgm_moveit_demo/` | Deterministic MoveIt executors |
| `tests/` | Safety, regression, agent, and dashboard tests |
| `.github/workflows/ci.yml` | Portable offline safety/regression CI |
| `LICENSE` | Apache-2.0 license text matching the ROS package metadata |
| `docs/` | Architecture, deployment, demo, and validation guides |

Large RGB-D captures, recordings, and logs remain under ignored
`isaac_sim/_output/`.
