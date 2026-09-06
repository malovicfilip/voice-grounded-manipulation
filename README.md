# Voice-Grounded Robotic Manipulation

A simulated Franka Panda that turns spoken instructions into **reviewed,
validated robot skills**. Built with NVIDIA Isaac Sim, ROS 2 Jazzy, MoveIt 2,
RGB-D perception, Whisper, and an LLM used only for language interpretation.

Example: **“Pick up the blue cube and place it on the yellow target.”**
The operator reviews the transcription and proposed task before confirming.
Deterministic code resolves IDs to fresh measured poses, MoveIt plans the
motion, and perception checks the result.

[Architecture](docs/architecture.md) · [Validation evidence](docs/all_phases_validation.md) ·
[Safety updates](docs/safety_hardening.md) · [Demo video guide](docs/demo_recording.md)

## What it demonstrates

- Spoken or typed commands, clarification, and browser-based operator confirmation.
- RGB-D grounding of six configured colored cubes and blue/yellow target markers.
- Bounded pick-and-place and multi-step tasks through ROS 2 and MoveIt 2.
- Schema validation, robot-state preconditions, collision checking, cancellation,
  explicit recovery, and perception-based outcome checks.
- GPU deployment on NVIDIA Brev with separate language, perception, planning,
  and execution components.

## Architecture and safety boundary

```text
Speech → Whisper transcript (or typed text)
       → LLM: high-level skills with object/target IDs only
       → JSON Schema + semantic/robot-state validation
       → operator review and confirmation
       → fresh 3D grounding + execution-time safety checks
       → deterministic task coordinator → MoveIt 2 → Isaac Sim Panda
       → perception-based placement verification
```

**The LLM must never directly control joints, Cartesian coordinates, velocities,
motors, torques, or trajectories.** It is a semantic parser, not a motion
controller. Deterministic layers retain execution authority.

STOP phrases and the browser STOP button bypass the LLM. This is software
cancellation, not a hardware emergency stop; spoken stopping still incurs
transcription latency.

The latest placement gate checks XY/Z error, attachment state, and two
observations consistent with rest. Released objects remain collision obstacles
during retreat. [Shared configuration](config/) defines the skill schema,
safety policy and authored scene geometry.

## Evidence and limitations

| Milestone | Evidence and scope |
| --- | --- |
| September 4, 2026 simulation campaign | Six live suites passed: invalid requests, cancellation, fault recovery, clarification, spoken manipulation, and a confirmed multi-step task. |
| Historical placement examples | Two tasks measured 3.98 mm and 5.04 mm **planar** error. Individual runs, not an accuracy benchmark or success rate. |
| September 6 safety deployment | Both ROS packages built; **121 automated tests passed** across the ROS and Isaac environments. Passive live checks observed six cubes, both targets, and a stationary, empty-handed robot. |
| Pending | Full live manipulation acceptance of the latest safety changes, including retained retreat collisions and stricter 3D outcome verification. |

The [historical acceptance record](docs/all_phases_validation.md) does not
certify the latest changes. See the [current validation checklist](docs/safety_hardening.md).

This is a **working simulation project**, not physical-robot deployment,
production safety certification, arbitrary household-object recognition, or a
statistically established manipulation success rate.

## Technology

- **Simulation:** NVIDIA Isaac Sim 6.0.1, Franka Panda, RGB-D camera.
- **Planning/runtime:** ROS 2 Jazzy, MoveIt 2, Python and C++.
- **Speech/language:** faster-whisper (`small.en`, CPU/int8), OpenAI Responses API.
- **Validation:** JSON Schema, deterministic scene/state checks, regression tests.
- **Environment:** Windows 11 + WSL2 Ubuntu 24.04; NVIDIA Brev L4 GPU VM.

The API key stays in WSL. Whisper transcribes on Brev; OpenAI receives text and
high-level identifiers, not audio, robot coordinates, or trajectories.
Credentials and generated recordings are excluded from Git.

## Run and explore

This is a multi-service robotics system, not a one-command fresh installation.
Start with [local development](docs/local_development.md) and
[Brev setup](docs/brev_development.md). Verify the automatic shutdown guard
before paid GPU work; keep its Windows host powered and online.

- **Speak to the robot:** [microphone → review → confirm](docs/browser_console.md).
  Requires a ready integrated session and a local console using that session ID.
- **Watch the demo:** [interactive streaming](docs/live_view.md).
- **Inspect/edit the scene:** [full Isaac Sim editor](docs/full_editor.md).
  Editor mode is separate from voice/MoveIt execution; do not run both simultaneously.
- **Reproduce acceptance:** [live validation runbook](docs/all_phases_validation.md).
  Its commands can move the simulated robot and call the API; review first.
- **Record a portfolio video:** [60–90 second plan](docs/demo_recording.md#short-portfolio-demo-6090-seconds).

VM addresses and session IDs change. Use current verified values, not historical
addresses copied from deployment records.

### Local checks

With the dependencies in `requirements-test.txt` available (recording tests
additionally need Pillow), run from the repository root:

```bash
python3 -m unittest discover -s tests -v
python3 isaac_sim/scripts/build_phase_1_scene.py --validate-only
```

These run regression tests, including local HTTP checks, and validate the scene
contract. Neither requires a GPU, cloud instance, API call, or robot motion.
They do not replace live acceptance.

## Phased roadmap

1. **Foundation:** Panda scene, ROS connectivity, MoveIt planning, skill boundary.
2. **Perception:** RGB-D grounding and confidence/freshness refusal paths.
3. **Voice:** transcription, constrained interpretation, clarification and confirmation.
4. **Task composition:** bounded multi-step execution, cancellation and recovery.

All four reached the historical configured-scene milestone. Next: live regression
acceptance of the safety upgrades, a short reproducible demo, and broader
robustness evaluation—not a claim of hardware readiness.

## Repository layout

| Directory | Contents |
| --- | --- |
| `isaac_sim/` | Scene builders, launchers, viewer and recording tools |
| `ros2_ws/src/` | Python runtime and C++ MoveIt integration |
| `config/` | Skill schema, canonical safety policy and scene configuration |
| `tests/` | Automated safety and regression tests |
| `docs/` | Architecture, setup, operator guides and validation evidence |

Large RGB-D captures, recordings and logs live under ignored
`isaac_sim/_output/`. Keep source and reviewed documentation in Git; back up
media separately before discarding the temporary cloud VM.
