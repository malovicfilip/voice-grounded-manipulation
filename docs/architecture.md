# Architecture

## Purpose

The system turns a spoken manipulation request into a safety-bounded simulated action by a Franka Panda. It is deliberately layered so that language reasoning cannot bypass perception, validation, motion planning, or execution safeguards. The LLM can operate either as a one-shot task interpreter or as a closed-loop high-level controller, but it never becomes the geometric or actuator controller.

## Components

- **User interaction** — the WSL operator console accepts a transcript or WAV, shows the exact task for confirmation, handles clarification replies, and provides stop/recovery controls.
- **Whisper speech-to-text** — uses local `faster-whisper` to convert an audio file to a transcript and refuses empty or low-confidence results.
- **RGB-D perception and scene grounding** — synchronizes RGB and metric depth, estimates cube-center XYZ and target-marker support-surface XYZ in the world frame, and records confidence, time, pixel support, and a revision covering objects and targets. No measured object Z is replaced with tabletop height.
- **LLM task interface** — asks `gpt-5.6-terra` for a strict-schema fixed task or, in agent mode, an operator-reviewed semantic mission envelope followed by one high-level capability decision at a time. Model-visible state contains IDs/relations and current allowed capabilities, never Cartesian poses, joints, trajectories, or motion limits. API storage is disabled. The API call and key remain on the workstation.
- **Agent capability gateway** — derives the exact actions currently available from fresh scene/holding state, the confirmed object/target/named-pose scope, deterministic completion conditions, target occupancy, and a finite action budget. An executable agent choice is converted back into the existing robot-skill schema before any backend request. Physical failure ends autonomous authority instead of triggering an LLM retry.
- **Skill validator and policy layer** — checks the exact schema, allowed skill/parameter combinations, replay protection, freshness, object confidence, workspace limits, and forbidden direct-control fields. It rejects anything invalid, ambiguous, unsupported, stale, or unsafe.
- **Pre-execution gate** — obtains a fresh RGB-D scene, rejects changed object identities or holding state and more than 1 cm of measured object drift, and regenerates the deterministic plan. A hash change caused only by bounded sensor jitter can be rebound to the new capture; stale current evidence cannot authorize motion.
- **Task coordinator** — expands a validator-minted skill into an immutable sequence of bounded task primitives. The current `pick_and_place` plan contains gripper, Cartesian approach/retreat, attach, and detach operations; no model-supplied coordinates are used.
- **Task session** — in fixed-task mode, checks the whole proposed sequence and target occupancy before confirmation, enforces pick/place holding preconditions, obtains a new observation before each skill, and latches faults until explicit recovery. Completed placements update only that object's tracked destination.
- **Agent session** — in closed-loop mode, confirms a finite semantic authority envelope and completion contract once, then captures fresh state before every LLM decision. It enforces mission scope, current capability options, request replay protection, a bounded action count, deterministic gating of `finish`, no finish while holding, and explicit stop/recovery boundaries.
- **MoveIt 2** — performs kinematic planning, collision checking, and trajectory generation within configured limits.
- **ROS 2 Jazzy** — provides communication, lifecycle management, transforms, robot state, and telemetry between the system components.
- **Isaac Sim** — hosts the simulated workspace, Franka Panda, sensors, physics, and simulation clock.
- **Safe execution and Panda controller interface** — accepts only allowlisted identifiers and validator-derived coordinates, adds the table/cubes to the planning scene, caps motion scaling, enforces the deadline, executes approved MoveIt-generated trajectories, and reports state.
- **Outcome validator and audit log** — requires XY and Z agreement with the perceived target, no attached object, and two separated RGB-D observations consistent with rest. Decisions are recorded without API keys, tokens, or raw audio.

## Data flow

Two operator-selectable language-control modes share the same physical execution boundary.

```text
                                 ┌──────── fixed-task mode ────────┐
Speech/text → transcript → LLM → exact ordered task → operator confirm
                                 └─────────────────────────────────┘

                                 ┌──────── agent mode ─────────────┐
Speech/text → transcript → LLM → mission scope/completion contract/budget → operator confirm
                                                   │
                                                   ▼
                                      fresh semantic observation
                                                   │
                                      allowed capabilities (code)
                                                   │
                                                   ▼
                                      LLM chooses one capability
                                                   │
                                      result / next observation ───┐
                                                   ▲                │
                                                   └────────────────┘
                                 └─────────────────────────────────┘

Both modes then use the same physical path for every executable skill:

capability/task skill
  → strict robot-skill schema + robot/scene policy
  → backend fresh capture + execution-time drift gate
  → deterministic measured geometry + policy offsets
  → MoveIt 2 collision-aware planning
  → ROS 2 controllers → Panda in Isaac Sim
  → fresh RGB-D / held-state outcome verification
```

The agent feedback loop is semantic. Exact measured geometry remains inside the
deterministic perception/execution side of the boundary.

## Authority boundaries

The LLM is a **task-level controller**, not an actuator or geometric controller. In fixed-task mode it proposes an ordered task. In agent mode it can repeatedly choose among supervisor-generated high-level capabilities. It must never directly control joints, Cartesian coordinates, velocities, motors, torques, collision-scene edits, safety limits, or trajectories.

Agent mode adds authority without bypassing the old boundary: the operator confirms object IDs, target IDs, named poses, deterministic completion conditions, and an action budget. `finish` is offered only after those conditions are deterministically true. Each subsequent model decision must be one of the exact capabilities offered for the fresh current state and remains inside that confirmed scope. Model-visible semantic observations deliberately omit `position_m` and other physical-control values.

Only the deterministic validation and execution path may authorize motion, regardless of language-control mode:

1. The validator accepts an allowed skill with complete, safe parameters.
2. Perception grounds referenced objects and confirms required confidence and scene conditions.
3. A second capture rejects stale state or measured scene drift immediately before execution.
4. The coordinator expands the skill using measured object/target positions and policy-owned offsets.
5. MoveIt 2 applies configured kinematic, collision, and trajectory constraints.
6. The controller interface executes the approved trajectory and reports the result.
7. Two fresh RGB-D captures and attached-object state verify the placement outcome.

Any failed validation, missing grounding, unsafe condition, planning failure,
execution fault, timeout, or failed outcome check makes the run fail closed.
Success means both motion execution and the independent observable outcome were
accepted.

## High-level skill boundary

The versioned schema allows `move_named_pose`, `open_gripper`, `close_gripper`,
`pick`, `place`, `pick_and_place`, `inspect`, `stop`, and `refuse`. Each skill has one
exact field combination. Grounded skills must copy the current scene revision
and use observed object and target IDs from the configured allowlists. Named poses are limited
to `ready`, `extended`, and `transport`.

The schema intentionally has no coordinate or control fields, sets
`additionalProperties` to false, and is executed directly with JSON Schema
Draft 2020-12 before semantic or robot-state checks. Task-step syntax is derived
from the same schema, not maintained as an independent set of combinations.
Recursive forbidden-field diagnostics identify joints, velocities, motors,
efforts, torques, and trajectories even if a malformed producer tries to nest
them. Only the validator can mint the in-process `ValidatedSkill` type required
by coordination.

The reusable executor supports separate `pick` and `place` skills as well as
`pick_and_place`, named poses, and opening an empty gripper. Standalone
`close_gripper` remains syntactically defined but is semantically refused:
closing around an object requires a validated pick sequence. Opening while
holding, another pick, or a named-pose motion while carrying are refused in
`SkillValidator` and checked again downstream. `inspect` returns a
fresh observed object without requesting motion. MoveIt's attached-object state
supplies logical holding information when a grasped cube is occluded. A final
two-frame camera observation remains mandatory for successful placement.

STOP/cancel/abort/halt phrases take a deterministic cancellation path before
model invocation, capture, task-state checks, or confirmation. The browser's
STOP button remains the quickest control during an in-flight operation; speech
still requires recording/transcription. This is software cancellation, not a
hardware-certified emergency stop. Clarification state stores the original
request and bounded question/answer turns, never recursively wrapped prompts.

`config/safety_policy.json` is the canonical physical safety policy; `config/agent_policy.json` separately bounds semantic agent capabilities and action count without changing motion limits; object dimensions,
target identities and table geometry are authored once in its referenced scene
file. Runtime target poses are never read from authored spawn positions. CMake
generates C++ constants from that effective policy. The executor checks a policy
digest against the Python launch configuration and refuses mismatched builds.
Released objects remain in the collision world through retreat; a conservative
world-aligned box covers the transformed release pose and settling to the
measured target support height. No release-time world-object removal is allowed.

See [safety hardening validation](safety_hardening.md) for offline evidence and
the required simulator deployment checks before using the revised executor.

`/vgm/stop` cancels through MoveIt and inhibits subsequent primitives. A wall-time
watchdog can cancel during a blocking motion, not only between primitives.
Faults latch at the session boundary. Ordinary recovery requires stationary
measured joints, no active executor, and no attached object; it performs no
motion. If an object is attached, the operator must explicitly confirm a
placement-only recovery to an allowlisted target. That operation passes the
same fresh-scene, target-occupancy, MoveIt, watchdog, and outcome gates. Neither
path silently resumes the failed task. An uncertain physical outcome can
require inspection and a fresh simulator session.

The comparison capture is historical evidence (maximum age 120 seconds), not
motion authorization. Its geometry and original proposal are checked against
the newest capture, which must be at most 10 seconds old. ROS stationary/held
state is sampled before rendering to avoid consuming the camera freshness
window on ROS discovery. Candidate colored regions must pass the spatial gate;
their existing combined color, depth, area, and location score selects the
supported cube region instead of a slightly nearer low-support fragment.

The grasp position is the estimated cube center, not the visible front surface.
Per-pixel metric depth is transformed into world points. A supported dominant
vertical face is offset inward by half the configured cube width; supported
top-face extents use their observed midpoint. Incomplete or ambiguous geometry
is refused. Expected destinations only guide region association and never
replace the measured grasp coordinates.

WSL's preliminary validation uses Brev's reported sensor-host clock, advanced
with a monotonic timer, so a small workstation/VM clock offset cannot make a
fresh frame look future-dated. Sensor timestamps are never rewritten. This
preliminary clock estimate cannot authorize motion: Brev performs the final
freshness checks against its own actual wall clock after a new capture.

## Deployment boundary

The workstation sends JSON requests over authenticated SSH; no application
listener is required. Only the transcript and allowlisted semantic information are sent to OpenAI. In agent mode that can additionally include visibility/holding/occupancy relations, the prior sanitized result, remaining action budget, completion-condition status, and supervisor-generated capability options. Coordinates and robot-control values remain local to the deterministic stack. WAV recordings are transcribed on Brev and are excluded from
Git and audit logs. API credentials remain local.

The GPU deployment is headless and uses NVIDIA Isaac Sim 6.0.1 in its pinned
container. ROS 2 Jazzy and MoveIt 2 stay in the repository's locked Pixi
environment. Docker host networking is used only for ROS discovery; no Docker
ports or cloud firewall rules are published. Generated images, depth arrays,
logs, credentials, and run manifests remain ignored, while code, configuration,
tests, and documentation are kept in Git.
