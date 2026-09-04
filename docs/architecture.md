# Architecture

## Purpose

The system turns a spoken manipulation request into a safe simulated action by a Franka Panda. It is deliberately layered so that language interpretation cannot bypass perception, validation, motion planning, or execution safeguards.

## Components

- **User interaction** — receives a spoken request and returns confirmations, clarifications, refusals, and status.
- **Whisper speech-to-text** — uses local `faster-whisper` to convert an audio file to a transcript and refuses empty or low-confidence results.
- **RGB-D perception and scene grounding** — synchronizes RGB and metric depth, identifies configured colored cubes, back-projects them into the world frame, and records object confidence, time, pixel support, and a stable scene revision.
- **LLM intent-to-skill interface** — asks `gpt-5.6-terra` for one tool-free, strict-schema high-level skill using only the transcript and allowlisted scene identifiers. API storage is disabled.
- **Skill validator and policy layer** — checks the exact schema, allowed skill/parameter combinations, replay protection, freshness, object confidence, workspace limits, and forbidden direct-control fields. It rejects anything invalid, ambiguous, unsupported, stale, or unsafe.
- **Pre-execution gate** — obtains a fresh RGB-D scene, revalidates the minted skill and policy version, rejects more than 1 cm of commanded-object drift, and regenerates the deterministic plan.
- **Task coordinator** — expands a validator-minted skill into an immutable sequence of bounded task primitives. The current `pick_and_place` plan contains gripper, Cartesian approach/retreat, attach, and detach operations; no model-supplied coordinates are used.
- **MoveIt 2** — performs kinematic planning, collision checking, and trajectory generation within configured limits.
- **ROS 2 Jazzy** — provides communication, lifecycle management, transforms, robot state, and telemetry between the system components.
- **Isaac Sim** — hosts the simulated workspace, Franka Panda, sensors, physics, and simulation clock.
- **Safe execution and Panda controller interface** — accepts only allowlisted identifiers and validator-derived coordinates, adds the table/cubes to the planning scene, caps motion scaling, enforces the deadline, executes approved MoveIt-generated trajectories, and reports state.
- **Outcome validator and audit log** — requires final RGB-D evidence of the placed cube within target tolerance and records decisions without API keys, tokens, or raw audio.

## Data flow

```text
Isaac RGB + metric depth ──> scene grounding ──> scene revision + object IDs
                                                     │
Spoken request ──> Whisper transcript ───────────────┤
                                                     v
                  constrained LLM skill proposal (no tools or coordinates)
                                                     │
                                                     v
                         strict schema + policy validation
                                                     │
fresh RGB-D scene ──> freshness/drift revalidation ──┤
                                                     v
                       deterministic task primitives
                                                     │
                                                     v
                    MoveIt 2 collision-aware planning
                                                     │
                                                     v
             ROS 2 controllers ──> Panda in Isaac Sim
                                                     │
                                                     v
           final RGB-D outcome check + status/audit result
```

## Authority boundaries

The LLM is an intent interpreter, not a robot controller. It must never directly control joints, velocities, motors, or trajectories. It can only emit a validated high-level skill proposal, for example `pick(object_id)` or `place(object_id, target_id)`, using an explicitly defined schema.

Only the deterministic validation and execution path may authorize motion:

1. The validator accepts an allowed skill with complete, safe parameters.
2. Perception grounds referenced objects and confirms required confidence and scene conditions.
3. A second capture rejects stale state or commanded-object drift immediately before execution.
4. The coordinator expands the skill using policy-owned positions and offsets.
5. MoveIt 2 applies configured kinematic, collision, and trajectory constraints.
6. The controller interface executes the approved trajectory and reports the result.
7. A final RGB-D capture confirms the expected placement outcome.

Any failed validation, missing grounding, unsafe condition, planning failure,
execution fault, timeout, or failed outcome check makes the run fail closed.
Success means both motion execution and the independent observable outcome were
accepted.

## High-level skill boundary

The versioned schema allows `move_named_pose`, `open_gripper`, `close_gripper`,
`pick`, `place`, `pick_and_place`, `stop`, and `refuse`. Each skill has one
exact field combination. Grounded skills must copy the current scene revision
and use observed object IDs plus configured target IDs. Named poses are limited
to `ready`, `extended`, and `transport`.

The schema intentionally has no coordinate or control fields, sets
`additionalProperties` to false, and is duplicated by deterministic semantic
checks. Recursive forbidden-field detection rejects joints, velocities, motors,
efforts, torques, and trajectories even if a malformed producer tries to nest
them. Only the validator can mint the in-process `ValidatedSkill` type required
by coordination.

The current integrated executor supports the complete `pick_and_place` path.
Other schema skills establish and test the task-level API boundary but are not
all exposed as standalone live simulator demos yet.

## Deployment boundary

The GPU deployment is headless and uses NVIDIA Isaac Sim 6.0.1 in its pinned
container. ROS 2 Jazzy and MoveIt 2 stay in the repository's locked Pixi
environment. Docker host networking is used only for ROS discovery; no Docker
ports or cloud firewall rules are published. Generated images, depth arrays,
logs, credentials, and run manifests remain ignored, while code, configuration,
tests, and documentation are kept in Git.
