# Architecture

## Purpose

The system turns a spoken manipulation request into a safe simulated action by a Franka Panda. It is deliberately layered so that language interpretation cannot bypass perception, validation, motion planning, or execution safeguards.

## Components

- **User interaction** — receives a spoken request and returns confirmations, clarifications, refusals, and status.
- **Whisper speech-to-text** — converts audio to a transcript with associated confidence or error signals.
- **LLM intent-to-skill interface** — proposes a structured, high-level skill request from the transcript and available task context. Its output is limited to the approved skill schema.
- **Skill validator and policy layer** — checks schema validity, allowed skill names and parameters, workspace and task policies, required object references, and safety preconditions. It rejects anything that is invalid, ambiguous, unsupported, or unsafe.
- **RGB-D perception and scene grounding** — detects or tracks relevant objects and estimates scene state, object poses, and confidence needed to resolve valid skill parameters.
- **Task coordinator** — expands an accepted high-level skill into deterministic robot-task actions and requests plans from MoveIt 2.
- **MoveIt 2** — performs kinematic planning, collision checking, and trajectory generation within configured limits.
- **ROS 2 Jazzy** — provides communication, lifecycle management, transforms, robot state, and telemetry between the system components.
- **Isaac Sim** — hosts the simulated workspace, Franka Panda, sensors, physics, and simulation clock.
- **Franka Panda controller interface** — executes approved MoveIt-generated trajectories in simulation and reports state.

## Data flow

```text
Spoken request
  -> Whisper transcript
  -> LLM structured high-level skill proposal
  -> Skill validator and policy checks
  -> RGB-D grounding and scene-state checks
  -> Task coordinator
  -> MoveIt 2 plan and collision checks
  -> ROS 2 control messages
  -> Franka Panda in Isaac Sim
  -> robot state, camera data, and execution result
  -> task coordinator and user interaction
```

## Authority boundaries

The LLM is an intent interpreter, not a robot controller. It must never directly control joints, velocities, motors, or trajectories. It can only emit a validated high-level skill proposal, for example `pick(object_id)` or `place(object_id, target_id)`, using an explicitly defined schema.

Only the deterministic validation and execution path may authorize motion:

1. The validator accepts an allowed skill with complete, safe parameters.
2. Perception grounds referenced objects and confirms required confidence and scene conditions.
3. The coordinator requests a plan from MoveIt 2.
4. MoveIt 2 applies configured kinematic, collision, and trajectory constraints.
5. The controller interface executes the approved trajectory and reports the result.

Any failed validation, missing grounding, unsafe condition, planning failure, or execution fault must stop the requested action and produce a refusal or recoverable error state.

## Initial high-level skill boundary

The first executable boundary is an allowlisted `move_named_pose` demonstration
with only `ready`, `extended`, and `transport`. The deterministic client rejects
any other identifier before starting simulation or requesting a plan, caps
velocity and acceleration scaling at 20%, and delegates all trajectory creation
and execution to MoveIt 2. The Isaac bridge consumes only the resulting ROS
control commands and does not expose a language-model control path.

This is a narrow Phase 1 proof, not the final LLM interface. The complete
structured skill schema, gripper operations, simulated stop/no-op, workspace
bounds, perception grounding, confirmation policy, and preconditions must still
be versioned and tested before voice-driven execution is enabled.
