# Voice-Grounded Manipulation

Voice-Grounded Manipulation is an embodied-AI robotics project for safely executing spoken manipulation tasks with a simulated Franka Panda robot. The system combines NVIDIA Isaac Sim, ROS 2 Jazzy, MoveIt 2, RGB-D perception, Whisper speech transcription, and an LLM that translates user intent into a small, validated vocabulary of high-level robot skills.

Safety is a core design constraint: the LLM must never directly control joints, velocities, motors, or trajectories. It may only propose structured high-level skills; deterministic validation, task planning, motion planning, collision checking, and robot-control layers retain authority over execution.

## Initial scope

The first milestone establishes a reproducible simulation and an end-to-end, safety-bounded command path. The target platform is a Franka Panda in Isaac Sim, connected through ROS 2 Jazzy and MoveIt 2, with RGB-D observations available to the perception stack.

## Roadmap

### Phase 1 — Foundation and validation

- Bring up the Panda scene in Isaac Sim and verify ROS 2 connectivity.
- Confirm MoveIt 2 can plan and execute a small set of safe simulated motions.
- Establish the accepted high-level skill schema and validation boundary.
- Verify that invalid, ambiguous, or unsafe requests are rejected without robot motion.

### Phase 2 — Perception and grounding

- Add RGB-D camera inputs and scene/object representations.
- Ground validated skills against observable objects, poses, and workspace constraints.
- Test perception failure handling and confidence-based refusal paths.

### Phase 3 — Voice interaction

- Integrate Whisper for speech-to-text.
- Connect transcription to constrained intent extraction and skill proposals.
- Add confirmations, clarifications, and audit logs for spoken commands.

### Phase 4 — Task-level autonomy

- Expand the validated skill library for pick, place, inspect, and related tasks.
- Compose multi-step tasks only through validated skill sequences.
- Evaluate robustness, recovery behavior, and safety limits in simulation before any hardware work.

## Repository layout

- `isaac_sim/` — Isaac Sim scenes and supporting scripts.
- `ros2_ws/` — ROS 2 Jazzy workspace source tree.
- `config/` — shared configuration files.
- `docs/` — architecture and validation criteria.
- `tests/` — future automated and scenario-based tests.

## Current status

This repository currently contains only the initial scaffold and planning documents. No dependencies, runtime configuration, or implementation code have been added.
