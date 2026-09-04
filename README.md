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
- `tests/` — automated offline and future simulation acceptance tests.

See [`docs/local_development.md`](docs/local_development.md) for the verified
Ubuntu 24.04 WSL, ROS 2 Jazzy, and workspace setup.

See [`docs/brev_development.md`](docs/brev_development.md) for the NVIDIA Brev
GPU environment, cost guard, SSH/VS Code connection, and persistence workflow.

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

## Current status

The local Ubuntu 24.04 development environment is configured with ROS 2 Jazzy
and the ROS development tools. The official publisher/subscriber demo and the
empty `ros2_ws` baseline build have passed. The versioned Phase 1 scene contract
and its builder pass all simulator-independent tests. A stoppable NVIDIA Brev
GPU environment has been verified with an NVIDIA L4, 8 CPUs, 32 GiB system RAM,
and persistent project storage under `/home/ubuntu/workspace`. The official
Isaac Sim 6.0.1 container passes NVIDIA's compatibility checker on that GPU, and
the Phase 1 Franka/RGB-D scene builds successfully in headless mode. The visual
demo captures RGB and metric-depth evidence without robot motion. MoveIt 2 and
application packages have not been installed yet.
