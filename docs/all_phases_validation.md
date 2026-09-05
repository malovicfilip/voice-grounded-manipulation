# All-phase simulation acceptance

The completion scope is the versioned six-cube Franka Panda scene in Isaac Sim
6.0.1, ROS 2 Jazzy, and MoveIt 2. A complete run must demonstrate speech input,
constrained LLM interpretation, RGB-D grounding, confirmed task execution,
independent outcome verification, live cancellation, and recovery. Physical
hardware, arbitrary household-object recognition, and grasp training are outside
this repository's simulated manipulation milestone.

## Acceptance matrix

| Phase | Required behavior | Evidence |
| --- | --- | --- |
| 1: foundation | Panda/ROS/MoveIt execute a bounded skill; malformed skills cause no motion; stop cancels actual motion; a fault prevents subsequent primitives | `boundary`, `stop`, and `fault` live suites, plus unit tests |
| 2: grounding | Synchronized RGB-D identifies configured objects; missing, stale, low-confidence, invalid-frame, or moved observations are refused; placement is measured from a final camera frame | RGB-D tests, live captures and placement outcome records |
| 3: voice | A WAV is transcribed by Whisper, interpreted by the real LLM, confirmed, and executed; ambiguity triggers clarification and no motion | `voice` and `dialogue` live suites |
| 4: autonomy | A bounded ordered sequence supports inspect, pick, place, and pick-and-place; holding and target-occupancy preconditions hold; faults latch; explicit recovery never silently resumes a failed task | `sequence`, `fault`, and `stop` live suites, plus task-session tests |

## Run the campaign

First verify the [Brev cost guard](brev_development.md#six-hour-shutdown-guard),
resume the existing instance, and build the two project packages. Then open a
reusable simulator session on Brev:

```bash
cd /home/ubuntu/workspace
ACCEPT_EULA=Y isaac_sim/scripts/run_voice_manipulation_demo.sh \
  --session --run-id my-session
```

`--session` keeps Isaac and MoveIt available for repeated captures and skills,
avoiding repeated camera startup. `--run-id` must be unique. The launcher prints
`Reusable simulator session ready` after the first frame and active controllers.
It limits the simulator session to two hours and cleans up its own processes;
the separate Brev guard is responsible for stopping compute billing.

From WSL, run the campaign using the existing local API credential:

```bash
python3 isaac_sim/scripts/validate_all_phases.py \
  --session my-session \
  --audio /tmp/vgm-spoken-command.wav \
  --output isaac_sim/_output/all-phases-acceptance.json
```

The audio should say “Pick the red cube and place it on the blue target.” The
runner uploads only this recording over SSH; Whisper runs on Brev, while the
OpenAI request and key stay in WSL. The runner explicitly confirms its fixed
test commands and records each proposal, result, and measured outcome. A failed
assertion returns nonzero and writes `status: failed`; it cannot produce a pass
report based only on successful process startup. `--suite` can select one test
when diagnosing a failed campaign.

## Operator console

```bash
PYTHONPATH=ros2_ws/src/vgm_runtime python3 -m vgm_runtime.task_cli \
  --session my-session
```

Enter a natural-language command or `audio /path/to/recording.wav`. The console
shows the exact bounded task and waits for `yes`. `no` cancels a pending task.
An ambiguous request asks for clarification; a reply is interpreted with the
original request. Ctrl+C requests cancellation during execution; `stop` can
also be entered at a prompt or sent from a second console with `--operation
stop`. `recover` checks for a
stationary robot and no held object, clears the fault, and requires a new
command and confirmation. Recovery never opens a loaded gripper or restarts a
failed sequence automatically.

If a stopped robot has a verified attached object, enter `recover place
blue_target` (or `yellow_target`) and confirm `yes`. This is an explicit
placement-only recovery, with the normal validation, occupied-target check,
MoveIt cancellation/deadline, and final camera gate. Its one-shot equivalent is:

```bash
PYTHONPATH=ros2_ws/src/vgm_runtime python3 -m vgm_runtime.task_cli \
  --session my-session --operation recover_place --target blue_target --confirm
```

`--confirm` authorizes this one recovery placement; it does not restart the
failed sequence. If holding or the final physical outcome is uncertain, keep
the session stopped for inspection and start a new scene when necessary.

If SSH fails during cancellation, `stop_unconfirmed` means the controller's
state is unknown, not that motion has stopped. Do not issue another task;
reconnect, inspect `--operation robot_state`, and explicitly recover. The remote
executor watchdog and separate Brev stop guard remain independent safeguards.

For a single supplied command, `--transcript` or `--audio` selects its source.
The explicit `--confirm` flag supports repeatable, unattended demos. The task
schema allows at most eight skills; confirmation expires after two minutes.
Every motion still receives fresh remote validation and the MoveIt executor's
120-second watchdog. A five-minute task deadline is checked between skills.
The LLM must never directly control joints, velocities,
motors, efforts, torques, or trajectories.

## Limits of the evidence

Perception uses the known colored-cube geometry and camera calibration. Object
height on the table comes from the versioned cube dimensions; RGB and metric
depth determine the horizontal surface points, and the known upright cube
geometry converts visible faces to a center estimate. The estimator is not a
general six-degree-of-freedom pose estimator. Confidence is a heuristic
quality score, not a calibrated probability. MoveIt attachment indicates logical
holding state; a subsequent observed placement confirms the physical simulated
outcome. Results do not establish safety for a real robot or unknown obstacles.
The voice fixture is a generated spoken WAV, not a microphone/noisy-room study.
There is no claim of arbitrary-object recognition or a statistically measured
task success rate from this small acceptance campaign.

The stop fixture must observe arm motion before cancellation, a live executor
stop acknowledgement, no following descent primitive, and less than 0.01 rad
joint drift over the final half-second observation window. The separate fault
fixture injects an inhibit-only failure before pickup approach and verifies
fault latching and explicit recovery.

## Regression history

The retained earlier runs are failures, not retroactively accepted results.
They exposed: scene-revision hash jitter, aging of a historical comparison
capture, a low-support color fragment outranking the cube, workstation/VM clock
offset, and a real missed pickup caused by using a front-surface point as the
grasp center. The last failure left the cube at its source; the independent
final RGB-D gate rejected the placement even though MoveIt had reported a
logical attachment and successful motion. Corrections have regression tests;
the final campaign must be repeated with fresh camera evidence after changes.

## Source references

- [Isaac Sim 6.0.1 MoveIt integration](https://docs.isaacsim.omniverse.nvidia.com/6.0.1/ros2_tutorials/tutorial_ros2_moveit.html)
- [MoveIt MoveGroupInterface API](https://moveit.picknik.ai/main/api/html/classmoveit_1_1planning__interface_1_1MoveGroupInterface.html)
- [OpenAI structured outputs](https://developers.openai.com/api/docs/guides/structured-outputs)
- [Brev instance lifecycle and billing](https://docs.nvidia.com/brev/concepts/gpu-instances)
