# Browser voice commands with confirmation

Use `http://localhost:8766/` in Windows while the local bridge is running.
No audio download or terminal submission is needed for each command.

1. Click **Start microphone** and grant browser microphone permission.
2. Speak a command, then click **Stop recording & review** (15-second maximum).
3. Wait for Whisper transcription and a validated high-level skill proposal.
4. Read **What was heard** and the listed object/target/skills.
5. Click **Confirm & execute**, or **Cancel / record again**.

There is also typed input. A rejected or ambiguous command cannot be confirmed;
enter the clarification or a new instruction. Confirmation expires after two
minutes and binds to the server's exact pending plan. A second click cannot
replay a completed task. The robot rechecks scene freshness, object movement,
and target occupancy before execution. The LLM never controls joints,
velocities, motors, or trajectories directly.

## Start the local bridge

The Brev simulation must already be ready and its cost guard verified. This
command does not resume cloud compute, open public ports, or move the robot:

```bash
cd /home/malov/voice-grounded-manipulation
PYTHONPATH=ros2_ws/src/vgm_runtime python3 -m vgm_runtime.browser_console \
  --session safety-live-v1 --port 8766
```

Use the current simulation's session ID. The existing local `.env.local` key
was explicitly authorized for reuse. Keep this terminal/process running while
using the browser page; use one browser console instead of concurrently
issuing tasks from the terminal console. The original file-based microphone
page at port 8765 remains a fallback.

## Stop and recovery

**STOP ROBOT** bypasses the model and the normal work queue. It invalidates
pending confirmation, inhibits further skills, and requests remote stopping.
If a prior operation is in flight, stopping is reasserted after it exits.
The page reports an unconfirmed stop as a fault, not success. After a stop or
fault, **Reset after stop / fault** requests explicit stationary, empty-handed
recovery. It never resumes the old task. Held-object placement recovery stays
in the existing operator console because it needs a separately confirmed target.

Closing a tab does not stop an executing task. If the browser connection fails,
use a separate WSL terminal:

```bash
PYTHONPATH=ros2_ws/src/vgm_runtime python3 -m vgm_runtime.task_cli \
  --session safety-live-v1 --operation stop
```

To finish the session, request simulator shutdown and run `brev stop
vgm-isaac-dev`, then verify `STOPPED` with `brev ls`. Neither closing the browser
nor stopping the local bridge stops compute billing. Keep Windows powered and
online for the scheduled shutdown guard.

## Data and request boundaries

- The server binds only `127.0.0.1`, not a public interface.
- Audio is validated as mono 16-bit PCM WAV, at most 15 seconds and 3 MB.
- Browser audio goes to WSL automatically, then over SSH to Whisper on Brev.
  The temporary WSL WAV is removed after transcription. Brev retains its most
  recent `command.wav`, as in the original console workflow.
- The LLM receives the transcript and scene identifiers through the existing
  adapter. The key remains in WSL, never in browser JavaScript or on Brev.
- Mutating requests require the exact local Host/Origin and an in-memory,
  per-process request token. No cross-origin permissions are enabled.
- Only fixed routes are served; arbitrary paths, executable commands, user
  supplied plans, and direct-control fields are not accepted by the web API.
- Preparation and execution are serialized. Stop remains independently
  available while transcription, model requests, or execution are pending.
- Audit records are kept in ignored `isaac_sim/_output/browser-audit.jsonl`.

The loopback HTTP tests require permission to bind local sockets. Run
`python3 -m unittest discover -s tests`. Tests use fake models/backends and
cover confirmation, replay, expiry, cancellation, stop races, recovery, WAV
limits, Host/Origin/token checks, and request-field validation. Real microphone
capture itself still depends on the user's browser permission and device.

On September 5, 2026, a live smoke test against `live-view-v2` submitted the
existing synthetic WAV through `/api/audio`, obtained the expected red-cube to
blue-target proposal, and cancelled it through `/api/cancel`. No confirmation
was sent. The robot remained stationary with measured joint drift 0.0 radians.
Windows HTTP access and JavaScript syntax were also verified. Motion after
confirmation is tested with fake backends here and uses the existing validated
TaskSession execution path; this smoke test did not initiate real motion.
