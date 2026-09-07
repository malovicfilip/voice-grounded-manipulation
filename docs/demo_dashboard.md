# Unified demo dashboard

The root `./demo` launcher provides one browser surface for command entry, mission
or task review, STOP/recovery, agent decisions, semantic state, and the Isaac viewer.
The live simulator is the primary visual surface rather than a small card: on desktop
it occupies the main workspace, mission controls sit immediately below it, and a
compact right rail carries execution history, grounded world state, completion
conditions, and the safety boundary. The HTTP command bridge remains loopback-only.

## Quick local sandbox

```bash
./demo replay
```

This starts an in-memory six-cube/two-target sandbox and the closed-loop agent UI.
It requires no GPU, ROS, SSH, OpenAI key, or cloud VM and never requests robot motion.
The table panel is a schematic browser visualization generated from the replay
backend, not an Isaac render.

Use planned-task mode instead:

```bash
./demo replay --control task
```

## Remote live demo

First start the existing reusable Isaac/MoveIt session and NVIDIA WebRTC viewer using
the deployment runbooks. Then, on the workstation:

```bash
./demo doctor \
  --mode remote \
  --session YOUR_SESSION_ID \
  --host vgm-isaac-dev \
  --remote-root /home/ubuntu/voice-grounded-manipulation \
  --viewer-url http://YOUR_SERVER_IPV4:8210/
```

If the passive checks look correct:

```bash
./demo run \
  --mode remote \
  --control agent \
  --session YOUR_SESSION_ID \
  --host vgm-isaac-dev \
  --remote-root /home/ubuntu/voice-grounded-manipulation \
  --viewer-url http://YOUR_SERVER_IPV4:8210/
```

Use `--remote-root` whenever the Brev checkout is not `/home/ubuntu/workspace`.
The remote path is syntax-validated before it is inserted into the fixed SSH command.

The dashboard prints and normally opens:

```text
http://localhost:8766/
```

The viewer URL is embedded as a cross-origin frame only after it passes a strict
`http`/`https`, host, optional-port-only parser. The page CSP authorizes exactly
that viewer origin; it does not use `frame-src *`.

Embedding is a convenience layer only. It does **not** proxy WebRTC signalling,
change NVIDIA's viewer configuration, expose robot-control endpoints, start the
simulator, or change host/cloud firewall rules. Streaming access still has to follow
`docs/live_view.md`.


### Presentation-only live view

The integrated Isaac scene authors a dedicated spectator framing and matte studio
backdrop for the WebRTC stream. These `/World/Presentation/*` prims are deliberately
visual-only: the builder applies no collision, rigid-body, or mass API to them. The
workspace RGB-D camera remains `/World/WorkspaceCamera` with its original calibrated
pose and is still the only camera used for grounding and outcome verification.
Changing the spectator framing therefore changes the video, not robot authority.

## Local live session

When a reusable Isaac/MoveIt session is already running on the same Linux machine:

```bash
./demo run \
  --mode local \
  --control agent \
  --session YOUR_SESSION_ID \
  --viewer-url http://127.0.0.1:8210/
```

The local adapter calls the same `SimulatorSession` backend directly rather than
crossing SSH. It does not start Isaac automatically.

## Doctor

`doctor` never requests robot motion.

```bash
./demo doctor --mode replay
./demo doctor --mode remote --session YOUR_SESSION_ID --host vgm-isaac-dev
```

Depending on mode/options, it checks:

- safety/skill/agent configuration and JSON Schemas;
- dashboard assets;
- local OpenAI key presence for live LLM modes;
- SSH executable and host reachability;
- reusable session status;
- stationary robot state;
- passive fresh RGB-D capture;
- viewer HTTP reachability.

Skip RGB-D capture if desired:

```bash
./demo doctor --mode remote --session YOUR_SESSION_ID --skip-capture
```

Machine-readable output:

```bash
./demo doctor --mode replay --json
```

## Dashboard security properties

The dashboard server:

- binds only `127.0.0.1`;
- accepts only localhost Host values;
- requires same-origin POST requests;
- requires a random per-process CSRF token;
- bounds JSON and WAV request sizes;
- accepts only mono 16-bit PCM WAV up to 15 seconds for browser recording;
- sends `no-store`, `nosniff`, frame-ancestor, referrer, and CSP headers;
- never logs transcripts/audio/tokens through the HTTP access logger;
- contains no shell or arbitrary proxy endpoint.

STOP bypasses the normal operation queue. If an operation is already in flight, the
browser controller reasserts backend STOP after that worker exits so a late response
cannot silently clear the stop state.

## Controls

### Agent mode

1. Type or record a goal.
2. Review goal summary, object scope, target scope, named-pose scope, deterministic completion conditions, and action budget.
3. Confirm the mission.
4. Watch fresh semantic observations and one-step LLM decisions in the timeline.
5. `finish` becomes selectable only after the confirmed completion conditions are
   deterministically satisfied. A placement can remain complete when the cube occludes
   its target marker because the two-frame outcome verifier records a verified placement
   pose and the next fresh object observation must remain within the normal drift bound.
   The run otherwise continues until the model asks for help, STOP/fault occurs, or the
   action budget is exhausted.

### Task mode

1. Type or record an instruction.
2. Review the exact proposed task steps.
3. Confirm that fixed task.
4. Existing `TaskSession` executes with fresh observation before every skill.

## Microphone behavior

Remote mode retains the existing workflow: the browser creates a bounded WAV, the
workstation uploads it over SSH, and Whisper transcribes on the remote host. Local
mode invokes the local faster-whisper adapter. Replay mode intentionally disables
microphone transcription so the zero-dependency demo does not imply speech services
are running.

## Shutdown

Ctrl+C on `./demo run` requests STOP before closing the dashboard server. Closing a
browser tab alone does not stop an executing task, remote simulator, WebRTC viewer,
or paid cloud compute. Keep the independent terminal STOP/shutdown commands from the
existing runbooks available during live demonstrations.
