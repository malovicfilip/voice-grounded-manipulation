# Record the simulation demo

## Verified recording: September 4, 2026

Session `full-demo-video-v1` completed all six live acceptance suites. Its
308.72-second recording contains 2,333 actual camera frames (640 x 480).
`isaac_sim/_output/full-demo-video-v1/full-demo.mp4` is a 5,790,450-byte H.264/AAC
MP4 with phase captions and explicitly labeled synthetic input audio replay.
The complete file was decoded successfully and copied to Windows Downloads as
`voice-grounded-manipulation-full-demo-2026-09-04.mp4`; both copies have SHA-256
`184f7b9f7f997945942d0c3ed8f88143ff262000561bcdadbff489cdfc0ba6c2`.

The original frames, capture timestamps, acceptance report, and raw timeline
are retained locally in the ignored session directory. The fixed camera shows
the tabletop work area; part of the raised arm is outside its view. This is a
complete real-time camera recording, not a recording of the Isaac Sim editor.

## Recording workflow

The integrated scene supports opt-in recording from its existing RGB-D camera.
It records real rendered frames, not reconstructed motion or generated video.
The LLM still emits only validated high-level skills; recording never commands
joints, velocities, motors, or trajectories.

Verify and rearm the [Brev shutdown guard](brev_development.md#six-hour-shutdown-guard)
before resuming the existing VM. Launch a reusable session as described in
[the demo runbook](all_phases_validation.md). No public ports are needed.

After `session.ready`, create `recording.start` in that session's output
directory on Brev. This enables JPEG capture at up to 10 frames per wall-clock
second. Run the existing full acceptance campaign with the synthetic spoken
command. Create `recording.stop` and wait for `recording.done` before stopping
the simulator or downloading the session directory using SCP.

Alternatively, from WSL the following command manages both recording markers,
runs all six suites, and saves phase timestamps. It uses the existing local API
credential and confirms only the campaign's fixed synthetic test commands:

```bash
python3 isaac_sim/scripts/record_full_demo.py \
  --session my-session --audio /tmp/vgm-spoken-command.wav
```

Use a new session ID; the local output directory must not already exist. The
runner does not resume or stop cloud compute. If interrupted during motion,
use the task console's stop operation before investigating or shutting down.

Recording stops automatically after 20 minutes or approximately 1 GB (the
byte limit may be exceeded by one frame). `recording/recording.json` contains
frame timestamps and the stop reason. Do not reuse an existing recording
directory. A hard-killed process may leave incomplete frames without metadata;
always wait for `recording.done` before normal shutdown.

Encode locally or on Brev where FFmpeg with libx264 is available:

```bash
python3 isaac_sim/scripts/encode_demo_video.py \
  isaac_sim/_output/my-session/recording \
  isaac_sim/_output/my-session/full-demo.mp4
```

This creates an H.264 MP4 suitable for ordinary video players and refuses to
overwrite an existing video. Wall-clock frame durations are preserved in a
30-fps playback container; repeated frames fill gaps, not invented intermediate
motion. Camera recording alone is silent. Optional `--timeline` and `--audio`
arguments add phase captions and replay the synthetic input WAV at the voice
chapter. The caption explicitly labels this as replayed input, not a live
microphone recording. Chapter timing is approximate (SSH startup latency),
while camera-frame spacing uses timestamps from the simulator host.
Keep the input WAV and acceptance report alongside it to document speech
transcription and confirmed skills.
Large media remain ignored; recording code and this runbook belong in GitHub.

Camera reference: [NVIDIA Isaac Sim 6.0.1 camera documentation](https://docs.isaacsim.omniverse.nvidia.com/6.0.1/sensors/isaacsim_sensors_camera.html).
