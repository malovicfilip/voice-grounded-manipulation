# Record the simulation demo

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
