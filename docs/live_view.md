# Interactive Isaac Sim viewing

This runbook describes the simulator and streaming services. Readiness must be
verified for each new session; a reachable viewer alone does not prove that
the robot or camera is ready. The September 6 safety deployment uses session
`safety-live-v1` (server IP `34.31.210.199`).
Use the existing integrated scene with `--session --stream-host SERVER_IPV4`.
This enables `omni.kit.livestream.app` using the installed Isaac Sim 6.0.1
standalone example. The existing ROS/MoveIt control boundary is unchanged.

## Access and cost prerequisites

Rearm and verify the [Brev shutdown guard](brev_development.md#six-hour-shutdown-guard)
before every paid session. Cloud firewall ports TCP 49100/8210 and UDP 47998
must be restricted to the client's current public IPv4, never all sources.
The console screenshot lists the three ports but does not display each saved
source restriction. Add a host-side restriction as defense in depth:

```bash
bash isaac_sim/scripts/restrict_streaming_ports.sh CLIENT_IPV4
```

Run this on the VM only after approval. It atomically creates the native
nftables `inet vgm_stream` table, accepts
loopback and the named client for only the streaming ports, drops other
sources for those ports, and blocks IPv6 streaming. SSH and default firewall
policies are unchanged. These host rules are not assumed persistent across reboots;
verify/reapply them before every streaming launch. The script refuses to
overwrite an existing table; inspect existing rules if it reports an error.
Inspect with `sudo nft list table inet vgm_stream`. Native nftables is needed
because Brev's boot configuration is not always compatible with iptables.

## Driver video libraries

The existing L4 driver is `595.71.05`. On September 5, 2026, after explicit
approval, the following two matching video libraries were installed with
no upgrades and no removals:

- `libnvidia-encode-595-server=595.71.05-0ubuntu0.22.04.1`
- `libnvidia-decode-595-server=595.71.05-0ubuntu0.22.04.1`

Refreshing `nvidia-cdi-refresh.service` was required to expose the newly
installed libraries to GPU containers. `ldconfig -p` inside Isaac Sim now
includes the encoder library. The launcher passes
`NVIDIA_DRIVER_CAPABILITIES=compute,utility,graphics,video,display`.

## Browser viewer

`isaac_sim/web_viewer/Dockerfile` builds NVIDIA's browser client independently
of the simulator, using scaffold version 1.14.2. Build with
`--build-arg ISAACSIM_HOST=SERVER_IPV4`; rebuild if the VM's public IP changes.
Run with host networking only after verifying the above restrictions. It
serves port 8210 and connects to signaling 49100 / media 47998. Do not run
NVIDIA's full two-service example alongside our launcher: that would start a
second simulator. Do not configure automatic container restart.

The viewer image builds successfully. npm reports advisories in the Mocha test
dependency and its serialize-javascript dependency. The serving image now
contains only the compiled browser assets and a dependency-free Node static
server, not node_modules, the test tooling, or Vite's development/preview
server. HTTP tests verify GET succeeds, POST is rejected, and a traversal
request cannot read the server source. This is not a comprehensive security
audit of the NVIDIA browser bundle.

Check HTTP delivery, signaling, GPU encoding, and actual browser-rendered
frames before claiming success. The stream is interactive and not read-only:
use viewport navigation, but avoid altering robot or physics state in the UI
while the project controller is running.

## Use the running scene

Start the simulator in its own terminal or named tmux session on Brev:

```bash
cd /home/ubuntu/workspace
ACCEPT_EULA=Y isaac_sim/scripts/run_voice_manipulation_demo.sh \
  --session --run-id NEW_UNIQUE_RUN_ID --stream-host SERVER_IPV4
```

Use a new run ID for each new session. After the initial camera warm-up and
`Reusable simulator session ready`, open `http://SERVER_IPV4:8210/` in Edge or
Chrome. Only one streaming client should be connected at once. The current
session example below uses `safety-live-v1`. In the earlier streaming setup,
HTTP delivery from Windows, an established
client signaling connection, the UDP media socket, and GPU encoder activity
have been verified. The first browser request exposed a 1080p/720p mismatch;
the viewer was rebuilt to request 1280 x 720 at 30 fps, matching the simulator.
Refresh the viewer after this update. A displayed live picture still requires
browser confirmation. No motion commands were issued during streaming setup.

Run the existing task console in WSL, from the repository root:

```bash
PYTHONPATH=ros2_ws/src/vgm_runtime python3 -m vgm_runtime.task_cli --session safety-live-v1
```

The console uses the existing local API key and SSH; the key stays in WSL.
Review each proposed task and explicitly confirm it. The LLM must never
directly control joints, velocities, motors, or trajectories.

## Microphone input

For the integrated **record → review → Confirm/Cancel** workflow without manual
downloads or terminal submission, use the [browser command console](browser_console.md)
at `http://localhost:8766/`. The file-based option below remains available as a
fallback.

The viewer does not itself interpret spoken robot commands. A separate,
local-only microphone page prepares a WAV for the existing task console:

```bash
python3 -m http.server 8765 --bind 127.0.0.1 --directory isaac_sim/microphone
```

Open `http://localhost:8765/` in Windows. Click Start, grant microphone
permission, speak, then Stop and download WAV. Both a timer and sample-count
limit bound recordings to 15 seconds. The page never uploads or executes
anything. Its JavaScript syntax and HTTP access from Windows were checked;
actual microphone capture requires the user's device and permission.

In the WSL task console, enter:

```text
audio /mnt/c/Users/malov/Downloads/vgm-command.wav
```

Adjust the filename if the browser renamed a repeated download. Audio is
uploaded over SSH to Whisper on Brev; its transcript feeds the existing
validated high-level skill path. Read the proposal and type `yes` only if
correct. This is record-then-submit, not hands-free or always-listening voice
control. Stop the localhost server with Ctrl+C when done.

## Stop the session

From WSL, request cleanup, stop the viewer, then stop paid compute:

```bash
PYTHONPATH=ros2_ws/src/vgm_runtime python3 -m vgm_runtime.task_cli --session safety-live-v1 --operation shutdown
ssh vgm-isaac-dev docker stop vgm-live-viewer
brev stop vgm-isaac-dev
brev ls
```

Verify `STOPPED`. Closing a browser tab does not stop compute. Keep Windows
powered and online for the workstation shutdown guard; it cannot enforce
the deadline if the workstation is shut down or disconnected.

References: [NVIDIA 6.0.1 livestream clients](https://docs.isaacsim.omniverse.nvidia.com/6.0.1/installation/manual_livestream_clients.html),
[official viewer source](https://github.com/isaac-sim/IsaacSim/tree/987015050efebfd0cd5d3736ae47fffe5adee308/tools/docker/web-viewer).
