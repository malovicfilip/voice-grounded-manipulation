# Interactive Isaac Sim viewing

Preparation is implemented; end-to-end browser streaming is **not verified**.
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

Run this on the VM only after approval. It creates `VGM-STREAM`, accepts
loopback and the named client for only the streaming ports, drops other
sources for those ports, and blocks IPv6 streaming. SSH and default firewall
policies are unchanged. These host rules are not persistent across reboots;
verify/reapply them before every streaming launch. The script refuses to
overwrite an existing chain; inspect existing rules if it reports an error.

## Missing driver libraries (approval required)

The existing L4 driver is `595.71.05`. Rendering works, but the VM and container
lack `libnvidia-encode.so.1`. A package dry run showed exactly two new packages,
no upgrades and no removals:

- `libnvidia-encode-595-server=595.71.05-0ubuntu0.22.04.1`
- `libnvidia-decode-595-server=595.71.05-0ubuntu0.22.04.1`

Installation was blocked by the permission reviewer under the earlier
no-dependencies instruction. Obtain explicit approval before installation;
do not substitute a different driver version or work around the restriction.
After approved installation, verify `ldconfig -p` inside a GPU container with
`NVIDIA_DRIVER_CAPABILITIES=compute,utility,graphics,video,display` includes the
encoder library. The launcher passes these capabilities.

## Browser viewer

`isaac_sim/web_viewer/Dockerfile` builds NVIDIA's browser client independently
of the simulator, using scaffold version 1.14.2. Build with
`--build-arg ISAACSIM_HOST=SERVER_IPV4`; rebuild if the VM's public IP changes.
Run with host networking only after verifying the above restrictions. It
serves port 8210 and connects to signaling 49100 / media 47998. Do not run
NVIDIA's full two-service example alongside our launcher: that would start a
second simulator. Do not configure automatic container restart.

The viewer image built successfully during preparation. npm reported one
moderate and one high dependency advisory; their applicability must be reviewed
before serving the viewer. No viewer container or simulator was started during
this preparation session.

Check HTTP delivery, signaling, GPU encoding, and actual browser-rendered
frames before claiming success. The stream is interactive and not read-only:
use viewport navigation, but avoid altering robot or physics state in the UI
while the project controller is running.

Microphone input is a separate path. The existing task console supports
`audio PATH` with explicit task confirmation; live push-to-talk is not yet
implemented. The viewer does not itself interpret spoken robot commands.

References: [NVIDIA 6.0.1 livestream clients](https://docs.isaacsim.omniverse.nvidia.com/6.0.1/installation/manual_livestream_clients.html),
[official viewer source](https://github.com/isaac-sim/IsaacSim/tree/987015050efebfd0cd5d3736ae47fffe5adee308/tools/docker/web-viewer).
