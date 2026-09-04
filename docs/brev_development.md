# NVIDIA Brev development environment

This project uses a stoppable NVIDIA Brev instance for GPU simulation work.
The verified development instance is `vgm-isaac-dev`, backed by GCP machine
type `g2-standard-8` with one NVIDIA L4 GPU, 8 CPUs, 32 GiB system memory, and
128 GiB flexible storage.

The commands in this guide operate on an existing instance. They do not create
or resize resources.

## Local WSL setup

Install the Brev CLI in Ubuntu WSL using NVIDIA's documented installer:

```bash
bash -c "$(curl -fsSL https://raw.githubusercontent.com/brevdev/brev-cli/main/bin/install-latest.sh)"
source ~/.profile
brev --version
```

The installer places `brev` in `~/.local/bin`. Reloading `.profile` makes that
directory available in the current shell; the version command verifies the
installation.

Authenticate without asking WSL to launch a browser, then synchronize Brev's
managed SSH configuration:

```bash
brev login --skip-browser
brev refresh
brev ls
```

`brev login` prints a browser URL and stores credentials plus SSH material in
`~/.brev`. `brev refresh` updates local SSH aliases without changing instance
lifecycle state. `brev ls` is a read-only state check.

## Cost and persistence boundary

According to NVIDIA Brev's instance lifecycle documentation:

- A running instance accrues compute charges.
- A stopped instance does not accrue compute charges; storage charges continue.
- `/home/ubuntu/workspace`, system packages, and Docker data persist across a
  stop, but not deletion.
- Restarting is subject to GPU capacity in the original provider and region.

Store the repository in `/home/ubuntu/workspace` and push work to GitHub before
every stop. Never use `brev delete` as a substitute for `brev stop`; deletion is
irreversible.

Official references:

- [GPU instance lifecycle and billing](https://docs.nvidia.com/brev/concepts/gpu-instances)
- [Brev instance management commands](https://docs.nvidia.com/brev/cli/instance-management)

## Six-hour shutdown guard

Brev CLI v0.6.334 exposes `brev stop` but no native delayed-stop flag. Before
starting paid work, create a one-time Windows Scheduled Task that runs the
authenticated WSL command. The first attempt below occurs after 5 hours 50
minutes and repeats every 5 minutes for one hour to tolerate a transient API or
network failure:

```powershell
$taskName = "Brev-Stop-vgm-isaac-dev"
$firstAttempt = (Get-Date).AddHours(5).AddMinutes(50)
$action = New-ScheduledTaskAction `
  -Execute "C:\Windows\System32\wsl.exe" `
  -Argument "-d Ubuntu-24.04 -u malov -- /home/malov/.local/bin/brev stop vgm-isaac-dev"
$trigger = New-ScheduledTaskTrigger `
  -Once -At $firstAttempt `
  -RepetitionInterval (New-TimeSpan -Minutes 5) `
  -RepetitionDuration (New-TimeSpan -Hours 1)
$settings = New-ScheduledTaskSettingsSet `
  -StartWhenAvailable -WakeToRun `
  -ExecutionTimeLimit (New-TimeSpan -Minutes 10) `
  -AllowStartIfOnBatteries -DontStopIfGoingOnBatteries
Register-ScheduledTask `
  -TaskName $taskName `
  -Action $action `
  -Trigger $trigger `
  -Settings $settings `
  -Description "Safety guard: stop Brev instance vgm-isaac-dev within six hours" `
  -Force
```

Verify the task before beginning GPU work:

```powershell
Get-ScheduledTask -TaskName "Brev-Stop-vgm-isaac-dev"
Get-ScheduledTaskInfo -TaskName "Brev-Stop-vgm-isaac-dev"
```

The task must be `Ready`, its action must name only `vgm-isaac-dev`, and its
next run must be less than six hours away. `WakeToRun` applies when supported by
Windows power and hardware settings. If Windows is fully powered off or cannot
reach Brev, the task cannot enforce the deadline; stop the instance manually
before shutting down the workstation and confirm `STOPPED` in `brev ls` or the
Brev console.

Re-arm and re-verify the guard at the beginning of every paid session.

## SSH and VS Code

After `brev refresh`, verify terminal access:

```bash
ssh vgm-isaac-dev
```

The repository lives directly at `/home/ubuntu/workspace` on this instance.

Install Microsoft's `ms-vscode-remote.remote-ssh` extension in Windows VS Code.
The Brev-generated SSH configuration enables Unix connection multiplexing,
which Windows OpenSSH does not support. Place this override before Brev's
`Include` line in `C:\Users\malov\.ssh\config`:

```sshconfig
Host vgm-isaac-dev
  ControlMaster no
  ControlPath none
  RequestTTY no
```

Set the VS Code user setting:

```json
"remote.SSH.path": "C:\\Windows\\System32\\OpenSSH\\ssh.exe"
```

Then choose **Remote-SSH: Connect to Host** and select `vgm-isaac-dev`, or open
the remote authority from a Windows terminal:

```powershell
code --new-window --remote ssh-remote+vgm-isaac-dev /home/ubuntu/workspace
```

This uses SSH and does not require exposing a public application port. NVIDIA
also documents a WSL SSH wrapper for instances whose SSH path uses Cloudflare
Tunnel:

- [VS Code setup](https://docs.nvidia.com/brev/latest/guides/development-tools/vscode-setup)
- [Windows/WSL SSH troubleshooting](https://docs.nvidia.com/brev/troubleshooting/ide-connectivity/vscode-windows-wsl)

## GitHub access on the temporary VM

Use GitHub CLI's browser/device flow rather than copying workstation tokens or
private keys into the VM:

```bash
gh auth login --hostname github.com --git-protocol https --web
gh auth setup-git --hostname github.com
gh auth status --hostname github.com
git fetch --dry-run origin
git push --dry-run origin main
```

On a headless VM without a credential vault, GitHub CLI stores its token in
`~/.config/gh/hosts.yml`. Confirm that the file mode is `600`. Before deleting
or transferring the VM, run `gh auth logout --hostname github.com` and revoke
the temporary authorization in GitHub account settings.

## Isaac Sim 6.0.1 container

The remote/headless deployment uses NVIDIA's official container:

```bash
docker pull nvcr.io/nvidia/isaac-sim:6.0.1
docker image inspect nvcr.io/nvidia/isaac-sim:6.0.1 \
  --format '{{.Id}} {{.Architecture}}'
```

`docker pull` downloads the versioned image without installing Isaac Sim into
the host OS. `docker image inspect` verifies the local content address and CPU
architecture. The image verified on this instance is `amd64` with image ID:

```text
sha256:783444c706538aa76cf5126e911ddc5e618779e6105305ad4af4260362a30aa9
```

The Brev base image originally included the NVIDIA compute stack but not the
matching Vulkan/OpenGL userspace package. Install only the package matching the
existing 595.71.05 server driver, then refresh both CDI specification paths:

```bash
sudo apt-get install -y libnvidia-gl-595-server
sudo systemctl restart nvidia-cdi-refresh.service
sudo nvidia-ctk cdi generate --output=/var/run/cdi/nvidia.yaml
```

The first command adds NVIDIA's graphics libraries without upgrading or
replacing the installed driver. Brev's custom refresh service regenerates
`/etc/cdi/nvidia.yaml` and restarts Docker. The final command refreshes the
standard runtime CDI file too; otherwise Docker can resolve the stale duplicate
in `/var/run/cdi` and omit the graphics libraries from containers.

Create persistent cache and state directories owned by Isaac Sim's rootless
container user. The final directory holds generated, git-ignored scene output:

```bash
sudo install -d -o 1234 -g 1234 \
  /home/ubuntu/docker/isaac-sim/cache/main \
  /home/ubuntu/docker/isaac-sim/cache/computecache \
  /home/ubuntu/docker/isaac-sim/config \
  /home/ubuntu/docker/isaac-sim/data \
  /home/ubuntu/docker/isaac-sim/logs \
  /home/ubuntu/docker/isaac-sim/pkg \
  /home/ubuntu/.cache/ov/hub \
  /home/ubuntu/workspace/isaac_sim/_output
```

`install -d` creates missing directories and assigns numeric UID/GID 1234 in a
single operation. Verify Vulkan, the L4, and the remaining requirements with
NVIDIA's bundled checker:

```bash
docker run --name isaac-sim-compat-6-0-1 \
  --entrypoint bash --gpus all --rm --network=host \
  -e ACCEPT_EULA=Y \
  nvcr.io/nvidia/isaac-sim:6.0.1 \
  ./isaac-sim.compatibility_check.sh --/app/quitAfter=10 --no-window
```

`--gpus all` supplies the L4, `--rm` removes the temporary container, and
`--no-window` runs headlessly. `--network=host` follows NVIDIA's checker and
cloud-container guidance; it does not publish a Docker port. Use
`ACCEPT_EULA=Y` only after accepting NVIDIA's license. Privacy telemetry remains
opted out because `PRIVACY_CONSENT` is deliberately unset. The verified result
on this instance is `System checking result: PASSED` with NVIDIA L4, Vulkan,
driver 595.71.05, and 24.15 GB VRAM.

Build the repository's Phase 1 scene without publishing ports:

```bash
docker run --name vgm-phase1-build --rm --gpus all \
  -e ACCEPT_EULA=Y -e PYTHONUNBUFFERED=1 -u 1234:1234 \
  -v /home/ubuntu/docker/isaac-sim/cache/main:/isaac-sim/.cache:rw \
  -v /home/ubuntu/docker/isaac-sim/cache/computecache:/isaac-sim/.nv/ComputeCache:rw \
  -v /home/ubuntu/docker/isaac-sim/logs:/isaac-sim/.nvidia-omniverse/logs:rw \
  -v /home/ubuntu/docker/isaac-sim/config:/isaac-sim/.nvidia-omniverse/config:rw \
  -v /home/ubuntu/docker/isaac-sim/data:/isaac-sim/.local/share/ov/data:rw \
  -v /home/ubuntu/docker/isaac-sim/pkg:/isaac-sim/.local/share/ov/pkg:rw \
  -v /home/ubuntu/.cache/ov/hub:/var/cache/hub:rw \
  -v /home/ubuntu/workspace:/workspace:ro \
  -v /home/ubuntu/workspace/isaac_sim/_output:/workspace/isaac_sim/_output:rw \
  -w /workspace --entrypoint /isaac-sim/python.sh \
  nvcr.io/nvidia/isaac-sim:6.0.1 -u \
  /workspace/isaac_sim/scripts/build_phase_1_scene.py --headless
```

The repository is mounted read-only while its ignored output subdirectory is a
separate writable mount. The builder has been verified to produce
`isaac_sim/_output/phase_1_scene.usd` containing the Franka asset reference, six
unique cubes, and an RGB-D camera with `OmniSensorAPI`. No WebRTC or other
public application ports have been opened. See NVIDIA's
[container installation guide](https://docs.isaacsim.omniverse.nvidia.com/6.0.1/installation/install_container.html)
and [RTX camera API](https://docs.isaacsim.omniverse.nvidia.com/6.0.1/py/source/extensions/isaacsim.sensors.experimental.rtx/docs/index.html).

## Phase 1 visual demo

Run the repository launcher from the Brev checkout:

```bash
cd /home/ubuntu/workspace
ACCEPT_EULA=Y isaac_sim/scripts/run_phase_1_demo.sh
```

`ACCEPT_EULA=Y` records the NVIDIA license acceptance for the container run.
The launcher authors and saves `phase_1_scene.usd` from the checked-in JSON
contract, then reads one frame from that live stage with Isaac Sim 6.0.1's
`CameraSensor`. RGB is stored as PNG. Depth remains a floating-point NumPy array
in meters, with a separately generated colorized PNG for visual inspection.
`PRIVACY_CONSENT` is not set, and no network ports are published.

The final console line identifies a unique directory such as:

```text
/home/ubuntu/workspace/isaac_sim/_output/demo-20260904T041500Z
```

Inspect the manifest and copy the small generated directory back to WSL:

```bash
ssh vgm-isaac-dev \
  'cat /home/ubuntu/workspace/isaac_sim/_output/<demo-id>/manifest.json'
scp -r \
  vgm-isaac-dev:/home/ubuntu/workspace/isaac_sim/_output/<demo-id> \
  isaac_sim/_output/
```

`ssh ... cat` displays the recorded resolution, scene, artifact names, and
observed depth range. `scp -r` transfers only that ignored demo directory over
the authenticated SSH connection; it does not expose a listening port.

The first verified L4 capture spent about six minutes inside Isaac Sim's first
render update before completing. During that interval the process can remain
CPU-bound with low GPU utilization. NVIDIA notes that robot assets may take
multiple minutes to load on first use, and Isaac Sim 6.0.1 has a reported
`simulation_app.update()` livelock that can recover without an application
error. Do not accept a run based on process activity alone: require a zero exit
status, `manifest.json` with `status: captured`, a nonempty RGB image, and a
finite positive depth range. See NVIDIA's
[Isaac Sim asset guidance](https://docs.isaacsim.omniverse.nvidia.com/6.0.1/assets/usd_assets_overview.html)
and the upstream
[Isaac Sim 6.0.1 update issue](https://github.com/isaac-sim/IsaacSim/issues/728).

The first accepted run produced a 640 x 480 RGB image and metric depth from
approximately 0.993 m to 1.743 m. The workspace camera is intentionally framed
for the six manipulation cubes; this sensor demo does not command or animate
the Franka.

An interactive WebRTC session is a separate option. NVIDIA documents TCP 49100
for signaling and UDP 47998 for video, and warns that the stream has no built-in
authentication or encryption. Do not expose those ports without restricting
them to the viewer's public IP and obtaining explicit approval first. See the
[Isaac Sim container streaming guide](https://docs.isaacsim.omniverse.nvidia.com/6.0.1/installation/install_container.html).

## ROS 2 Jazzy and MoveIt 2

The verified Brev host is Ubuntu 22.04 (Jammy), while ROS 2 Jazzy targets Ubuntu
24.04 (Noble). Do not add Noble ROS apt repositories to the Jammy host. Instead,
keep Jazzy and MoveIt isolated in a project-local Pixi environment using the
RoboStack Jazzy packages declared by `ros2_ws/pixi.toml`. Isaac Sim remains in
the pinned NVIDIA container, so the environment does not download a duplicate
Isaac Sim or PyTorch installation.

Install Pixi in the remote user's home and verify it:

```bash
curl -fsSL https://pixi.sh/install.sh | bash
/home/ubuntu/.pixi/bin/pixi --version
```

The first command runs Pixi's official user-level installer; the second proves
which binary is available without relying on a newly modified shell `PATH`.
Clone NVIDIA's matching ROS workspace tag and verify the pinned revision:

```bash
git clone --branch IsaacSim-6.0.1 --depth 1 \
  https://github.com/isaac-sim/IsaacSim-ros_workspaces.git \
  /home/ubuntu/IsaacSim-ros_workspaces
git -C /home/ubuntu/IsaacSim-ros_workspaces rev-parse HEAD
```

The checkout supplies NVIDIA's Franka MoveIt configuration,
`topic_based_ros2_control`, and Fast DDS profile. The verified tag resolves to
commit `dd3eeed`. Install the locked project environment:

```bash
cd /home/ubuntu/workspace
/home/ubuntu/.pixi/bin/pixi install --manifest-path ros2_ws/pixi.toml
```

`pixi install` resolves or reuses `ros2_ws/pixi.lock` and materializes the
ignored `ros2_ws/.pixi` environment. Build only the five NVIDIA packages needed
by this demo into the project workspace, then build the project package:

```bash
cd /home/ubuntu/workspace/ros2_ws
/home/ubuntu/.pixi/bin/pixi run --manifest-path pixi.toml \
  colcon build --base-paths /home/ubuntu/IsaacSim-ros_workspaces/jazzy_ws/src \
  --packages-select isaac_moveit moveit_resources \
  moveit_resources_panda_description moveit_resources_panda_moveit_config \
  topic_based_ros2_control
/home/ubuntu/.pixi/bin/pixi run --manifest-path pixi.toml \
  bash -c 'source install/setup.bash && colcon build \
  --packages-select vgm_moveit_demo'
```

The first build imports the pinned upstream sources but writes all colcon
outputs beneath this repository's ignored `ros2_ws/{build,install,log}` paths.
The second sources that overlay and compiles the constrained application.

Run the live headless demo from the repository root:

```bash
cd /home/ubuntu/workspace
ACCEPT_EULA=Y isaac_sim/scripts/run_moveit_demo.sh extended
```

The launcher validates the requested named pose before setup, starts the exact
Isaac Sim 6.0.1 image, waits for a synchronized initial RGB capture, starts the
headless MoveIt stack, requires active controllers, and invokes the constrained
client. The client accepts only `ready`, `extended`, or `transport`, caps both
velocity and acceleration scaling at 20%, and delegates planning and execution
to MoveIt. It has no direct joint, velocity, motor, or trajectory publisher.

The Isaac bridge uses NVIDIA's Fast DDS profile and Docker `--network=host` so
ROS discovery works between the container and host. It publishes no Docker
ports (`-p`/`--publish`) and this workflow makes no cloud firewall change. A
successful output directory contains `initial.png`, `final.png`, initial and
final joint-state YAML, Isaac/MoveIt/skill logs, and `manifest.json`. The
launcher also stops its exact container and MoveIt process group on success or
failure.

The verified live run planned and executed both `ready` and `extended`. For the
`extended` result, joint 4 settled at approximately `-0.0698` rad instead of the
SRDF's nominal zero, demonstrating enforcement of the Panda joint limit rather
than direct command passthrough. The final automated `moveit-visual-v3` run
exited with status 0 and produced its two 640 x 480 RGB frames, joint-state
snapshots, logs, and success manifest before cleaning up its container and
MoveIt process group. NVIDIA's matching references are the
[Isaac Sim 6.0.1 MoveIt tutorial](https://docs.isaacsim.omniverse.nvidia.com/6.0.1/ros2_tutorials/tutorial_ros2_moveit.html),
[ROS 2 installation guide](https://docs.isaacsim.omniverse.nvidia.com/6.0.1/installation/install_ros.html),
and [IsaacSim ROS workspaces](https://github.com/isaac-sim/IsaacSim-ros_workspaces/tree/IsaacSim-6.0.1).

## Session shutdown

Commit and push all work, stop the instance, and verify its state:

```bash
git status --short --branch
git push origin main
brev stop vgm-isaac-dev
brev ls
```

Do not assume that closing SSH, VS Code, Jupyter, or a browser stops billing.
Only proceed after Brev reports the instance as `STOPPED`.
