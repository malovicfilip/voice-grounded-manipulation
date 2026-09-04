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
