"""Workstation transport: credentials remain local; only JSON crosses SSH."""

from __future__ import annotations

import json
import re
import shlex
import subprocess

from .serialization import scene_from_mapping


class SSHBackend:
    def __init__(self, session: str, *, host: str = "vgm-isaac-dev"):
        if not re.fullmatch(r"[A-Za-z0-9][A-Za-z0-9_-]{0,63}", session):
            raise ValueError("invalid session name")
        if not re.fullmatch(r"[A-Za-z0-9][A-Za-z0-9_.-]{0,100}", host):
            raise ValueError("invalid SSH host alias")
        self.session, self.host = session, host

    def request(self, operation: str, **fields):
        script = (
            "cd /home/ubuntu/workspace && "
            "export FASTRTPS_DEFAULT_PROFILES_FILE=/home/ubuntu/IsaacSim-ros_workspaces/jazzy_ws/fastdds.xml && "
            "/home/ubuntu/.pixi/bin/pixi run --manifest-path ros2_ws/pixi.toml "
            "bash -c 'source ros2_ws/install/setup.bash; "
            "export PYTHONPATH=/home/ubuntu/workspace/ros2_ws/src/vgm_runtime; "
            "exec python -m vgm_runtime.session_backend \"$1\"' bash " + shlex.quote(self.session)
        )
        completed = subprocess.run(
            ["ssh", "-T", "-o", "BatchMode=yes", "-o", "ConnectTimeout=10", self.host, script],
            input=json.dumps({"operation": operation, **fields}, allow_nan=False),
            capture_output=True, text=True, timeout=180,
        )
        # ROS writes informational startup lines on some installations. The
        # protocol response is the final JSON line, never a shell expression.
        lines = completed.stdout.strip().splitlines()
        if not lines:
            raise RuntimeError(f"SSH session request failed (exit {completed.returncode})")
        response = json.loads(lines[-1])
        if completed.returncode or not response.get("ok"):
            raise RuntimeError(response.get("message", "remote operation failed"))
        return response["result"]

    def capture(self):
        return scene_from_mapping(self.request("capture"))

    def execute(self, proposal, scene):
        observed = scene.objects.get(proposal["object_id"])
        return self.request("execute", proposal=proposal,
                            baseline={"scene": scene.to_mapping(),
                                      "object_position": list(observed.position_m) if observed else None})

    def stop(self):
        return self.request("stop")

    def recover(self):
        return self.request("recover")
