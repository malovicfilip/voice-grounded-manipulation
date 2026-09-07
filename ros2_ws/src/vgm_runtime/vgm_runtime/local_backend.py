"""Same-machine adapter for an already running reusable Isaac/MoveIt session."""

from __future__ import annotations

import time
from pathlib import Path

from .session_backend import SimulatorSession


class LocalBackend:
    """Mirror :class:`SSHBackend` without crossing SSH.

    This adapter does not start Isaac Sim. It attaches to a reusable session
    created by ``run_voice_manipulation_demo.sh --session`` on this machine.
    """

    def __init__(self, session: str, *, repository_root: Path | None = None) -> None:
        import re
        if not re.fullmatch(r"[A-Za-z0-9][A-Za-z0-9_-]{0,63}", session):
            raise ValueError("invalid session name")
        root = repository_root or Path(__file__).resolve().parents[4]
        self.session = session
        self.directory = root / "isaac_sim" / "_output" / session
        self.session_backend = SimulatorSession(self.directory)

    @staticmethod
    def clock() -> float:
        return time.time()

    def capture(self):
        return self.session_backend.capture()

    def execute(self, proposal, scene):
        observed = scene.objects.get(proposal["object_id"])
        return self.session_backend.execute(
            proposal,
            {
                "scene": scene.to_mapping(),
                "object_position": list(observed.position_m) if observed else None,
            },
        )

    def stop(self):
        return self.session_backend.stop()

    def recover(self):
        return self.session_backend.recover()
