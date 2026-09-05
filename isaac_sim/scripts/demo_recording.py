"""Bounded, opt-in camera recording; never issues robot commands."""

import json
import time
from pathlib import Path

import numpy as np
from PIL import Image


class DemoRecording:
    def __init__(self, output, fps=10, max_seconds=1200, max_bytes=1_000_000_000):
        self.output = Path(output)
        self.directory = self.output / "recording"
        self.interval = 1 / fps
        self.max_seconds = max_seconds
        self.max_bytes = max_bytes
        self.started = None
        self.previous = None
        self.frames = []
        self.bytes = 0
        self.finished = False

    def update(self, rgb, now=None):
        if self.finished or not (self.output / "recording.start").exists():
            return
        now = time.monotonic() if now is None else now
        if self.started is None:
            self.directory.mkdir(exist_ok=False)
            self.started = now
        if (self.output / "recording.stop").exists():
            self.finish("requested", now)
            return
        if now - self.started >= self.max_seconds or self.bytes >= self.max_bytes:
            self.finish("limit", now)
            return
        if rgb is None or (self.previous is not None and now - self.previous < self.interval):
            return
        array = np.asarray(rgb)[..., :3]
        if np.issubdtype(array.dtype, np.floating):
            array = np.clip(array * 255, 0, 255)
        path = self.directory / f"frame-{len(self.frames):06d}.jpg"
        Image.fromarray(array.astype(np.uint8)).save(path, quality=90)
        self.frames.append({"file": path.name, "elapsed_s": now - self.started})
        self.bytes += path.stat().st_size
        self.previous = now

    def finish(self, reason="simulator_exit", now=None):
        if self.finished or self.started is None:
            return
        now = time.monotonic() if now is None else now
        metadata = {"status": reason, "duration_s": now - self.started,
                    "bytes": self.bytes, "frames": self.frames,
                    "timing": "wall-clock; no motion interpolation"}
        (self.directory / "recording.json").write_text(json.dumps(metadata, indent=2) + "\n")
        self.finished = True
        (self.output / "recording.done").touch()
