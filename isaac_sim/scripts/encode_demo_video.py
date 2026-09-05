#!/usr/bin/env python3
"""Encode captured camera frames as a real-time H.264 MP4 using FFmpeg."""

import argparse
import json
from pathlib import Path
import re
import subprocess


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("recording", type=Path)
    parser.add_argument("output", type=Path)
    parser.add_argument("--ffmpeg", default="ffmpeg")
    args = parser.parse_args()
    root = args.recording.resolve()
    value = json.loads((root / "recording.json").read_text())
    frames = value["frames"]
    if not frames:
        parser.error("recording contains no frames")
    lines = ["ffconcat version 1.0"]
    for index, frame in enumerate(frames):
        name = frame["file"]
        if not re.fullmatch(r"frame-[0-9]{6}\.jpg", name) or not (root / name).is_file():
            parser.error("invalid or missing recorded frame")
        end = frames[index + 1]["elapsed_s"] if index + 1 < len(frames) else value["duration_s"]
        duration = end - frame["elapsed_s"]
        if duration <= 0:
            parser.error("frame timestamps must increase")
        lines.extend([f"file '{name}'", f"duration {duration:.6f}"])
    lines.append(f"file '{frames[-1]['file']}'")
    playlist = root / "frames.ffconcat"
    playlist.write_text("\n".join(lines) + "\n")
    subprocess.run([args.ffmpeg, "-hide_banner", "-nostdin", "-n", "-safe", "1",
                    "-f", "concat", "-i", str(playlist), "-vf", "fps=30",
                    "-c:v", "libx264", "-crf", "20", "-pix_fmt", "yuv420p",
                    "-movflags", "+faststart", str(args.output.resolve())], check=True)


if __name__ == "__main__":
    main()
