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
    parser.add_argument("--timeline", type=Path, help="Optional campaign timeline for chapter captions")
    parser.add_argument("--audio", type=Path, help="Replay synthetic input audio at the voice chapter")
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
    command = [args.ffmpeg, "-hide_banner", "-nostdin", "-n", "-safe", "1",
               "-f", "concat", "-i", str(playlist)]
    filters = "fps=30"
    voice_start = None
    if args.timeline:
        events = json.loads(args.timeline.read_text())
        titles = {
            "boundary": "Safety: invalid commands rejected before motion",
            "stop": "Safety: stop during motion, then explicit recovery",
            "fault": "Safety: injected fault, motion inhibited, explicit recovery",
            "dialogue": "LLM: clarify the request, then confirm inspection",
            "voice": "Whisper + LLM: red cube to blue target\nSynthetic input audio replayed here",
            "sequence": "Multi-step: inspect, pick, place on yellow, inspect",
        }
        chapters = []
        previous_pass = None
        for event in events:
            if event["text"].startswith("PASS "):
                previous_pass = event["elapsed_s"]
            match = re.fullmatch(r"Running (\w+) acceptance", event["text"])
            if match and match[1] in titles:
                # Suites run consecutively. The preceding PASS marks the
                # transition even in older, line-buffered capture timelines.
                start = previous_pass if previous_pass is not None else event["elapsed_s"]
                chapters.append((start, titles[match[1]]))
                if match[1] == "voice":
                    voice_start = start
        def timestamp(seconds):
            milliseconds = max(0, round(seconds * 1000))
            seconds, milliseconds = divmod(milliseconds, 1000)
            minutes, seconds = divmod(seconds, 60)
            hours, minutes = divmod(minutes, 60)
            return f"{hours:02}:{minutes:02}:{seconds:02},{milliseconds:03}"
        subtitles = []
        for index, (start, title) in enumerate(chapters):
            end = chapters[index + 1][0] if index + 1 < len(chapters) else value["duration_s"]
            subtitles.append(f"{index + 1}\n{timestamp(start)} --> {timestamp(end)}\n{title}\n")
        (root / "chapters.srt").write_text("\n".join(subtitles))
        filters += ",subtitles=chapters.srt:force_style='FontSize=14,Alignment=8,MarginV=10'"
    if args.audio:
        if voice_start is None:
            parser.error("audio replay requires a timeline with a voice chapter")
        command += ["-i", str(args.audio.resolve()), "-af", f"adelay={round(voice_start * 1000)}:all=1,apad",
                    "-c:a", "aac", "-b:a", "128k"]
    command += ["-vf", filters, "-t", str(value["duration_s"] - frames[0]["elapsed_s"]),
                "-c:v", "libx264", "-crf", "20", "-pix_fmt", "yuv420p",
                "-movflags", "+faststart", str(args.output.resolve())]
    subprocess.run(command, check=True, cwd=root)


if __name__ == "__main__":
    main()
