#!/usr/bin/env python3
"""Record the existing full acceptance campaign; does not start cloud compute."""

import argparse
import json
import os
from pathlib import Path
import re
import selectors
import subprocess
import sys
import time

ROOT = Path(__file__).resolve().parents[2]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--session", required=True)
    parser.add_argument("--audio", type=Path, required=True)
    args = parser.parse_args()
    if not re.fullmatch(r"[A-Za-z0-9][A-Za-z0-9_-]{0,63}", args.session):
        parser.error("invalid session")
    if not args.audio.is_file():
        parser.error("audio file missing")
    output = ROOT / "isaac_sim/_output" / args.session
    output.mkdir(exist_ok=False)
    remote = f"/home/ubuntu/workspace/isaac_sim/_output/{args.session}"
    ssh = ["ssh", "-T", "-o", "BatchMode=yes", "-o", "ConnectTimeout=10", "vgm-isaac-dev"]
    subprocess.run(ssh + [f"test -f {remote}/session.ready && test ! -e {remote}/recording.start && touch {remote}/recording.start"], check=True, timeout=30)
    started = time.monotonic()
    events = []
    process = None
    try:
        process = subprocess.Popen([sys.executable, "-u", str(ROOT / "isaac_sim/scripts/validate_all_phases.py"),
                                    "--session", args.session, "--audio", str(args.audio.resolve()),
                                    "--output", str(output / "acceptance.json")],
                                   stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
        selector = selectors.DefaultSelector()
        selector.register(process.stdout, selectors.EVENT_READ)
        pending = b""
        while True:
            if time.monotonic() - started > 1140:
                raise TimeoutError("recorded campaign exceeded 19 minutes")
            if not selector.select(timeout=1):
                continue
            chunk = os.read(process.stdout.fileno(), 65536)
            if not chunk:
                break
            pending += chunk
            while b"\n" in pending:
                line, pending = pending.split(b"\n", 1)
                text = line.decode("utf-8", errors="replace")
                events.append({"elapsed_s": time.monotonic() - started, "text": text})
                print(text, flush=True)
        process.wait(timeout=10)
    finally:
        if process is not None and process.poll() is None:
            process.terminate()
            try:
                process.wait(timeout=10)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait()
        (output / "timeline.json").write_text(json.dumps(events, indent=2) + "\n")
        subprocess.run(ssh + [f"touch {remote}/recording.stop"], check=True, timeout=30)
        for _ in range(20):
            result = subprocess.run(ssh + [f"test -f {remote}/recording.done"], timeout=15)
            if result.returncode == 0:
                break
            time.sleep(1)
        else:
            raise RuntimeError("recording did not finalize; preserve the running session for inspection")
    return process.returncode


if __name__ == "__main__":
    raise SystemExit(main())
