"""Operator console for confirmed voice/text tasks in a remote simulator session."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import subprocess
import threading

from .audit import AuditLogger
from .openai_intent import load_local_api_key
from .ssh_backend import SSHBackend
from .tasks import OpenAITaskModel, TaskSession


def upload_and_transcribe(backend, audio: Path):
    audio = audio.expanduser().resolve()
    if not audio.is_file() or not 0 < audio.stat().st_size <= 20_000_000:
        raise ValueError("audio must be a nonempty file no larger than 20 MB")
    destination = f"{backend.host}:/home/ubuntu/workspace/isaac_sim/_output/{backend.session}/command.wav"
    subprocess.run(["scp", "--", str(audio), destination], check=True, timeout=60)
    return backend.request("transcribe")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--session", required=True)
    parser.add_argument("--host", default="vgm-isaac-dev")
    source = parser.add_mutually_exclusive_group()
    source.add_argument("--transcript")
    source.add_argument("--audio", type=Path)
    parser.add_argument("--confirm", action="store_true", help="explicitly confirm this supplied command for unattended execution")
    parser.add_argument("--operation", choices=("stop", "recover", "capture", "robot_state", "shutdown"))
    parser.add_argument("--audit", type=Path, default=Path("isaac_sim/_output/operator-audit.jsonl"))
    args = parser.parse_args()
    backend = SSHBackend(args.session, host=args.host)
    if args.operation:
        print(json.dumps(backend.request(args.operation), indent=2))
        return
    load_local_api_key()
    session = TaskSession(OpenAITaskModel(), backend, AuditLogger(args.audit))

    def execute_pending(token):
        # Keep Ctrl+C responsive while the SSH execution is pending.
        outcome = []
        worker = threading.Thread(target=lambda: outcome.append(session.confirm(token)))
        worker.start()
        try:
            while worker.is_alive():
                worker.join(timeout=0.2)
        except KeyboardInterrupt:
            print(json.dumps(session.stop(), indent=2))
            worker.join(timeout=15)
            if worker.is_alive():
                raise RuntimeError("stop requested; remote completion is still pending")
        return outcome[0] if outcome else {"status": "stopped"}

    if args.transcript is not None or args.audio is not None:
        transcript = args.transcript
        if args.audio is not None:
            speech = upload_and_transcribe(backend, args.audio)
            print(json.dumps({"transcription": speech}, indent=2))
            transcript = speech["text"]
        pending = session.prepare(transcript)
        print(json.dumps(pending, indent=2))
        if pending["status"] != "awaiting_confirmation":
            raise SystemExit(2)
        confirmed = args.confirm or input("Execute this task? [yes/no] ").strip().lower() == "yes"
        if not confirmed:
            print("Cancelled before motion.")
            return
        result = execute_pending(pending["confirmation"])
        print(json.dumps(result, indent=2))
        raise SystemExit(0 if result["status"] == "completed" else 2)

    print("Enter a command, 'audio PATH', 'yes', 'no', 'stop', 'recover', or 'quit'. Ctrl+C requests stop.")
    while True:
        try:
            text = input("vgm> ").strip()
            if text.lower() in {"quit", "exit"}:
                session.stop()
                return
            if text.lower() == "yes":
                if session.pending is None:
                    print("No task is awaiting confirmation.")
                    continue
                result = execute_pending(session.pending.confirmation)
            elif text.lower() == "no":
                if session.state == "awaiting_confirmation":
                    session.pending = None
                    session.state = "idle"
                result = {"status": "cancelled_before_motion"}
            elif text.lower() == "recover":
                result = session.recover()
            elif text.lower() == "stop":
                result = session.stop()
            else:
                if text.startswith("audio "):
                    speech = upload_and_transcribe(backend, Path(text[6:]))
                    print(json.dumps({"transcription": speech}, indent=2))
                    text = speech["text"]
                result = session.prepare(text)
            print(json.dumps(result, indent=2))
        except (KeyboardInterrupt, EOFError):
            print(json.dumps(session.stop(), indent=2))
            return
        except Exception as error:
            print(f"Refused: {error}")


if __name__ == "__main__":
    main()
