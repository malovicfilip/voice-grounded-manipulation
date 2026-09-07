"""One-command launcher and passive diagnostics for the VGM demo dashboard."""

from __future__ import annotations

import argparse
from dataclasses import dataclass, asdict
import json
import os
from pathlib import Path
import shutil
import signal
import subprocess
import sys
import urllib.request
import webbrowser

from jsonschema import Draft202012Validator

from .agent import OpenAIAgentActionModel, OpenAIMissionModel, action_schema, mission_schema
from .audit import AuditLogger
from .browser_console import AgentBrowserController, BrowserController, make_server, normalize_viewer_url
from .config import load_json_config
from .openai_intent import load_local_api_key
from .replay import ReplayAgentActionModel, ReplayBackend, ReplayMissionModel, ReplayTaskModel
from .tasks import OpenAITaskModel
from .validator import SkillValidator

REPOSITORY_ROOT = Path(__file__).resolve().parents[4]


@dataclass(frozen=True)
class Check:
    name: str
    status: str
    detail: str


def _check(name, function, *, optional=False):
    try:
        detail = function()
        return Check(name, "ok", str(detail or "ready"))
    except Exception as error:
        return Check(name, "warn" if optional else "fail", f"{type(error).__name__}: {error}")


def _require_session(args):
    if args.mode in {"remote", "local"} and not args.session:
        raise ValueError(f"--session is required in {args.mode} mode")
    return args.session or "replay"


def _backend(args):
    session = _require_session(args)
    if args.mode == "replay":
        return ReplayBackend()
    if args.mode == "remote":
        from .ssh_backend import SSHBackend
        return SSHBackend(session, host=args.host, remote_root=args.remote_root)
    from .local_backend import LocalBackend
    return LocalBackend(session)


def _local_transcribe(_backend, path):
    from .whisper_transcriber import FasterWhisperTranscriber
    transcript = FasterWhisperTranscriber().transcribe(path)
    return {"text": transcript.text, "confidence": transcript.confidence, "language": transcript.language}


def _controller(args, backend, audit):
    if args.mode == "replay":
        if args.control == "agent":
            return AgentBrowserController(ReplayMissionModel(), ReplayAgentActionModel(), backend, audit,
                                          transcribe=lambda *_: (_ for _ in ()).throw(RuntimeError("replay has no speech transcriber")))
        return BrowserController(ReplayTaskModel(), backend, audit,
                                 transcribe=lambda *_: (_ for _ in ()).throw(RuntimeError("replay has no speech transcriber")))

    load_local_api_key()
    if not os.environ.get("OPENAI_API_KEY"):
        raise RuntimeError("OPENAI_API_KEY is unavailable; add it to .env.local or the local environment")
    transcribe = _local_transcribe if args.mode == "local" else None
    if args.control == "agent":
        kwargs = {} if transcribe is None else {"transcribe": transcribe}
        return AgentBrowserController(OpenAIMissionModel(), OpenAIAgentActionModel(), backend, audit, **kwargs)
    kwargs = {} if transcribe is None else {"transcribe": transcribe}
    return BrowserController(OpenAITaskModel(), backend, audit, **kwargs)


def _viewer_check(url):
    normalized = normalize_viewer_url(url)
    if not normalized:
        return "not configured"
    request = urllib.request.Request(normalized, method="HEAD")
    with urllib.request.urlopen(request, timeout=5) as response:
        if response.status >= 400:
            raise RuntimeError(f"HTTP {response.status}")
    return normalized


def doctor(args) -> int:
    session = _require_session(args)
    checks = []

    def configuration():
        SkillValidator()
        Draft202012Validator.check_schema(mission_schema())
        Draft202012Validator.check_schema(action_schema())
        policy = load_json_config("agent_policy.json")
        if policy.get("policy_version") != 1 or not 1 <= policy.get("maximum_actions", 0) <= 10:
            raise RuntimeError("agent policy bounds are invalid")
        return "skill, safety, and agent policies valid"
    checks.append(_check("configuration", configuration))
    checks.append(_check("dashboard assets", lambda: "present" if all(
        (REPOSITORY_ROOT / "isaac_sim/browser_console" / name).is_file() for name in ("index.html", "app.js"))
        else (_ for _ in ()).throw(RuntimeError("dashboard asset missing"))))

    if args.mode != "replay":
        load_local_api_key()
        checks.append(Check("OpenAI key", "ok" if os.environ.get("OPENAI_API_KEY") else "fail",
                            "configured locally" if os.environ.get("OPENAI_API_KEY") else "OPENAI_API_KEY unavailable"))

    backend = None
    if args.mode == "remote":
        checks.append(Check("ssh executable", "ok" if shutil.which("ssh") else "fail",
                            shutil.which("ssh") or "not found"))
        def ssh_host():
            completed = subprocess.run(
                ["ssh", "-T", "-o", "BatchMode=yes", "-o", "ConnectTimeout=5", args.host, "true"],
                capture_output=True, text=True, timeout=10,
            )
            if completed.returncode:
                raise RuntimeError(f"SSH exit {completed.returncode}")
            return args.host
        checks.append(_check("SSH host", ssh_host))
        try:
            backend = _backend(args)
        except Exception as error:
            checks.append(Check("remote backend", "fail", str(error)))
    elif args.mode == "local":
        checks.append(_check("local session marker", lambda: str(
            (REPOSITORY_ROOT / "isaac_sim/_output" / session / "session.ready").resolve())
            if (REPOSITORY_ROOT / "isaac_sim/_output" / session / "session.ready").is_file()
            else (_ for _ in ()).throw(RuntimeError("session.ready is missing"))))
        try:
            backend = _backend(args)
        except Exception as error:
            checks.append(Check("local backend", "fail", str(error)))
    else:
        backend = _backend(args)
        checks.append(_check("replay backend", lambda: f"{len(backend.capture().objects)} objects grounded"))

    if backend is not None and args.mode in {"remote", "local"}:
        if args.mode == "remote":
            checks.append(_check("remote session", lambda: json.dumps(backend.request("execution_progress"), sort_keys=True)))
            checks.append(_check("robot state", lambda: "stationary" if backend.request("robot_state").get("stationary") else (_ for _ in ()).throw(RuntimeError("robot is moving"))))
        if not args.skip_capture:
            checks.append(_check("fresh RGB-D capture", lambda: f"scene {backend.capture().revision}"))
        else:
            checks.append(Check("fresh RGB-D capture", "warn", "skipped by operator"))

    if args.viewer_url:
        checks.append(_check("viewer HTTP", lambda: _viewer_check(args.viewer_url), optional=True))
    else:
        checks.append(Check("viewer HTTP", "warn", "no --viewer-url supplied"))

    if args.json:
        print(json.dumps({"mode": args.mode, "session": session, "checks": [asdict(item) for item in checks]}, indent=2))
    else:
        print(f"VGM demo doctor — mode={args.mode} session={session}")
        for item in checks:
            mark = {"ok": "✓", "warn": "!", "fail": "✗"}[item.status]
            print(f"{mark} {item.name}: {item.detail}")
    return 2 if any(item.status == "fail" for item in checks) else 0


def _stop_and_wait(controller, timeout_s=245.0) -> bool:
    """Request controller STOP and wait for its cancellation worker to settle."""
    controller.stop()
    changed = getattr(controller, "changed", None)
    if changed is None:
        return True
    with changed:
        return changed.wait_for(lambda: not getattr(controller, "stopping", False), timeout=timeout_s)


def run_dashboard(args) -> int:
    session = _require_session(args)
    viewer_url = normalize_viewer_url(args.viewer_url)
    backend = _backend(args)
    output = REPOSITORY_ROOT / "isaac_sim/_output"
    output.mkdir(parents=True, exist_ok=True)
    audit = AuditLogger(output / f"dashboard-{args.mode}-{session}.jsonl")
    controller = _controller(args, backend, audit)
    server = make_server(
        controller,
        args.port,
        session_name=session,
        viewer_url=viewer_url,
        backend_mode=args.mode,
        microphone_enabled=args.mode != "replay",
    )
    url = f"http://localhost:{server.server_port}/"
    print(f"VGM dashboard: {url}")
    print(f"Control: {args.control} · backend: {args.mode} · session: {session}")
    if args.mode == "replay":
        print("Replay mode is local, API-free, and performs no robot/ROS/SSH operation.")
    elif viewer_url:
        print(f"Live Isaac viewer embedded from: {viewer_url}")
    else:
        print("No live viewer URL configured; pass --viewer-url to embed the Isaac WebRTC viewer.")
    print("Press Ctrl+C to request STOP and close the dashboard.")
    if not args.no_open:
        try:
            webbrowser.open(url)
        except Exception:
            pass
    exit_code = 0
    previous_handlers = {}

    def interrupt(_signum, _frame):
        raise KeyboardInterrupt

    for name in ("SIGINT", "SIGTERM", "SIGHUP"):
        sig = getattr(signal, name, None)
        if sig is not None:
            previous_handlers[sig] = signal.getsignal(sig)
            signal.signal(sig, interrupt)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\nRequesting STOP before dashboard shutdown…", flush=True)
        if not _stop_and_wait(controller):
            print("STOP did not settle before the shutdown deadline.", file=sys.stderr, flush=True)
            exit_code = 2
        else:
            final_status = controller.snapshot().get("status")
            if final_status != "stopped":
                print(f"STOP was not confirmed (dashboard state: {final_status}).", file=sys.stderr, flush=True)
                exit_code = 2
    finally:
        for sig, handler in previous_handlers.items():
            signal.signal(sig, handler)
        server.server_close()
    return exit_code


def build_parser():
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest="command", required=True)

    run = sub.add_parser("run", help="launch the unified dashboard")
    run.add_argument("--mode", choices=("remote", "local", "replay"), default="remote")
    run.add_argument("--control", choices=("agent", "task"), default="agent")
    run.add_argument("--session")
    run.add_argument("--host", default="vgm-isaac-dev")
    run.add_argument("--remote-root", default="/home/ubuntu/workspace")
    run.add_argument("--viewer-url", default="")
    run.add_argument("--port", type=int, default=8766)
    run.add_argument("--no-open", action="store_true")

    replay = sub.add_parser("replay", help="launch an API-free, zero-motion local dashboard")
    replay.add_argument("--control", choices=("agent", "task"), default="agent")
    replay.add_argument("--port", type=int, default=8766)
    replay.add_argument("--no-open", action="store_true")

    diagnostic = sub.add_parser("doctor", help="run passive readiness checks; never requests motion")
    diagnostic.add_argument("--mode", choices=("remote", "local", "replay"), default="remote")
    diagnostic.add_argument("--session")
    diagnostic.add_argument("--host", default="vgm-isaac-dev")
    diagnostic.add_argument("--remote-root", default="/home/ubuntu/workspace")
    diagnostic.add_argument("--viewer-url", default="")
    diagnostic.add_argument("--skip-capture", action="store_true", help="skip passive RGB-D capture")
    diagnostic.add_argument("--json", action="store_true")
    return parser


def main(argv=None):
    parser = build_parser()
    args = parser.parse_args(argv)
    if args.command == "replay":
        args.mode = "replay"
        args.session = "replay"
        args.host = "vgm-isaac-dev"
        args.remote_root = "/home/ubuntu/workspace"
        args.viewer_url = ""
        return run_dashboard(args)
    if args.command == "doctor":
        return doctor(args)
    return run_dashboard(args)


if __name__ == "__main__":
    raise SystemExit(main())
