"""Loopback-only unified robot dashboard with explicit operator confirmation."""

from __future__ import annotations

import argparse
import io
import json
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
import secrets
import tempfile
import threading
from urllib.parse import urlparse
import wave

from .agent import AgentSession, OpenAIAgentActionModel, OpenAIMissionModel
from .audit import AuditLogger
from .openai_intent import load_local_api_key
from .ssh_backend import SSHBackend
from .task_cli import upload_and_transcribe
from .tasks import OpenAITaskModel, TaskSession

MAX_AUDIO_BYTES = 3_000_000
ASSETS = Path(__file__).resolve().parents[4] / "isaac_sim/browser_console"


def validate_wav(data):
    if not 44 <= len(data) <= MAX_AUDIO_BYTES:
        raise ValueError("Audio must be a WAV smaller than 3 MB")
    with wave.open(io.BytesIO(data), "rb") as audio:
        frames, rate = audio.getnframes(), audio.getframerate()
        if (audio.getnchannels() != 1 or audio.getsampwidth() != 2
                or audio.getcomptype() != "NONE" or not 8000 <= rate <= 96000
                or not 0 < frames <= 15 * rate):
            raise ValueError("Use mono 16-bit PCM WAV, up to 15 seconds")
        if len(audio.readframes(frames)) != frames * 2:
            raise ValueError("Truncated WAV")


def normalize_viewer_url(value: str | None) -> str:
    if not value:
        return ""
    parsed = urlparse(value)
    if parsed.scheme not in {"http", "https"} or not parsed.hostname or parsed.username or parsed.password:
        raise ValueError("viewer URL must be an http(s) URL without embedded credentials")
    if parsed.path not in {"", "/"} or parsed.query or parsed.fragment:
        raise ValueError("viewer URL must contain only scheme, host, optional port, and / path")
    # Rebuild from parsed components instead of echoing untrusted input.
    host = f"[{parsed.hostname}]" if ":" in parsed.hostname else parsed.hostname
    port = f":{parsed.port}" if parsed.port else ""
    return f"{parsed.scheme}://{host}{port}/"


class CancellableBackend:
    def __init__(self, backend):
        self.backend = backend
        self.cancelled = threading.Event()

    def __getattr__(self, name):
        return getattr(self.backend, name)

    def capture(self):
        if self.cancelled.is_set():
            raise RuntimeError("Operator stop requested")
        return self.backend.capture()

    def execute(self, proposal, scene):
        if self.cancelled.is_set():
            raise RuntimeError("Operator stop requested")
        return self.backend.execute(proposal, scene)


def _with_visual_state(view: dict, backend: CancellableBackend, *, busy: bool) -> dict:
    result = {**view, "busy": busy}
    visual = getattr(backend.backend, "visual_state", None)
    if callable(visual):
        try:
            result["demo_scene"] = visual()
        except Exception:
            pass
    return result


class BrowserController:
    """Original operator-reviewed planned-task workflow."""

    control_mode = "task"

    def __init__(self, model, backend, audit, transcribe=upload_and_transcribe):
        self.backend = CancellableBackend(backend)
        self.task = TaskSession(model, self.backend, audit)
        self.transcribe = transcribe
        self.lock = threading.RLock()
        self.changed = threading.Condition(self.lock)
        self.workers = 0
        self.generation = 0
        self.stopping = False
        self.view = {"status": "idle", "transcript": "", "steps": [], "timeline": []}

    def snapshot(self):
        with self.lock:
            return _with_visual_state(self.view, self.backend, busy=self.workers > 0)

    def _launch(self, phase, function):
        if self.workers:
            raise ValueError("An operation is already in progress")
        self.workers += 1
        self.generation += 1
        generation = self.generation
        self.view = {**self.view, "status": phase, "confirmation": None}

        def work():
            try:
                result = function()
            except Exception as error:
                result = {"status": "error", "message": str(error)}
            with self.lock:
                self.workers -= 1
                if generation == self.generation:
                    self.view = {**self.view, **result}
                else:
                    self.task.pending = None
                    self.task.state = "faulted" if self.view["status"] == "faulted" else "stopped"
                self.changed.notify_all()
        threading.Thread(target=work, daemon=True).start()

    def prepare(self, *, audio=None, transcript=None):
        from .stop_intent import is_stop_request
        if is_stop_request(transcript):
            self.stop()
            return
        if audio is not None:
            validate_wav(audio)
        elif not isinstance(transcript, str) or not 0 < len(transcript.strip()) <= 2000:
            raise ValueError("Enter a command of 1–2000 characters")
        with self.lock:
            if self.workers or self.task.state not in {"idle", "awaiting_confirmation", "completed", "refused"}:
                raise ValueError("Wait for the current operation, or recover after a stop/fault")
            self.task.pending = None
            self.view = {"status": "preparing", "steps": [], "timeline": [], "transcript": transcript or ""}

            def prepare():
                text = transcript
                speech = None
                if audio is not None:
                    with tempfile.TemporaryDirectory(prefix="vgm-browser-audio-") as directory:
                        path = Path(directory) / "command.wav"
                        path.write_bytes(audio)
                        speech = self.transcribe(self.backend.backend, path)
                        text = speech["text"]
                if is_stop_request(text):
                    self.stop()
                    return {"status": "stopping"}
                if self.backend.cancelled.is_set():
                    return {"status": "stopped"}
                result = self.task.prepare(text)
                return {**result, "transcript": text, "speech": speech}
            self._launch("preparing", prepare)

    def confirm(self, token):
        with self.lock:
            pending = self.task.pending
            if self.workers or not pending or self.task.state != "awaiting_confirmation" or token != pending.confirmation:
                raise ValueError("Confirmation does not match the pending task")
            self._launch("running", lambda: self.task.confirm(token))

    def cancel(self):
        with self.lock:
            if self.workers:
                raise ValueError("Use Stop while an operation is running")
            if self.task.state == "awaiting_confirmation":
                self.task.pending = None
                self.task.state = "idle"
                self.task._clarification = None
                self.view = {"status": "cancelled", "transcript": "", "steps": [], "timeline": []}
                self.task.audit.record("browser_cancel", {"status": "cancelled_before_motion"})

    def stop(self):
        with self.lock:
            if self.stopping:
                return
            self.stopping = True
            had_worker = self.workers > 0
            self.generation += 1
            self.workers += 1
            self.backend.cancelled.set()
            self.task._stop.set()
            self.task._clarification = None
            self.task.pending = None
            self.view = {**self.view, "status": "stopping", "confirmation": None}

            def stop():
                try:
                    result = self.backend.backend.stop()
                    if had_worker:
                        with self.changed:
                            if not self.changed.wait_for(lambda: self.workers == 1, timeout=240):
                                raise RuntimeError("In-flight operation did not exit after stop")
                        result = self.backend.backend.stop()
                    state = "stopped" if result.get("status") == "stopped" else "faulted"
                except Exception as error:
                    result, state = {"message": str(error), "status": "stop_unconfirmed"}, "faulted"
                with self.lock:
                    self.task.state = state
                    self.task.pending = None
                    self.view = {**self.view, "status": state, "stop_result": result}
                    self.workers -= 1
                    self.stopping = False
                    self.task.audit.record("browser_stop", result)
                    self.changed.notify_all()
            threading.Thread(target=stop, daemon=True).start()

    def recover(self):
        with self.lock:
            if self.task.state not in {"stopped", "faulted"}:
                raise ValueError("Recovery requires a stopped or faulted session")

            def recover():
                result = self.task.recover()
                if result.get("status") == "recovered":
                    self.backend.cancelled.clear()
                    return {"status": "idle", "steps": [], "timeline": [], "recovery": result}
                return {"status": "faulted", "recovery": result}
            self._launch("recovering", recover)


class AgentBrowserController:
    """Browser wrapper for the bounded closed-loop :class:`AgentSession`."""

    control_mode = "agent"

    def __init__(self, mission_model, action_model, backend, audit, transcribe=upload_and_transcribe):
        self.backend = CancellableBackend(backend)
        self.transcribe = transcribe
        self.lock = threading.RLock()
        self.changed = threading.Condition(self.lock)
        self.workers = 0
        self.generation = 0
        self.stopping = False
        self.view = {"status": "idle", "transcript": "", "mission": None, "timeline": []}
        self.agent = AgentSession(
            mission_model, action_model, self.backend, audit, event_callback=self._agent_event
        )

    def snapshot(self):
        with self.lock:
            return _with_visual_state(self.view, self.backend, busy=self.workers > 0)

    def _agent_event(self, event, payload):
        with self.lock:
            if event == "agent_observation":
                self.view = {
                    **self.view,
                    "semantic_state": {
                        "action_index": payload.get("action_index"),
                        "remaining_actions": payload.get("remaining_actions"),
                        "scene": payload.get("scene"),
                        "last_result": payload.get("last_result"),
                        "completion_status": payload.get("completion_status"),
                        "allowed_capabilities": payload.get("allowed_capabilities"),
                    },
                }
            elif event == "agent_decision":
                timeline = list(self.view.get("timeline") or [])
                timeline.append({
                    "index": payload.get("index"), "capability": payload.get("capability"),
                    "object_id": payload.get("object_id"), "target_id": payload.get("target_id"),
                    "pose_name": payload.get("pose_name"), "reason": payload.get("reason"),
                    "status": "selected",
                })
                self.view = {**self.view, "timeline": timeline}
            elif event == "agent_action_result":
                timeline = list(self.view.get("timeline") or [])
                index = payload.get("index")
                for item in reversed(timeline):
                    if item.get("index") == index:
                        item["status"] = (payload.get("result") or {}).get("status", "unknown")
                        break
                self.view = {**self.view, "timeline": timeline, "last_result": payload.get("result")}
            self.changed.notify_all()

    def _launch(self, phase, function):
        if self.workers:
            raise ValueError("An operation is already in progress")
        self.workers += 1
        self.generation += 1
        generation = self.generation
        self.view = {**self.view, "status": phase, "confirmation": None}

        def work():
            try:
                result = function()
            except Exception as error:
                result = {"status": "error", "message": str(error)}
            with self.lock:
                self.workers -= 1
                if generation == self.generation:
                    self.view = {**self.view, **result}
                else:
                    self.agent.pending = None
                    self.agent.state = "faulted" if self.view["status"] == "faulted" else "stopped"
                self.changed.notify_all()
        threading.Thread(target=work, daemon=True).start()

    def prepare(self, *, audio=None, transcript=None):
        from .stop_intent import is_stop_request
        if is_stop_request(transcript):
            self.stop()
            return
        if audio is not None:
            validate_wav(audio)
        elif not isinstance(transcript, str) or not 0 < len(transcript.strip()) <= 2000:
            raise ValueError("Enter a command of 1–2000 characters")
        with self.lock:
            if self.workers or self.agent.state not in {"idle", "completed", "refused", "needs_human"}:
                raise ValueError("Wait for the current operation, or recover after a stop/fault")
            self.agent.pending = None
            self.view = {"status": "preparing", "transcript": transcript or "", "mission": None, "timeline": []}

            def prepare():
                text = transcript
                speech = None
                if audio is not None:
                    with tempfile.TemporaryDirectory(prefix="vgm-browser-audio-") as directory:
                        path = Path(directory) / "command.wav"
                        path.write_bytes(audio)
                        speech = self.transcribe(self.backend.backend, path)
                        text = speech["text"]
                if is_stop_request(text):
                    self.stop()
                    return {"status": "stopping"}
                if self.backend.cancelled.is_set():
                    return {"status": "stopped"}
                result = self.agent.prepare(text)
                return {**result, "transcript": text, "speech": speech}
            self._launch("preparing", prepare)

    def confirm(self, token):
        with self.lock:
            pending = self.agent.pending
            if self.workers or not pending or self.agent.state != "awaiting_confirmation" or token != pending.confirmation:
                raise ValueError("Confirmation does not match the pending mission")
            self._launch("running", lambda: self.agent.confirm(token))

    def cancel(self):
        with self.lock:
            if self.workers:
                raise ValueError("Use Stop while an operation is running")
            if self.agent.state != "awaiting_confirmation":
                raise ValueError("There is no pending mission to cancel")
            self.agent.cancel()
            self.view = {"status": "cancelled", "transcript": "", "mission": None, "timeline": []}

    def stop(self):
        with self.lock:
            if self.stopping:
                return
            self.stopping = True
            had_worker = self.workers > 0
            self.generation += 1
            self.workers += 1
            self.backend.cancelled.set()
            self.agent._stop.set()
            self.agent.pending = None
            self.view = {**self.view, "status": "stopping", "confirmation": None}

            def stop():
                try:
                    result = self.backend.backend.stop()
                    if had_worker:
                        with self.changed:
                            if not self.changed.wait_for(lambda: self.workers == 1, timeout=240):
                                raise RuntimeError("In-flight operation did not exit after stop")
                        result = self.backend.backend.stop()
                    state = "stopped" if result.get("status") == "stopped" else "faulted"
                except Exception as error:
                    result, state = {"message": str(error), "status": "stop_unconfirmed"}, "faulted"
                with self.lock:
                    self.agent.state = state
                    self.agent.pending = None
                    self.view = {**self.view, "status": state, "stop_result": result}
                    self.workers -= 1
                    self.stopping = False
                    self.agent.audit.record("browser_agent_stop", result)
                    self.changed.notify_all()
            threading.Thread(target=stop, daemon=True).start()

    def recover(self):
        with self.lock:
            if self.agent.state not in {"stopped", "faulted"}:
                raise ValueError("Recovery requires a stopped or faulted session")

            def recover():
                result = self.agent.recover()
                if result.get("status") == "recovered":
                    self.backend.cancelled.clear()
                    return {"status": "idle", "mission": None, "timeline": [], "recovery": result}
                return {"status": "faulted", "recovery": result}
            self._launch("recovering", recover)


def make_server(
    controller,
    port=8766,
    assets=ASSETS,
    session_name="",
    *,
    viewer_url="",
    backend_mode="remote",
    microphone_enabled=True,
):
    csrf = secrets.token_urlsafe(32)
    viewer_url = normalize_viewer_url(viewer_url)
    viewer_origin = ""
    if viewer_url:
        parsed = urlparse(viewer_url)
        host = f"[{parsed.hostname}]" if parsed.hostname and ":" in parsed.hostname else parsed.hostname
        viewer_origin = f"{parsed.scheme}://{host}" + (f":{parsed.port}" if parsed.port else "")

    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *_):
            pass

        def _allowed_host(self):
            return self.headers.get("Host") in {f"localhost:{self.server.server_port}", f"127.0.0.1:{self.server.server_port}"}

        def _respond(self, value, code=200, content_type="application/json"):
            body = json.dumps(value, allow_nan=False).encode() if content_type == "application/json" else value
            self.send_response(code)
            self.send_header("Content-Type", content_type)
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Cache-Control", "no-store")
            self.send_header("X-Content-Type-Options", "nosniff")
            self.send_header("X-Frame-Options", "DENY")
            self.send_header("Referrer-Policy", "no-referrer")
            frame_src = viewer_origin if viewer_origin else "'none'"
            self.send_header(
                "Content-Security-Policy",
                "default-src 'self'; script-src 'self'; style-src 'self' 'unsafe-inline'; "
                f"connect-src 'self'; frame-src {frame_src}; frame-ancestors 'none'; base-uri 'none'",
            )
            self.end_headers()
            try:
                self.wfile.write(body)
            except (BrokenPipeError, ConnectionResetError):
                pass

        def do_GET(self):
            if not self._allowed_host() or self.headers.get("Sec-Fetch-Site") == "cross-site":
                return self._respond({"error": "Local access only"}, 403)
            if self.path == "/api/config":
                return self._respond({
                    "csrf": csrf,
                    "session": session_name,
                    "control_mode": getattr(controller, "control_mode", "task"),
                    "backend_mode": backend_mode,
                    "viewer_url": viewer_url,
                    "microphone_enabled": bool(microphone_enabled),
                })
            if self.path == "/api/health":
                return self._respond({"status": "ok", "control_mode": getattr(controller, "control_mode", "task"), "backend_mode": backend_mode})
            if self.path == "/api/state":
                if not secrets.compare_digest(self.headers.get("X-VGM-CSRF", ""), csrf):
                    return self._respond({"error": "Invalid request token"}, 403)
                return self._respond(controller.snapshot())
            files = {"/": ("index.html", "text/html; charset=utf-8"), "/app.js": ("app.js", "text/javascript")}
            if self.path not in files:
                return self._respond({"error": "Not found"}, 404)
            name, mime = files[self.path]
            self._respond((assets / name).read_bytes(), content_type=mime)

        def do_POST(self):
            if (not self._allowed_host()
                    or self.headers.get("Origin") != "http://" + self.headers.get("Host", "")
                    or not secrets.compare_digest(self.headers.get("X-VGM-CSRF", ""), csrf)):
                return self._respond({"error": "Invalid local origin or request token"}, 403)
            try:
                if self.headers.get("Transfer-Encoding"):
                    raise ValueError("Chunked requests are not supported")
                length = int(self.headers.get("Content-Length", "-1"))
                limit = MAX_AUDIO_BYTES if self.path == "/api/audio" else 8192
                if not 0 <= length <= limit:
                    return self._respond({"error": "Request too large or missing length"}, 413)
                self.connection.settimeout(10)
                data = self.rfile.read(length)
                if len(data) != length:
                    raise ValueError("Incomplete request")
                if self.path == "/api/audio":
                    if not microphone_enabled:
                        raise ValueError("Microphone transcription is unavailable in this demo mode")
                    if self.headers.get_content_type() != "audio/wav":
                        raise ValueError("Expected audio/wav")
                    controller.prepare(audio=data)
                else:
                    if self.headers.get_content_type() != "application/json":
                        raise ValueError("Expected application/json")
                    value = json.loads(data)
                    fields = {"/api/text": {"transcript"}, "/api/confirm": {"confirmation"},
                              "/api/cancel": set(), "/api/stop": set(), "/api/recover": set()}
                    if self.path not in fields or not isinstance(value, dict) or set(value) != fields[self.path]:
                        raise ValueError("Invalid operation or fields")
                    if self.path == "/api/text":
                        controller.prepare(transcript=value["transcript"])
                    elif self.path == "/api/confirm":
                        controller.confirm(value["confirmation"])
                    elif self.path == "/api/cancel":
                        controller.cancel()
                    elif self.path == "/api/stop":
                        controller.stop()
                    else:
                        controller.recover()
                self._respond(controller.snapshot(), 202)
            except Exception as error:
                self._respond({"error": str(error)}, 400)

    return ThreadingHTTPServer(("127.0.0.1", port), Handler)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--session", required=True)
    parser.add_argument("--host", default="vgm-isaac-dev")
    parser.add_argument("--port", type=int, default=8766)
    parser.add_argument("--viewer-url", default="")
    parser.add_argument("--control", choices=("agent", "task"), default="agent")
    args = parser.parse_args()
    backend = SSHBackend(args.session, host=args.host)
    load_local_api_key()
    audit = AuditLogger(ASSETS.parent / "_output" / "browser-audit.jsonl")
    if args.control == "agent":
        controller = AgentBrowserController(OpenAIMissionModel(), OpenAIAgentActionModel(), backend, audit)
    else:
        controller = BrowserController(OpenAITaskModel(), backend, audit)
    server = make_server(controller, args.port, session_name=args.session, viewer_url=args.viewer_url, backend_mode="remote")
    print(f"VGM dashboard: http://localhost:{server.server_port}/ (session {args.session}, {args.control})", flush=True)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        controller.stop()
    finally:
        server.server_close()


if __name__ == "__main__":
    main()
