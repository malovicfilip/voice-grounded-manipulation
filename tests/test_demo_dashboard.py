"""Unified dashboard HTTP and launcher safety checks."""

from __future__ import annotations

import json
from pathlib import Path
import sys
import tempfile
import threading
import time
import unittest
import urllib.error
import urllib.request

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "ros2_ws/src/vgm_runtime"))

from vgm_runtime.audit import AuditLogger
from vgm_runtime.browser_console import AgentBrowserController, make_server, normalize_viewer_url
from vgm_runtime.replay import ReplayAgentActionModel, ReplayBackend, ReplayMissionModel


class DashboardTests(unittest.TestCase):
    def setUp(self):
        directory = tempfile.TemporaryDirectory()
        self.addCleanup(directory.cleanup)
        self.backend = ReplayBackend()
        self.controller = AgentBrowserController(
            ReplayMissionModel(), ReplayAgentActionModel(), self.backend,
            AuditLogger(Path(directory.name) / "audit.jsonl"),
        )

    def settle(self, timeout=3):
        deadline = time.monotonic() + timeout
        while self.controller.snapshot()["busy"] and time.monotonic() < deadline:
            time.sleep(.005)
        self.assertFalse(self.controller.snapshot()["busy"])
        return self.controller.snapshot()

    def test_replay_browser_requires_confirmation_then_runs_agent(self):
        self.controller.prepare(transcript="Put the red cube on the yellow target")
        pending = self.settle()
        self.assertEqual(pending["status"], "awaiting_confirmation")
        self.assertEqual(self.backend.executed, [])
        self.controller.confirm(pending["confirmation"])
        result = self.settle()
        self.assertEqual(result["status"], "completed")
        self.assertEqual(len(self.backend.executed), 1)
        self.assertTrue(result["demo_scene"]["label"].startswith("Local replay"))

    def test_viewer_url_rejects_credentials_paths_query_and_non_http(self):
        self.assertEqual(normalize_viewer_url("http://127.0.0.1:8210/"), "http://127.0.0.1:8210/")
        for value in (
            "file:///tmp/viewer", "http://user:pass@example.test/", "http://example.test/path",
            "http://example.test/?token=secret", "javascript:alert(1)",
        ):
            with self.assertRaises(ValueError, msg=value):
                normalize_viewer_url(value)

    def test_http_config_exposes_mode_without_exposing_control_endpoints_cross_origin(self):
        server = make_server(self.controller, port=0, session_name="replay", backend_mode="replay", microphone_enabled=False)
        worker = threading.Thread(target=server.serve_forever, daemon=True)
        worker.start()
        self.addCleanup(server.server_close)
        self.addCleanup(server.shutdown)
        origin = f"http://127.0.0.1:{server.server_port}"
        with urllib.request.urlopen(origin + "/api/config") as response:
            config = json.load(response)
            csp = response.headers["Content-Security-Policy"]
        self.assertEqual(config["control_mode"], "agent")
        self.assertEqual(config["backend_mode"], "replay")
        self.assertFalse(config["microphone_enabled"])
        self.assertIn("frame-src 'none'", csp)

        request = urllib.request.Request(
            origin + "/api/text", data=json.dumps({"transcript": "move red"}).encode(),
            headers={"Origin": "https://evil.example", "X-VGM-CSRF": config["csrf"], "Content-Type": "application/json"},
        )
        with self.assertRaises(urllib.error.HTTPError) as caught:
            urllib.request.urlopen(request)
        self.assertEqual(caught.exception.code, 403)
        self.assertEqual(self.backend.executed, [])

    def test_configured_viewer_origin_is_the_only_frame_source(self):
        server = make_server(
            self.controller, port=0, session_name="replay", backend_mode="replay",
            viewer_url="https://viewer.example:8210/", microphone_enabled=False,
        )
        worker = threading.Thread(target=server.serve_forever, daemon=True)
        worker.start()
        self.addCleanup(server.server_close)
        self.addCleanup(server.shutdown)
        origin = f"http://127.0.0.1:{server.server_port}"
        with urllib.request.urlopen(origin + "/api/config") as response:
            config = json.load(response)
            csp = response.headers["Content-Security-Policy"]
        self.assertEqual(config["viewer_url"], "https://viewer.example:8210/")
        self.assertIn("frame-src https://viewer.example:8210", csp)
        self.assertNotIn("frame-src *", csp)


if __name__ == "__main__":
    unittest.main()
