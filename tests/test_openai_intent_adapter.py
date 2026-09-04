"""Contract tests for the tool-free OpenAI Structured Outputs adapter."""

from __future__ import annotations

import json
import sys
import unittest
from pathlib import Path


REPOSITORY_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPOSITORY_ROOT / "ros2_ws" / "src" / "vgm_runtime"))

from vgm_runtime.openai_intent import IntentModelError, OpenAIIntentModel  # noqa: E402
from vgm_runtime.types import GroundedScene, ObjectObservation  # noqa: E402
from vgm_runtime.validator import SkillValidator  # noqa: E402


class CapturingTransport:
    def __init__(self, output_factory):
        self.output_factory = output_factory
        self.calls = []

    def __call__(self, url, headers, payload, timeout_s):
        self.calls.append((url, headers, payload, timeout_s))
        output = self.output_factory(payload)
        return {
            "output": [
                {
                    "type": "message",
                    "content": [{"type": "output_text", "text": json.dumps(output)}],
                }
            ]
        }


def scene():
    observation = ObjectObservation(
        "red_cube", (-0.1, -0.18, 0.775), 0.99, 100.0, pixel_count=100
    )
    return GroundedScene("0123456789abcdef", 100.0, {"red_cube": observation})


class OpenAIIntentAdapterTest(unittest.TestCase):
    def test_request_has_strict_schema_no_tools_and_no_coordinates(self):
        def output(payload):
            context = json.loads(payload["input"])
            return {
                "schema_version": 1,
                "request_id": context["request_id"],
                "skill": "pick",
                "object_id": "red_cube",
                "target_id": None,
                "pose_name": None,
                "reason": None,
                "scene_revision": context["scene_revision"],
            }

        transport = CapturingTransport(output)
        validator = SkillValidator(clock=lambda: 100.0)
        adapter = OpenAIIntentModel(
            api_key="test-only-not-real",
            schema=validator.schema,
            transport=transport,
            request_id_factory=lambda: "req_openai_1",
        )
        result = adapter.propose("pick the red cube", scene(), validator.policy)
        self.assertEqual(result["skill"], "pick")
        url, headers, payload, timeout_s = transport.calls[0]
        self.assertEqual(url, "https://api.openai.com/v1/responses")
        self.assertEqual(payload["model"], "gpt-5.6-terra")
        self.assertFalse(payload["store"])
        self.assertNotIn("tools", payload)
        self.assertEqual(payload["reasoning"], {"effort": "low"})
        self.assertTrue(payload["text"]["format"]["strict"])
        self.assertEqual(payload["text"]["format"]["type"], "json_schema")
        self.assertNotIn("position", payload["input"])
        self.assertNotIn("test-only-not-real", json.dumps(payload))
        self.assertTrue(headers["Authorization"].startswith("Bearer "))
        self.assertGreater(timeout_s, 0.0)

    def test_changed_request_id_fails_closed(self):
        transport = CapturingTransport(
            lambda payload: {
                "schema_version": 1,
                "request_id": "changed",
                "skill": "stop",
                "object_id": None,
                "target_id": None,
                "pose_name": None,
                "reason": "stop",
                "scene_revision": None,
            }
        )
        adapter = OpenAIIntentModel(
            api_key="test-only-not-real",
            transport=transport,
            request_id_factory=lambda: "req_expected",
        )
        with self.assertRaises(IntentModelError):
            adapter.propose("stop", scene(), SkillValidator().policy)

    def test_missing_key_and_missing_output_fail_closed(self):
        adapter = OpenAIIntentModel(api_key="", transport=lambda *args: {})
        with self.assertRaises(IntentModelError):
            adapter.propose("stop", scene(), SkillValidator().policy)
        adapter = OpenAIIntentModel(
            api_key="test-only-not-real", transport=lambda *args: {}
        )
        with self.assertRaises(IntentModelError):
            adapter.propose("stop", scene(), SkillValidator().policy)


if __name__ == "__main__":
    unittest.main()
