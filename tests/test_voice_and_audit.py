"""Offline Whisper adapter and audit-log behavior tests."""

from __future__ import annotations

import json
import math
import sys
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace


REPOSITORY_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPOSITORY_ROOT / "ros2_ws" / "src" / "vgm_runtime"))

from vgm_runtime.audit import AuditLogger  # noqa: E402
from vgm_runtime.whisper_transcriber import (  # noqa: E402
    FasterWhisperTranscriber,
    WhisperTranscriptionError,
)


class FakeWhisper:
    def __init__(self, segments, language="en"):
        self.segments = segments
        self.language = language
        self.calls = []

    def transcribe(self, path, **options):
        self.calls.append((path, options))
        return iter(self.segments), SimpleNamespace(language=self.language)


class VoiceAndAuditTest(unittest.TestCase):
    def test_whisper_combines_segments_and_reports_confidence(self):
        fake = FakeWhisper(
            [
                SimpleNamespace(text=" Pick the red cube", avg_logprob=-0.1),
                SimpleNamespace(text=" and place it.", avg_logprob=-0.2),
            ]
        )
        with tempfile.TemporaryDirectory() as temporary_directory:
            path = Path(temporary_directory) / "voice.wav"
            path.write_bytes(b"RIFFtest")
            result = FasterWhisperTranscriber(
                model=fake, minimum_confidence=0.5
            ).transcribe(path)
        self.assertEqual(result.text, "Pick the red cube and place it.")
        self.assertAlmostEqual(
            result.confidence, (math.exp(-0.1) + math.exp(-0.2)) / 2.0
        )
        self.assertEqual(result.language, "en")
        self.assertTrue(fake.calls[0][1]["vad_filter"])

    def test_whisper_rejects_empty_low_confidence_and_missing_audio(self):
        with tempfile.TemporaryDirectory() as temporary_directory:
            path = Path(temporary_directory) / "voice.wav"
            path.write_bytes(b"RIFFtest")
            with self.assertRaises(WhisperTranscriptionError):
                FasterWhisperTranscriber(
                    model=FakeWhisper([SimpleNamespace(text="", avg_logprob=-0.1)])
                ).transcribe(path)
            with self.assertRaises(WhisperTranscriptionError):
                FasterWhisperTranscriber(
                    model=FakeWhisper(
                        [SimpleNamespace(text="unclear", avg_logprob=-10.0)]
                    )
                ).transcribe(path)
            with self.assertRaises(WhisperTranscriptionError):
                FasterWhisperTranscriber(model=FakeWhisper([])).transcribe(
                    Path(temporary_directory) / "missing.wav"
                )

    def test_audit_log_omits_keys_tokens_and_audio(self):
        with tempfile.TemporaryDirectory() as temporary_directory:
            path = Path(temporary_directory) / "audit.jsonl"
            AuditLogger(path).record(
                "test",
                {
                    "request_id": "req_1",
                    "api_key": "secret",
                    "usage_tokens": 123,
                    "audio_path": "/secret.wav",
                },
            )
            value = json.loads(path.read_text())
            self.assertEqual(value, {"event": "test", "request_id": "req_1"})
            self.assertEqual(path.stat().st_mode & 0o777, 0o600)


if __name__ == "__main__":
    unittest.main()
