import io
import json
from pathlib import Path
import sys
import tempfile
import threading
import time
import unittest
import urllib.error
import urllib.request
import wave
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'ros2_ws/src/vgm_runtime'))
from vgm_runtime.audit import AuditLogger
from vgm_runtime.browser_console import BrowserController, make_server, validate_wav
from test_tasks import Backend, Model, step


def audio(seconds=.25, channels=1):
    output = io.BytesIO()
    with wave.open(output, 'wb') as wav:
        wav.setnchannels(channels)
        wav.setsampwidth(2)
        wav.setframerate(16000)
        wav.writeframes(b'\0\0' * int(seconds * 16000) * channels)
    return output.getvalue()


class BrowserTests(unittest.TestCase):
    def setUp(self):
        directory = tempfile.TemporaryDirectory()
        self.addCleanup(directory.cleanup)
        self.backend = Backend()
        self.model = Model([step('pick_and_place', 'red_cube', 'blue_target')])
        self.transcribed = []
        def transcribe(backend, path):
            self.transcribed.append(path)
            self.assertTrue(path.exists())
            return {'text': 'Move red to blue', 'confidence': .9}
        self.controller = BrowserController(self.model, self.backend,
            AuditLogger(Path(directory.name) / 'audit.jsonl'), transcribe)

    def settle(self):
        deadline = time.monotonic() + 3
        while self.controller.snapshot()['busy'] and time.monotonic() < deadline:
            time.sleep(.005)
        self.assertFalse(self.controller.snapshot()['busy'])
        return self.controller.snapshot()

    def test_audio_prepares_without_execution_and_removes_temporary_file(self):
        self.controller.prepare(audio=audio())
        result = self.settle()
        self.assertEqual(result['status'], 'awaiting_confirmation')
        self.assertEqual(result['transcript'], 'Move red to blue')
        self.assertEqual(self.backend.executed, [])
        self.assertFalse(self.transcribed[0].exists())

    def test_confirmation_only_and_replay_denied(self):
        self.controller.prepare(transcript='Move red')
        token = self.settle()['confirmation']
        with self.assertRaises(ValueError):
            self.controller.confirm('wrong')
        self.assertFalse(self.backend.executed)
        self.controller.confirm(token)
        self.assertEqual(self.settle()['status'], 'completed')
        self.assertEqual(len(self.backend.executed), 1)
        with self.assertRaises(ValueError):
            self.controller.confirm(token)

    def test_cancel_invalidates_confirmation(self):
        self.controller.prepare(transcript='Move red')
        token = self.settle()['confirmation']
        self.controller.cancel()
        with self.assertRaises(ValueError):
            self.controller.confirm(token)
        self.assertFalse(self.backend.executed)

    def test_expired_confirmation_does_not_move(self):
        self.controller.prepare(transcript='Move red')
        token = self.settle()['confirmation']
        self.controller.task.clock = lambda: time.time() + 200
        self.controller.confirm(token)
        self.assertEqual(self.settle()['code'], 'confirmation_expired')
        self.assertFalse(self.backend.executed)

    def test_stop_during_model_request_cannot_restore_pending(self):
        entered, release = threading.Event(), threading.Event()
        original = self.model.propose
        def blocked(*args):
            entered.set(); release.wait(2)
            return original(*args)
        with patch.object(self.model, 'propose', side_effect=blocked):
            self.controller.prepare(transcript='Move red')
            self.assertTrue(entered.wait(1))
            with self.assertRaises(ValueError):
                self.controller.prepare(transcript='duplicate')
            self.controller.stop()
            release.set()
            self.assertEqual(self.settle()['status'], 'stopped')
        self.assertIsNone(self.controller.task.pending)
        self.assertFalse(self.backend.executed)
        with self.assertRaises(ValueError):
            self.controller.prepare(transcript='Move red')
        self.controller.recover()
        self.assertEqual(self.settle()['status'], 'idle')
        self.assertFalse(self.backend.executed)

    def test_stop_failure_is_not_reported_as_stopped(self):
        with patch.object(self.backend, 'stop', side_effect=RuntimeError('disconnected')):
            self.controller.stop()
            result = self.settle()
        self.assertEqual(result['status'], 'faulted')
        self.assertEqual(result['stop_result']['status'], 'stop_unconfirmed')

    def test_stop_during_execution_prevents_following_skill(self):
        entered, release = threading.Event(), threading.Event()
        self.model.steps = [step('inspect', 'red_cube'), step('inspect', 'red_cube')]
        self.controller.prepare(transcript='Inspect twice')
        token = self.settle()['confirmation']
        original = self.backend.execute
        def blocked(*args):
            entered.set(); release.wait(2)
            return original(*args)
        with patch.object(self.backend, 'execute', side_effect=blocked):
            self.controller.confirm(token)
            self.assertTrue(entered.wait(1))
            self.controller.stop()
            release.set()
            self.assertEqual(self.settle()['status'], 'stopped')
        self.assertEqual(len(self.backend.executed), 1)
        self.assertGreaterEqual(self.backend.stops, 2)

    def test_invalid_audio_rejected(self):
        for value in (b'bad', audio(16), audio(channels=2), audio()[:-2], audio(0)):
            with self.assertRaises((ValueError, wave.Error)):
                validate_wav(value)

    def test_http_origin_token_host_and_body_gates(self):
        server = make_server(self.controller, port=0)
        worker = threading.Thread(target=server.serve_forever, daemon=True)
        worker.start()
        self.addCleanup(server.server_close)
        self.addCleanup(server.shutdown)
        origin = f'http://127.0.0.1:{server.server_port}'
        with urllib.request.urlopen(origin + '/api/config') as response:
            token = json.load(response)['csrf']
        def post(path, value, extra=None):
            headers = {'Origin': origin, 'X-VGM-CSRF': token, 'Content-Type': 'application/json'}
            headers.update(extra or {})
            request = urllib.request.Request(origin + path, data=json.dumps(value).encode(), headers=headers)
            try:
                with urllib.request.urlopen(request) as response:
                    return response.status
            except urllib.error.HTTPError as error:
                return error.code
        self.assertEqual(post('/api/text', {'transcript': 'move'}, {'Origin': 'https://evil.example'}), 403)
        self.assertEqual(post('/api/text', {'transcript': 'move'}, {'X-VGM-CSRF': 'bad'}), 403)
        self.assertEqual(post('/api/text', {'transcript': 'move'}, {'Host': 'evil.example'}), 403)
        self.assertEqual(post('/api/text', {'transcript': 'move', 'joint_targets': [0]}), 400)
        self.assertEqual(post('/api/text', {'transcript': 'x' * 9000}), 413)
        self.assertEqual(post('/api/confirm', {'confirmation': 'made_up'}), 400)
        self.assertEqual(post('/api/text', {'transcript': 'Move red'}), 202)
        self.assertEqual(self.settle()['status'], 'awaiting_confirmation')
        self.assertFalse(self.backend.executed)
        self.assertEqual(post('/api/cancel', {}), 202)
        self.assertFalse(self.backend.executed)


if __name__ == '__main__':
    unittest.main()
