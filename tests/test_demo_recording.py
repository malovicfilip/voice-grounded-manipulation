import importlib.util
import json
from pathlib import Path
import tempfile
import unittest

import numpy as np

spec = importlib.util.spec_from_file_location("demo_recording", Path(__file__).resolve().parents[1] / "isaac_sim/scripts/demo_recording.py")
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


class RecordingTests(unittest.TestCase):
    def test_opt_in_timing_and_stop(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            recorder = module.DemoRecording(root)
            frame = np.zeros((8, 8, 4), dtype=np.uint8)
            recorder.update(frame, 0)
            self.assertFalse((root / "recording").exists())
            (root / "recording.start").touch()
            for now in (1, 1.01, 1.2):
                recorder.update(frame, now)
            (root / "recording.stop").touch()
            recorder.update(frame, 2)
            value = json.loads((root / "recording/recording.json").read_text())
            self.assertEqual(len(value["frames"]), 2)
            self.assertEqual(value["duration_s"], 1)
            self.assertTrue((root / "recording.done").exists())

    def test_duration_limit(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            (root / "recording.start").touch()
            recorder = module.DemoRecording(root, max_seconds=1)
            recorder.update(np.ones((8, 8, 3)), 0)
            recorder.update(None, 1)
            self.assertTrue(recorder.finished)

    def test_byte_limit_and_no_restart(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            (root / "recording.start").touch()
            recorder = module.DemoRecording(root, max_bytes=1)
            recorder.update(np.zeros((8, 8, 3), dtype=np.uint8), 0)
            recorder.update(None, 1)
            recorder.update(np.ones((8, 8, 3)), 2)
            self.assertEqual(len(recorder.frames), 1)
            value = json.loads((root / "recording/recording.json").read_text())
            self.assertEqual(value["status"], "limit")

    def test_refuses_existing_frame_directory(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            (root / "recording.start").touch()
            (root / "recording").mkdir()
            with self.assertRaises(FileExistsError):
                module.DemoRecording(root).update(None, 0)


if __name__ == "__main__":
    unittest.main()
