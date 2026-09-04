"""Task-level behavioral acceptance: no motion before confirmation or after failure."""

import copy
from dataclasses import replace
import json
from pathlib import Path
import sys
import tempfile
import time
import unittest
import jsonschema

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "ros2_ws/src/vgm_runtime"))
from vgm_runtime.audit import AuditLogger
from vgm_runtime.tasks import OpenAITaskModel, TaskSession, task_schema
from vgm_runtime.serialization import scene_from_mapping
from vgm_runtime.types import GroundedScene, ObjectObservation
from vgm_runtime.validator import SkillValidationError, SkillValidator


def step(skill, object_id=None, target_id=None, pose_name=None, reason=None):
    return dict(skill=skill, object_id=object_id, target_id=target_id, pose_name=pose_name, reason=reason)


class Model:
    def __init__(self, steps):
        self.steps = steps
        self.calls = 0
        self.transcripts = []
    def propose(self, transcript, scene, policy):
        self.calls += 1
        self.transcripts.append(transcript)
        return dict(schema_version=1, request_id=f"task_{self.calls}", scene_revision=scene.revision,
                    steps=copy.deepcopy(self.steps))


class Backend:
    def __init__(self):
        self.executed = []
        self.stops = 0
        self.fail_at = None
        self.held = None
        self.positions = {"red_cube": (-0.1, -0.18, 0.775), "green_cube": (0.1, -0.18, 0.775)}
    def capture(self):
        now = time.time()
        return GroundedScene("0123456789abcdef", now, {
            key: ObjectObservation(key, value, .98, now, pixel_count=100)
            for key, value in self.positions.items() if key != self.held
        }, self.held)
    def execute(self, proposal, scene):
        self.executed.append(proposal)
        if self.fail_at == len(self.executed):
            return {"status": "failed", "code": "injected_failure"}
        if proposal["skill"] == "pick":
            self.held = proposal["object_id"]
        if proposal["skill"] in {"place", "pick_and_place"}:
            self.held = None
            self.positions[proposal["object_id"]] = (0.0, 0.3 if proposal["target_id"] == "blue_target" else -0.3, .775)
        return {"status": "succeeded"}
    def stop(self):
        self.stops += 1
        return {"status": "stopped"}
    def recover(self):
        return {"status": "recovered" if self.held is None else "refused"}


class TaskTests(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory()
        self.addCleanup(self.directory.cleanup)
        self.backend = Backend()
    def session(self, steps):
        return TaskSession(Model(steps), self.backend, AuditLogger(Path(self.directory.name) / "audit.jsonl"))
    def run_task(self, session):
        pending = session.prepare("operator command")
        self.assertEqual(pending["status"], "awaiting_confirmation", pending)
        return session.confirm(pending["confirmation"])
    def test_confirmation_required_and_bound_to_exact_pending_task(self):
        session = self.session([step("pick_and_place", "red_cube", "blue_target")])
        pending = session.prepare("move red")
        self.assertEqual(self.backend.executed, [])
        with self.assertRaises(ValueError):
            session.confirm("wrong_confirmation")
        result = session.confirm(pending["confirmation"])
        self.assertEqual(result["status"], "completed")
        with self.assertRaises(ValueError):
            session.confirm(pending["confirmation"])
    def test_inspect_pick_place_and_inspect_complete_in_order(self):
        session = self.session([step("inspect", "red_cube"), step("pick", "red_cube"),
                                step("place", "red_cube", "blue_target"), step("inspect", "red_cube")])
        result = self.run_task(session)
        self.assertEqual(result["status"], "completed", result)
        self.assertEqual([p["skill"] for p in self.backend.executed], ["inspect", "pick", "place", "inspect"])
        self.assertIsNone(self.backend.held)
    def test_clarification_produces_no_motion_and_replacement_can_be_confirmed(self):
        session = self.session([step("refuse", reason="Which cube should I move?")])
        self.assertEqual(session.prepare("move it")["code"], "clarification_required")
        self.assertFalse(self.backend.executed)
        session.model.steps = [step("pick_and_place", "red_cube", "blue_target")]
        self.assertEqual(self.run_task(session)["status"], "completed")
        self.assertIn("Original request: move it", session.model.transcripts[-1])
    def test_unpaired_pick_and_place_without_holding_are_refused(self):
        for steps in ([step("pick", "red_cube")], [step("place", "red_cube", "blue_target")]):
            session = self.session(steps)
            self.assertEqual(session.prepare("invalid sequence")["status"], "refused")
        self.assertFalse(self.backend.executed)
    def test_fault_prevents_next_skill_and_requires_explicit_recovery(self):
        self.backend.fail_at = 1
        session = self.session([step("inspect", "red_cube"), step("pick_and_place", "red_cube", "blue_target")])
        self.assertEqual(self.run_task(session)["status"], "faulted")
        self.assertEqual(len(self.backend.executed), 1)
        with self.assertRaises(RuntimeError):
            session.prepare("try again")
        self.assertEqual(session.recover()["status"], "recovered")
        self.assertIsNone(session.pending)
        self.assertEqual(len(self.backend.executed), 1)
    def test_stop_bypasses_model_and_invalidates_confirmation(self):
        session = self.session([step("inspect", "red_cube")])
        pending = session.prepare("inspect red")
        self.assertEqual(session.prepare("stop")["status"], "stopped")
        self.assertEqual(session.model.calls, 1)
        with self.assertRaises(ValueError):
            session.confirm(pending["confirmation"])
        self.assertFalse(self.backend.executed)
    def test_drift_after_confirmation_request_refuses_execution(self):
        session = self.session([step("pick_and_place", "red_cube", "blue_target")])
        pending = session.prepare("move red")
        self.backend.positions["red_cube"] = (-.05, -.18, .775)
        result = session.confirm(pending["confirmation"])
        self.assertEqual(result["code"], "object_moved")
        self.assertFalse(self.backend.executed)
    def test_expired_confirmation_does_not_execute(self):
        session = self.session([step("inspect", "red_cube")])
        pending = session.prepare("inspect red")
        session.clock = lambda: time.time() + 121
        self.assertEqual(session.confirm(pending["confirmation"])["code"], "confirmation_expired")
        self.assertFalse(self.backend.executed)
    def test_eight_step_bound_and_nested_control_injection(self):
        for steps in ([step("inspect", "red_cube")] * 9,
                      [{**step("inspect", "red_cube"), "debug": {"joint_targets": [0]}}]):
            self.assertEqual(self.session(steps).prepare("invalid")["status"], "refused")
        self.assertFalse(self.backend.executed)
    def test_target_occupied_by_another_cube_is_refused(self):
        self.backend.positions["green_cube"] = (0, .3, .775)
        session = self.session([step("pick_and_place", "red_cube", "blue_target")])
        self.assertEqual(session.prepare("move red")["code"], "target_occupied")
    def test_two_steps_cannot_reserve_the_same_occupied_target(self):
        session = self.session([step("pick_and_place", "red_cube", "blue_target"),
                                step("pick_and_place", "green_cube", "blue_target")])
        self.assertEqual(session.prepare("stack cubes")["code"], "target_occupied")
        self.assertFalse(self.backend.executed)
    def test_nonfinite_sensor_confidence_and_times_fail_closed(self):
        scene = self.backend.capture()
        proposal = dict(schema_version=1, request_id="bad_sensor", scene_revision=scene.revision,
                        **step("inspect", "red_cube"))
        original = scene.objects["red_cube"]
        for observation in (replace(original, confidence=float("nan")),
                            replace(original, confidence=float("inf")),
                            replace(original, observed_at_s=float("nan"))):
            corrupted = replace(scene, objects={"red_cube": observation})
            with self.assertRaises(SkillValidationError):
                SkillValidator().validate(proposal, corrupted)
    def test_task_model_request_stays_tool_free_and_coordinate_free(self):
        captured = []
        def transport(url, headers, payload, timeout):
            captured.append(payload)
            context = json.loads(payload["input"])
            return {"output_text": json.dumps({"schema_version": 1, "request_id": context["request_id"],
                    "scene_revision": context["scene_revision"], "steps": [step("inspect", "red_cube")]})}
        model = OpenAITaskModel(api_key="test-key", transport=transport)
        model.propose("inspect red", self.backend.capture(), SkillValidator().policy)
        self.assertNotIn("tools", captured[0])
        self.assertNotIn("position", captured[0]["input"])
        self.assertFalse(captured[0]["store"])
        self.assertEqual(captured[0]["text"]["format"]["schema"], task_schema())
    def test_scene_roundtrip_preserves_controller_held_state(self):
        self.backend.held = "red_cube"
        scene = self.backend.capture()
        restored = scene_from_mapping(scene.to_mapping())
        self.assertEqual(restored, scene)
        legacy = scene.to_mapping()
        del legacy["held_object_id"]
        self.assertIsNone(scene_from_mapping(legacy).held_object_id)
    def test_task_json_schema_rejects_irrelevant_inspect_parameters(self):
        value = {"schema_version": 1, "request_id": "schema_regression",
                 "scene_revision": "0123456789abcdef", "steps": [step("inspect", "red_cube")]}
        jsonschema.Draft202012Validator.check_schema(task_schema())
        jsonschema.validate(value, task_schema())
        for extra in ({"reason": "inspect without motion"}, {"target_id": "blue_target"},
                      {"pose_name": "ready"}, {"joint_positions": [0.0]}):
            bad = copy.deepcopy(value)
            bad["steps"][0].update(extra)
            with self.assertRaises(jsonschema.ValidationError):
                jsonschema.validate(bad, task_schema())
    def test_serialized_sensor_numbers_frames_and_extra_fields_are_rejected(self):
        for change in ({"confidence": float("nan")}, {"confidence": True},
                       {"frame_id": "camera"}, {"position_m": ["0", 0, 0]}):
            raw = self.backend.capture().to_mapping()
            raw["objects"][0].update(change)
            with self.assertRaises(ValueError):
                scene_from_mapping(raw)
    def test_nested_audit_secrets_are_redacted(self):
        path = Path(self.directory.name) / "nested.jsonl"
        AuditLogger(path).record("nested", {"response": [{"api_key": "secret", "text": "ok"}]})
        self.assertEqual(json.loads(path.read_text())["response"], [{"text": "ok"}])


if __name__ == "__main__":
    unittest.main()
