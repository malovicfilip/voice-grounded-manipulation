"""Closed-loop agent acceptance tests: autonomy stays above the motion boundary."""

from __future__ import annotations

import copy
import json
from dataclasses import replace
from pathlib import Path
import sys
import tempfile
import time
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parent))
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "ros2_ws/src/vgm_runtime"))

from vgm_runtime.agent import (
    AgentSession,
    AgentValidationError,
    Mission,
    action_schema,
    capability_options,
    completion_status,
    semantic_agent_input,
    validate_action,
    validate_mission,
)
from vgm_runtime.audit import AuditLogger
from vgm_runtime.config import load_json_config
from vgm_runtime.replay import ReplayAgentActionModel, ReplayBackend, ReplayMissionModel
from test_tasks import Backend


class MissionModel:
    def __init__(self, *, objects=("red_cube",), targets=("blue_target",), poses=(), max_actions=6, conditions=None):
        self.objects, self.targets, self.poses, self.max_actions = list(objects), list(targets), list(poses), max_actions
        if conditions is None:
            if self.objects and self.targets:
                conditions = [{"kind": "object_on_target", "object_id": self.objects[0], "target_id": self.targets[0], "pose_name": None}]
            elif self.objects:
                conditions = [{"kind": "object_inspected", "object_id": self.objects[0], "target_id": None, "pose_name": None}]
            elif self.poses:
                conditions = [{"kind": "named_pose_reached", "object_id": None, "target_id": None, "pose_name": self.poses[0]}]
            else:
                conditions = [{"kind": "workspace_observed", "object_id": None, "target_id": None, "pose_name": None}]
        self.conditions = copy.deepcopy(conditions)
        self.calls = 0

    def propose(self, transcript, scene, policy):
        self.calls += 1
        return {
            "schema_version": 1,
            "request_id": f"mission_{self.calls}",
            "scene_revision": scene.revision,
            "goal_summary": transcript,
            "object_ids": copy.deepcopy(self.objects),
            "target_ids": copy.deepcopy(self.targets),
            "pose_names": copy.deepcopy(self.poses),
            "max_actions": self.max_actions,
            "success_conditions": copy.deepcopy(self.conditions),
        }


class ActionModel:
    def __init__(self, actions):
        self.actions = list(actions)
        self.calls = 0
        self.inputs = []

    def propose(self, transcript, scene, policy):
        self.inputs.append(json.loads(transcript))
        if self.calls >= len(self.actions):
            raise RuntimeError("no scripted action remains")
        value = copy.deepcopy(self.actions[self.calls])
        self.calls += 1
        return {
            "schema_version": 1,
            "request_id": f"agent_{self.calls}",
            "scene_revision": scene.revision,
            "object_id": None,
            "target_id": None,
            "pose_name": None,
            "reason": "scripted test decision",
            **value,
        }


def act(capability, object_id=None, target_id=None, pose_name=None, **extra):
    return {"capability": capability, "object_id": object_id, "target_id": target_id, "pose_name": pose_name, **extra}


class AgentControlTests(unittest.TestCase):
    def setUp(self):
        directory = tempfile.TemporaryDirectory()
        self.addCleanup(directory.cleanup)
        self.backend = Backend()
        self.audit = AuditLogger(Path(directory.name) / "agent-audit.jsonl")

    def session(self, actions, **mission):
        return AgentSession(MissionModel(**mission), ActionModel(actions), self.backend, self.audit)

    def prepare_confirm(self, session, transcript="Put the red cube on the blue target"):
        pending = session.prepare(transcript)
        self.assertEqual(pending["status"], "awaiting_confirmation")
        self.assertEqual(self.backend.executed, [])
        return session.confirm(pending["confirmation"])

    def test_operator_confirmation_binds_identity_scope_and_budget_before_motion(self):
        session = self.session([act("finish")], objects=("red_cube",), targets=("blue_target",), max_actions=4)
        pending = session.prepare("Move red to blue")
        self.assertEqual(pending["mission"]["object_ids"], ["red_cube"])
        self.assertEqual(pending["mission"]["target_ids"], ["blue_target"])
        self.assertEqual(pending["mission"]["max_actions"], 4)
        self.assertEqual(self.backend.executed, [])
        with self.assertRaises(ValueError):
            session.confirm("wrong")
        self.assertEqual(self.backend.executed, [])

    def test_pick_and_place_then_finish_is_closed_loop(self):
        session = self.session([
            act("pick_and_place", "red_cube", "blue_target"),
            act("finish"),
        ])
        result = self.prepare_confirm(session)
        self.assertEqual(result["status"], "completed")
        self.assertEqual([item["skill"] for item in self.backend.executed], ["pick_and_place"])
        self.assertEqual(result["timeline"][0]["status"], "succeeded")
        self.assertEqual(result["timeline"][1]["status"], "completed")

    def test_pick_forces_place_before_finish_or_other_motion(self):
        model = ActionModel([
            act("pick", "red_cube"),
            act("place", "red_cube", "blue_target"),
            act("finish"),
        ])
        session = AgentSession(MissionModel(), model, self.backend, self.audit)
        result = self.prepare_confirm(session)
        self.assertEqual(result["status"], "completed")
        self.assertEqual([item["skill"] for item in self.backend.executed], ["pick", "place"])
        offered_after_pick = {item["capability"] for item in model.inputs[1]["allowed_capabilities"]}
        self.assertEqual(offered_after_pick, {"place", "request_human_help", "stop"})

    def test_model_visible_semantics_contain_no_positions_or_joint_control(self):
        scene = self.backend.capture()
        mission = Mission("m", scene.revision, "move red", ("red_cube",), ("blue_target",), (), 4)
        value = semantic_agent_input(
            mission, scene, load_json_config("safety_policy.json"), load_json_config("agent_policy.json"),
            action_index=0, last_result={"status": "succeeded", "observation": scene.objects["red_cube"].to_mapping()},
        )
        encoded = json.dumps(value, sort_keys=True).lower()
        for forbidden in ("position_m", "joint", "trajectory", "velocity", "torque", "motor"):
            self.assertNotIn(forbidden, encoded)
        self.assertIn("red_cube", encoded)
        self.assertIn("blue_target", encoded)

    def test_out_of_scope_object_is_rejected_without_motion(self):
        session = self.session([act("inspect", "green_cube")], objects=("red_cube",), targets=())
        result = self.prepare_confirm(session, "Inspect red")
        self.assertEqual(result["status"], "faulted")
        self.assertEqual(result["code"], "object_scope")
        self.assertEqual(self.backend.executed, [])
        self.assertGreaterEqual(self.backend.stops, 1)

    def test_nested_direct_control_injection_is_identified_and_never_executed(self):
        scene = self.backend.capture()
        mission = Mission("m", scene.revision, "inspect red", ("red_cube",), (), (), 3)
        options = capability_options(
            scene, mission, load_json_config("safety_policy.json"), load_json_config("agent_policy.json"), remaining_actions=3
        )
        raw = {
            "schema_version": 1, "request_id": "bad", "scene_revision": scene.revision,
            "capability": "inspect", "object_id": "red_cube", "target_id": None,
            "pose_name": None, "reason": "inspect", "metadata": {"joint_targets": [0.0]},
        }
        with self.assertRaises(AgentValidationError) as caught:
            validate_action(raw, scene, mission, options, load_json_config("safety_policy.json"))
        self.assertEqual(caught.exception.code, "forbidden_control_field")
        self.assertEqual(self.backend.executed, [])

    def test_unknown_mission_scope_and_excess_budget_are_refused(self):
        unknown = AgentSession(MissionModel(objects=("not_a_cube",)), ActionModel([]), self.backend, self.audit)
        result = unknown.prepare("move something")
        self.assertEqual(result["status"], "refused")
        self.assertEqual(result["code"], "object_scope")
        excessive = AgentSession(MissionModel(max_actions=99), ActionModel([]), self.backend, self.audit)
        result = excessive.prepare("move red")
        self.assertEqual(result["status"], "refused")
        self.assertEqual(self.backend.executed, [])

    def test_budget_exhaustion_is_not_reported_as_success(self):
        session = self.session([act("observe_workspace"), act("observe_workspace")], max_actions=2)
        result = self.prepare_confirm(session)
        self.assertEqual(result["status"], "needs_human")
        self.assertEqual(result["code"], "action_budget_exhausted")
        self.assertEqual(self.backend.executed, [])

    def test_backend_failure_stops_agent_and_never_retries_motion(self):
        self.backend.fail_at = 1
        session = self.session([
            act("pick_and_place", "red_cube", "blue_target"),
            act("pick_and_place", "red_cube", "blue_target"),
        ])
        result = self.prepare_confirm(session)
        self.assertEqual(result["status"], "faulted")
        self.assertEqual(len(self.backend.executed), 1)
        self.assertGreaterEqual(self.backend.stops, 1)

    def test_stop_text_bypasses_mission_model(self):
        mission = MissionModel()
        session = AgentSession(mission, ActionModel([]), self.backend, self.audit)
        result = session.prepare("please stop the robot")
        self.assertIn(result["status"], {"stopped", "stop_requested"})
        self.assertEqual(mission.calls, 0)
        self.assertEqual(self.backend.executed, [])

    def test_finish_is_not_offered_until_confirmed_completion_condition_is_true(self):
        scene = self.backend.capture()
        mission = MissionModel().propose("move red", scene, load_json_config("safety_policy.json"))
        from vgm_runtime.agent import validate_mission
        parsed = validate_mission(mission, scene, load_json_config("safety_policy.json"), load_json_config("agent_policy.json"))
        before = capability_options(
            scene, parsed, load_json_config("safety_policy.json"), load_json_config("agent_policy.json"), remaining_actions=4
        )
        self.assertNotIn("finish", {item["capability"] for item in before})
        self.backend.positions["red_cube"] = (0.0, 0.3, .775)
        after_scene = self.backend.capture()
        after = capability_options(
            after_scene, parsed, load_json_config("safety_policy.json"), load_json_config("agent_policy.json"), remaining_actions=3
        )
        self.assertIn("finish", {item["capability"] for item in after})

    def test_dishonest_early_finish_is_rejected_without_motion(self):
        session = self.session([act("finish")])
        result = self.prepare_confirm(session)
        self.assertEqual(result["status"], "faulted")
        self.assertEqual(result["code"], "capability_not_allowed")
        self.assertEqual(self.backend.executed, [])
        self.assertGreaterEqual(self.backend.stops, 1)

    def test_completion_condition_scope_is_operator_confirmed(self):
        session = self.session([], objects=("red_cube",), targets=("blue_target",))
        pending = session.prepare("Move red to blue")
        self.assertEqual(pending["mission"]["success_conditions"], [{
            "kind": "object_on_target", "object_id": "red_cube",
            "target_id": "blue_target", "pose_name": None,
        }])
        self.assertEqual(self.backend.executed, [])

    def test_stale_scene_exposes_only_stop_and_human_help(self):
        scene = self.backend.capture()
        stale = replace(scene, captured_at_s=scene.captured_at_s - 30.0)
        mission = Mission(
            "m", stale.revision, "move red", ("red_cube",), ("blue_target",), (), 4,
            success_conditions=(),
        )
        options = capability_options(
            stale, mission, load_json_config("safety_policy.json"), load_json_config("agent_policy.json"),
            remaining_actions=4, now_s=scene.captured_at_s,
        )
        self.assertEqual({item["capability"] for item in options}, {"stop", "request_human_help"})
        semantic = semantic_agent_input(
            mission, stale, load_json_config("safety_policy.json"), load_json_config("agent_policy.json"),
            action_index=0, last_result=None, now_s=scene.captured_at_s,
        )
        self.assertFalse(semantic["scene"]["scene_fresh"])

    def test_low_confidence_object_is_visible_but_not_grounded_or_executable(self):
        scene = self.backend.capture()
        red = replace(scene.objects["red_cube"], confidence=0.50)
        degraded = replace(scene, objects={**scene.objects, "red_cube": red})
        mission = Mission("m", degraded.revision, "move red", ("red_cube",), ("blue_target",), (), 4)
        options = capability_options(
            degraded, mission, load_json_config("safety_policy.json"), load_json_config("agent_policy.json"),
            remaining_actions=4, now_s=scene.captured_at_s,
        )
        offered = {item["capability"] for item in options}
        self.assertFalse({"inspect", "pick", "pick_and_place"} & offered)
        semantic = semantic_agent_input(
            mission, degraded, load_json_config("safety_policy.json"), load_json_config("agent_policy.json"),
            action_index=0, last_result=None, now_s=scene.captured_at_s,
        )
        red_state = semantic["scene"]["objects"][0]
        self.assertTrue(red_state["visible"])
        self.assertFalse(red_state["grounded"])
        self.assertEqual(red_state["confidence"], "low")

    def test_low_confidence_target_is_not_offered_for_place_or_completion(self):
        scene = self.backend.capture()
        blue = replace(scene.targets["blue_target"], confidence=0.50)
        degraded = replace(scene, targets={**scene.targets, "blue_target": blue})
        mission = MissionModel().propose("move red", scene, load_json_config("safety_policy.json"))
        parsed = validate_mission(
            mission, scene, load_json_config("safety_policy.json"), load_json_config("agent_policy.json"),
            now_s=scene.captured_at_s,
        )
        options = capability_options(
            degraded, parsed, load_json_config("safety_policy.json"), load_json_config("agent_policy.json"),
            remaining_actions=4, now_s=scene.captured_at_s,
        )
        self.assertNotIn("pick_and_place", {item["capability"] for item in options})
        placed = replace(
            degraded,
            objects={**degraded.objects, "red_cube": replace(degraded.objects["red_cube"], position_m=(0.0, 0.3, .775))},
        )
        status = completion_status(
            parsed, placed, load_json_config("safety_policy.json"), set(), now_s=scene.captured_at_s,
        )
        self.assertFalse(status[0]["satisfied"])

    def test_mission_scope_rejects_unusable_grounding_before_confirmation(self):
        class LowConfidenceBackend(Backend):
            def capture(inner_self):
                scene = super(LowConfidenceBackend, inner_self).capture()
                red = replace(scene.objects["red_cube"], confidence=0.5)
                return replace(scene, objects={**scene.objects, "red_cube": red})

        backend = LowConfidenceBackend()
        session = AgentSession(MissionModel(), ActionModel([]), backend, self.audit)
        result = session.prepare("Move red to blue")
        self.assertEqual(result["status"], "refused")
        self.assertEqual(result["code"], "object_scope")
        self.assertEqual(backend.executed, [])

    def test_replay_agent_completes_transfer_without_api_or_robot(self):
        backend = ReplayBackend()
        session = AgentSession(ReplayMissionModel(), ReplayAgentActionModel(), backend, self.audit)
        pending = session.prepare("Put the red cube on the yellow target")
        self.assertEqual(pending["status"], "awaiting_confirmation")
        result = session.confirm(pending["confirmation"])
        self.assertEqual(result["status"], "completed")
        self.assertEqual(len(backend.executed), 1)
        self.assertEqual(backend.executed[0]["skill"], "pick_and_place")
        self.assertEqual(backend.executed[0]["target_id"], "yellow_target")


if __name__ == "__main__":
    unittest.main()
