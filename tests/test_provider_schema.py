"""Provider compatibility must never relax local mission/action validation."""
import copy
import json
import unittest

from jsonschema import Draft202012Validator
from test_openai_intent_adapter import scene
from test_agent_control import MissionModel
from vgm_runtime.agent import OpenAIMissionModel, OpenAIAgentActionModel, mission_schema, action_schema
from vgm_runtime.openai_intent import IntentModelError, provider_schema_copy
from vgm_runtime.validator import SkillValidator


class ProviderSchemaTests(unittest.TestCase):
    def test_recursive_projection_preserves_schema_names_literals_and_anyof(self):
        excluded = ("uniqueItems", "allOf", "oneOf", "not", "if", "then", "else",
                    "dependentRequired", "dependentSchemas", "dependencies")
        leaf = {"type": "array", "items": {"type": "string"}, "maxItems": 6,
                **{key: True for key in excluded}}
        original = {
            "type": "object", "additionalProperties": False,
            "required": ["uniqueItems"],
            "properties": {"uniqueItems": {"anyOf": [leaf, {"type": "null"}]}},
            "$defs": {"allOf": leaf},
            "enum": [{"uniqueItems": ["allOf"]}],
            "const": {"if": "then"},
        }
        before = copy.deepcopy(original)
        sent = provider_schema_copy(original)
        expected = {"type": "array", "items": {"type": "string"}, "maxItems": 6}
        self.assertEqual(sent["properties"]["uniqueItems"]["anyOf"][0], expected)
        self.assertEqual(sent["$defs"]["allOf"], expected)
        self.assertEqual(sent["enum"], before["enum"])
        self.assertEqual(sent["const"], before["const"])
        self.assertEqual(sent["required"], ["uniqueItems"])
        self.assertFalse(sent["additionalProperties"])
        sent["properties"]["uniqueItems"]["anyOf"][0]["items"]["type"] = "integer"
        sent["enum"][0]["uniqueItems"].append("modified")
        self.assertEqual(original, before)

    def run_model(self, model_type, value):
        calls = []
        def transport(url, headers, payload, timeout):
            calls.append(payload)
            return {"output_text": json.dumps(value)}
        model = model_type(api_key="test-only", transport=transport,
                           request_id_factory=lambda: value["request_id"])
        before = copy.deepcopy(model.schema)
        try:
            result = model.propose("inspect red cube", scene(), SkillValidator().policy)
        finally:
            self.assertEqual(model.schema, before)
            self.assertEqual(calls[0]["text"]["format"]["schema"], provider_schema_copy(before))
            self.assertTrue(calls[0]["text"]["format"]["strict"])
            self.assertNotIn("tools", calls[0])
        return result

    def test_mission_transport_strips_uniqueness_but_local_validation_rejects_duplicates(self):
        value = MissionModel().propose("inspect red", scene(), {})
        self.assertEqual(self.run_model(OpenAIMissionModel, value), value)
        for key in ("object_ids", "target_ids", "pose_names"):
            with self.subTest(key=key):
                bad = copy.deepcopy(value)
                bad[key] = ["red_cube", "red_cube"]
                Draft202012Validator(provider_schema_copy(mission_schema())).validate(bad)
                self.assertFalse(Draft202012Validator(mission_schema()).is_valid(bad))
                with self.assertRaisesRegex(IntentModelError, "JSON Schema validation"):
                    self.run_model(OpenAIMissionModel, bad)

    def test_action_local_conditionals_still_reject_inconsistent_parameters(self):
        value = dict(schema_version=1, request_id="action_test", scene_revision=scene().revision,
                     capability="inspect", object_id="red_cube", target_id=None,
                     pose_name=None, reason="Inspect the selected cube")
        self.assertEqual(self.run_model(OpenAIAgentActionModel, value), value)
        value["target_id"] = "blue_target"
        Draft202012Validator(provider_schema_copy(action_schema())).validate(value)
        self.assertFalse(Draft202012Validator(action_schema()).is_valid(value))
        with self.assertRaisesRegex(IntentModelError, "JSON Schema validation"):
            self.run_model(OpenAIAgentActionModel, value)

    def test_current_agent_payloads_use_supported_keywords_and_keep_constraints(self):
        allowed = {"type", "properties", "required", "additionalProperties", "items",
                   "anyOf", "$defs", "$ref", "enum", "const", "pattern", "minLength",
                   "maxLength", "minItems", "maxItems", "minimum", "maximum"}
        def check(schema):
            self.assertLessEqual(set(schema), allowed)
            for child in schema.get("properties", {}).values():
                check(child)
            if "items" in schema:
                check(schema["items"])
            for child in schema.get("anyOf", []):
                check(child)
        for original in (mission_schema(), action_schema()):
            sent = provider_schema_copy(original)
            check(sent)
            Draft202012Validator.check_schema(sent)
            self.assertEqual(sent["required"], original["required"])
            self.assertFalse(sent["additionalProperties"])
        props = provider_schema_copy(mission_schema())["properties"]
        self.assertEqual(props["object_ids"]["maxItems"], 6)
        self.assertEqual(props["max_actions"], mission_schema()["properties"]["max_actions"])
