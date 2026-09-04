"""Confirmed, bounded task sequences with observation between every skill.

The model selects only identifiers. The task session owns sequencing, holding
preconditions, confirmation, cancellation, and recovery. Backends must validate
the proposal independently before requesting any MoveIt operation.
"""

from __future__ import annotations

import hashlib
import json
import math
import threading
import time
from dataclasses import dataclass, replace
from typing import Any, Protocol

from .audit import AuditLogger
from .config import load_json_config
from .openai_intent import OpenAIIntentModel
from .types import GroundedScene
from .validator import SkillValidationError, SkillValidator


MAX_STEPS = 8
CONFIRMATION_TTL_S = 120.0
TASK_DEADLINE_S = 300.0
STEP_FIELDS = {"skill", "object_id", "target_id", "pose_name", "reason"}


def task_schema() -> dict[str, Any]:
    skill = load_json_config("robot_skill.schema.json")
    return {
        "type": "object", "additionalProperties": False,
        "required": ["schema_version", "request_id", "scene_revision", "steps"],
        "properties": {
            "schema_version": {"type": "integer", "const": 1},
            "request_id": skill["properties"]["request_id"],
            "scene_revision": {"type": "string", "pattern": "^[a-f0-9]{16}$"},
            "steps": {
                "type": "array", "minItems": 1, "maxItems": MAX_STEPS,
                "items": {
                    "type": "object", "additionalProperties": False,
                    "required": sorted(STEP_FIELDS),
                    "properties": {key: skill["properties"][key] for key in sorted(STEP_FIELDS)},
                },
            },
        },
    }


class OpenAITaskModel(OpenAIIntentModel):
    def __init__(self, **kwargs):
        super().__init__(
            schema=task_schema(), response_name="robot_task", max_output_tokens=1800,
            instructions=(
                "Convert the transcript to an ordered task of at most eight high-level skills. "
                "Copy request_id and scene_revision exactly. Use only available IDs. "
                "Never output coordinates, joints, velocities, motors, or trajectories. "
                "Use pick_and_place for one transfer. Use separate pick then place only if requested. "
                "Inspect observes an object without moving. A pick must be followed by place of "
                "the same object before picking anything else. Do not leave an object held at task end. "
                "For ambiguity return a single refuse step with a concrete clarification question. "
                "A stop request produces only one stop step. Unused fields must be null."
            ), **kwargs,
        )


class TaskBackend(Protocol):
    def capture(self) -> GroundedScene: ...
    def execute(self, proposal: dict, scene: GroundedScene) -> dict: ...
    def stop(self) -> dict: ...
    def recover(self) -> dict: ...


def proposal_for(step: dict, task_id: str, index: int, scene: GroundedScene) -> dict:
    return {
        "schema_version": 1,
        "request_id": hashlib.sha256(f"{task_id}:{index}".encode()).hexdigest()[:32],
        "scene_revision": scene.revision,
        **step,
    }


def check_preconditions(step: dict, held: str | None, scene: GroundedScene, policy: dict) -> None:
    skill = step["skill"]
    if skill == "place" and held != step["object_id"]:
        raise SkillValidationError("not_holding_object", "place requires the same object to be held")
    if held and skill not in {"place", "stop", "refuse"}:
        raise SkillValidationError("object_already_held", "place the held object before another action")
    if skill in {"place", "pick_and_place"}:
        target = policy["targets"][step["target_id"]]["position_m"]
        for obj in scene.objects.values():
            if obj.object_id != step["object_id"] and math.dist(obj.position_m[:2], target[:2]) < 0.08:
                raise SkillValidationError("target_occupied", "the destination is occupied by another object")


@dataclass(frozen=True)
class PendingTask:
    request_id: str
    steps_json: str
    scene: GroundedScene
    confirmation: str
    expires_at: float

    @property
    def steps(self):
        return json.loads(self.steps_json)


class TaskSession:
    def __init__(self, model, backend: TaskBackend, audit: AuditLogger, *, clock=time.time):
        self.model, self.backend, self.audit, self.clock = model, backend, audit, clock
        self.policy = load_json_config("safety_policy.json")
        self.pending: PendingTask | None = None
        self.state = "idle"
        self._used: set[str] = set()
        self._stop = threading.Event()

    def prepare(self, transcript: str) -> dict:
        if self.state not in {"idle", "awaiting_confirmation", "completed", "refused"}:
            raise RuntimeError("recover the stopped/faulted session before preparing a new task")
        self.pending = None
        if not isinstance(transcript, str) or not transcript.strip() or len(transcript) > 2000:
            return self._refuse("invalid_transcript", "Provide a nonempty command up to 2000 characters.")
        if transcript.strip().lower().rstrip(".!?") in {"stop", "cancel", "emergency stop", "stop now"}:
            return self.stop()
        try:
            scene = self.backend.capture()
            raw = self.model.propose(transcript, scene, self.policy)
            validator = SkillValidator(clock=self.clock)
            validator._reject_direct_control_fields(raw)
            if not isinstance(raw, dict) or set(raw) != {"schema_version", "request_id", "scene_revision", "steps"}:
                raise ValueError("task fields are invalid")
            if type(raw["schema_version"]) is not int or raw["schema_version"] != 1:
                raise ValueError("task schema version is invalid")
            if raw["scene_revision"] != scene.revision:
                raise ValueError("model changed the scene revision")
            task_id = raw["request_id"]
            if not isinstance(task_id, str) or not task_id or len(task_id) > 64 or task_id in self._used:
                raise ValueError("invalid or replayed task ID")
            steps = raw["steps"]
            if not isinstance(steps, list) or not 1 <= len(steps) <= MAX_STEPS:
                raise ValueError("task requires one to eight steps")
            held = scene.held_object_id
            for index, step in enumerate(steps):
                if not isinstance(step, dict) or set(step) != STEP_FIELDS:
                    raise ValueError("step fields are invalid")
                virtual = replace(scene, held_object_id=held)
                validator.validate(proposal_for(step, task_id, index, virtual), virtual)
                if step["skill"] in {"stop", "refuse"}:
                    if len(steps) != 1:
                        raise ValueError("stop/refuse must be the only step")
                    if step["skill"] == "stop":
                        return self.stop()
                    return self._refuse("clarification_required", step["reason"])
                check_preconditions(step, held, virtual, self.policy)
                if step["skill"] == "pick":
                    held = step["object_id"]
                elif step["skill"] == "place":
                    held = None
            if held:
                raise ValueError("the task must finish by placing its held object")
            canonical = json.dumps(steps, sort_keys=True)
            token = hashlib.sha256(f"{task_id}:{canonical}".encode()).hexdigest()[:16]
            self.pending = PendingTask(task_id, canonical, scene, token, self.clock() + CONFIRMATION_TTL_S)
            self.state = "awaiting_confirmation"
            result = {"status": self.state, "request_id": task_id, "steps": steps, "confirmation": token}
            self.audit.record("task_prepared", {"transcript": transcript, **result})
            return result
        except Exception as error:
            return self._refuse(getattr(error, "code", "invalid_task"), str(error))

    def confirm(self, token: str) -> dict:
        pending = self.pending
        if pending is None or self.state != "awaiting_confirmation" or token != pending.confirmation:
            raise ValueError("confirmation must match the pending task")
        self.pending = None
        if self.clock() > pending.expires_at:
            return self._refuse("confirmation_expired", "Please repeat the command to obtain a fresh plan.")
        self._used.add(pending.request_id)
        self._stop.clear()
        self.state = "running"
        completed = []
        baseline = dict(pending.scene.objects)
        deadline = time.monotonic() + TASK_DEADLINE_S
        try:
            for index, step in enumerate(pending.steps):
                if self._stop.is_set():
                    break
                if time.monotonic() > deadline:
                    raise RuntimeError("task deadline exceeded")
                scene = self.backend.capture()
                proposal = proposal_for(step, pending.request_id, index, scene)
                SkillValidator(clock=self.clock).validate(proposal, scene)
                check_preconditions(step, scene.held_object_id, scene, self.policy)
                if step["skill"] in {"pick", "pick_and_place"}:
                    original = baseline.get(step["object_id"])
                    current = scene.objects[step["object_id"]]
                    if original is None or math.dist(original.position_m, current.position_m) > self.policy["maximum_object_drift_m"]:
                        raise SkillValidationError("object_moved", "object moved after confirmation was requested")
                if self._stop.is_set():
                    break
                result = self.backend.execute(proposal, scene)
                self.audit.record("task_step", {"index": index, "proposal": proposal, "result": result})
                if result.get("status") != "succeeded":
                    raise RuntimeError(f"skill failed: {result.get('code', 'backend_failure')}")
                completed.append({"skill": step["skill"], "result": result})
                # A new camera observation after successful placement becomes the
                # baseline only for the object our own verified step moved.
                if step["skill"] in {"place", "pick_and_place"}:
                    latest = self.backend.capture()
                    baseline[step["object_id"]] = latest.objects[step["object_id"]]
            self.state = "stopped" if self._stop.is_set() else "completed"
            result = {"status": self.state, "completed_steps": completed}
        except Exception as error:
            self.state = "stopped" if self._stop.is_set() else "faulted"
            try:
                self.backend.stop()
            finally:
                result = {"status": self.state, "code": getattr(error, "code", "execution_failure"),
                          "message": str(error), "completed_steps": completed}
        self.audit.record("task_result", result)
        return result

    def stop(self) -> dict:
        self._stop.set()
        self.pending = None
        self.state = "stopped"
        result = self.backend.stop()
        self.audit.record("operator_stop", result)
        return {"status": "stopped", "backend": result}

    def recover(self) -> dict:
        if self.state not in {"stopped", "faulted"}:
            raise RuntimeError("recovery requires a stopped or faulted task")
        result = self.backend.recover()
        if result.get("status") != "recovered":
            return result
        self._stop.clear()
        self.pending = None
        self.state = "idle"
        self.audit.record("operator_recovery", result)
        return result

    def _refuse(self, code: str, message: str) -> dict:
        self.pending = None
        self.state = "refused"
        result = {"status": "refused", "code": code, "message": message}
        self.audit.record("task_refused", result)
        return result
