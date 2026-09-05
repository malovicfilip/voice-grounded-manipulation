"""Fail-closed transcript-to-plan pipeline."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Protocol

from .audit import AuditLogger
from .coordinator import TaskCoordinator
from .types import GroundedScene, TaskPlan, ValidatedSkill
from .validator import SkillValidationError, SkillValidator
from .stop_intent import is_stop_request, stop_proposal


class IntentModel(Protocol):
    def propose(self, transcript: str, scene: GroundedScene, policy) -> dict[str, Any]: ...


@dataclass(frozen=True, slots=True)
class PipelineDecision:
    accepted: bool
    code: str
    message: str
    raw_proposal: dict[str, Any] | None = None
    validated_skill: ValidatedSkill | None = None
    plan: TaskPlan | None = None


class IntentPipeline:
    def __init__(
        self,
        intent_model: IntentModel,
        validator: SkillValidator,
        coordinator: TaskCoordinator,
        audit_logger: AuditLogger | None = None,
    ) -> None:
        self.intent_model = intent_model
        self.validator = validator
        self.coordinator = coordinator
        self.audit_logger = audit_logger

    def decide(self, transcript: str, scene: GroundedScene) -> PipelineDecision:
        try:
            proposal = stop_proposal() if is_stop_request(transcript) else self.intent_model.propose(
                transcript, scene, self.validator.policy
            )
        except Exception as error:
            decision = PipelineDecision(
                False,
                "intent_model_failure",
                f"intent model failed closed: {type(error).__name__}",
            )
            self._audit(transcript, scene, decision)
            return decision
        try:
            validated = self.validator.validate(proposal, scene)
            plan = self.coordinator.create_plan(validated, scene)
        except SkillValidationError as error:
            decision = PipelineDecision(
                False,
                error.code,
                str(error),
                raw_proposal=proposal,
            )
            self._audit(transcript, scene, decision)
            return decision
        except Exception as error:
            decision = PipelineDecision(
                False,
                "coordination_failure",
                f"coordination failed closed: {type(error).__name__}",
                raw_proposal=proposal,
            )
            self._audit(transcript, scene, decision)
            return decision

        accepted = validated.proposal.skill not in {"refuse"}
        code = "accepted" if accepted else "model_refusal"
        decision = PipelineDecision(
            accepted,
            code,
            validated.proposal.reason or "validated",
            proposal,
            validated,
            plan,
        )
        self._audit(transcript, scene, decision)
        return decision

    def _audit(
        self,
        transcript: str,
        scene: GroundedScene,
        decision: PipelineDecision,
    ) -> None:
        if self.audit_logger is None:
            return
        self.audit_logger.record(
            "intent_decision",
            {
                "transcript": transcript,
                "scene_revision": scene.revision,
                "accepted": decision.accepted,
                "code": decision.code,
                "message": decision.message,
                "proposal": decision.raw_proposal,
            },
        )
