"""Command-line entry point for transcript-to-validated-plan evaluation."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

from .audit import AuditLogger
from .coordinator import TaskCoordinator
from .openai_intent import OpenAIIntentModel, load_local_api_key
from .pipeline import IntentPipeline
from .rule_intent import RuleBasedIntentModel
from .serialization import load_scene, plan_to_mapping
from .validator import SkillValidator


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--transcript", required=True)
    parser.add_argument("--scene-json", required=True, type=Path)
    parser.add_argument("--provider", choices=("openai", "rules"), default="openai")
    parser.add_argument("--audit-jsonl", type=Path)
    return parser


def main() -> None:
    args = build_parser().parse_args()
    scene = load_scene(args.scene_json)
    validator = SkillValidator()
    if args.provider == "openai":
        load_local_api_key()
        model = OpenAIIntentModel(schema=validator.schema)
    else:
        model = RuleBasedIntentModel()
    pipeline = IntentPipeline(
        model,
        validator,
        TaskCoordinator(validator.policy),
        AuditLogger(args.audit_jsonl) if args.audit_jsonl else None,
    )
    decision = pipeline.decide(args.transcript, scene)
    output = {
        "accepted": decision.accepted,
        "code": decision.code,
        "message": decision.message,
        "validated_skill": (
            decision.validated_skill.to_mapping()
            if decision.validated_skill is not None
            else None
        ),
        "plan": plan_to_mapping(decision.plan) if decision.plan is not None else None,
    }
    print(json.dumps(output, indent=2, sort_keys=True))
    raise SystemExit(0 if decision.accepted else 2)


if __name__ == "__main__":
    main()
