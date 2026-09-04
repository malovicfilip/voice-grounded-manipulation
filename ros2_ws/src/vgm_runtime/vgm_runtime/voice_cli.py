"""Transcribe a local audio file, then run the validated intent pipeline."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

from .audit import AuditLogger
from .coordinator import TaskCoordinator
from .openai_intent import OpenAIIntentModel, load_local_api_key
from .pipeline import IntentPipeline
from .serialization import load_scene, plan_to_mapping
from .validator import SkillValidator
from .whisper_transcriber import FasterWhisperTranscriber


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("audio", type=Path)
    parser.add_argument("--scene-json", required=True, type=Path)
    parser.add_argument("--audit-jsonl", type=Path, required=True)
    parser.add_argument("--whisper-model", default="small.en")
    args = parser.parse_args()

    transcript = FasterWhisperTranscriber(model_size=args.whisper_model).transcribe(
        args.audio
    )
    scene = load_scene(args.scene_json)
    validator = SkillValidator()
    load_local_api_key()
    pipeline = IntentPipeline(
        OpenAIIntentModel(schema=validator.schema),
        validator,
        TaskCoordinator(validator.policy),
        AuditLogger(args.audit_jsonl),
    )
    decision = pipeline.decide(transcript.text, scene)
    print(
        json.dumps(
            {
                "transcript": transcript.text,
                "transcript_confidence": transcript.confidence,
                "accepted": decision.accepted,
                "code": decision.code,
                "validated_skill": (
                    decision.validated_skill.to_mapping()
                    if decision.validated_skill is not None
                    else None
                ),
                "plan": (
                    plan_to_mapping(decision.plan)
                    if decision.plan is not None
                    else None
                ),
            },
            indent=2,
            sort_keys=True,
        )
    )
    raise SystemExit(0 if decision.accepted else 2)


if __name__ == "__main__":
    main()
