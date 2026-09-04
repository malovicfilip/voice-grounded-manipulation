"""Validate final RGB-D evidence against the gated placement outcome."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

from .config import load_json_config
from .outcome import OutcomeValidationError, validate_placement_outcome
from .serialization import load_scene


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--execution-plan", required=True, type=Path)
    parser.add_argument("--final-scene", required=True, type=Path)
    parser.add_argument("--output-json", required=True, type=Path)
    args = parser.parse_args()
    try:
        execution = json.loads(args.execution_plan.read_text(encoding="utf-8"))
        result = validate_placement_outcome(
            execution, load_scene(args.final_scene), load_json_config("safety_policy.json")
        )
        args.output_json.write_text(
            json.dumps(result, indent=2, sort_keys=True) + "\n", encoding="utf-8"
        )
    except (OSError, TypeError, ValueError, json.JSONDecodeError, OutcomeValidationError) as error:
        print(
            json.dumps(
                {
                    "accepted": False,
                    "code": getattr(error, "code", "outcome_validation_failure"),
                    "message": str(error),
                },
                sort_keys=True,
            )
        )
        raise SystemExit(2) from error


if __name__ == "__main__":
    main()
