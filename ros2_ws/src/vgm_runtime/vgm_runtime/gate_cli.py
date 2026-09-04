"""Revalidate an accepted decision immediately before MoveIt execution."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

from .execution_gate import gate_decision
from .serialization import load_scene
from .validator import SkillValidationError, SkillValidator


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--decision-json", required=True, type=Path)
    parser.add_argument("--latest-scene-json", required=True, type=Path)
    parser.add_argument("--output-json", required=True, type=Path)
    args = parser.parse_args()

    try:
        decision = json.loads(args.decision_json.read_text(encoding="utf-8"))
        gated = gate_decision(
            decision,
            load_scene(args.latest_scene_json),
            SkillValidator(),
        )
        args.output_json.write_text(
            json.dumps(gated, indent=2, sort_keys=True) + "\n", encoding="utf-8"
        )
    except (OSError, TypeError, ValueError, json.JSONDecodeError, SkillValidationError) as error:
        print(
            json.dumps(
                {
                    "accepted": False,
                    "code": getattr(error, "code", "execution_gate_failure"),
                    "message": str(error),
                },
                sort_keys=True,
            )
        )
        raise SystemExit(2) from error


if __name__ == "__main__":
    main()
