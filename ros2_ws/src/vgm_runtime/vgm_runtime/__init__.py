"""Safety-bounded runtime for the voice-grounded manipulation project."""

from .types import GroundedScene, ObjectObservation, TargetObservation, SkillProposal, ValidatedSkill


def __getattr__(name):
    # Isaac imports perception/types without the ROS command-runtime dependencies.
    # Requesting a validator still imports jsonschema normally and fails closed
    # if it is missing; this is not an optional validation fallback.
    if name in {"SkillValidationError", "SkillValidator"}:
        from .validator import SkillValidationError, SkillValidator
        return {"SkillValidationError": SkillValidationError, "SkillValidator": SkillValidator}[name]
    raise AttributeError(f"module {__name__!r} has no attribute {name!r}")

__all__ = [
    "GroundedScene",
    "ObjectObservation",
    "TargetObservation",
    "SkillProposal",
    "SkillValidationError",
    "SkillValidator",
    "ValidatedSkill",
]
