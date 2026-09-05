"""Explicit synthetic observations, not production fallback coordinates."""
from vgm_runtime.types import TargetObservation


def targets(now):
    return {key: TargetObservation(key, (0., y, .751), .98, now, pixel_count=300)
            for key, y in (("blue_target", .3), ("yellow_target", -.3))}
