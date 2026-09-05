"""Operator cancellation has no model, scene, key, or confirmation dependency."""

import re
import uuid


def is_stop_request(text: str) -> bool:
    # Whole words deliberately favor stopping even in a longer instruction.
    # 'stopper' and 'unstoppable' are not stop words; negated stops also stop.
    return isinstance(text, str) and bool(re.search(r"\b(stop|cancel|abort|halt|freeze)\b", text, re.I))


def stop_proposal() -> dict:
    return dict(schema_version=1, request_id="stop_" + uuid.uuid4().hex,
                skill="stop", object_id=None, target_id=None, pose_name=None,
                reason="operator requested stop", scene_revision=None)
