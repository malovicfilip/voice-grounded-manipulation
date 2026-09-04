"""Append-only JSONL evidence without audio or API credentials."""

from __future__ import annotations

import json
import os
from pathlib import Path
from typing import Any, Mapping


class AuditLogger:
    def __init__(self, path: Path) -> None:
        self.path = path

    def record(self, event: str, values: Mapping[str, Any]) -> None:
        self.path.parent.mkdir(parents=True, exist_ok=True)
        safe_values = dict(values)
        for key in tuple(safe_values):
            if "key" in key.lower() or "token" in key.lower() or "audio" in key.lower():
                safe_values.pop(key)
        line = json.dumps({"event": event, **safe_values}, sort_keys=True)
        descriptor = os.open(
            self.path,
            os.O_APPEND | os.O_CREAT | os.O_WRONLY,
            0o600,
        )
        try:
            os.write(descriptor, (line + "\n").encode("utf-8"))
        finally:
            os.close(descriptor)
