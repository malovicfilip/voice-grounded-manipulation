"""OpenAI Responses API adapter that can only return the skill JSON schema."""

from __future__ import annotations

import json
import os
import urllib.error
import urllib.request
import uuid
from collections.abc import Callable, Mapping
from pathlib import Path
from typing import Any

from .config import config_directory, load_json_config
from .types import GroundedScene


class IntentModelError(RuntimeError):
    """The remote intent proposal failed closed."""


def load_local_api_key(path: Path | None = None) -> None:
    """Load only OPENAI_API_KEY from an ignored local env file, if needed."""
    if os.environ.get("OPENAI_API_KEY"):
        return
    env_path = path or config_directory().parent / ".env.local"
    try:
        lines = env_path.read_text(encoding="utf-8").splitlines()
    except FileNotFoundError:
        return
    for line in lines:
        if line.startswith("OPENAI_API_KEY="):
            value = line.partition("=")[2].strip()
            if value:
                os.environ["OPENAI_API_KEY"] = value
            return


def _default_transport(
    url: str,
    headers: Mapping[str, str],
    payload: Mapping[str, Any],
    timeout_s: float,
) -> dict[str, Any]:
    request = urllib.request.Request(
        url,
        data=json.dumps(payload).encode("utf-8"),
        headers=dict(headers),
        method="POST",
    )
    try:
        with urllib.request.urlopen(request, timeout=timeout_s) as response:
            body = response.read()
    except urllib.error.HTTPError as error:
        api_code = None
        parameter = None
        api_message = None
        try:
            error_body = json.loads(error.read())
            api_error = error_body.get("error", {})
            api_code = api_error.get("code") or api_error.get("type")
            parameter = api_error.get("param")
            raw_message = api_error.get("message")
            if isinstance(raw_message, str):
                api_message = " ".join(raw_message.split())[:400]
        except (AttributeError, TypeError, json.JSONDecodeError):
            pass
        metadata = " ".join(
            part
            for part in (
                f"api_code={api_code}" if api_code else "",
                f"param={parameter}" if parameter else "",
                f"detail={api_message}" if api_message else "",
            )
            if part
        )
        raise IntentModelError(
            f"OpenAI request failed: HTTPError status={error.code} {metadata}".rstrip()
        ) from error
    except (OSError, urllib.error.URLError) as error:
        status = getattr(error, "code", None)
        suffix = f" status={status}" if isinstance(status, int) else ""
        raise IntentModelError(
            f"OpenAI request failed: {type(error).__name__}{suffix}"
        ) from error
    try:
        value = json.loads(body)
    except (TypeError, json.JSONDecodeError) as error:
        raise IntentModelError("OpenAI response was not valid JSON") from error
    if not isinstance(value, dict):
        raise IntentModelError("OpenAI response was not an object")
    return value


class OpenAIIntentModel:
    """Request one tool-free, schema-constrained high-level skill proposal."""

    ENDPOINT = "https://api.openai.com/v1/responses"

    def __init__(
        self,
        *,
        model: str = "gpt-5.6-terra",
        schema: Mapping[str, Any] | None = None,
        api_key: str | None = None,
        timeout_s: float = 20.0,
        transport: Callable[..., dict[str, Any]] = _default_transport,
        request_id_factory: Callable[[], str] | None = None,
    ) -> None:
        self.model = model
        self.schema = dict(schema or load_json_config("robot_skill.schema.json"))
        self.api_key = api_key
        self.timeout_s = float(timeout_s)
        self.transport = transport
        self.request_id_factory = request_id_factory or (
            lambda: f"req_{uuid.uuid4().hex[:16]}"
        )

    def propose(
        self, transcript: str, scene: GroundedScene, policy: Mapping[str, Any]
    ) -> dict[str, Any]:
        transcript = transcript.strip()
        if not transcript:
            raise IntentModelError("transcript is empty")
        key = self.api_key or os.environ.get("OPENAI_API_KEY")
        if not key:
            raise IntentModelError("OPENAI_API_KEY is unavailable")

        request_id = self.request_id_factory()
        context = {
            "request_id": request_id,
            "scene_revision": scene.revision,
            "available_object_ids": sorted(scene.objects),
            "available_target_ids": sorted(policy["targets"]),
            "allowed_named_poses": list(policy["allowed_named_poses"]),
            "transcript": transcript,
        }
        payload = {
            "model": self.model,
            "instructions": (
                "Convert the transcript into exactly one high-level robot skill. "
                "Copy request_id and scene_revision exactly. Never invent object IDs. "
                "Never emit joints, velocities, motors, torques, efforts, or trajectories. "
                "Use refuse with a short reason for ambiguity, missing grounding, unsupported "
                "requests, or uncertainty. Use stop immediately for a stop request."
            ),
            "input": json.dumps(context, separators=(",", ":")),
            "reasoning": {"effort": "low"},
            "max_output_tokens": 512,
            "store": False,
            "text": {
                "format": {
                    "type": "json_schema",
                    "name": "robot_skill_proposal",
                    "strict": True,
                    "schema": self.schema,
                }
            },
        }
        response = self.transport(
            self.ENDPOINT,
            {
                "Authorization": f"Bearer {key}",
                "Content-Type": "application/json",
            },
            payload,
            self.timeout_s,
        )
        output_text = self._extract_output_text(response)
        try:
            proposal = json.loads(output_text)
        except json.JSONDecodeError as error:
            raise IntentModelError("structured output was not valid JSON") from error
        if not isinstance(proposal, dict):
            raise IntentModelError("structured output was not an object")
        if proposal.get("request_id") != request_id:
            raise IntentModelError("model changed the request_id")
        return proposal

    @staticmethod
    def _extract_output_text(response: Mapping[str, Any]) -> str:
        direct = response.get("output_text")
        if isinstance(direct, str) and direct:
            return direct
        for item in response.get("output", []):
            if not isinstance(item, Mapping) or item.get("type") != "message":
                continue
            for content in item.get("content", []):
                if isinstance(content, Mapping) and content.get("type") == "output_text":
                    text = content.get("text")
                    if isinstance(text, str) and text:
                        return text
                if isinstance(content, Mapping) and content.get("type") == "refusal":
                    raise IntentModelError("model refused without structured output")
        raise IntentModelError("OpenAI response contained no output text")
