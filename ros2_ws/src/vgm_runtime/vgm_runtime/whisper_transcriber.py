"""Local faster-whisper adapter; imported lazily to keep tests lightweight."""

from __future__ import annotations

import math
from dataclasses import dataclass
from pathlib import Path
from typing import Any


@dataclass(frozen=True, slots=True)
class TranscriptResult:
    text: str
    confidence: float
    language: str | None


class WhisperTranscriptionError(RuntimeError):
    pass


class FasterWhisperTranscriber:
    def __init__(
        self,
        *,
        model_size: str = "small.en",
        device: str = "cpu",
        compute_type: str = "int8",
        minimum_confidence: float = 0.55,
        model: Any | None = None,
    ) -> None:
        self.model_size = model_size
        self.device = device
        self.compute_type = compute_type
        self.minimum_confidence = float(minimum_confidence)
        self._model = model

    def _load_model(self):
        if self._model is None:
            try:
                from faster_whisper import WhisperModel
            except ImportError as error:
                raise WhisperTranscriptionError(
                    "faster-whisper is not installed in this environment"
                ) from error
            self._model = WhisperModel(
                self.model_size,
                device=self.device,
                compute_type=self.compute_type,
            )
        return self._model

    def transcribe(self, audio_path: Path) -> TranscriptResult:
        path = audio_path.expanduser().resolve()
        if not path.is_file() or path.stat().st_size == 0:
            raise WhisperTranscriptionError("audio file is missing or empty")
        try:
            segments, info = self._load_model().transcribe(
                str(path),
                beam_size=5,
                vad_filter=True,
                condition_on_previous_text=False,
            )
            captured = list(segments)
        except Exception as error:
            raise WhisperTranscriptionError(
                f"Whisper transcription failed: {type(error).__name__}"
            ) from error
        text = " ".join(segment.text.strip() for segment in captured).strip()
        if not text:
            raise WhisperTranscriptionError("Whisper returned an empty transcript")
        if not all(math.isfinite(float(segment.avg_logprob)) for segment in captured):
            raise WhisperTranscriptionError("Whisper returned invalid confidence values")
        probabilities = [
            math.exp(min(0.0, float(segment.avg_logprob))) for segment in captured
        ]
        confidence = sum(probabilities) / len(probabilities) if probabilities else 0.0
        if confidence < self.minimum_confidence:
            raise WhisperTranscriptionError("transcript confidence is below threshold")
        return TranscriptResult(
            text=text,
            confidence=confidence,
            language=getattr(info, "language", None),
        )
