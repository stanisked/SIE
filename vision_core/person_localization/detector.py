"""Versioned backend-neutral contracts for AR0234 person detection.

No detector model or backend adapter is bundled at this stage. The manifest is
validated syntactically here; an approved future backend adapter must verify
the referenced artifact bytes before it can produce detections.
"""

from __future__ import annotations

import hashlib
import hmac
import json
import math
import re
from collections.abc import Mapping, Sequence
from dataclasses import dataclass, field
from pathlib import Path
from types import MappingProxyType
from typing import Protocol, TypeAlias

import numpy as np

from .models import PersonDetection


JsonValue: TypeAlias = None | bool | str | int | float | list["JsonValue"] | dict[str, "JsonValue"]

ARTIFACT_SCHEMA_VERSION = "sie.person_detector_artifact.v1"
OUTPUT_CONTRACT_VERSION = "sie.person_detection_output.v1"
CONFIDENCE_SEMANTICS = "normalized_score_0_1"
_SAFE_TOKEN = re.compile(r"^[A-Za-z0-9][A-Za-z0-9._-]{0,127}$")
_SHA256 = re.compile(r"^[0-9a-f]{64}$")


def _validate_token(name: str, value: object) -> str:
    if not isinstance(value, str) or not value:
        raise ValueError(f"{name} must be a non-empty string")
    if value != value.strip() or any(ord(character) < 32 or ord(character) == 127 for character in value):
        raise ValueError(f"{name} must not contain surrounding whitespace or control characters")
    if not _SAFE_TOKEN.fullmatch(value):
        raise ValueError(f"{name} must match {_SAFE_TOKEN.pattern}")
    return value


def _json_safe_copy(value: object) -> JsonValue:
    """Return a detached JSON-only value or reject implementation-specific data."""
    value_type = type(value)
    if value is None or value_type in (bool, str, int):
        return value
    if value_type is float:
        if not math.isfinite(value):
            raise ValueError("inference parameters must not contain NaN or infinity")
        return value
    if value_type is list:
        return [_json_safe_copy(item) for item in value]
    if isinstance(value, Mapping):
        copied: dict[str, JsonValue] = {}
        for key, item in value.items():
            if type(key) is not str:
                raise ValueError("inference parameter mapping keys must be strings")
            copied[key] = _json_safe_copy(item)
        return copied
    raise ValueError(f"inference parameters contain unsupported type {value_type.__name__}")


def _freeze_json(value: JsonValue) -> object:
    if type(value) is list:
        return tuple(_freeze_json(item) for item in value)
    if type(value) is dict:
        return MappingProxyType({key: _freeze_json(item) for key, item in value.items()})
    return value


@dataclass(frozen=True)
class DetectorArtifact:
    """Syntactically valid, versioned detector-artifact manifest.

    This object deliberately does not claim that artifact bytes were verified.
    A future approved backend adapter must call ``verify_artifact_bytes`` before
    loading or using its model.
    """

    schema_version: str
    backend_id: str
    backend_version: str
    model_id: str
    model_version: str
    artifact_sha256: str
    output_contract_version: str
    person_label: str
    confidence_semantics: str
    confidence_threshold: float
    inference_parameters: Mapping[str, JsonValue]
    _inference_parameters_json: str = field(init=False, repr=False, compare=False)

    def __post_init__(self) -> None:
        if self.schema_version != ARTIFACT_SCHEMA_VERSION:
            raise ValueError(f"schema_version must be {ARTIFACT_SCHEMA_VERSION}")
        if self.output_contract_version != OUTPUT_CONTRACT_VERSION:
            raise ValueError(f"output_contract_version must be {OUTPUT_CONTRACT_VERSION}")
        if self.confidence_semantics != CONFIDENCE_SEMANTICS:
            raise ValueError(f"confidence_semantics must be {CONFIDENCE_SEMANTICS}")
        for name in ("backend_id", "backend_version", "model_id", "model_version", "person_label"):
            _validate_token(name, getattr(self, name))
        if not isinstance(self.artifact_sha256, str) or not _SHA256.fullmatch(self.artifact_sha256):
            raise ValueError("artifact_sha256 must be a 64-character lowercase hexadecimal SHA-256")
        if type(self.confidence_threshold) not in (int, float) or not math.isfinite(
            self.confidence_threshold
        ):
            raise ValueError("confidence_threshold must be finite")
        if not 0.0 <= float(self.confidence_threshold) <= 1.0:
            raise ValueError("confidence_threshold must be in [0, 1]")
        copied = _json_safe_copy(self.inference_parameters)
        if type(copied) is not dict:
            raise ValueError("inference_parameters must be a mapping")
        serialized = json.dumps(copied, allow_nan=False, sort_keys=True, separators=(",", ":"))
        object.__setattr__(self, "_inference_parameters_json", serialized)
        object.__setattr__(self, "inference_parameters", _freeze_json(copied))

    def metadata(self) -> dict[str, JsonValue]:
        """Return detached, JSON-serializable provenance for an Observation."""
        return {
            "schema_version": self.schema_version,
            "backend_id": self.backend_id,
            "backend_version": self.backend_version,
            "model_id": self.model_id,
            "model_version": self.model_version,
            "artifact_sha256": self.artifact_sha256,
            "output_contract_version": self.output_contract_version,
            "person_label": self.person_label,
            "confidence_semantics": self.confidence_semantics,
            "confidence_threshold": float(self.confidence_threshold),
            "inference_parameters": json.loads(self._inference_parameters_json),
        }


def verify_artifact_bytes(artifact_path: Path, expected_sha256: str) -> bool:
    """Compare bytes of a future backend artifact with a declared SHA-256.

    This helper performs no loading or inference and is not invoked by the
    current runner. Its result must be recorded by a future approved adapter.
    """
    if not isinstance(expected_sha256, str) or not _SHA256.fullmatch(expected_sha256):
        raise ValueError("expected artifact SHA-256 must be lowercase hexadecimal")
    digest = hashlib.sha256()
    with artifact_path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return hmac.compare_digest(digest.hexdigest(), expected_sha256)


class PersonDetector(Protocol):
    """Injectable detector boundary. It returns all candidates, never a target."""

    artifact: DetectorArtifact

    def detect(self, frame_bgr: np.ndarray) -> Sequence[PersonDetection]:
        """Return all person candidates for one BGR uint8 image."""
