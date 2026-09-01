"""Fail-closed, offline CPU benchmark framework for approved ONNX artifacts.

This module deliberately has no runnable detector approval. A concrete model
adapter belongs in a separately reviewed change that pins an artifact, its
provenance, and its verified graph contract.
"""

from __future__ import annotations

import hashlib
import hmac
import json
import math
import os
import platform
import errno
import secrets
import stat
import subprocess
import sys
from types import MappingProxyType
from collections.abc import Callable, Mapping, Sequence
from dataclasses import dataclass, field
from datetime import datetime, timezone
from pathlib import Path
from time import perf_counter_ns
from typing import Any, Protocol

import cv2
import numpy as np


BENCHMARK_MANIFEST_SCHEMA_VERSION = "sie.person_detector_benchmark_manifest.v2"
REPORT_SCHEMA_VERSION = "sie.person_detector_cpu_benchmark_report.v2"
_IMAGE_SUFFIXES = frozenset({".bmp", ".jpeg", ".jpg", ".png"})
_MANIFEST_KEYS = frozenset(
    {
        "schema_version", "model_family", "model_id", "model_version",
        "official_source_url", "artifact_filename", "artifact_sha256",
        "license_identifier", "license_source_url", "adapter_id",
        "adapter_version", "input_contract_id", "output_contract_id",
        "person_class_id", "confidence_threshold", "inference_parameters",
    }
)
_APPROVAL_FIELDS = (
    "artifact_sha256", "model_family", "model_id", "model_version",
    "artifact_filename", "official_source_url", "license_identifier",
    "license_source_url", "adapter_id", "adapter_version", "input_contract_id",
    "output_contract_id", "inference_parameters_json",
)


class BenchmarkBlockedError(RuntimeError):
    """A controlled safety refusal that the CLI renders without a traceback."""


@dataclass(frozen=True)
class ReportPublicationResult:
    """The namespace and crash-durability state of a published report."""

    target_path: str
    overwrite: bool
    published: bool
    durability_confirmed: bool


class ReportPublishedDurabilityUncertainError(BenchmarkBlockedError):
    """The report is visible, but the directory fsync did not confirm durability."""

    def __init__(self, result: ReportPublicationResult, cause: OSError) -> None:
        self.result = result
        self.published = result.published
        self.durability_confirmed = result.durability_confirmed
        self.target_path = result.target_path
        self.overwrite = result.overwrite
        self.cause = cause
        super().__init__(
            "REPORT_PUBLISHED_DURABILITY_UNCERTAIN: target is published in the current "
            "namespace, but crash durability was not confirmed"
        )


@dataclass(frozen=True)
class BoundingBox:
    x_min: int
    y_min: int
    x_max: int
    y_max: int

    def validate(self, width: int, height: int) -> None:
        values = (self.x_min, self.y_min, self.x_max, self.y_max)
        if any(type(value) is not int for value in values):
            raise BenchmarkBlockedError("INVALID_FINAL_DETECTION")
        if self.x_min < 0 or self.y_min < 0 or self.x_max <= self.x_min or self.y_max <= self.y_min:
            raise BenchmarkBlockedError("INVALID_FINAL_DETECTION")
        if self.x_max > width or self.y_max > height:
            raise BenchmarkBlockedError("INVALID_FINAL_DETECTION")


@dataclass(frozen=True)
class Detection:
    """A final, model-specific-postprocessed person candidate from an adapter."""

    bbox: BoundingBox
    confidence: float
    class_id: int
    label: str = "person"


class DetectionStatus:
    PERSON_LOST = "PERSON_LOST"
    SINGLE_PERSON = "SINGLE_PERSON"
    MULTIPLE_PERSONS = "MULTIPLE_PERSONS"


@dataclass(frozen=True)
class BenchmarkManifest:
    schema_version: str
    model_family: str
    model_id: str
    model_version: str
    official_source_url: str
    artifact_filename: str
    artifact_sha256: str
    license_identifier: str
    license_source_url: str
    adapter_id: str
    adapter_version: str
    input_contract_id: str
    output_contract_id: str
    person_class_id: int
    confidence_threshold: float
    inference_parameters: Mapping[str, object]
    inference_parameters_json: str = field(init=False, repr=False, compare=True)

    def __post_init__(self) -> None:
        if self.schema_version != BENCHMARK_MANIFEST_SCHEMA_VERSION:
            raise ValueError("unsupported benchmark manifest schema_version")
        for name in (
            "model_family", "model_id", "model_version", "artifact_filename",
            "license_identifier", "adapter_id", "adapter_version", "input_contract_id",
            "output_contract_id",
        ):
            _require_nonempty_string(name, getattr(self, name))
        for name in ("official_source_url", "license_source_url"):
            value = getattr(self, name)
            if type(value) is not str or not value.startswith("https://"):
                raise ValueError(f"{name} must be an https URL")
        if type(self.artifact_sha256) is not str or len(self.artifact_sha256) != 64:
            raise ValueError("artifact_sha256 must be a lowercase SHA-256 hex digest")
        if any(character not in "0123456789abcdef" for character in self.artifact_sha256):
            raise ValueError("artifact_sha256 must be a lowercase SHA-256 hex digest")
        if type(self.person_class_id) is not int or self.person_class_id < 0:
            raise ValueError("person_class_id must be a non-negative integer")
        if type(self.confidence_threshold) not in (int, float) or not math.isfinite(self.confidence_threshold):
            raise ValueError("confidence_threshold must be finite")
        if not 0.0 <= self.confidence_threshold <= 1.0:
            raise ValueError("confidence_threshold must be in [0, 1]")
        if type(self.inference_parameters) is not dict:
            raise ValueError("inference_parameters must be a JSON object")
        frozen_parameters = _freeze_json_value(self.inference_parameters, "inference_parameters")
        if not isinstance(frozen_parameters, Mapping):
            raise ValueError("inference_parameters must be a JSON object")
        object.__setattr__(self, "inference_parameters", frozen_parameters)
        object.__setattr__(
            self,
            "inference_parameters_json",
            json.dumps(_thaw_json_tree(frozen_parameters), sort_keys=True, separators=(",", ":"), allow_nan=False),
        )


@dataclass(frozen=True)
class TrustedApproval:
    artifact_sha256: str
    model_family: str
    model_id: str
    model_version: str
    artifact_filename: str
    official_source_url: str
    license_identifier: str
    license_source_url: str
    adapter_id: str
    adapter_version: str
    input_contract_id: str
    output_contract_id: str
    inference_parameters_json: str


@dataclass(frozen=True)
class DatasetFrame:
    path: str
    encoded_sha256: str
    pixel_bytes: bytes
    image_shape: tuple[int, int, int]
    image_dtype: str

    @property
    def image_width(self) -> int:
        return self.image_shape[1]

    @property
    def image_height(self) -> int:
        return self.image_shape[0]

    def new_image_view(self) -> np.ndarray:
        """Create one adapter-local view from immutable snapshot bytes."""
        dtype = np.dtype(self.image_dtype)
        image = np.frombuffer(self.pixel_bytes, dtype=dtype).reshape(self.image_shape)
        if (
            image.dtype != np.uint8
            or image.shape != self.image_shape
            or image.ndim != 3
            or image.shape[2] != 3
            or image.flags.writeable
        ):
            raise BenchmarkBlockedError("DATASET_IMAGE_NOT_IMMUTABLE")
        return image


@dataclass(frozen=True)
class DatasetSnapshot:
    directory: str
    frames: tuple[DatasetFrame, ...]


class BenchmarkAdapter(Protocol):
    """Generic boundary: all model-specific processing ends at postprocess()."""

    adapter_id: str
    adapter_version: str

    def preprocess(self, image_bgr: np.ndarray, manifest: BenchmarkManifest) -> np.ndarray: ...

    def postprocess(
        self,
        output: object,
        manifest: BenchmarkManifest,
        *,
        image_width: int,
        image_height: int,
    ) -> Sequence[Detection]: ...


ApprovalResolver = Callable[[BenchmarkManifest], TrustedApproval | None]
AdapterResolver = Callable[[TrustedApproval], BenchmarkAdapter | None]
RootDiscovery = Callable[[Path], tuple[Path, ...]]
OnnxLoader = Callable[[np.ndarray], Any]
ImageDecoder = Callable[[np.ndarray, int], np.ndarray | None]

# This is intentionally the whole production approval boundary for this scaffold.
PRODUCTION_TRUSTED_APPROVALS: tuple[TrustedApproval, ...] = ()
PRODUCTION_ADAPTERS: Mapping[str, BenchmarkAdapter] = MappingProxyType({})


def production_approval_resolver(manifest: BenchmarkManifest) -> TrustedApproval | None:
    for approval in PRODUCTION_TRUSTED_APPROVALS:
        if _approval_matches(manifest, approval):
            return approval
    return None


def production_adapter_resolver(approval: TrustedApproval) -> BenchmarkAdapter | None:
    return PRODUCTION_ADAPTERS.get(f"{approval.adapter_id}:{approval.adapter_version}")


def _require_nonempty_string(name: str, value: object) -> None:
    if type(value) is not str or not value or value != value.strip():
        raise ValueError(f"{name} must be a non-empty trimmed string")


def _freeze_json_value(value: object, name: str) -> object:
    """Validate and freeze one strict-JSON tree without revisiting user containers."""
    value_type = type(value)
    if value is None or value_type in (str, bool, int):
        return value
    if value_type is float:
        if not math.isfinite(value):
            raise ValueError(f"{name} must contain finite JSON numbers")
        return value
    if value_type is list:
        return tuple(_freeze_json_value(child, f"{name}[{index}]") for index, child in enumerate(value))
    if value_type is dict:
        frozen: dict[str, object] = {}
        for key, child in value.items():
            if type(key) is not str:
                raise ValueError(f"{name} contains a non-string key")
            frozen[key] = _freeze_json_value(child, f"{name}.{key}")
        return MappingProxyType(frozen)
    raise ValueError(f"{name} must contain only standard JSON values")


def _thaw_json_tree(value: object) -> object:
    """Return a detached, JSON-safe copy suitable for evidence serialization."""
    if isinstance(value, Mapping):
        return {key: _thaw_json_tree(child) for key, child in value.items()}
    if isinstance(value, tuple):
        return [_thaw_json_tree(child) for child in value]
    return value


def manifest_report_metadata(manifest: BenchmarkManifest) -> dict[str, object]:
    """Build report metadata without exposing a manifest-owned mutable object."""
    return {
        "schema_version": manifest.schema_version,
        "model_family": manifest.model_family,
        "model_id": manifest.model_id,
        "model_version": manifest.model_version,
        "official_source_url": manifest.official_source_url,
        "artifact_filename": manifest.artifact_filename,
        "artifact_sha256": manifest.artifact_sha256,
        "license_identifier": manifest.license_identifier,
        "license_source_url": manifest.license_source_url,
        "adapter_id": manifest.adapter_id,
        "adapter_version": manifest.adapter_version,
        "input_contract_id": manifest.input_contract_id,
        "output_contract_id": manifest.output_contract_id,
        "person_class_id": manifest.person_class_id,
        "confidence_threshold": manifest.confidence_threshold,
        "inference_parameters": _thaw_json_tree(manifest.inference_parameters),
    }


def _no_duplicate_pairs(pairs: list[tuple[str, object]]) -> dict[str, object]:
    result: dict[str, object] = {}
    for key, value in pairs:
        if key in result:
            raise ValueError(f"duplicate JSON key: {key}")
        result[key] = value
    return result


def _reject_json_constant(value: str) -> object:
    raise ValueError(f"non-finite JSON constant: {value}")


def parse_manifest(path: Path) -> BenchmarkManifest:
    if not path.is_absolute():
        raise ValueError("manifest path must be absolute")
    try:
        payload = json.loads(
            path.read_text(encoding="utf-8"),
            object_pairs_hook=_no_duplicate_pairs,
            parse_constant=_reject_json_constant,
        )
    except (OSError, UnicodeDecodeError, json.JSONDecodeError, ValueError) as error:
        raise BenchmarkBlockedError(f"INVALID_MANIFEST: {error}") from error
    if not isinstance(payload, dict) or set(payload) != _MANIFEST_KEYS:
        present = set(payload) if isinstance(payload, dict) else set()
        raise BenchmarkBlockedError(
            f"INVALID_MANIFEST: missing={sorted(_MANIFEST_KEYS - present)}, "
            f"unknown={sorted(present - _MANIFEST_KEYS)}"
        )
    try:
        return BenchmarkManifest(**payload)  # type: ignore[arg-type]
    except (TypeError, ValueError) as error:
        raise BenchmarkBlockedError(f"INVALID_MANIFEST: {error}") from error


def _approval_matches(manifest: BenchmarkManifest, approval: TrustedApproval) -> bool:
    return all(getattr(manifest, field) == getattr(approval, field) for field in _APPROVAL_FIELDS)


def require_approved_manifest(
    manifest: BenchmarkManifest, resolver: ApprovalResolver = production_approval_resolver
) -> TrustedApproval:
    approval = resolver(manifest)
    if approval is None or not _approval_matches(manifest, approval):
        raise BenchmarkBlockedError("ARTIFACT_NOT_APPROVED")
    return approval


def canonical_repository_root() -> Path:
    root = Path(__file__).resolve().parents[2]
    try:
        top_level = subprocess.check_output(
            ["git", "-C", str(root), "rev-parse", "--show-toplevel"], text=True, stderr=subprocess.DEVNULL
        ).strip()
    except (OSError, subprocess.CalledProcessError) as error:
        raise BenchmarkBlockedError("CANONICAL_GIT_ROOT_UNAVAILABLE") from error
    if Path(top_level).resolve() != root.resolve():
        raise BenchmarkBlockedError("CANONICAL_GIT_ROOT_UNAVAILABLE")
    return root


def discover_worktree_roots(canonical_root: Path) -> tuple[Path, ...]:
    try:
        text = subprocess.check_output(
            ["git", "-C", str(canonical_root), "worktree", "list", "--porcelain"],
            text=True,
            stderr=subprocess.DEVNULL,
        )
    except (OSError, subprocess.CalledProcessError) as error:
        raise BenchmarkBlockedError("GIT_WORKTREE_DISCOVERY_UNAVAILABLE") from error
    roots = _parse_worktree_porcelain(text)
    if not roots or canonical_root.resolve() not in roots:
        raise BenchmarkBlockedError("GIT_WORKTREE_DISCOVERY_UNAVAILABLE")
    return roots


def _parse_worktree_porcelain(text: str) -> tuple[Path, ...]:
    """Read only explicit ``worktree <path>`` records; spaces are part of the path."""
    return tuple(Path(line[len("worktree ") :]).resolve() for line in text.splitlines() if line.startswith("worktree "))


def _require_external_path(path: Path, worktree_roots: Sequence[Path], *, label: str) -> Path:
    if not path.is_absolute():
        raise BenchmarkBlockedError(f"{label.upper()}_PATH_NOT_ABSOLUTE")
    try:
        resolved = path.resolve(strict=False)
    except OSError as error:
        raise BenchmarkBlockedError(f"{label.upper()}_PATH_INVALID") from error
    for root in worktree_roots:
        try:
            resolved.relative_to(root.resolve())
        except ValueError:
            continue
        raise BenchmarkBlockedError(f"{label.upper()}_MUST_BE_EXTERNAL")
    return resolved


def _snapshot_regular_file(path: Path, *, label: str) -> bytes:
    try:
        before = os.lstat(path)
    except OSError as error:
        raise BenchmarkBlockedError(f"{label.upper()}_UNREADABLE") from error
    if stat.S_ISLNK(before.st_mode) or not stat.S_ISREG(before.st_mode):
        raise BenchmarkBlockedError(f"{label.upper()}_NOT_REGULAR_FILE")
    flags = os.O_RDONLY
    if hasattr(os, "O_CLOEXEC"):
        flags |= os.O_CLOEXEC
    if hasattr(os, "O_NOFOLLOW"):
        flags |= os.O_NOFOLLOW
    try:
        descriptor = os.open(path, flags)
    except OSError as error:
        raise BenchmarkBlockedError(f"{label.upper()}_UNREADABLE") from error
    try:
        opened = os.fstat(descriptor)
        if not stat.S_ISREG(opened.st_mode):
            raise BenchmarkBlockedError(f"{label.upper()}_NOT_REGULAR_FILE")
        pieces: list[bytes] = []
        while True:
            piece = os.read(descriptor, 1024 * 1024)
            if not piece:
                break
            pieces.append(piece)
        snapshot = b"".join(pieces)
    finally:
        os.close(descriptor)
    if not snapshot:
        raise BenchmarkBlockedError(f"{label.upper()}_EMPTY")
    return snapshot


def _sha256(snapshot: bytes) -> str:
    return hashlib.sha256(snapshot).hexdigest()


def _snapshot_decoded_pixels(decoded: np.ndarray) -> tuple[bytes, tuple[int, int, int], str]:
    """Detach decoded pixels into immutable bytes and immutable geometry metadata."""
    contiguous = np.ascontiguousarray(decoded)
    pixel_bytes = contiguous.tobytes()
    shape = tuple(int(dimension) for dimension in contiguous.shape)
    if len(shape) != 3 or contiguous.dtype != np.uint8:
        raise BenchmarkBlockedError("DATASET_IMAGE_NOT_IMMUTABLE")
    return pixel_bytes, shape, contiguous.dtype.name


def load_verified_onnx_net(
    artifact_path: Path,
    manifest: BenchmarkManifest,
    *,
    worktree_roots: Sequence[Path],
    loader: OnnxLoader = cv2.dnn.readNetFromONNX,
) -> tuple[Any, float]:
    """Load only the byte snapshot that was just verified, never a filesystem path."""
    if not artifact_path.is_absolute():
        raise BenchmarkBlockedError("MODEL_ARTIFACT_PATH_NOT_ABSOLUTE")
    _require_external_path(artifact_path, worktree_roots, label="model_artifact")
    if artifact_path.name != manifest.artifact_filename:
        raise BenchmarkBlockedError("MODEL_FILENAME_MISMATCH")
    snapshot = _snapshot_regular_file(artifact_path, label="model_artifact")
    if not hmac.compare_digest(_sha256(snapshot), manifest.artifact_sha256):
        raise BenchmarkBlockedError("MODEL_SHA256_MISMATCH")
    buffer = np.frombuffer(snapshot, dtype=np.uint8)
    started_ns = perf_counter_ns()
    try:
        net = loader(buffer)
    except (cv2.error, TypeError, ValueError) as error:
        raise BenchmarkBlockedError("ONNX_BUFFER_LOADER_UNSUPPORTED") from error
    except Exception as error:
        raise BenchmarkBlockedError("ONNX_BUFFER_LOADER_FAILED") from error
    load_ms = (perf_counter_ns() - started_ns) / 1_000_000
    try:
        net.setPreferableBackend(cv2.dnn.DNN_BACKEND_OPENCV)
        net.setPreferableTarget(cv2.dnn.DNN_TARGET_CPU)
    except Exception as error:
        raise BenchmarkBlockedError("ONNX_NET_CONFIGURATION_FAILED") from error
    return net, load_ms


def build_dataset_snapshot(
    directory: Path,
    *,
    worktree_roots: Sequence[Path],
    decoder: ImageDecoder = cv2.imdecode,
) -> DatasetSnapshot:
    resolved = _require_external_path(directory, worktree_roots, label="dataset")
    try:
        original_directory_stat = os.lstat(directory)
    except OSError as error:
        raise BenchmarkBlockedError("DATASET_UNREADABLE") from error
    if stat.S_ISLNK(original_directory_stat.st_mode):
        raise BenchmarkBlockedError("DATASET_NOT_DIRECTORY")
    try:
        directory_stat = os.lstat(resolved)
        entries = list(resolved.iterdir())
    except OSError as error:
        raise BenchmarkBlockedError("DATASET_UNREADABLE") from error
    if stat.S_ISLNK(directory_stat.st_mode) or not stat.S_ISDIR(directory_stat.st_mode):
        raise BenchmarkBlockedError("DATASET_NOT_DIRECTORY")
    supported: list[Path] = []
    for entry in entries:
        try:
            entry_stat = os.lstat(entry)
        except OSError as error:
            raise BenchmarkBlockedError("DATASET_UNREADABLE") from error
        if stat.S_ISLNK(entry_stat.st_mode):
            raise BenchmarkBlockedError("DATASET_SYMLINK_ENTRY")
        if not stat.S_ISREG(entry_stat.st_mode):
            raise BenchmarkBlockedError("DATASET_NON_REGULAR_ENTRY")
        if entry.suffix.lower() in _IMAGE_SUFFIXES:
            supported.append(entry)
    if not supported:
        raise BenchmarkBlockedError("DATASET_EMPTY")
    frames: list[DatasetFrame] = []
    for entry in sorted(supported, key=lambda value: value.name):
        snapshot = _snapshot_regular_file(entry, label="dataset_image")
        decoded = decoder(np.frombuffer(snapshot, dtype=np.uint8), cv2.IMREAD_COLOR)
        if (
            not isinstance(decoded, np.ndarray)
            or decoded.dtype != np.uint8
            or decoded.ndim != 3
            or decoded.shape[0] <= 0
            or decoded.shape[1] <= 0
            or decoded.shape[2] != 3
        ):
            raise BenchmarkBlockedError("DATASET_IMAGE_DECODE_FAILED")
        pixel_bytes, image_shape, image_dtype = _snapshot_decoded_pixels(decoded)
        frames.append(DatasetFrame(str(entry), _sha256(snapshot), pixel_bytes, image_shape, image_dtype))
    return DatasetSnapshot(directory=str(resolved), frames=tuple(frames))


def classify_person_detections(detections: Sequence[Detection]) -> str:
    if len(detections) == 0:
        return DetectionStatus.PERSON_LOST
    if len(detections) == 1:
        return DetectionStatus.SINGLE_PERSON
    return DetectionStatus.MULTIPLE_PERSONS


def validate_final_detections(
    detections: object, *, manifest: BenchmarkManifest, image_width: int, image_height: int
) -> tuple[Detection, ...]:
    if not isinstance(detections, Sequence) or isinstance(detections, (str, bytes, bytearray)):
        raise BenchmarkBlockedError("INVALID_ADAPTER_RESULT")
    checked: list[Detection] = []
    for detection in detections:
        if not isinstance(detection, Detection):
            raise BenchmarkBlockedError("INVALID_FINAL_DETECTION")
        if not isinstance(detection.bbox, BoundingBox):
            raise BenchmarkBlockedError("INVALID_FINAL_DETECTION")
        if type(detection.confidence) not in (int, float) or not math.isfinite(detection.confidence):
            raise BenchmarkBlockedError("INVALID_FINAL_DETECTION")
        if not 0.0 <= detection.confidence <= 1.0:
            raise BenchmarkBlockedError("INVALID_FINAL_DETECTION")
        if type(detection.class_id) is not int or detection.class_id != manifest.person_class_id:
            raise BenchmarkBlockedError("INVALID_FINAL_DETECTION")
        if type(detection.label) is not str or detection.label != "person":
            raise BenchmarkBlockedError("INVALID_FINAL_DETECTION")
        detection.bbox.validate(image_width, image_height)
        checked.append(detection)
    return tuple(checked)


def median_and_p95_ms(samples_ms: Sequence[float]) -> tuple[float, float]:
    if not samples_ms or any(type(value) not in (int, float) or not math.isfinite(value) or value < 0 for value in samples_ms):
        raise ValueError("latency samples must be finite non-negative numbers")
    ordered = sorted(float(value) for value in samples_ms)
    middle = len(ordered) // 2
    median = ordered[middle] if len(ordered) % 2 else (ordered[middle - 1] + ordered[middle]) / 2
    return median, ordered[math.ceil(len(ordered) * 0.95) - 1]


def _cpu_model() -> tuple[str, str]:
    try:
        with open("/proc/cpuinfo", encoding="utf-8") as handle:
            for line in handle:
                if line.lower().startswith("model name") and ":" in line:
                    value = line.split(":", 1)[1].strip()
                    if value:
                        return value, "/proc/cpuinfo:model name"
    except OSError:
        pass
    return platform.processor() or "unknown", "platform.processor"


def _git_commit(repository_root: Path) -> str:
    try:
        return subprocess.check_output(
            ["git", "-C", str(repository_root), "rev-parse", "HEAD"], text=True, stderr=subprocess.DEVNULL
        ).strip()
    except (OSError, subprocess.CalledProcessError) as error:
        raise BenchmarkBlockedError("CANONICAL_GIT_COMMIT_UNAVAILABLE") from error


def _run_one(
    *,
    net: Any,
    adapter: BenchmarkAdapter,
    frame: DatasetFrame,
    manifest: BenchmarkManifest,
    clock_ns: Callable[[], int] = perf_counter_ns,
) -> tuple[float, float, str]:
    started_ns = clock_ns()
    image = frame.new_image_view()
    try:
        tensor = adapter.preprocess(image, manifest)
        if not isinstance(tensor, np.ndarray):
            raise BenchmarkBlockedError("INVALID_ADAPTER_RESULT")
        net.setInput(tensor)
        forward_started_ns = clock_ns()
        output = net.forward()
        forward_ms = (clock_ns() - forward_started_ns) / 1_000_000
        candidates = adapter.postprocess(
            output, manifest, image_width=frame.image_width, image_height=frame.image_height
        )
        detections = validate_final_detections(
            candidates, manifest=manifest, image_width=frame.image_width, image_height=frame.image_height
        )
    except BenchmarkBlockedError:
        raise
    except Exception as error:
        raise BenchmarkBlockedError("DETECTOR_ITERATION_FAILED") from error
    status = classify_person_detections(detections)
    pipeline_ms = (clock_ns() - started_ns) / 1_000_000
    return forward_ms, pipeline_ms, status


def run_offline_benchmark(
    *,
    artifact_path: Path,
    manifest_path: Path,
    image_directory: Path,
    warmup_count: int,
    opencv_threads: int,
    approval_resolver: ApprovalResolver = production_approval_resolver,
    adapter_resolver: AdapterResolver = production_adapter_resolver,
    root_discovery: RootDiscovery = discover_worktree_roots,
    canonical_root_provider: Callable[[], Path] = canonical_repository_root,
    loader: OnnxLoader = cv2.dnn.readNetFromONNX,
    decoder: ImageDecoder = cv2.imdecode,
    clock_ns: Callable[[], int] = perf_counter_ns,
) -> dict[str, object]:
    """Benchmark one full validated dataset pass after successful warm-up."""
    if type(warmup_count) is not int or warmup_count < 0 or type(opencv_threads) is not int or opencv_threads <= 0:
        raise ValueError("warmup_count must be non-negative and opencv_threads positive integers")
    root = canonical_root_provider()
    roots = root_discovery(root)
    _require_external_path(manifest_path, roots, label="manifest")
    manifest = parse_manifest(manifest_path)
    approval = require_approved_manifest(manifest, approval_resolver)
    adapter = adapter_resolver(approval)
    if adapter is None or adapter.adapter_id != approval.adapter_id or adapter.adapter_version != approval.adapter_version:
        raise BenchmarkBlockedError("ADAPTER_NOT_APPROVED")
    _require_external_path(artifact_path, roots, label="model_artifact")
    snapshot = build_dataset_snapshot(image_directory, worktree_roots=roots, decoder=decoder)
    cv2.setNumThreads(opencv_threads)
    actual_threads = cv2.getNumThreads()
    if actual_threads != opencv_threads:
        raise BenchmarkBlockedError("OPENCV_THREAD_CONFIGURATION_MISMATCH")
    net, model_load_ms = load_verified_onnx_net(
        artifact_path, manifest, worktree_roots=roots, loader=loader
    )
    warmup_completed = 0
    for _ in range(warmup_count):
        _run_one(net=net, adapter=adapter, frame=snapshot.frames[0], manifest=manifest, clock_ns=clock_ns)
        warmup_completed += 1
    forward_samples: list[float] = []
    detector_pipeline_samples: list[float] = []
    statuses = {status: 0 for status in (DetectionStatus.PERSON_LOST, DetectionStatus.SINGLE_PERSON, DetectionStatus.MULTIPLE_PERSONS)}
    measured_completed = 0
    for frame in snapshot.frames:
        forward_ms, pipeline_ms, status = _run_one(
            net=net, adapter=adapter, frame=frame, manifest=manifest, clock_ns=clock_ns
        )
        forward_samples.append(forward_ms)
        detector_pipeline_samples.append(pipeline_ms)
        statuses[status] += 1
        measured_completed += 1
    forward_median, forward_p95 = median_and_p95_ms(forward_samples)
    pipeline_median, pipeline_p95 = median_and_p95_ms(detector_pipeline_samples)
    cpu_model, cpu_model_source = _cpu_model()
    return {
        "schema_version": REPORT_SCHEMA_VERSION,
        "timestamp_utc": datetime.now(timezone.utc).isoformat(),
        "git_commit": _git_commit(root),
        "environment": {
            "opencv_threads_requested": opencv_threads,
            "opencv_threads_actual": actual_threads,
            "opencv_version": cv2.__version__,
            "python_version": sys.version,
            "platform": platform.platform(),
            "cpu_model": cpu_model,
            "cpu_model_source": cpu_model_source,
            "dnn_backend": cv2.dnn.DNN_BACKEND_OPENCV,
            "dnn_target": cv2.dnn.DNN_TARGET_CPU,
        },
        "model": {**manifest_report_metadata(manifest), "artifact_path": str(artifact_path)},
        "dataset": {
            "path": snapshot.directory,
            "input_files": [{"path": frame.path, "sha256": frame.encoded_sha256} for frame in snapshot.frames],
        },
        "iterations_semantics": "one measured detector inference per validated dataset image",
        "warmup_iterations_requested": warmup_count,
        "warmup_iterations_completed": warmup_completed,
        "measured_iterations_requested": len(snapshot.frames),
        "measured_samples_completed": measured_completed,
        "latency_unit": "ms",
        "p95_method": "nearest_rank",
        "model_load_latency_ms": model_load_ms,
        "forward_latency_ms": {"median": forward_median, "p95": forward_p95},
        "detector_pipeline_latency_ms": {"median": pipeline_median, "p95": pipeline_p95},
        "status_counts": statuses,
        "pipeline_boundary": "preprocess, setInput, forward, adapter postprocess, validation, and status; excludes dataset snapshot and decode",
    }


def benchmark_report_json(report: Mapping[str, object]) -> str:
    return json.dumps(report, allow_nan=False, sort_keys=True, separators=(",", ":"))


def _directory_open_flags() -> int:
    flags = os.O_RDONLY
    for name in ("O_DIRECTORY", "O_CLOEXEC", "O_NOFOLLOW"):
        flags |= getattr(os, name, 0)
    return flags


def _temporary_open_flags() -> int:
    flags = os.O_CREAT | os.O_EXCL | os.O_WRONLY
    for name in ("O_CLOEXEC", "O_NOFOLLOW"):
        flags |= getattr(os, name, 0)
    return flags


def _open_report_directory(report_path: Path, worktree_roots: Sequence[Path]) -> tuple[int, str]:
    _require_external_path(report_path, worktree_roots, label="report")
    target_name = report_path.name
    if target_name in {"", ".", ".."} or os.sep in target_name or (os.altsep and os.altsep in target_name):
        raise BenchmarkBlockedError("REPORT_TARGET_INVALID")
    try:
        parent_before = os.lstat(report_path.parent)
    except OSError as error:
        raise BenchmarkBlockedError("REPORT_DIRECTORY_UNREADABLE") from error
    if stat.S_ISLNK(parent_before.st_mode):
        raise BenchmarkBlockedError("REPORT_PATH_SYMLINK")
    try:
        directory_fd = os.open(report_path.parent, _directory_open_flags())
    except OSError as error:
        raise BenchmarkBlockedError("REPORT_DIRECTORY_UNREADABLE") from error
    try:
        directory_stat = os.fstat(directory_fd)
        if not stat.S_ISDIR(directory_stat.st_mode):
            raise BenchmarkBlockedError("REPORT_DIRECTORY_INVALID")
    except Exception:
        os.close(directory_fd)
        raise
    return directory_fd, target_name


def _entry_lstat(directory_fd: int, name: str) -> os.stat_result | None:
    try:
        return os.stat(name, dir_fd=directory_fd, follow_symlinks=False)
    except FileNotFoundError:
        return None
    except OSError as error:
        raise BenchmarkBlockedError("REPORT_TARGET_UNREADABLE") from error


def _create_report_temporary(directory_fd: int, target_name: str) -> tuple[str, int]:
    for _ in range(128):
        temporary_name = f".{target_name}.{secrets.token_hex(16)}.tmp"
        try:
            temporary_fd = os.open(temporary_name, _temporary_open_flags(), 0o600, dir_fd=directory_fd)
        except FileExistsError:
            continue
        except OSError as error:
            raise BenchmarkBlockedError("REPORT_TEMPORARY_CREATE_FAILED") from error
        try:
            temporary_stat = os.fstat(temporary_fd)
            if not stat.S_ISREG(temporary_stat.st_mode):
                raise BenchmarkBlockedError("REPORT_TEMPORARY_NOT_REGULAR")
        except Exception:
            os.close(temporary_fd)
            try:
                os.unlink(temporary_name, dir_fd=directory_fd)
            except OSError:
                pass
            raise
        return temporary_name, temporary_fd
    raise BenchmarkBlockedError("REPORT_TEMPORARY_NAME_EXHAUSTED")


def _write_and_fsync_report(temporary_fd: int, payload: bytes) -> None:
    """Write through a buffered handle so flush and fsync are both explicit."""
    duplicate_fd = os.dup(temporary_fd)
    with os.fdopen(duplicate_fd, "wb", closefd=True) as handle:
        handle.write(payload)
        handle.flush()
        os.fsync(handle.fileno())


def write_report_atomic(
    report_path: Path,
    report: Mapping[str, object],
    *,
    worktree_roots: Sequence[Path],
    overwrite: bool = False,
) -> ReportPublicationResult:
    payload = (benchmark_report_json(report) + "\n").encode("utf-8")
    directory_fd: int | None = None
    temporary_fd: int | None = None
    temporary_name: str | None = None
    publication_result: ReportPublicationResult | None = None
    try:
        directory_fd, target_name = _open_report_directory(report_path, worktree_roots)
        existing_target = _entry_lstat(directory_fd, target_name)
        if existing_target is not None:
            if stat.S_ISLNK(existing_target.st_mode):
                raise BenchmarkBlockedError("REPORT_PATH_SYMLINK")
            if not stat.S_ISREG(existing_target.st_mode):
                raise BenchmarkBlockedError("REPORT_TARGET_NOT_REGULAR")
            if not overwrite:
                raise BenchmarkBlockedError("REPORT_EXISTS_USE_OVERWRITE")
        temporary_name, temporary_fd = _create_report_temporary(directory_fd, target_name)
        _write_and_fsync_report(temporary_fd, payload)
        os.close(temporary_fd)
        temporary_fd = None
        if overwrite:
            try:
                os.replace(temporary_name, target_name, src_dir_fd=directory_fd, dst_dir_fd=directory_fd)
            except OSError as error:
                raise BenchmarkBlockedError("REPORT_REPLACE_FAILED") from error
            temporary_name = None
        else:
            try:
                os.link(
                    temporary_name,
                    target_name,
                    src_dir_fd=directory_fd,
                    dst_dir_fd=directory_fd,
                    follow_symlinks=False,
                )
            except FileExistsError as error:
                raise BenchmarkBlockedError("REPORT_EXISTS_USE_OVERWRITE") from error
            except OSError as error:
                if error.errno == errno.EEXIST:
                    raise BenchmarkBlockedError("REPORT_EXISTS_USE_OVERWRITE") from error
                raise BenchmarkBlockedError("REPORT_SAFE_PUBLISH_FAILED") from error
            try:
                os.unlink(temporary_name, dir_fd=directory_fd)
            except OSError:
                pass
            else:
                temporary_name = None
        publication_result = ReportPublicationResult(
            target_path=str(report_path),
            overwrite=overwrite,
            published=True,
            durability_confirmed=True,
        )
        try:
            os.fsync(directory_fd)
        except OSError as error:
            uncertain = ReportPublicationResult(
                target_path=publication_result.target_path,
                overwrite=publication_result.overwrite,
                published=True,
                durability_confirmed=False,
            )
            raise ReportPublishedDurabilityUncertainError(uncertain, error) from error
        return publication_result
    except BenchmarkBlockedError:
        raise
    except (OSError, TypeError, ValueError) as error:
        raise BenchmarkBlockedError("REPORT_WRITE_FAILED") from error
    finally:
        if temporary_fd is not None:
            try:
                os.close(temporary_fd)
            except OSError:
                pass
        if temporary_name is not None and directory_fd is not None:
            try:
                os.unlink(temporary_name, dir_fd=directory_fd)
            except OSError:
                pass
        if directory_fd is not None:
            try:
                os.close(directory_fd)
            except OSError:
                pass
