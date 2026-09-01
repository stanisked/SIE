from __future__ import annotations

import ctypes
import hashlib
import json
import os
from datetime import datetime, timedelta, timezone
from itertools import permutations
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch

import cv2
import numpy as np
import pytest

import vision_core.person_localization.ar0234 as ar0234_module
import vision_core.tools.run_person_localization_ar0234 as person_runner_module
from vision_core.person_localization.ar0234 import (
    AR0234_BY_ID,
    AR0234Capture,
    AR0234CaptureConfig,
    V4L2Capabilities,
    V4L2_CAP_DEVICE_CAPS,
    V4L2_CAP_STREAMING,
    V4L2_CAP_VIDEO_CAPTURE,
    V4L2_CAP_VIDEO_CAPTURE_MPLANE,
    check_v4l2_capture_capability,
    query_v4l2_capabilities,
    resolve_ar0234_device,
)
from vision_core.person_localization.detector import (
    ARTIFACT_SCHEMA_VERSION,
    CONFIDENCE_SEMANTICS,
    OUTPUT_CONTRACT_VERSION,
    DetectorArtifact,
    verify_artifact_bytes,
)
from vision_core.person_localization.models import BoundingBox, PersonDetection
from vision_core.person_localization.pipeline import (
    AR0234_OPTICAL_FRAME,
    PersonLocalizationPipeline,
    PersonLocalizationPolicy,
)


NOW = datetime(2026, 8, 31, 12, 0, 0, tzinfo=timezone.utc)
REPOSITORY_ROOT = Path(__file__).resolve().parents[2]


def make_artifact(**overrides: object) -> DetectorArtifact:
    values: dict[str, object] = {
        "schema_version": ARTIFACT_SCHEMA_VERSION,
        "backend_id": "test_backend",
        "backend_version": "1.0.0",
        "model_id": "test_person_model",
        "model_version": "1.0.0",
        "artifact_sha256": "a" * 64,
        "output_contract_version": OUTPUT_CONTRACT_VERSION,
        "person_label": "person",
        "confidence_semantics": CONFIDENCE_SEMANTICS,
        "confidence_threshold": 0.5,
        "inference_parameters": {"score_threshold": 0.5, "nested": [True, None]},
    }
    values.update(overrides)
    return DetectorArtifact(**values)  # type: ignore[arg-type]


ARTIFACT = make_artifact()
_DEFAULT_FRAME = object()


class FakeDetector:
    def __init__(self, candidates: object, artifact: DetectorArtifact = ARTIFACT) -> None:
        self.candidates = candidates
        self.artifact = artifact
        self.calls = 0

    def detect(self, _frame: np.ndarray) -> object:
        self.calls += 1
        return self.candidates


class RaisingDetector(FakeDetector):
    def __init__(self, error: BaseException) -> None:
        super().__init__([])
        self.error = error

    def detect(self, _frame: np.ndarray) -> object:
        self.calls += 1
        raise self.error


class ArtifactChangingDetector:
    """Backend double that changes or fails when its artifact is read post-detect."""

    def __init__(
        self,
        replacement: object,
        *,
        property_error: BaseException | None = None,
    ) -> None:
        self._initial_artifact = ARTIFACT
        self._replacement = replacement
        self._property_error = property_error
        self._detected = False
        self.calls = 0

    @property
    def artifact(self) -> object:
        if self._detected and self._property_error is not None:
            raise self._property_error
        if self._detected:
            return self._replacement
        return self._initial_artifact

    def detect(self, _frame: np.ndarray) -> object:
        self.calls += 1
        self._detected = True
        return [PersonDetection(BoundingBox(1, 1, 4, 4), 0.9)]


class MetadataTrapArtifact:
    person_label = "person"
    confidence_threshold = 0.5

    def __init__(self) -> None:
        self.metadata_calls = 0

    def metadata(self) -> dict[str, object]:
        self.metadata_calls += 1
        return {"invalid": np.array([1])}


class FakeCapture:
    def __init__(
        self,
        *,
        opened: bool = True,
        set_result: bool = True,
        frame: object = _DEFAULT_FRAME,
        read_ok: bool = True,
        read_error: Exception | None = None,
        get_error: Exception | None = None,
    ) -> None:
        self.opened = opened
        self.set_result = set_result
        self.frame = (
            np.zeros((1200, 1920, 3), dtype=np.uint8) if frame is _DEFAULT_FRAME else frame
        )
        self.read_ok = read_ok
        self.read_error = read_error
        self.get_error = get_error
        self.release_calls = 0

    def isOpened(self) -> bool:
        return self.opened

    def set(self, _property_id: int, _value: float) -> bool:
        return self.set_result

    def get(self, property_id: int) -> float:
        if self.get_error is not None:
            raise self.get_error
        return {
            cv2.CAP_PROP_FRAME_WIDTH: 1920.0,
            cv2.CAP_PROP_FRAME_HEIGHT: 1200.0,
            cv2.CAP_PROP_FPS: 60.0,
        }[property_id]

    def read(self) -> tuple[bool, object]:
        if self.read_error is not None:
            raise self.read_error
        return self.read_ok, self.frame

    def release(self) -> None:
        self.release_calls += 1


def make_pipeline(
    candidates: object,
    *,
    artifact: DetectorArtifact = ARTIFACT,
    now_utc: datetime = NOW,
    **policy_kwargs: float,
) -> tuple[PersonLocalizationPipeline, FakeDetector]:
    detector = FakeDetector(candidates, artifact)
    return (
        PersonLocalizationPipeline(
            detector,
            PersonLocalizationPolicy(**policy_kwargs),
            now_utc=lambda: now_utc,
        ),
        detector,
    )


def process(candidates: object, **policy_kwargs: float):
    pipeline, detector = make_pipeline(candidates, **policy_kwargs)
    result = pipeline.process(
        np.zeros((100, 200, 3), dtype=np.uint8), captured_at_utc=NOW, cycle_id="42"
    )
    return result, detector


def make_open_capture(fake_capture: FakeCapture) -> AR0234Capture:
    return AR0234Capture(
        capture_factory=lambda *_args: fake_capture,
        device_resolver=lambda _device: Path("/dev/video99"),
    )


def test_detector_artifact_is_versioned_immutable_and_json_safe() -> None:
    parameters = {"score_threshold": 0.5, "nested": ["x", 1]}
    artifact = make_artifact(inference_parameters=parameters)
    parameters["score_threshold"] = 0.9
    parameters["nested"].append("changed")

    assert artifact.inference_parameters["score_threshold"] == 0.5
    assert artifact.inference_parameters["nested"] == ("x", 1)
    with pytest.raises(TypeError):
        artifact.inference_parameters["score_threshold"] = 0.1  # type: ignore[index]
    metadata = artifact.metadata()
    assert json.loads(json.dumps(metadata, allow_nan=False)) == metadata
    assert metadata["schema_version"] == ARTIFACT_SCHEMA_VERSION
    assert metadata["output_contract_version"] == OUTPUT_CONTRACT_VERSION
    assert metadata["confidence_semantics"] == CONFIDENCE_SEMANTICS


def test_verify_artifact_bytes_hashes_exact_bytes_without_creating_verified_state(tmp_path: Path) -> None:
    artifact_path = tmp_path / "detector.bin"
    artifact_bytes = b"person-detector-artifact-v1\x00"
    artifact_path.write_bytes(artifact_bytes)
    expected_sha256 = hashlib.sha256(artifact_bytes).hexdigest()

    assert verify_artifact_bytes(artifact_path, expected_sha256) is True
    assert verify_artifact_bytes(artifact_path, "0" * 64) is False
    artifact_path.write_bytes(artifact_bytes[:-1] + b"1")
    assert verify_artifact_bytes(artifact_path, expected_sha256) is False
    assert artifact_bytes == b"person-detector-artifact-v1\x00"
    with pytest.raises(ValueError, match="lowercase hexadecimal"):
        verify_artifact_bytes(artifact_path, "not-a-sha256")


def test_v4l2_uapi_layout_and_request_match_independent_linux_literals() -> None:
    assert ctypes.sizeof(ar0234_module._V4L2CapabilityLayout) == 104
    assert {
        name: getattr(ar0234_module._V4L2CapabilityLayout, name).offset
        for name, _field_type in ar0234_module._V4L2CapabilityLayout._fields_
    } == {
        "driver": 0,
        "card": 16,
        "bus_info": 48,
        "version": 80,
        "capabilities": 84,
        "device_caps": 88,
        "reserved": 92,
    }
    assert ar0234_module._vidioc_querycap_request() == 0x80685600
    assert ar0234_module.V4L2_CAP_VIDEO_CAPTURE == 0x00000001
    assert ar0234_module.V4L2_CAP_VIDEO_CAPTURE_MPLANE == 0x00001000
    assert ar0234_module.V4L2_CAP_STREAMING == 0x04000000
    assert ar0234_module.V4L2_CAP_DEVICE_CAPS == 0x80000000


@pytest.mark.parametrize(
    ("field", "value"),
    [
        ("backend_id", ""),
        ("backend_id", " backend"),
        ("backend_version", "1.0\n0"),
        ("model_id", "model name"),
        ("model_version", "version 1"),
        ("person_label", "person\tlabel"),
        ("artifact_sha256", "A" * 64),
        ("artifact_sha256", "a" * 63),
        ("schema_version", "other"),
        ("output_contract_version", "other"),
        ("confidence_semantics", "unknown"),
        ("confidence_threshold", float("nan")),
        ("confidence_threshold", 1.01),
    ],
)
def test_detector_artifact_rejects_invalid_required_values(field: str, value: object) -> None:
    with pytest.raises(ValueError):
        make_artifact(**{field: value})


@pytest.mark.parametrize(
    "parameters",
    [
        {"array": np.array([1])},
        {"scalar": np.float64(1.0)},
        {"path": Path("model.onnx")},
        {"bytes": b"model"},
        {"set": {"value"}},
        {"nan": float("nan")},
        {1: "wrong-key"},
        ("not", "a mapping"),
    ],
)
def test_detector_artifact_rejects_non_json_inference_parameters(parameters: object) -> None:
    with pytest.raises(ValueError):
        make_artifact(inference_parameters=parameters)


def test_ar0234_accepts_only_approved_stable_identity() -> None:
    assert str(AR0234_BY_ID).endswith("DECXIN_CAMERA_01.00.00-video-index0")
    with pytest.raises(ValueError, match="does not permit"):
        AR0234CaptureConfig(device=Path("/dev/video4"))


def test_production_resolver_calls_capability_checker_without_real_device() -> None:
    target = Path("/dev/video99")
    checker_calls: list[Path] = []
    with (
        patch.object(Path, "is_symlink", return_value=True),
        patch.object(Path, "resolve", return_value=target),
        patch.object(Path, "stat", return_value=SimpleNamespace(st_mode=0o020000)),
    ):
        assert resolve_ar0234_device(capability_checker=checker_calls.append) == target

    assert checker_calls == [target]


@pytest.mark.parametrize(
    "capabilities",
    [
        V4L2_CAP_VIDEO_CAPTURE | V4L2_CAP_STREAMING,
        V4L2_CAP_VIDEO_CAPTURE_MPLANE | V4L2_CAP_STREAMING,
    ],
)
def test_v4l2_capture_capability_accepts_capture_streaming(capabilities: int) -> None:
    check_v4l2_capture_capability(
        Path("/dev/video99"),
        query_capabilities=lambda _device: V4L2Capabilities(capabilities, 0),
    )


def test_v4l2_device_caps_override_global_capabilities() -> None:
    check_v4l2_capture_capability(
        Path("/dev/video99"),
        query_capabilities=lambda _device: V4L2Capabilities(
            V4L2_CAP_DEVICE_CAPS,
            V4L2_CAP_VIDEO_CAPTURE | V4L2_CAP_STREAMING,
        ),
    )


@pytest.mark.parametrize(
    "capabilities",
    [
        0x00800000 | V4L2_CAP_STREAMING,
        0x00000002 | V4L2_CAP_STREAMING,
        V4L2_CAP_VIDEO_CAPTURE,
    ],
)
def test_v4l2_capture_capability_rejects_metadata_output_or_nonstreaming(
    capabilities: int,
) -> None:
    with pytest.raises(RuntimeError):
        check_v4l2_capture_capability(
            Path("/dev/video99"),
            query_capabilities=lambda _device: V4L2Capabilities(capabilities, 0),
        )


def test_v4l2_query_closes_descriptor_on_success_and_ioctl_failure() -> None:
    closed: list[int] = []

    def ioctl_success(_fd: int, _request: int, buffer: bytearray, _mutate: bool) -> int:
        parsed = ar0234_module._V4L2CapabilityLayout.from_buffer(buffer)
        parsed.capabilities = V4L2_CAP_VIDEO_CAPTURE | V4L2_CAP_STREAMING
        return 0

    result = query_v4l2_capabilities(
        Path("/dev/video99"),
        open_fn=lambda _path, _flags: 31,
        ioctl_fn=ioctl_success,
        close_fn=closed.append,
    )
    assert result.effective == V4L2_CAP_VIDEO_CAPTURE | V4L2_CAP_STREAMING
    assert closed == [31]

    with pytest.raises(RuntimeError, match="capability query failed"):
        query_v4l2_capabilities(
            Path("/dev/video99"),
            open_fn=lambda _path, _flags: 32,
            ioctl_fn=lambda *_args: (_ for _ in ()).throw(OSError("ioctl failed")),
            close_fn=closed.append,
        )
    assert closed == [31, 32]

    with pytest.raises(RuntimeError, match="capability query failed"):
        query_v4l2_capabilities(
            Path("/dev/video99"),
            open_fn=lambda _path, _flags: (_ for _ in ()).throw(OSError("open failed")),
            ioctl_fn=ioctl_success,
            close_fn=closed.append,
        )
    assert closed == [31, 32]


def test_capture_open_failure_releases_exactly_once() -> None:
    capture = FakeCapture(set_result=False)
    adapter = make_open_capture(capture)

    with pytest.raises(RuntimeError, match="rejected required capture property"):
        adapter.open()
    adapter.close()

    assert capture.release_calls == 1


def test_capture_get_failure_releases_exactly_once() -> None:
    capture = FakeCapture(get_error=RuntimeError("get failed"))
    adapter = make_open_capture(capture)

    with pytest.raises(RuntimeError, match="get failed"):
        adapter.open()
    adapter.close()

    assert capture.release_calls == 1


def test_capture_mode_validation_failure_releases_exactly_once() -> None:
    class ModeMismatchCapture(FakeCapture):
        def get(self, property_id: int) -> float:
            if property_id == cv2.CAP_PROP_FRAME_WIDTH:
                return 1.0
            return super().get(property_id)

    capture = ModeMismatchCapture()
    adapter = make_open_capture(capture)

    with pytest.raises(RuntimeError, match="mode mismatch"):
        adapter.open()
    adapter.close()

    assert capture.release_calls == 1


@pytest.mark.parametrize(
    "frame,read_ok,read_error",
    [
        (None, True, RuntimeError("read failed")),
        (None, False, None),
        (None, True, None),
        (np.zeros((0, 1920, 3), dtype=np.uint8), True, None),
        (np.zeros((1200, 1920, 1), dtype=np.uint8), True, None),
        (np.zeros((1200, 1920, 3), dtype=np.float32), True, None),
    ],
)
def test_capture_read_or_validation_failure_closes_exactly_once(
    frame: object | None,
    read_ok: bool,
    read_error: Exception | None,
) -> None:
    capture = FakeCapture(frame=frame, read_ok=read_ok, read_error=read_error)
    adapter = make_open_capture(capture)
    adapter.open()

    with pytest.raises(RuntimeError):
        adapter.read()
    adapter.close()

    assert capture.release_calls == 1


def test_capture_context_manager_does_not_double_release_after_read_failure() -> None:
    capture = FakeCapture(frame=np.zeros((1200, 1920, 1), dtype=np.uint8))
    adapter = make_open_capture(capture)

    with pytest.raises(RuntimeError):
        with adapter:
            adapter.read()

    assert capture.release_calls == 1


def test_runner_help_and_wrong_device_do_not_open_camera() -> None:
    import subprocess
    import sys

    environment = {**os.environ, "PYTHONPATH": str(REPOSITORY_ROOT)}
    command = [sys.executable, "vision_core/tools/run_person_localization_ar0234.py"]
    help_result = subprocess.run(
        [*command, "--help"],
        cwd=REPOSITORY_ROOT,
        env=environment,
        capture_output=True,
        check=False,
        text=True,
    )
    wrong_device_result = subprocess.run(
        [*command, "--device", "/dev/video4", "--detector-artifact", "/not/opened.json"],
        cwd=REPOSITORY_ROOT,
        env=environment,
        capture_output=True,
        check=False,
        text=True,
    )

    assert help_result.returncode == 0
    assert "detector-artifact" in help_result.stdout
    assert wrong_device_result.returncode == 2
    assert "only the approved AR0234" in wrong_device_result.stderr


def _artifact_manifest() -> dict[str, object]:
    return ARTIFACT.metadata()


def test_runner_accepts_exact_artifact_manifest_schema(tmp_path: Path) -> None:
    manifest_path = tmp_path / "detector.json"
    manifest_path.write_text(json.dumps(_artifact_manifest()), encoding="utf-8")

    assert person_runner_module._load_artifact(manifest_path) == ARTIFACT


@pytest.mark.parametrize(
    "mutation",
    [
        lambda manifest: manifest.pop("model_id"),
        lambda manifest: manifest.__setitem__("verified", True),
        lambda manifest: manifest.__setitem__("modeld_id", manifest.pop("model_id")),
    ],
    ids=["missing", "unknown-verified", "typo"],
)
def test_runner_rejects_nonexact_artifact_manifest_schema(
    tmp_path: Path,
    mutation: object,
) -> None:
    manifest = _artifact_manifest()
    mutation(manifest)  # type: ignore[operator]
    manifest_path = tmp_path / "invalid-detector.json"
    manifest_path.write_text(json.dumps(manifest), encoding="utf-8")

    with pytest.raises(ValueError, match="keys mismatch"):
        person_runner_module._load_artifact(manifest_path)


def test_runner_invalid_manifest_does_not_open_device(tmp_path: Path) -> None:
    import subprocess
    import sys

    manifest = _artifact_manifest()
    manifest["verified"] = True
    manifest_path = tmp_path / "invalid-detector.json"
    manifest_path.write_text(json.dumps(manifest), encoding="utf-8")
    result = subprocess.run(
        [
            sys.executable,
            "vision_core/tools/run_person_localization_ar0234.py",
            "--device",
            str(AR0234_BY_ID),
            "--detector-artifact",
            str(manifest_path),
        ],
        cwd=REPOSITORY_ROOT,
        env={**os.environ, "PYTHONPATH": str(REPOSITORY_ROOT)},
        capture_output=True,
        check=False,
        text=True,
    )

    assert result.returncode == 2
    assert "keys mismatch" in result.stderr


def test_single_person_creates_bbox_only_observation() -> None:
    result, detector = process([PersonDetection(BoundingBox(20, 10, 80, 50), 0.8)])

    assert result.status == "SINGLE_PERSON"
    assert result.observation is not None
    assert result.observation.observation_type == "single_person_bbox"
    assert result.observation.payload["reference_frame"] == AR0234_OPTICAL_FRAME
    assert result.observation.payload["unit"] == "px"
    assert result.observation.payload["bounding_box_xyxy_px"] == [20, 10, 80, 50]
    assert "mask" not in result.observation.payload
    assert not hasattr(result, "mask")
    assert not hasattr(PersonDetection(BoundingBox(1, 1, 2, 2), 0.5), "mask")
    assert json.loads(json.dumps(result.observation.to_dict(), allow_nan=False))
    assert detector.calls == 1


def test_no_person_is_reported_without_observation() -> None:
    result, _detector = process([])

    assert result.status == "PERSON_LOST"
    assert result.observation is None


@pytest.mark.parametrize(
    "detections",
    [
        [
            PersonDetection(BoundingBox(1, 1, 40, 40), 0.9),
            PersonDetection(BoundingBox(100, 1, 150, 50), 0.8),
        ],
        [
            PersonDetection(BoundingBox(20, 20, 80, 80), 0.7),
            PersonDetection(BoundingBox(20, 20, 80, 80), 0.7),
        ],
    ],
)
def test_every_permutation_of_two_valid_detections_is_multiple_persons(
    detections: list[PersonDetection],
) -> None:
    for ordered in permutations(detections):
        result, _detector = process(list(ordered))
        assert result.status == "MULTIPLE_PERSONS"
        assert result.candidate_count == 2
        assert result.observation is None


def test_three_valid_detections_are_multiple_persons_in_any_order() -> None:
    detections = [
        PersonDetection(BoundingBox(1, 1, 10, 10), 0.9),
        PersonDetection(BoundingBox(20, 1, 30, 10), 0.8),
        PersonDetection(BoundingBox(40, 1, 50, 10), 0.7),
    ]
    for ordered in permutations(detections):
        result, _detector = process(list(ordered))
        assert result.status == "MULTIPLE_PERSONS"
        assert result.candidate_count == 3


def test_stale_and_future_timestamps_block_before_detector_execution() -> None:
    pipeline, detector = make_pipeline([PersonDetection(BoundingBox(1, 1, 4, 4), 0.9)])
    stale = pipeline.process(
        np.zeros((10, 10, 3), dtype=np.uint8),
        captured_at_utc=NOW - timedelta(seconds=0.500001),
        cycle_id="stale",
    )
    future = pipeline.process(
        np.zeros((10, 10, 3), dtype=np.uint8),
        captured_at_utc=NOW + timedelta(microseconds=1),
        cycle_id="future",
    )

    assert stale.status == "STALE_FRAME"
    assert future.status == "FUTURE_TIMESTAMP"
    assert detector.calls == 0


def test_timestamp_boundaries_are_allowed() -> None:
    policy = {"maximum_frame_age_s": 0.5, "allowed_future_skew_s": 0.2}
    pipeline, detector = make_pipeline([PersonDetection(BoundingBox(1, 1, 4, 4), 0.9)], **policy)
    frame = np.zeros((10, 10, 3), dtype=np.uint8)

    at_stale_boundary = pipeline.process(
        frame, captured_at_utc=NOW - timedelta(seconds=0.5), cycle_id="stale-boundary"
    )
    at_future_boundary = pipeline.process(
        frame, captured_at_utc=NOW + timedelta(seconds=0.2), cycle_id="future-boundary"
    )
    beyond_future_boundary = pipeline.process(
        frame, captured_at_utc=NOW + timedelta(seconds=0.200001), cycle_id="future-blocked"
    )

    assert at_stale_boundary.status == "SINGLE_PERSON"
    assert at_future_boundary.status == "SINGLE_PERSON"
    assert beyond_future_boundary.status == "FUTURE_TIMESTAMP"
    assert detector.calls == 2


@pytest.mark.parametrize(
    "frame",
    [
        np.zeros((10, 10), dtype=np.uint8),
        np.zeros((0, 10, 3), dtype=np.uint8),
        np.zeros((10, 10, 3), dtype=np.float32),
    ],
)
def test_invalid_or_empty_frame_is_blocked_before_detector_execution(frame: np.ndarray) -> None:
    pipeline, detector = make_pipeline([])
    result = pipeline.process(frame, captured_at_utc=NOW, cycle_id="invalid")

    assert result.status == "INVALID_FRAME"
    assert detector.calls == 0


def forge_detection(*, box: object, confidence: object, label: object) -> PersonDetection:
    forged = object.__new__(PersonDetection)
    object.__setattr__(forged, "bounding_box", box)
    object.__setattr__(forged, "confidence", confidence)
    object.__setattr__(forged, "label", label)
    return forged


@pytest.mark.parametrize(
    "candidates",
    [
        object(),
        [object()],
        [PersonDetection(BoundingBox(190, 1, 210, 4), 0.9)],
        [PersonDetection(BoundingBox(201, 1, 210, 4), 0.9)],
        [forge_detection(box=BoundingBox(1, 1, 4, 4), confidence=0.9, label="cat")],
        [forge_detection(box="not-a-box", confidence=0.9, label="person")],
        [forge_detection(box=BoundingBox(1, 1, 4, 4), confidence=float("nan"), label="person")],
        [forge_detection(box=BoundingBox(1, 1, 4, 4), confidence=float("inf"), label="person")],
        [forge_detection(box=BoundingBox(1, 1, 4, 4), confidence=-0.1, label="person")],
        [forge_detection(box=BoundingBox(1, 1, 4, 4), confidence=1.1, label="person")],
    ],
)
def test_malformed_detector_output_fails_closed(candidates: object) -> None:
    result, _detector = process(candidates)

    assert result.status == "MALFORMED_OUTPUT"
    assert result.observation is None


@pytest.mark.parametrize("coordinate", ["1", 1.5, True, -1])
def test_malformed_bbox_coordinate_fails_closed(coordinate: object) -> None:
    box = object.__new__(BoundingBox)
    object.__setattr__(box, "x_min", coordinate)
    object.__setattr__(box, "y_min", 1)
    object.__setattr__(box, "x_max", 4)
    object.__setattr__(box, "y_max", 4)
    result, _detector = process([forge_detection(box=box, confidence=0.9, label="person")])

    assert result.status == "MALFORMED_OUTPUT"


def test_degenerate_forged_bbox_fails_closed() -> None:
    box = object.__new__(BoundingBox)
    object.__setattr__(box, "x_min", 4)
    object.__setattr__(box, "y_min", 1)
    object.__setattr__(box, "x_max", 4)
    object.__setattr__(box, "y_max", 4)
    result, _detector = process([forge_detection(box=box, confidence=0.9, label="person")])

    assert result.status == "MALFORMED_OUTPUT"


def test_detector_runtime_exception_is_malformed_output() -> None:
    pipeline = PersonLocalizationPipeline(RaisingDetector(RuntimeError("backend failed")), now_utc=lambda: NOW)
    result = pipeline.process(
        np.zeros((10, 10, 3), dtype=np.uint8), captured_at_utc=NOW, cycle_id="backend-error"
    )

    assert result.status == "MALFORMED_OUTPUT"
    assert result.observation is None


@pytest.mark.parametrize(
    "replacement",
    [
        make_artifact(person_label="operator"),
        make_artifact(confidence_threshold=0.9),
    ],
    ids=["changed-label", "changed-threshold"],
)
def test_artifact_replacement_after_detect_fails_closed(replacement: DetectorArtifact) -> None:
    detector = ArtifactChangingDetector(replacement)
    pipeline = PersonLocalizationPipeline(detector, now_utc=lambda: NOW)  # type: ignore[arg-type]

    result = pipeline.process(
        np.zeros((10, 10, 3), dtype=np.uint8), captured_at_utc=NOW, cycle_id="artifact-change"
    )

    assert result.status == "MALFORMED_OUTPUT"
    assert result.observation is None


def test_replacement_metadata_is_never_used_after_detect() -> None:
    replacement = MetadataTrapArtifact()
    detector = ArtifactChangingDetector(replacement)
    pipeline = PersonLocalizationPipeline(detector, now_utc=lambda: NOW)  # type: ignore[arg-type]

    result = pipeline.process(
        np.zeros((10, 10, 3), dtype=np.uint8), captured_at_utc=NOW, cycle_id="metadata-trap"
    )

    assert result.status == "MALFORMED_OUTPUT"
    assert result.observation is None
    assert replacement.metadata_calls == 0


def test_artifact_property_exception_after_detect_fails_closed() -> None:
    detector = ArtifactChangingDetector(ARTIFACT, property_error=RuntimeError("artifact unavailable"))
    pipeline = PersonLocalizationPipeline(detector, now_utc=lambda: NOW)  # type: ignore[arg-type]

    result = pipeline.process(
        np.zeros((10, 10, 3), dtype=np.uint8), captured_at_utc=NOW, cycle_id="artifact-error"
    )

    assert result.status == "MALFORMED_OUTPUT"
    assert result.observation is None


@pytest.mark.parametrize("error", [KeyboardInterrupt(), SystemExit(4)])
def test_artifact_property_base_exceptions_after_detect_are_not_swallowed(error: BaseException) -> None:
    detector = ArtifactChangingDetector(ARTIFACT, property_error=error)
    pipeline = PersonLocalizationPipeline(detector, now_utc=lambda: NOW)  # type: ignore[arg-type]

    with pytest.raises(type(error)):
        pipeline.process(
            np.zeros((10, 10, 3), dtype=np.uint8),
            captured_at_utc=NOW,
            cycle_id="artifact-interrupt",
        )


def test_observation_uses_detached_artifact_metadata_snapshot() -> None:
    pipeline, _detector = make_pipeline([PersonDetection(BoundingBox(1, 1, 4, 4), 0.9)])
    result = pipeline.process(
        np.zeros((10, 10, 3), dtype=np.uint8), captured_at_utc=NOW, cycle_id="snapshot-one"
    )
    assert result.observation is not None
    result.observation.payload["detector"]["model_id"] = "mutated"  # type: ignore[index]

    next_result = pipeline.process(
        np.zeros((10, 10, 3), dtype=np.uint8), captured_at_utc=NOW, cycle_id="snapshot-two"
    )
    assert next_result.observation is not None
    assert next_result.observation.payload["detector"]["model_id"] == ARTIFACT.model_id
    assert json.loads(json.dumps(next_result.observation.to_dict(), allow_nan=False))


@pytest.mark.parametrize("error", [KeyboardInterrupt(), SystemExit(3)])
def test_base_exceptions_are_not_swallowed(error: BaseException) -> None:
    pipeline = PersonLocalizationPipeline(RaisingDetector(error), now_utc=lambda: NOW)
    with pytest.raises(type(error)):
        pipeline.process(
            np.zeros((10, 10, 3), dtype=np.uint8), captured_at_utc=NOW, cycle_id="interrupt"
        )


def test_low_confidence_candidates_do_not_become_person_observations() -> None:
    result, _detector = process(
        [PersonDetection(BoundingBox(1, 1, 4, 4), 0.2)], min_confidence=0.5
    )

    assert result.status == "PERSON_LOST"
    assert result.observation is None


def test_artifact_confidence_threshold_is_applied_by_pipeline() -> None:
    strict_artifact = make_artifact(confidence_threshold=0.8)
    pipeline, _detector = make_pipeline(
        [PersonDetection(BoundingBox(1, 1, 4, 4), 0.7)],
        artifact=strict_artifact,
        min_confidence=0.0,
    )
    result = pipeline.process(
        np.zeros((10, 10, 3), dtype=np.uint8), captured_at_utc=NOW, cycle_id="threshold"
    )

    assert result.status == "PERSON_LOST"


def test_naive_capture_timestamp_is_rejected() -> None:
    pipeline, _detector = make_pipeline([])
    with pytest.raises(ValueError, match="timezone-aware"):
        pipeline.process(
            np.zeros((10, 10, 3), dtype=np.uint8),
            captured_at_utc=datetime(2026, 8, 31, 12, 0, 0),
            cycle_id="naive",
        )
