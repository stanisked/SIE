from __future__ import annotations

import hashlib
import json
import os
import runpy
import socket
import stat
import subprocess
import sys
from dataclasses import replace
from pathlib import Path

import cv2
import numpy as np
import pytest

import vision_core.person_detector_benchmark.benchmark as benchmark_module

from vision_core.person_detector_benchmark.benchmark import (
    BENCHMARK_MANIFEST_SCHEMA_VERSION,
    PRODUCTION_ADAPTERS,
    PRODUCTION_TRUSTED_APPROVALS,
    BenchmarkBlockedError,
    BenchmarkManifest,
    BoundingBox,
    Detection,
    DetectionStatus,
    ReportPublishedDurabilityUncertainError,
    TrustedApproval,
    benchmark_report_json,
    build_dataset_snapshot,
    classify_person_detections,
    load_verified_onnx_net,
    median_and_p95_ms,
    parse_manifest,
    run_offline_benchmark,
    validate_final_detections,
    write_report_atomic,
)


REPOSITORY_ROOT = Path(__file__).resolve().parents[2]


def manifest_payload(**overrides: object) -> dict[str, object]:
    payload: dict[str, object] = {
        "schema_version": BENCHMARK_MANIFEST_SCHEMA_VERSION,
        "model_family": "synthetic_cpu_detector", "model_id": "synthetic-person-detector",
        "model_version": "test-v1", "official_source_url": "https://example.invalid/model",
        "artifact_filename": "model.onnx", "artifact_sha256": "a" * 64,
        "license_identifier": "Test-License", "license_source_url": "https://example.invalid/license",
        "adapter_id": "synthetic-final-detection-adapter", "adapter_version": "test-v1",
        "input_contract_id": "synthetic-input-v1", "output_contract_id": "synthetic-final-detections-v1",
        "person_class_id": 0, "confidence_threshold": 0.5,
        "inference_parameters": {"resize": {"width": 8, "height": 8}},
    }
    payload.update(overrides)
    return payload


def write_manifest(tmp_path: Path, payload: dict[str, object]) -> Path:
    path = tmp_path / "manifest.json"
    path.write_text(json.dumps(payload), encoding="utf-8")
    return path


def valid_manifest(**overrides: object) -> BenchmarkManifest:
    return BenchmarkManifest(**manifest_payload(**overrides))  # type: ignore[arg-type]


def approval_for(manifest: BenchmarkManifest) -> TrustedApproval:
    return TrustedApproval(
        artifact_sha256=manifest.artifact_sha256, model_family=manifest.model_family,
        model_id=manifest.model_id, model_version=manifest.model_version,
        artifact_filename=manifest.artifact_filename, official_source_url=manifest.official_source_url,
        license_identifier=manifest.license_identifier, license_source_url=manifest.license_source_url,
        adapter_id=manifest.adapter_id, adapter_version=manifest.adapter_version,
        input_contract_id=manifest.input_contract_id, output_contract_id=manifest.output_contract_id,
        inference_parameters_json=manifest.inference_parameters_json,
    )


class FakeNet:
    def __init__(self, output: object = object(), *, fail_forward: bool = False) -> None:
        self.output, self.fail_forward = output, fail_forward
        self.backend: int | None = None
        self.target: int | None = None
        self.inputs: list[np.ndarray] = []

    def setPreferableBackend(self, value: int) -> None:
        self.backend = value

    def setPreferableTarget(self, value: int) -> None:
        self.target = value

    def setInput(self, value: np.ndarray) -> None:
        self.inputs.append(value)

    def forward(self) -> object:
        if self.fail_forward:
            raise RuntimeError("synthetic forward fault")
        return self.output


class SyntheticAdapter:
    adapter_id = "synthetic-final-detection-adapter"
    adapter_version = "test-v1"

    def __init__(self, detections: object) -> None:
        self.detections = detections
        self.preprocess_calls = 0

    def preprocess(self, image_bgr: np.ndarray, _manifest: BenchmarkManifest) -> np.ndarray:
        self.preprocess_calls += 1
        return image_bgr

    def postprocess(
        self, _output: object, _manifest: BenchmarkManifest, *, image_width: int, image_height: int
    ) -> object:
        assert (image_width, image_height) == (8, 6)
        return self.detections


def root_provider() -> Path:
    return REPOSITORY_ROOT


def roots(_root: Path) -> tuple[Path, ...]:
    return (REPOSITORY_ROOT, REPOSITORY_ROOT.parent / "main-worktree")


def decoder_from_bytes(buffer: np.ndarray, _flag: int) -> np.ndarray | None:
    assert buffer.dtype == np.uint8
    if bytes(buffer) == b"broken":
        return None
    return np.zeros((6, 8, 3), dtype=np.uint8)


def external_artifacts(tmp_path: Path) -> tuple[Path, Path, Path, BenchmarkManifest]:
    artifact = tmp_path / "model.onnx"
    model_bytes = b"verified synthetic bytes"
    artifact.write_bytes(model_bytes)
    manifest = valid_manifest(artifact_sha256=hashlib.sha256(model_bytes).hexdigest())
    manifest_path = write_manifest(tmp_path, {**manifest_payload(), "artifact_sha256": manifest.artifact_sha256})
    images = tmp_path / "images"
    images.mkdir()
    (images / "b.png").write_bytes(b"b")
    (images / "a.png").write_bytes(b"a")
    return artifact, manifest_path, images, manifest


@pytest.mark.parametrize(
    "text",
    [
        '{"schema_version":"x","schema_version":"x"}',
        '{"schema_version":"x","inference_parameters":{"x":1,"x":2}}',
        '{"schema_version":"x","inference_parameters":{"normalization":{"x":1,"x":2}}}',
    ],
)
def test_manifest_rejects_duplicate_keys_at_every_depth(tmp_path: Path, text: str) -> None:
    path = tmp_path / "manifest.json"
    path.write_text(text, encoding="utf-8")
    with pytest.raises(BenchmarkBlockedError, match="duplicate JSON key"):
        parse_manifest(path)


def test_manifest_is_strict_no_coercion_and_no_nonfinite_values(tmp_path: Path) -> None:
    missing = manifest_payload()
    missing.pop("model_id")
    with pytest.raises(BenchmarkBlockedError, match="missing"):
        parse_manifest(write_manifest(tmp_path, missing))
    unknown = manifest_payload(extra="no")
    with pytest.raises(BenchmarkBlockedError, match="unknown"):
        parse_manifest(write_manifest(tmp_path, unknown))
    with pytest.raises(BenchmarkBlockedError, match="person_class_id"):
        parse_manifest(write_manifest(tmp_path, manifest_payload(person_class_id=True)))
    with pytest.raises(BenchmarkBlockedError, match="confidence_threshold"):
        parse_manifest(write_manifest(tmp_path, manifest_payload(confidence_threshold=True)))
    raw = json.dumps(manifest_payload()).replace("0.5", "NaN", 1)
    path = tmp_path / "nan.json"
    path.write_text(raw, encoding="utf-8")
    with pytest.raises(BenchmarkBlockedError, match="non-finite"):
        parse_manifest(path)
    with pytest.raises(ValueError, match="standard JSON"):
        BenchmarkManifest(**manifest_payload(inference_parameters={"bad": {1, 2}}))  # type: ignore[arg-type]


def test_production_registry_is_empty_and_has_no_model_specific_allowlist() -> None:
    assert PRODUCTION_TRUSTED_APPROVALS == ()
    assert dict(PRODUCTION_ADAPTERS) == {}
    assert "yolox" not in repr(PRODUCTION_TRUSTED_APPROVALS).lower()
    assert "yolo26" not in repr(PRODUCTION_TRUSTED_APPROVALS).lower()


def test_verified_loader_receives_the_exact_checked_snapshot_and_uses_cpu_literals(tmp_path: Path) -> None:
    artifact = tmp_path / "model.onnx"
    source = b"unique model snapshot"
    artifact.write_bytes(source)
    manifest = valid_manifest(artifact_sha256=hashlib.sha256(source).hexdigest())
    net = FakeNet()
    received: list[bytes] = []

    def loader(buffer: np.ndarray) -> FakeNet:
        received.append(bytes(buffer))
        return net

    loaded, _ = load_verified_onnx_net(
        artifact, manifest, worktree_roots=roots(REPOSITORY_ROOT), loader=loader
    )
    assert loaded is net
    assert received == [source]
    assert net.backend == cv2.dnn.DNN_BACKEND_OPENCV
    assert net.target == cv2.dnn.DNN_TARGET_CPU


def test_model_sha_mismatch_or_symlink_never_reaches_loader(tmp_path: Path) -> None:
    artifact = tmp_path / "model.onnx"
    artifact.write_bytes(b"one")
    calls: list[np.ndarray] = []
    with pytest.raises(BenchmarkBlockedError, match="MODEL_SHA256_MISMATCH"):
        load_verified_onnx_net(
            artifact, valid_manifest(artifact_sha256="b" * 64),
            worktree_roots=roots(REPOSITORY_ROOT), loader=calls.append,
        )
    assert calls == []
    link = tmp_path / "link.onnx"
    link.symlink_to(artifact)
    manifest = valid_manifest(artifact_filename="link.onnx", artifact_sha256=hashlib.sha256(b"one").hexdigest())
    with pytest.raises(BenchmarkBlockedError, match="NOT_REGULAR"):
        load_verified_onnx_net(link, manifest, worktree_roots=roots(REPOSITORY_ROOT), loader=calls.append)
    assert calls == []
    with pytest.raises(BenchmarkBlockedError, match="PATH_NOT_ABSOLUTE"):
        load_verified_onnx_net(
            Path("model.onnx"), manifest, worktree_roots=roots(REPOSITORY_ROOT), loader=calls.append
        )


def test_dataset_snapshot_is_external_sorted_and_decodes_hashed_bytes(tmp_path: Path) -> None:
    images = tmp_path / "images"
    images.mkdir()
    (images / "z.png").write_bytes(b"z")
    (images / "a.JPG").write_bytes(b"a")
    observed: list[bytes] = []

    def decoder(buffer: np.ndarray, _flag: int) -> np.ndarray:
        observed.append(bytes(buffer))
        return np.zeros((6, 8, 3), dtype=np.uint8)

    snapshot = build_dataset_snapshot(images, worktree_roots=roots(REPOSITORY_ROOT), decoder=decoder)
    assert [Path(frame.path).name for frame in snapshot.frames] == ["a.JPG", "z.png"]
    assert observed == [b"a", b"z"]
    assert [frame.encoded_sha256 for frame in snapshot.frames] == [hashlib.sha256(b"a").hexdigest(), hashlib.sha256(b"z").hexdigest()]


def test_dataset_rejects_symlink_nonregular_empty_and_broken_image(tmp_path: Path) -> None:
    images = tmp_path / "images"
    images.mkdir()
    (images / "target.png").write_bytes(b"image")
    (images / "linked.png").symlink_to(images / "target.png")
    with pytest.raises(BenchmarkBlockedError, match="SYMLINK"):
        build_dataset_snapshot(images, worktree_roots=roots(REPOSITORY_ROOT), decoder=decoder_from_bytes)
    (images / "linked.png").unlink()
    os.mkfifo(images / "pipe.png")
    with pytest.raises(BenchmarkBlockedError, match="NON_REGULAR"):
        build_dataset_snapshot(images, worktree_roots=roots(REPOSITORY_ROOT), decoder=decoder_from_bytes)
    (images / "pipe.png").unlink()
    (images / "target.png").unlink()
    with pytest.raises(BenchmarkBlockedError, match="EMPTY"):
        build_dataset_snapshot(images, worktree_roots=roots(REPOSITORY_ROOT), decoder=decoder_from_bytes)
    (images / "broken.png").write_bytes(b"broken")
    with pytest.raises(BenchmarkBlockedError, match="DECODE_FAILED"):
        build_dataset_snapshot(images, worktree_roots=roots(REPOSITORY_ROOT), decoder=decoder_from_bytes)


def test_dataset_rejects_all_special_entries_before_suffix_filter(tmp_path: Path) -> None:
    images = tmp_path / "images"
    images.mkdir()
    (images / "a.png").write_bytes(b"a")
    (images / "ordinary.txt").write_text("ignored", encoding="utf-8")
    directory = images / "folder.unknown"
    directory.mkdir()
    with pytest.raises(BenchmarkBlockedError, match="NON_REGULAR"):
        build_dataset_snapshot(images, worktree_roots=roots(REPOSITORY_ROOT), decoder=decoder_from_bytes)
    directory.rmdir()
    fifo = images / "pipe.unknown"
    os.mkfifo(fifo)
    with pytest.raises(BenchmarkBlockedError, match="NON_REGULAR"):
        build_dataset_snapshot(images, worktree_roots=roots(REPOSITORY_ROOT), decoder=decoder_from_bytes)
    fifo.unlink()
    unknown_link = images / "link.unknown"
    unknown_link.symlink_to(images / "ordinary.txt")
    with pytest.raises(BenchmarkBlockedError, match="SYMLINK"):
        build_dataset_snapshot(images, worktree_roots=roots(REPOSITORY_ROOT), decoder=decoder_from_bytes)
    unknown_link.unlink()
    snapshot = build_dataset_snapshot(images, worktree_roots=roots(REPOSITORY_ROOT), decoder=decoder_from_bytes)
    assert [Path(frame.path).name for frame in snapshot.frames] == ["a.png"]


def test_generic_policy_never_deduplicates_final_detections() -> None:
    detection = Detection(BoundingBox(1, 1, 4, 4), 0.9, 0)
    assert classify_person_detections([]) == DetectionStatus.PERSON_LOST
    assert classify_person_detections([detection]) == DetectionStatus.SINGLE_PERSON
    assert classify_person_detections([detection, detection]) == DetectionStatus.MULTIPLE_PERSONS


@pytest.mark.parametrize(
    "candidate",
    [
        "not-a-sequence", [object()], [Detection(BoundingBox(1, 1, 4, 4), float("nan"), 0)],
        [Detection(BoundingBox(1, 1, 4, 4), 1.1, 0)], [Detection(BoundingBox(1, 1, 4, 4), 0.9, True)],
        [Detection(BoundingBox(1, 1, 4, 4), 0.9, 1)], [Detection(BoundingBox(1, 1, 4, 4), 0.9, 0, "not-person")],
        [Detection(BoundingBox(-1, 1, 4, 4), 0.9, 0)], [Detection(BoundingBox(1, 1, 1, 4), 0.9, 0)],
        [Detection(BoundingBox(1, 1, 9, 4), 0.9, 0)],
    ],
)
def test_generic_final_detection_validation_fails_closed(candidate: object) -> None:
    with pytest.raises(BenchmarkBlockedError):
        validate_final_detections(candidate, manifest=valid_manifest(), image_width=8, image_height=6)


def test_latency_statistics_use_literal_nearest_rank_and_validate() -> None:
    assert median_and_p95_ms([5]) == (5.0, 5.0)
    assert median_and_p95_ms([4, 1, 3, 2]) == (2.5, 4.0)
    assert median_and_p95_ms(list(range(1, 21))) == (10.5, 19.0)
    with pytest.raises(ValueError):
        median_and_p95_ms([])
    with pytest.raises(ValueError):
        median_and_p95_ms([True])


def test_unapproved_or_mismatched_approval_never_reaches_loader(tmp_path: Path) -> None:
    artifact, manifest_path, images, manifest = external_artifacts(tmp_path)
    calls: list[np.ndarray] = []
    kwargs = dict(artifact_path=artifact, manifest_path=manifest_path, image_directory=images,
                  warmup_count=0, opencv_threads=1, canonical_root_provider=root_provider,
                  root_discovery=roots, loader=calls.append, decoder=decoder_from_bytes)
    with pytest.raises(BenchmarkBlockedError, match="ARTIFACT_NOT_APPROVED"):
        run_offline_benchmark(**kwargs)
    assert calls == []
    wrong = replace(approval_for(manifest), official_source_url="https://different.invalid")
    with pytest.raises(BenchmarkBlockedError, match="ARTIFACT_NOT_APPROVED"):
        run_offline_benchmark(**kwargs, approval_resolver=lambda _manifest: wrong)
    assert calls == []


@pytest.mark.parametrize("field", ["artifact_sha256", "official_source_url", "license_identifier", "adapter_id", "input_contract_id", "output_contract_id"])
def test_every_approval_field_mismatch_blocks_loader(tmp_path: Path, field: str) -> None:
    artifact, manifest_path, images, manifest = external_artifacts(tmp_path)
    wrong = replace(approval_for(manifest), **{field: "b" * 64 if field == "artifact_sha256" else "different"})
    calls: list[np.ndarray] = []
    with pytest.raises(BenchmarkBlockedError, match="ARTIFACT_NOT_APPROVED"):
        run_offline_benchmark(artifact_path=artifact, manifest_path=manifest_path, image_directory=images,
                              warmup_count=0, opencv_threads=1, canonical_root_provider=root_provider,
                              root_discovery=roots, approval_resolver=lambda _manifest: wrong,
                              loader=calls.append, decoder=decoder_from_bytes)
    assert calls == []


def test_benchmark_warmup_is_separate_and_report_has_exact_counts(tmp_path: Path) -> None:
    artifact, manifest_path, images, manifest = external_artifacts(tmp_path)
    net = FakeNet()
    adapter = SyntheticAdapter([Detection(BoundingBox(1, 1, 4, 4), 0.9, 0)])
    report = run_offline_benchmark(artifact_path=artifact, manifest_path=manifest_path, image_directory=images,
                                   warmup_count=3, opencv_threads=1, canonical_root_provider=root_provider,
                                   root_discovery=roots, approval_resolver=lambda _manifest: approval_for(manifest),
                                   adapter_resolver=lambda _approval: adapter, loader=lambda _snapshot: net,
                                   decoder=decoder_from_bytes)
    assert report["warmup_iterations_requested"] == 3
    assert report["warmup_iterations_completed"] == 3
    assert report["measured_iterations_requested"] == 2
    assert report["measured_samples_completed"] == 2
    assert report["status_counts"][DetectionStatus.SINGLE_PERSON] == 2  # type: ignore[index]
    assert len(net.inputs) == 5
    assert report["latency_unit"] == "ms"
    assert report["p95_method"] == "nearest_rank"


def test_warmup_or_forward_failure_blocks_measured_loop(tmp_path: Path) -> None:
    artifact, manifest_path, images, manifest = external_artifacts(tmp_path)
    adapter = SyntheticAdapter([Detection(BoundingBox(1, 1, 4, 4), 0.9, 0)])
    net = FakeNet(fail_forward=True)
    with pytest.raises(BenchmarkBlockedError, match="DETECTOR_ITERATION_FAILED"):
        run_offline_benchmark(artifact_path=artifact, manifest_path=manifest_path, image_directory=images,
                              warmup_count=1, opencv_threads=1, canonical_root_provider=root_provider,
                              root_discovery=roots, approval_resolver=lambda _manifest: approval_for(manifest),
                              adapter_resolver=lambda _approval: adapter, loader=lambda _snapshot: net,
                              decoder=decoder_from_bytes)
    assert len(net.inputs) == 1


def test_paths_in_registered_worktrees_rejected_external_accepted(tmp_path: Path) -> None:
    with pytest.raises(BenchmarkBlockedError, match="DATASET_MUST_BE_EXTERNAL"):
        build_dataset_snapshot(REPOSITORY_ROOT / "vision_core", worktree_roots=roots(REPOSITORY_ROOT), decoder=decoder_from_bytes)
    main_images = REPOSITORY_ROOT.parent / "main-worktree" / "images"
    with pytest.raises(BenchmarkBlockedError, match="DATASET_MUST_BE_EXTERNAL"):
        build_dataset_snapshot(main_images, worktree_roots=roots(REPOSITORY_ROOT), decoder=decoder_from_bytes)
    images = tmp_path / "external"
    images.mkdir()
    (images / "a.png").write_bytes(b"a")
    assert len(build_dataset_snapshot(images, worktree_roots=roots(REPOSITORY_ROOT), decoder=decoder_from_bytes).frames) == 1


def test_atomic_report_refuses_overwrite_and_is_json_safe(tmp_path: Path) -> None:
    report_path = tmp_path / "report.json"
    report = {"number": 1, "nested": {"ok": True}}
    write_report_atomic(report_path, report, worktree_roots=roots(REPOSITORY_ROOT))
    assert json.loads(report_path.read_text(encoding="utf-8")) == report
    with pytest.raises(BenchmarkBlockedError, match="REPORT_EXISTS"):
        write_report_atomic(report_path, report, worktree_roots=roots(REPOSITORY_ROOT))
    write_report_atomic(report_path, {"number": 2}, worktree_roots=roots(REPOSITORY_ROOT), overwrite=True)
    assert json.loads(report_path.read_text(encoding="utf-8")) == {"number": 2}
    with pytest.raises(ValueError):
        benchmark_report_json({"bad": float("nan")})
    link = tmp_path / "linked-report.json"
    link.symlink_to(report_path)
    with pytest.raises(BenchmarkBlockedError, match="SYMLINK"):
        write_report_atomic(link, report, worktree_roots=roots(REPOSITORY_ROOT), overwrite=True)


def test_cli_help_is_safe_and_has_no_project_root() -> None:
    result = subprocess.run([sys.executable, "vision_core/tools/benchmark_person_detector_onnx.py", "--help"],
                            cwd=REPOSITORY_ROOT, env={**os.environ, "PYTHONPATH": str(REPOSITORY_ROOT)},
                            capture_output=True, check=False, text=True)
    assert result.returncode == 0
    assert "--project-root" not in result.stdout
    assert "external ONNX model path" in result.stdout


def test_production_import_boundary_has_no_hardware_or_runtime_dependencies() -> None:
    source = "\n".join((REPOSITORY_ROOT / path).read_text(encoding="utf-8") for path in (
        "vision_core/person_detector_benchmark/benchmark.py", "vision_core/tools/benchmark_person_detector_onnx.py",
    )).lower()
    for forbidden in ("videocapture", "camera", "stereo", "esp32", "motor", "guarded_runtime", "torch", "ultralytics", "onnxruntime", "openvino", "cuda"):
        assert forbidden not in source


def test_manifest_json_tree_is_fully_immutable_and_report_metadata_is_detached() -> None:
    source = {"top": 1, "nested": {"items": [1, {"value": 2}]}}
    manifest = valid_manifest(inference_parameters=source)
    with pytest.raises(TypeError):
        manifest.inference_parameters["top"] = 2  # type: ignore[index]
    nested = manifest.inference_parameters["nested"]
    assert isinstance(nested, dict) is False
    with pytest.raises(TypeError):
        nested["items"] = ()  # type: ignore[index]
    items = nested["items"]  # type: ignore[index]
    assert isinstance(items, tuple)
    with pytest.raises(TypeError):
        items[0] = 2  # type: ignore[index]
    with pytest.raises(TypeError):
        items[1]["value"] = 3  # type: ignore[index]
    approval = approval_for(manifest)
    metadata = benchmark_module.manifest_report_metadata(manifest)
    metadata["inference_parameters"]["nested"]["items"][1]["value"] = 99  # type: ignore[index]
    source["nested"]["items"].append({"value": 3})
    source["nested"]["items"][1]["value"] = 77
    assert json.loads(manifest.inference_parameters_json)["nested"]["items"][1]["value"] == 2
    assert json.loads(manifest.inference_parameters_json) == {
        "nested": {"items": [1, {"value": 2}]}, "top": 1
    }
    assert benchmark_module.require_approved_manifest(manifest, lambda _manifest: approval) == approval


def test_manifest_rejects_container_subclasses_without_visiting_them() -> None:
    class StatefulDict(dict[str, object]):
        def items(self) -> object:
            raise AssertionError("subclass must be rejected before items()")

    class StatefulList(list[object]):
        def __iter__(self) -> object:
            raise AssertionError("subclass must be rejected before iteration")

    with pytest.raises(ValueError, match="JSON object"):
        valid_manifest(inference_parameters=StatefulDict())
    with pytest.raises(ValueError, match="standard JSON"):
        valid_manifest(inference_parameters={"nested": StatefulDict()})
    with pytest.raises(ValueError, match="standard JSON"):
        valid_manifest(inference_parameters={"nested": StatefulList()})


def test_snapshot_pixels_are_new_adapter_local_views_across_warmup_and_measurement(tmp_path: Path) -> None:
    artifact, manifest_path, images, manifest = external_artifacts(tmp_path)

    class MetadataMutatingAdapter(SyntheticAdapter):
        def __init__(self) -> None:
            super().__init__([Detection(BoundingBox(1, 1, 4, 4), 0.9, 0)])
            self.inputs: list[np.ndarray] = []
            self.input_bytes: list[bytes] = []
            self.input_shapes: list[tuple[int, ...]] = []
            self.input_dtypes: list[np.dtype[object]] = []
            self.mutation_errors: list[type[Exception]] = []

        def preprocess(self, image_bgr: np.ndarray, config: BenchmarkManifest) -> np.ndarray:
            self.inputs.append(image_bgr)
            self.input_bytes.append(bytes(image_bgr))
            self.input_shapes.append(image_bgr.shape)
            self.input_dtypes.append(image_bgr.dtype)
            def mutate_manifest() -> None:
                config.inference_parameters["tamper"] = True  # type: ignore[index]

            for mutation in (lambda: image_bgr.setflags(write=True), lambda: image_bgr.__setitem__((0, 0, 0), 255), mutate_manifest):
                try:
                    mutation()
                except (TypeError, ValueError) as error:
                    self.mutation_errors.append(type(error))
            if len(self.inputs) == 1:
                image_bgr.dtype = np.int8
                image_bgr.shape = (3, 6, 8)
            return image_bgr.copy()

    adapter = MetadataMutatingAdapter()
    report = run_offline_benchmark(
        artifact_path=artifact, manifest_path=manifest_path, image_directory=images,
        warmup_count=1, opencv_threads=1, canonical_root_provider=root_provider, root_discovery=roots,
        approval_resolver=lambda _manifest: approval_for(manifest), adapter_resolver=lambda _approval: adapter,
        loader=lambda _snapshot: FakeNet(), decoder=decoder_from_bytes,
    )
    assert len(adapter.inputs) == 3
    assert len({id(image) for image in adapter.inputs}) == 3
    assert adapter.input_shapes == [(6, 8, 3)] * 3
    assert adapter.input_dtypes == [np.dtype(np.uint8)] * 3
    assert len(set(adapter.input_bytes)) == 1
    assert adapter.mutation_errors == [ValueError, ValueError, TypeError] * 3
    assert json.loads(manifest.inference_parameters_json)["resize"]["width"] == 8
    assert report["dataset"]["input_files"] == [  # type: ignore[index]
        {"path": str(images / "a.png"), "sha256": hashlib.sha256(b"a").hexdigest()},
        {"path": str(images / "b.png"), "sha256": hashlib.sha256(b"b").hexdigest()},
    ]


@pytest.mark.parametrize(
    "field",
    [
        "artifact_sha256", "model_family", "model_id", "model_version", "artifact_filename",
        "official_source_url", "license_identifier", "license_source_url", "adapter_id",
        "adapter_version", "input_contract_id", "output_contract_id", "inference_parameters_json",
    ],
)
def test_all_approval_fields_must_match_before_loader(tmp_path: Path, field: str) -> None:
    artifact, manifest_path, images, manifest = external_artifacts(tmp_path)
    different = "b" * 64 if field == "artifact_sha256" else "different"
    wrong = replace(approval_for(manifest), **{field: different})
    loader_calls: list[np.ndarray] = []
    with pytest.raises(BenchmarkBlockedError, match="ARTIFACT_NOT_APPROVED"):
        run_offline_benchmark(
            artifact_path=artifact, manifest_path=manifest_path, image_directory=images,
            warmup_count=0, opencv_threads=1, canonical_root_provider=root_provider, root_discovery=roots,
            approval_resolver=lambda _manifest: wrong, loader=loader_calls.append, decoder=decoder_from_bytes,
        )
    assert loader_calls == []


def test_model_path_rejects_directory_fifo_and_character_device(tmp_path: Path) -> None:
    def check(path: Path) -> None:
        manifest = valid_manifest(artifact_filename=path.name)
        with pytest.raises(BenchmarkBlockedError, match="NOT_REGULAR"):
            load_verified_onnx_net(path, manifest, worktree_roots=roots(REPOSITORY_ROOT), loader=lambda _buffer: FakeNet())

    directory = tmp_path / "model-directory"
    directory.mkdir()
    check(directory)
    fifo = tmp_path / "model-fifo"
    os.mkfifo(fifo)
    check(fifo)
    check(Path("/dev/null"))


def test_model_and_dataset_reject_unix_socket_when_supported(tmp_path: Path) -> None:
    unix_socket = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    images = tmp_path / "images"
    images.mkdir()
    (images / "a.png").write_bytes(b"a")
    socket_path = images / "special.unknown"
    try:
        try:
            unix_socket.bind(str(socket_path))
        except PermissionError as error:
            pytest.skip(f"Unix-domain socket creation is unavailable in this sandbox: {error}")
        manifest = valid_manifest(artifact_filename=socket_path.name)
        with pytest.raises(BenchmarkBlockedError, match="NOT_REGULAR"):
            load_verified_onnx_net(
                socket_path, manifest, worktree_roots=roots(REPOSITORY_ROOT), loader=lambda _buffer: FakeNet()
            )
        with pytest.raises(BenchmarkBlockedError, match="NON_REGULAR"):
            build_dataset_snapshot(images, worktree_roots=roots(REPOSITORY_ROOT), decoder=decoder_from_bytes)
    finally:
        unix_socket.close()


def test_model_socket_mode_is_rejected_before_loader(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> None:
    artifact = tmp_path / "model.onnx"
    artifact.write_bytes(b"model")
    loader_calls: list[np.ndarray] = []
    original_lstat = benchmark_module.os.lstat

    def socket_lstat(path: str | Path) -> os.stat_result:
        if Path(path) == artifact:
            return os.stat_result((stat.S_IFSOCK,) + (0,) * 9)
        return original_lstat(path)

    monkeypatch.setattr(benchmark_module.os, "lstat", socket_lstat)
    with pytest.raises(BenchmarkBlockedError, match="MODEL_ARTIFACT_NOT_REGULAR_FILE"):
        load_verified_onnx_net(
            artifact,
            valid_manifest(artifact_sha256=hashlib.sha256(b"model").hexdigest()),
            worktree_roots=roots(REPOSITORY_ROOT),
            loader=loader_calls.append,
        )
    assert loader_calls == []


def test_dataset_socket_mode_is_rejected_before_decoder(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> None:
    images = tmp_path / "images"
    images.mkdir()
    socket_entry = images / "special.unknown"
    socket_entry.write_bytes(b"not-decoded")
    decoder_calls: list[np.ndarray] = []
    original_lstat = benchmark_module.os.lstat

    def socket_lstat(path: str | Path) -> os.stat_result:
        if Path(path) == socket_entry:
            return os.stat_result((stat.S_IFSOCK,) + (0,) * 9)
        return original_lstat(path)

    monkeypatch.setattr(benchmark_module.os, "lstat", socket_lstat)
    with pytest.raises(BenchmarkBlockedError, match="DATASET_NON_REGULAR_ENTRY"):
        build_dataset_snapshot(
            images, worktree_roots=roots(REPOSITORY_ROOT), decoder=decoder_calls.append
        )
    assert decoder_calls == []


def test_model_manifest_and_report_inside_second_worktree_are_rejected(tmp_path: Path) -> None:
    artifact, manifest_path, images, manifest = external_artifacts(tmp_path)
    second_root = REPOSITORY_ROOT.parent / "main-worktree"
    inside_model = second_root / "model.onnx"
    with pytest.raises(BenchmarkBlockedError, match="MODEL_ARTIFACT_MUST_BE_EXTERNAL"):
        load_verified_onnx_net(inside_model, manifest, worktree_roots=roots(REPOSITORY_ROOT), loader=lambda _buffer: FakeNet())
    with pytest.raises(BenchmarkBlockedError, match="MANIFEST_MUST_BE_EXTERNAL"):
        run_offline_benchmark(
            artifact_path=artifact, manifest_path=second_root / "manifest.json", image_directory=images,
            warmup_count=0, opencv_threads=1, canonical_root_provider=root_provider, root_discovery=roots,
        )
    with pytest.raises(BenchmarkBlockedError, match="REPORT_MUST_BE_EXTERNAL"):
        write_report_atomic(second_root / "report.json", {"ok": True}, worktree_roots=roots(REPOSITORY_ROOT))
    assert manifest_path.exists()


def test_worktree_porcelain_parser_preserves_spaces_and_multiple_roots(tmp_path: Path) -> None:
    first = tmp_path / "first root"
    second = tmp_path / "second root"
    text = f"worktree {first}\nHEAD aaa\n\nworktree {second}\nHEAD bbb\n\n"
    assert benchmark_module._parse_worktree_porcelain(text) == (first.resolve(), second.resolve())


@pytest.mark.parametrize(
    "bbox",
    ["not-a-bbox", BoundingBox(1.0, 1, 4, 4), BoundingBox("1", 1, 4, 4)],
)
def test_generic_validation_rejects_wrong_bbox_type_and_coordinates(bbox: object) -> None:
    with pytest.raises(BenchmarkBlockedError, match="INVALID_FINAL_DETECTION"):
        validate_final_detections([Detection(bbox, 0.9, 0)], manifest=valid_manifest(), image_width=8, image_height=6)  # type: ignore[arg-type]


def test_loader_exception_and_buffer_overload_failure_are_controlled(tmp_path: Path) -> None:
    artifact = tmp_path / "model.onnx"
    artifact.write_bytes(b"model")
    manifest = valid_manifest(artifact_sha256=hashlib.sha256(b"model").hexdigest())
    with pytest.raises(BenchmarkBlockedError, match="ONNX_BUFFER_LOADER_FAILED"):
        load_verified_onnx_net(
            artifact, manifest, worktree_roots=roots(REPOSITORY_ROOT),
            loader=lambda _buffer: (_ for _ in ()).throw(RuntimeError("loader fault")),
        )
    with pytest.raises(BenchmarkBlockedError, match="ONNX_BUFFER_LOADER_UNSUPPORTED"):
        load_verified_onnx_net(
            artifact, manifest, worktree_roots=roots(REPOSITORY_ROOT),
            loader=lambda _buffer: (_ for _ in ()).throw(TypeError("buffer unsupported")),
        )


def test_pipeline_clock_includes_status_calculation(monkeypatch: pytest.MonkeyPatch) -> None:
    frame = benchmark_module.DatasetFrame("frame", "hash", bytes(6 * 8 * 3), (6, 8, 3), "uint8")
    adapter = SyntheticAdapter([Detection(BoundingBox(1, 1, 4, 4), 0.9, 0)])
    clock_values = iter((0, 10, 20, 30, 40))

    def clock() -> int:
        return next(clock_values)

    original = benchmark_module.classify_person_detections

    def classify_with_cost(detections: object) -> str:
        assert clock() == 30
        return original(detections)  # type: ignore[arg-type]

    monkeypatch.setattr(benchmark_module, "classify_person_detections", classify_with_cost)
    forward_ms, pipeline_ms, status = benchmark_module._run_one(
        net=FakeNet(), adapter=adapter, frame=frame, manifest=valid_manifest(), clock_ns=clock
    )
    assert forward_ms == 10 / 1_000_000
    assert pipeline_ms == 40 / 1_000_000
    assert status == DetectionStatus.SINGLE_PERSON


def _temporary_entries(directory: Path, target_name: str) -> list[Path]:
    return sorted(directory.glob(f".{target_name}.*.tmp"))


def test_report_no_clobber_race_preserves_concurrent_target_and_cleans_temp(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    target = tmp_path / "report.json"
    original_link = benchmark_module.os.link

    def create_competing_target(src: str, dst: str, **kwargs: object) -> None:
        descriptor = os.open(dst, os.O_CREAT | os.O_EXCL | os.O_WRONLY, 0o600, dir_fd=kwargs["dst_dir_fd"])  # type: ignore[arg-type]
        try:
            os.write(descriptor, b"concurrent")
        finally:
            os.close(descriptor)
        original_link(src, dst, **kwargs)

    monkeypatch.setattr(benchmark_module.os, "link", create_competing_target)
    with pytest.raises(BenchmarkBlockedError, match="REPORT_EXISTS"):
        write_report_atomic(target, {"new": True}, worktree_roots=roots(REPOSITORY_ROOT))
    assert target.read_bytes() == b"concurrent"
    assert _temporary_entries(tmp_path, target.name) == []


def test_report_publish_success_overwrite_and_failure_cleanup(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    target = tmp_path / "report.json"
    first_result = write_report_atomic(target, {"value": 1}, worktree_roots=roots(REPOSITORY_ROOT))
    assert first_result.published is True
    assert first_result.durability_confirmed is True
    assert first_result.overwrite is False
    assert json.loads(target.read_text()) == {"value": 1}
    with pytest.raises(BenchmarkBlockedError, match="REPORT_EXISTS"):
        write_report_atomic(target, {"value": 2}, worktree_roots=roots(REPOSITORY_ROOT))
    overwrite_result = write_report_atomic(target, {"value": 2}, worktree_roots=roots(REPOSITORY_ROOT), overwrite=True)
    assert overwrite_result.published is True
    assert overwrite_result.durability_confirmed is True
    assert overwrite_result.overwrite is True
    assert json.loads(target.read_text()) == {"value": 2}

    def write_fault(_fd: int, _payload: bytes) -> None:
        raise OSError("write fault")

    monkeypatch.setattr(benchmark_module, "_write_and_fsync_report", write_fault)
    failed = tmp_path / "failed.json"
    with pytest.raises(BenchmarkBlockedError, match="REPORT_WRITE_FAILED"):
        write_report_atomic(failed, {"value": 3}, worktree_roots=roots(REPOSITORY_ROOT))
    assert not failed.exists()
    assert _temporary_entries(tmp_path, failed.name) == []


def test_report_fsync_failure_cleans_temp(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> None:
    target = tmp_path / "fsync-failed.json"

    def fsync_fault(_fd: int) -> None:
        raise OSError("fsync fault")

    monkeypatch.setattr(benchmark_module.os, "fsync", fsync_fault)
    with pytest.raises(BenchmarkBlockedError, match="REPORT_WRITE_FAILED"):
        write_report_atomic(target, {"value": 1}, worktree_roots=roots(REPOSITORY_ROOT))
    assert not target.exists()
    assert _temporary_entries(tmp_path, target.name) == []


@pytest.mark.parametrize("overwrite", [False, True])
def test_report_directory_fsync_failure_reports_published_durability_uncertain(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch, overwrite: bool
) -> None:
    target = tmp_path / "report.json"
    if overwrite:
        write_report_atomic(target, {"value": "old"}, worktree_roots=roots(REPOSITORY_ROOT))
    original_fsync = benchmark_module.os.fsync
    fsync_calls = 0

    def fail_only_directory_fsync(descriptor: int) -> None:
        nonlocal fsync_calls
        fsync_calls += 1
        if fsync_calls == 2:
            raise OSError("directory fsync fault")
        original_fsync(descriptor)

    monkeypatch.setattr(benchmark_module.os, "fsync", fail_only_directory_fsync)
    with pytest.raises(ReportPublishedDurabilityUncertainError) as raised:
        write_report_atomic(target, {"value": "new"}, worktree_roots=roots(REPOSITORY_ROOT), overwrite=overwrite)
    error = raised.value
    assert error.published is True
    assert error.durability_confirmed is False
    assert error.target_path == str(target)
    assert error.overwrite is overwrite
    assert isinstance(error.cause, OSError)
    assert json.loads(target.read_text(encoding="utf-8")) == {"value": "new"}
    assert _temporary_entries(tmp_path, target.name) == []


@pytest.mark.parametrize("operation,status", [("link", "REPORT_SAFE_PUBLISH_FAILED"), ("replace", "REPORT_REPLACE_FAILED")])
def test_report_link_and_replace_errors_clean_temp(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch, operation: str, status: str
) -> None:
    target = tmp_path / f"{operation}.json"

    def fault(*_args: object, **_kwargs: object) -> None:
        raise OSError("publish fault")

    monkeypatch.setattr(benchmark_module.os, operation, fault)
    with pytest.raises(BenchmarkBlockedError, match=status):
        write_report_atomic(target, {"value": 1}, worktree_roots=roots(REPOSITORY_ROOT), overwrite=operation == "replace")
    assert not target.exists()
    assert _temporary_entries(tmp_path, target.name) == []


def test_report_parent_and_target_symlinks_are_rejected(tmp_path: Path) -> None:
    actual_parent = tmp_path / "actual"
    actual_parent.mkdir()
    linked_parent = tmp_path / "linked-parent"
    linked_parent.symlink_to(actual_parent, target_is_directory=True)
    with pytest.raises(BenchmarkBlockedError, match="SYMLINK"):
        write_report_atomic(linked_parent / "report.json", {"ok": True}, worktree_roots=roots(REPOSITORY_ROOT))
    target = actual_parent / "report.json"
    target.write_text("old", encoding="utf-8")
    link = actual_parent / "linked-report.json"
    link.symlink_to(target)
    with pytest.raises(BenchmarkBlockedError, match="SYMLINK"):
        write_report_atomic(link, {"ok": True}, worktree_roots=roots(REPOSITORY_ROOT), overwrite=True)


def test_help_does_not_call_any_runtime_boundary(monkeypatch: pytest.MonkeyPatch) -> None:
    def fail(*_args: object, **_kwargs: object) -> object:
        raise AssertionError("runtime boundary called")

    for name in (
        "canonical_repository_root", "discover_worktree_roots", "run_offline_benchmark",
        "write_report_atomic", "parse_manifest", "build_dataset_snapshot", "load_verified_onnx_net",
        "production_adapter_resolver",
    ):
        monkeypatch.setattr(benchmark_module, name, fail)
    monkeypatch.setattr(sys, "argv", ["benchmark_person_detector_onnx.py", "--help"])
    with pytest.raises(SystemExit) as result:
        runpy.run_path(str(REPOSITORY_ROOT / "vision_core/tools/benchmark_person_detector_onnx.py"), run_name="__main__")
    assert result.value.code == 0


def test_cli_reports_published_durability_uncertain_with_distinct_exit_code(
    monkeypatch: pytest.MonkeyPatch, capsys: pytest.CaptureFixture[str]
) -> None:
    report_path = Path("/tmp/external-report.json")
    result = benchmark_module.ReportPublicationResult(
        target_path=str(report_path), overwrite=True, published=True, durability_confirmed=False
    )

    def fail_after_publication(*_args: object, **_kwargs: object) -> object:
        raise ReportPublishedDurabilityUncertainError(result, OSError("directory fsync fault"))

    monkeypatch.setattr(benchmark_module, "canonical_repository_root", lambda: REPOSITORY_ROOT)
    monkeypatch.setattr(benchmark_module, "discover_worktree_roots", lambda _root: roots(REPOSITORY_ROOT))
    monkeypatch.setattr(benchmark_module, "run_offline_benchmark", lambda **_kwargs: {"ok": True})
    monkeypatch.setattr(benchmark_module, "write_report_atomic", fail_after_publication)
    monkeypatch.setattr(
        sys,
        "argv",
        [
            "benchmark_person_detector_onnx.py", "--model", "/tmp/model.onnx", "--manifest", "/tmp/manifest.json",
            "--images", "/tmp/images", "--report", str(report_path), "--overwrite",
        ],
    )
    with pytest.raises(SystemExit) as exited:
        runpy.run_path(str(REPOSITORY_ROOT / "vision_core/tools/benchmark_person_detector_onnx.py"), run_name="__main__")
    assert exited.value.code == 3
    assert "REPORT_PUBLISHED_DURABILITY_UNCERTAIN" in capsys.readouterr().err
