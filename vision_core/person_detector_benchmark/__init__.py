"""Offline, fail-closed benchmark utilities, isolated from person localization."""

from .benchmark import (
    BENCHMARK_MANIFEST_SCHEMA_VERSION,
    PRODUCTION_ADAPTERS,
    PRODUCTION_TRUSTED_APPROVALS,
    BenchmarkBlockedError,
    BenchmarkManifest,
    BoundingBox,
    Detection,
    DetectionStatus,
    ReportPublicationResult,
    ReportPublishedDurabilityUncertainError,
    TrustedApproval,
    benchmark_report_json,
    build_dataset_snapshot,
    canonical_repository_root,
    load_verified_onnx_net,
    median_and_p95_ms,
    parse_manifest,
    run_offline_benchmark,
    write_report_atomic,
)

__all__ = [
    "BENCHMARK_MANIFEST_SCHEMA_VERSION", "PRODUCTION_ADAPTERS", "PRODUCTION_TRUSTED_APPROVALS",
    "BenchmarkBlockedError", "BenchmarkManifest", "BoundingBox", "Detection",
    "DetectionStatus", "ReportPublicationResult", "ReportPublishedDurabilityUncertainError",
    "TrustedApproval", "benchmark_report_json",
    "build_dataset_snapshot", "canonical_repository_root", "load_verified_onnx_net",
    "median_and_p95_ms", "parse_manifest", "run_offline_benchmark", "write_report_atomic",
]
