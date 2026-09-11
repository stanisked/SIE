"""AR0234 single-person localization primitives for SIE Vision Core."""

from .ar0234 import AR0234_BY_ID, AR0234Capture, AR0234CaptureConfig, resolve_ar0234_device
from .close_range_locator import (
    CloseRangeDetection,
    CloseRangeTargetStatus,
    OpenCvHaarFaceLocator,
    build_close_range_target_record,
)
from .detector import DetectorArtifact, PersonDetector, verify_artifact_bytes
from .models import (
    BoundingBox,
    PersonDetection,
    PersonLocalizationResult,
    PersonLocalizationStatus,
)
from .pipeline import PersonLocalizationPipeline, PersonLocalizationPolicy

__all__ = [
    "AR0234_BY_ID",
    "AR0234Capture",
    "AR0234CaptureConfig",
    "BoundingBox",
    "CloseRangeDetection",
    "CloseRangeTargetStatus",
    "DetectorArtifact",
    "OpenCvHaarFaceLocator",
    "PersonDetection",
    "PersonDetector",
    "PersonLocalizationPipeline",
    "PersonLocalizationPolicy",
    "PersonLocalizationResult",
    "PersonLocalizationStatus",
    "build_close_range_target_record",
    "resolve_ar0234_device",
    "verify_artifact_bytes",
]
