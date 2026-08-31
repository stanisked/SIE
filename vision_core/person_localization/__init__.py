"""AR0234 single-person localization primitives for SIE Vision Core."""

from .ar0234 import AR0234_BY_ID, AR0234Capture, AR0234CaptureConfig
from .detector import HOGPersonDetector, PersonDetector
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
    "HOGPersonDetector",
    "PersonDetection",
    "PersonDetector",
    "PersonLocalizationPipeline",
    "PersonLocalizationPolicy",
    "PersonLocalizationResult",
    "PersonLocalizationStatus",
]
