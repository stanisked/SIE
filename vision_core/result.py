from dataclasses import dataclass, field
from typing import Any


@dataclass(frozen=True)
class VisionFrameResult:
    cycle_id: str
    timestamp: str

    observations: list[Any] = field(default_factory=list)
    measurements: list[Any] = field(default_factory=list)

    quality: Any | None = None

    frame_id: int | None = None
    source_id: str = "stereo_camera_01"
    reference_frame: str = "stereo_camera_01_left_optical_frame"