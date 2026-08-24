from dataclasses import dataclass
from typing import Optional


@dataclass(frozen=True)
class QualityReport:
    valid: bool
    confidence: float

    valid_points: int
    roi_size_px: tuple[int, int]

    depth_std_mm: Optional[float] = None
    sharpness_left: Optional[float] = None
    sharpness_right: Optional[float] = None

    notes: Optional[str] = None

    def to_dict(self):
        return {
            "valid": self.valid,
            "confidence": self.confidence,
            "valid_points": self.valid_points,
            "roi_size_px": self.roi_size_px,
            "depth_std_mm": self.depth_std_mm,
            "sharpness_left": self.sharpness_left,
            "sharpness_right": self.sharpness_right,
            "notes": self.notes,
        }
