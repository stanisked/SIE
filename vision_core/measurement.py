from dataclasses import dataclass, field
from datetime import datetime, timezone


@dataclass(frozen=True)
class Measurement:
    measurement_id: str
    measurement_type: str
    value: float | None
    unit: str
    reference_frame: str
    confidence: float
    quality: dict
    source_observations: tuple[str, ...]
    timestamp: str
    status: str
    metadata: dict = field(default_factory=dict)

    @classmethod
    def from_roi_depth(
        cls,
        value,
        observation,
        reference_frame,
        confidence,
        quality,
        status,
        cycle_id="current",
        estimator="ROIDepthEstimator",
    ):
        return cls(
            measurement_id=f"measurement.roi_depth.{cycle_id}",
            measurement_type="roi_depth",
            value=value,
            unit="m",
            reference_frame=reference_frame,
            confidence=float(confidence),
            quality=quality,
            source_observations=(observation.observation_id,),
            timestamp=datetime.now(timezone.utc).isoformat(),
            status=status,
            metadata={
                "estimator": estimator,
            },
        )

    def to_dict(self):
        return {
            "measurement_id": self.measurement_id,
            "measurement_type": self.measurement_type,
            "value": self.value,
            "unit": self.unit,
            "reference_frame": self.reference_frame,
            "confidence": self.confidence,
            "quality": self.quality,
            "source_observations": list(self.source_observations),
            "timestamp": self.timestamp,
            "status": self.status,
            "metadata": self.metadata,
        }
