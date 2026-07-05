from dataclasses import dataclass, field
from datetime import datetime, timezone

import numpy as np


@dataclass(frozen=True)
class Observation:
    observation_id: str
    source: str
    timestamp: str
    cycle_id: str
    observation_type: str
    payload: dict
    confidence: float
    quality: dict
    evidence_ids: tuple[str, ...] = field(default_factory=tuple)

    @classmethod
    def from_depth_pipeline(
        cls,
        disparity,
        depth,
        confidence,
        reference_frame,
        cycle_id="current",
        evidence_ids=("evidence.stereo_frame.current",),
    ):
        valid_depth = np.isfinite(depth)
        payload = {
            "reference_frame": reference_frame,
            "unit": "m",
            "depth_shape": tuple(depth.shape),
            "disparity_shape": tuple(disparity.shape),
            "valid_depth_ratio": float(np.mean(valid_depth)),
        }
        quality = {
            "confidence_signals": confidence,
            "valid_depth_ratio": payload["valid_depth_ratio"],
        }

        return cls(
            observation_id=f"observation.depth.{cycle_id}",
            source="vision_core",
            timestamp=datetime.now(timezone.utc).isoformat(),
            cycle_id=cycle_id,
            observation_type="depth_observation",
            payload=payload,
            confidence=float(confidence["confidence"]),
            quality=quality,
            evidence_ids=evidence_ids,
        )

    def to_dict(self):
        return {
            "observation_id": self.observation_id,
            "source": self.source,
            "timestamp": self.timestamp,
            "cycle_id": self.cycle_id,
            "observation_type": self.observation_type,
            "payload": self.payload,
            "confidence": self.confidence,
            "quality": self.quality,
            "evidence_ids": list(self.evidence_ids),
        }
