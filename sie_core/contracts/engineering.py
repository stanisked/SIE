from dataclasses import dataclass, field


@dataclass(frozen=True)
class SpatialValue:
    value: float | None
    unit: str
    reference_frame: str
    confidence: float

    def to_dict(self):
        return {
            "value": self.value,
            "unit": self.unit,
            "reference_frame": self.reference_frame,
            "confidence": self.confidence,
        }


@dataclass(frozen=True)
class Evidence:
    evidence_id: str
    source: str
    kind: str
    metadata: dict = field(default_factory=dict)

    def to_dict(self):
        return {
            "evidence_id": self.evidence_id,
            "source": self.source,
            "kind": self.kind,
            "metadata": self.metadata,
        }


@dataclass(frozen=True)
class Observation:
    observation_id: str
    kind: str
    reference_frame: str
    confidence: float
    evidence_ids: tuple[str, ...]
    metadata: dict = field(default_factory=dict)

    def to_dict(self):
        return {
            "observation_id": self.observation_id,
            "kind": self.kind,
            "reference_frame": self.reference_frame,
            "confidence": self.confidence,
            "evidence_ids": list(self.evidence_ids),
            "metadata": self.metadata,
        }


@dataclass(frozen=True)
class Measurement:
    measurement_id: str
    kind: str
    value: SpatialValue
    evidence_ids: tuple[str, ...]
    status: str
    metadata: dict = field(default_factory=dict)

    def to_dict(self):
        return {
            "measurement_id": self.measurement_id,
            "kind": self.kind,
            "value": self.value.to_dict(),
            "evidence_ids": list(self.evidence_ids),
            "status": self.status,
            "metadata": self.metadata,
        }
