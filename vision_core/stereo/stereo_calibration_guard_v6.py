#!/usr/bin/env python3

"""Fail-closed runtime guard for conditionally active V6 stereo calibration."""

from __future__ import annotations

import hashlib
import json
import math
import time
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Callable

import numpy as np


REQUIRED_CHANNELS = {"ambient", "camera_left", "camera_right"}
REQUIRED_GATING_CHANNELS = {"camera_left", "camera_right"}
REQUIRED_OBSERVATIONAL_CHANNELS = {"ambient"}
REQUIRED_CALIBRATION_KEYS = {
    "K1",
    "D1",
    "K2",
    "D2",
    "R1",
    "R2",
    "P1",
    "P2",
    "Q",
    "T",
    "size",
    "baseline_mm",
    "calibration_id",
}


class CalibrationGuardError(RuntimeError):
    """A stable, machine-readable reason for blocking depth processing."""

    def __init__(self, reason: str, detail: str):
        self.reason = reason
        self.detail = detail
        super().__init__(f"{reason}: {detail}")


@dataclass(frozen=True)
class TemperatureGateResult:
    checked_at_utc: str
    state_age_s: float
    temperatures_c: dict[str, float]


@dataclass(frozen=True)
class GuardedCalibration:
    calibration_id: str
    calibration_sha256: str
    activation_record_sha256: str
    parameters: dict[str, np.ndarray]
    temperature_gate: TemperatureGateResult


def file_sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def load_json_object(path: Path, label: str) -> dict[str, Any]:
    if not path.is_file():
        raise CalibrationGuardError("MISSING_FILE", f"{label}: {path}")
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, UnicodeDecodeError, json.JSONDecodeError) as error:
        raise CalibrationGuardError(
            "INVALID_JSON", f"{label}: {path}: {error}"
        ) from error
    if not isinstance(value, dict):
        raise CalibrationGuardError("INVALID_JSON", f"{label} is not an object")
    return value


def resolve_inside_project(project_root: Path, value: str, label: str) -> Path:
    candidate = Path(value)
    if not candidate.is_absolute():
        candidate = project_root / candidate
    resolved = candidate.resolve()
    try:
        resolved.relative_to(project_root)
    except ValueError as error:
        raise CalibrationGuardError(
            "PATH_OUTSIDE_PROJECT", f"{label}: {resolved}"
        ) from error
    return resolved


def scalar_text(value: np.ndarray, key: str) -> str:
    array = np.asarray(value)
    if array.size != 1:
        raise CalibrationGuardError(
            "INVALID_CALIBRATION_METADATA", f"{key} must contain one value"
        )
    return str(array.reshape(-1)[0])


class StereoCalibrationGuard:
    """Verify activation once and temperature eligibility before every depth cycle."""

    def __init__(
        self,
        policy_path: Path,
        *,
        project_root: Path,
        now_unix_s: Callable[[], float] = time.time,
    ) -> None:
        self.project_root = project_root.resolve()
        self.policy_path = policy_path.resolve()
        self.now_unix_s = now_unix_s
        self.policy = load_json_object(self.policy_path, "runtime policy")
        self.gating_channels: frozenset[str] = frozenset()
        self.observational_channels: frozenset[str] = frozenset()
        self._validate_policy()

        self.activation_record_path = resolve_inside_project(
            self.project_root,
            str(self.policy["activation_record_path"]),
            "activation record",
        )
        self.calibration_path = resolve_inside_project(
            self.project_root,
            str(self.policy["calibration_path"]),
            "calibration",
        )
        self.temperature_state_path = Path(
            str(self.policy["temperature_state_file"])
        )
        self.activation_record: dict[str, Any] | None = None
        self.envelope: dict[str, dict[str, float]] | None = None

    @classmethod
    def from_policy(
        cls,
        policy_path: str | Path,
        *,
        project_root: str | Path,
        now_unix_s: Callable[[], float] = time.time,
    ) -> "StereoCalibrationGuard":
        return cls(
            Path(policy_path),
            project_root=Path(project_root),
            now_unix_s=now_unix_s,
        )

    def _validate_policy(self) -> None:
        if self.policy.get("schema_version") != "sie_stereo_runtime_policy_v1":
            raise CalibrationGuardError(
                "INVALID_POLICY", "unexpected schema_version"
            )
        if self.policy.get("status") != "ENABLED":
            raise CalibrationGuardError("POLICY_DISABLED", "policy status is not ENABLED")
        if self.policy.get("required_activation_status") != "ACTIVE_CONDITIONAL":
            raise CalibrationGuardError(
                "INVALID_POLICY", "required activation status must be ACTIVE_CONDITIONAL"
            )
        maximum_age_s = float(self.policy.get("maximum_temperature_state_age_s", 0))
        if not math.isfinite(maximum_age_s) or maximum_age_s <= 0:
            raise CalibrationGuardError(
                "INVALID_POLICY", "maximum temperature age must be positive"
            )
        mapping = self.policy.get("expected_rom_mapping")
        if not isinstance(mapping, dict) or set(mapping) != REQUIRED_CHANNELS:
            raise CalibrationGuardError(
                "INVALID_POLICY", "expected ROM mapping must contain three channels"
            )
        normalized = {str(key): str(value).upper() for key, value in mapping.items()}
        if len(set(normalized.values())) != 3:
            raise CalibrationGuardError("INVALID_POLICY", "ROM values must be unique")
        for rom in normalized.values():
            if len(rom) != 16 or any(char not in "0123456789ABCDEF" for char in rom):
                raise CalibrationGuardError("INVALID_POLICY", f"invalid ROM: {rom}")
        self.policy["expected_rom_mapping"] = normalized
        gating_channels = self.policy.get("temperature_gating_channels")
        observational_channels = self.policy.get(
            "temperature_observational_channels"
        )
        if (
            not isinstance(gating_channels, list)
            or set(gating_channels) != REQUIRED_GATING_CHANNELS
            or len(gating_channels) != len(REQUIRED_GATING_CHANNELS)
        ):
            raise CalibrationGuardError(
                "INVALID_POLICY",
                "temperature_gating_channels must be exactly "
                "camera_left and camera_right",
            )
        if (
            not isinstance(observational_channels, list)
            or set(observational_channels) != REQUIRED_OBSERVATIONAL_CHANNELS
            or len(observational_channels) != len(REQUIRED_OBSERVATIONAL_CHANNELS)
        ):
            raise CalibrationGuardError(
                "INVALID_POLICY",
                "temperature_observational_channels must be exactly ambient",
            )
        self.gating_channels = frozenset(str(value) for value in gating_channels)
        self.observational_channels = frozenset(
            str(value) for value in observational_channels
        )

    def _validate_activation_record(self) -> dict[str, Any]:
        expected_sha256 = str(self.policy["activation_record_sha256"]).lower()
        actual_sha256 = file_sha256(self.activation_record_path)
        if actual_sha256 != expected_sha256:
            raise CalibrationGuardError(
                "ACTIVATION_SHA_MISMATCH",
                f"expected={expected_sha256}, actual={actual_sha256}",
            )
        record = load_json_object(self.activation_record_path, "activation record")
        if record.get("schema_version") != "sie_calibration_activation_v1":
            raise CalibrationGuardError(
                "INVALID_ACTIVATION_RECORD", "unexpected schema_version"
            )
        if record.get("status") != self.policy["required_activation_status"]:
            raise CalibrationGuardError(
                "CALIBRATION_NOT_ACTIVE", f"status={record.get('status')!r}"
            )
        if record.get("calibration_id") != self.policy["calibration_id"]:
            raise CalibrationGuardError(
                "CALIBRATION_ID_MISMATCH", "activation record calibration_id differs"
            )
        if record.get("calibration_sha256") != self.policy["calibration_sha256"]:
            raise CalibrationGuardError(
                "CALIBRATION_SHA_MISMATCH", "activation record points to another NPZ"
            )
        requirements = record.get("runtime_requirements")
        if not isinstance(requirements, dict):
            raise CalibrationGuardError(
                "INVALID_ACTIVATION_RECORD", "runtime_requirements missing"
            )
        if requirements.get("outside_envelope_action") != "BLOCK_DEPTH_MEASUREMENT":
            raise CalibrationGuardError(
                "INVALID_ACTIVATION_RECORD", "outside-envelope action is not fail-closed"
            )
        if requirements.get("hidden_depth_scale_or_offset_correction_allowed") is not False:
            raise CalibrationGuardError(
                "INVALID_ACTIVATION_RECORD", "hidden depth correction must be forbidden"
            )
        if requirements.get("capture_mode") != self.policy["capture_mode"]:
            raise CalibrationGuardError(
                "CAPTURE_MODE_MISMATCH", "policy and activation record differ"
            )
        if set(record.get("temperature_gating_channels", [])) != set(
            self.gating_channels
        ):
            raise CalibrationGuardError(
                "INVALID_ACTIVATION_RECORD",
                "activation temperature gating channels differ from policy",
            )
        if set(record.get("temperature_observational_channels", [])) != set(
            self.observational_channels
        ):
            raise CalibrationGuardError(
                "INVALID_ACTIVATION_RECORD",
                "activation observational channels differ from policy",
            )
        envelope = record.get("validated_temperature_envelope_c")
        if not isinstance(envelope, dict) or set(envelope) != set(
            self.gating_channels
        ):
            raise CalibrationGuardError(
                "INVALID_ACTIVATION_RECORD",
                "temperature envelope must contain exactly the gating channels",
            )
        normalized_envelope: dict[str, dict[str, float]] = {}
        for channel in sorted(self.gating_channels):
            limits = envelope[channel]
            if not isinstance(limits, dict):
                raise CalibrationGuardError(
                    "INVALID_ACTIVATION_RECORD", f"invalid envelope for {channel}"
                )
            minimum = float(limits["minimum_c"])
            maximum = float(limits["maximum_c"])
            if not all(map(math.isfinite, (minimum, maximum))) or minimum > maximum:
                raise CalibrationGuardError(
                    "INVALID_ACTIVATION_RECORD", f"invalid limits for {channel}"
                )
            normalized_envelope[channel] = {
                "minimum_c": minimum,
                "maximum_c": maximum,
            }
        self.envelope = normalized_envelope
        self.activation_record = record
        return record

    def check_before_measurement(self) -> TemperatureGateResult:
        if self.activation_record is None or self.envelope is None:
            raise CalibrationGuardError(
                "GUARD_NOT_STARTED", "startup() must succeed before measurement checks"
            )
        state = load_json_object(self.temperature_state_path, "temperature state")
        if state.get("status") != "OK":
            raise CalibrationGuardError(
                "TEMPERATURE_STATE_NOT_OK", f"status={state.get('status')!r}"
            )
        if state.get("reset_detected") is True:
            raise CalibrationGuardError(
                "TEMPERATURE_CONTROLLER_RESET", "ESP32 reset detected"
            )
        try:
            updated_at = float(state["updated_at_unix_s"])
        except (KeyError, TypeError, ValueError) as error:
            raise CalibrationGuardError(
                "INVALID_TEMPERATURE_STATE", "updated_at_unix_s missing or invalid"
            ) from error
        age_s = float(self.now_unix_s()) - updated_at
        maximum_age_s = float(self.policy["maximum_temperature_state_age_s"])
        if age_s < -1.0:
            raise CalibrationGuardError(
                "TEMPERATURE_TIMESTAMP_IN_FUTURE", f"age={age_s:.3f}s"
            )
        if age_s > maximum_age_s:
            raise CalibrationGuardError(
                "TEMPERATURE_STATE_STALE",
                f"age={age_s:.3f}s, limit={maximum_age_s:.3f}s",
            )
        mapping = state.get("rom_mapping")
        if not isinstance(mapping, dict):
            raise CalibrationGuardError("ROM_MAPPING_MISMATCH", "mapping missing")
        normalized_mapping = {
            str(key): str(value).upper() for key, value in mapping.items()
        }
        if normalized_mapping != self.policy["expected_rom_mapping"]:
            raise CalibrationGuardError(
                "ROM_MAPPING_MISMATCH",
                f"expected={self.policy['expected_rom_mapping']}, actual={normalized_mapping}",
            )
        temperatures = state.get("temperatures_c")
        if not isinstance(temperatures, dict) or set(temperatures) != REQUIRED_CHANNELS:
            raise CalibrationGuardError(
                "INVALID_TEMPERATURE_STATE", "exactly three channels are required"
            )
        normalized_temperatures: dict[str, float] = {}
        violations = []
        for channel in sorted(REQUIRED_CHANNELS):
            try:
                value = float(temperatures[channel])
            except (TypeError, ValueError) as error:
                raise CalibrationGuardError(
                    "INVALID_TEMPERATURE_STATE", f"invalid {channel} value"
                ) from error
            if not math.isfinite(value):
                raise CalibrationGuardError(
                    "INVALID_TEMPERATURE_STATE", f"non-finite {channel} value"
                )
            normalized_temperatures[channel] = value
            if channel in self.gating_channels:
                limits = self.envelope[channel]
                if not limits["minimum_c"] <= value <= limits["maximum_c"]:
                    violations.append(
                        f"{channel}={value:.4f}C outside "
                        f"[{limits['minimum_c']:.4f}, "
                        f"{limits['maximum_c']:.4f}]C"
                    )
        if violations:
            raise CalibrationGuardError(
                "TEMPERATURE_OUTSIDE_VALIDATED_ENVELOPE", "; ".join(violations)
            )
        return TemperatureGateResult(
            checked_at_utc=datetime.now(timezone.utc).isoformat(),
            state_age_s=age_s,
            temperatures_c=normalized_temperatures,
        )

    def startup(self) -> GuardedCalibration:
        self._validate_activation_record()
        temperature_gate = self.check_before_measurement()

        expected_sha256 = str(self.policy["calibration_sha256"]).lower()
        if not self.calibration_path.is_file():
            raise CalibrationGuardError(
                "MISSING_FILE", f"calibration: {self.calibration_path}"
            )
        actual_sha256 = file_sha256(self.calibration_path)
        if actual_sha256 != expected_sha256:
            raise CalibrationGuardError(
                "CALIBRATION_SHA_MISMATCH",
                f"expected={expected_sha256}, actual={actual_sha256}",
            )
        try:
            with np.load(str(self.calibration_path), allow_pickle=False) as archive:
                missing = sorted(REQUIRED_CALIBRATION_KEYS - set(archive.files))
                if missing:
                    raise CalibrationGuardError(
                        "INVALID_CALIBRATION", f"missing keys: {missing}"
                    )
                parameters = {key: np.array(archive[key], copy=True) for key in archive.files}
        except CalibrationGuardError:
            raise
        except Exception as error:
            raise CalibrationGuardError(
                "INVALID_CALIBRATION", f"cannot load NPZ: {error}"
            ) from error

        calibration_id = scalar_text(parameters["calibration_id"], "calibration_id")
        if calibration_id != self.policy["calibration_id"]:
            raise CalibrationGuardError(
                "CALIBRATION_ID_MISMATCH",
                f"expected={self.policy['calibration_id']}, actual={calibration_id}",
            )
        return GuardedCalibration(
            calibration_id=calibration_id,
            calibration_sha256=actual_sha256,
            activation_record_sha256=str(
                self.policy["activation_record_sha256"]
            ).lower(),
            parameters=parameters,
            temperature_gate=temperature_gate,
        )
