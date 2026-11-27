"""Wheel kinematics and conservative boot-scoped odometry integration."""
from dataclasses import dataclass, fields
from math import cos, isfinite, pi, remainder, sin, tau

from firmware.esp32.tools.wheel_encoder_telemetry_v1 import (
    BOOL_FIELDS, EncoderSample, EncoderSampleContinuity,
)


@dataclass(frozen=True)
class WheelGeometry:
    physical_right_wheel_diameter_m: float = 0.068
    physical_left_wheel_diameter_m: float = 0.068
    effective_right_wheel_diameter_m: float = 0.0646
    effective_left_wheel_diameter_m: float = 0.0652
    right_encoder_ticks_per_rev: int = 205
    left_encoder_ticks_per_rev: int = 206
    track_width_m: float = 0.290

    def __post_init__(self):
        for field in fields(self):
            value = getattr(self, field.name)
            if field.name.endswith("ticks_per_rev"):
                valid = type(value) is int and value > 0
            else:
                valid = type(value) in (int, float) and isfinite(value) and value > 0
            if not valid:
                raise ValueError(f"Invalid wheel parameter: {field.name}")

    def wheel_displacements(self, right_ticks, left_ticks):
        return (
            pi * self.effective_right_wheel_diameter_m * (right_ticks / self.right_encoder_ticks_per_rev),
            pi * self.effective_left_wheel_diameter_m * (left_ticks / self.left_encoder_ticks_per_rev),
        )


@dataclass(frozen=True)
class Pose:
    x: float = 0.0
    y: float = 0.0
    yaw: float = 0.0


def integrate_arc(pose, distance, angle):
    half = angle / 2
    chord = distance * (sin(half) / half if half else 1.0)
    heading = pose.yaw + half
    return Pose(
        pose.x + chord * cos(heading), pose.y + chord * sin(heading),
        remainder(pose.yaw + angle, tau),
    )


@dataclass(frozen=True)
class Estimate:
    pose: Pose
    linear_x: float
    angular_z: float
    distance_travelled: float
    heading_travelled: float
    discontinuities: int


@dataclass(frozen=True)
class Update:
    reason: str
    disposition: str | None = None
    estimate: Estimate | None = None


class WheelOdometry:
    def __init__(self, geometry: WheelGeometry, *, max_interval_us: int):
        if type(max_interval_us) is not int or max_interval_us <= 0:
            raise ValueError("max_interval_us must be a positive integer")
        self.geometry = geometry
        self.max_interval_us = max_interval_us
        self.pose = Pose()
        self.anchor: EncoderSample | None = None
        self.continuity = EncoderSampleContinuity()
        self.discontinuities = 0
        self.distance_travelled = 0.0
        self.heading_travelled = 0.0

    def interrupt(self, reason, disposition=None):
        if self.anchor is not None:
            self.anchor = None
            self.discontinuities += 1
        return Update(reason, disposition)

    def observe(self, sample, *, verified_boot, timing_refusal=None):
        disposition = self.continuity.observe(sample).status
        flags = [key.upper() for key in BOOL_FIELDS if getattr(sample, key)]
        if flags or sample.controller_state == "FAULT":
            return self.interrupt(",".join(flags) or "CONTROLLER_FAULT", disposition)
        if disposition not in ("BASELINE", "REBOOT_BASELINE", "CONSECUTIVE", "SAMPLE_GAP"):
            return self.interrupt(disposition, disposition)
        if timing_refusal:
            return self.interrupt(timing_refusal, disposition)
        if sample.boot_session_id != verified_boot:
            return self.interrupt("FIRMWARE_STATUS_REQUIRED", disposition)
        if disposition in ("REBOOT_BASELINE", "SAMPLE_GAP"):
            self.interrupt(disposition)
            self.anchor = sample
            return Update(disposition, disposition)
        if self.anchor is None:
            self.anchor = sample
            return Update("BASELINE" if disposition == "BASELINE" else "RECOVERY_BASELINE", disposition)

        previous = self.anchor
        elapsed_us = sample.sample_monotonic_us - previous.sample_monotonic_us
        if elapsed_us > self.max_interval_us:
            self.interrupt("DEVICE_SAMPLE_INTERVAL_EXCEEDED")
            self.anchor = sample
            return Update("DEVICE_SAMPLE_INTERVAL_EXCEEDED", disposition)
        right, left = self.geometry.wheel_displacements(
            sample.right_total_signed_ticks - previous.right_total_signed_ticks,
            sample.left_total_signed_ticks - previous.left_total_signed_ticks,
        )
        distance = right / 2 + left / 2
        angle = (right - left) / self.geometry.track_width_m
        if not all(isfinite(value) for value in (distance, angle)):
            return self.interrupt("KINEMATICS_NONFINITE", disposition)
        pose = integrate_arc(self.pose, distance, angle)
        elapsed_s = elapsed_us / 1_000_000
        estimate = Estimate(
            pose, distance / elapsed_s, angle / elapsed_s,
            self.distance_travelled + abs(right) / 2 + abs(left) / 2,
            self.heading_travelled + abs(angle), self.discontinuities,
        )
        if not all(isfinite(value) for value in (
            pose.x, pose.y, pose.yaw, estimate.linear_x, estimate.angular_z,
            estimate.distance_travelled, estimate.heading_travelled,
        )):
            return self.interrupt("KINEMATICS_NONFINITE", disposition)
        self.pose = pose
        self.anchor = sample
        self.distance_travelled = estimate.distance_travelled
        self.heading_travelled = estimate.heading_travelled
        return Update("VALID", disposition, estimate)
