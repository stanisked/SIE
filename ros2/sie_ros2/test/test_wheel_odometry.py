"""Synthetic wheel counts, with no physical or calibration evidence."""
from dataclasses import FrozenInstanceError, replace
from math import cos, pi, sin

import pytest

from firmware.esp32.tools.wheel_encoder_telemetry_v1 import EncoderSample
from sie_ros2.wheel_odometry import Pose, WheelGeometry, WheelOdometry, integrate_arc

BOOT = "0123456789ABCDEF"
REBOOT = "1123456789ABCDEF"


def sample(seq, right=0, left=0, **overrides):
    base = EncoderSample(BOOT, seq, seq * 100_000, right, left, "READY",
                         False, False, False, False, False)
    return replace(base, **overrides)


def engine(geometry=None):
    return WheelOdometry(geometry or WheelGeometry(), max_interval_us=350_000)


@pytest.mark.parametrize("right,left", [(205, 206), (-205, -206), (0, 0), (205, -206), (-205, 206)])
def test_straight_reverse_stationary_and_both_turn_signs(right, left):
    model = engine(WheelGeometry(effective_right_wheel_diameter_m=0.065,
                                 effective_left_wheel_diameter_m=0.065))
    assert model.observe(sample(1, 500, -20), verified_boot=BOOT).estimate is None
    outcome = model.observe(sample(2, 500 + right, -20 + left), verified_boot=BOOT)
    d_right = pi * .065 * right / 205
    d_left = pi * .065 * left / 206
    assert outcome.reason == "VALID"
    assert (outcome.estimate.pose.x, outcome.estimate.pose.y, outcome.estimate.pose.yaw) == pytest.approx(
        ((d_right + d_left) / 2, 0.0, (d_right - d_left) / .290)
    )
    assert outcome.estimate.linear_x == pytest.approx((d_right + d_left) / .2)
    assert outcome.estimate.angular_z == pytest.approx((d_right - d_left) / .029)


def test_exact_curve_with_different_effective_diameters_and_body_frame():
    geometry = WheelGeometry()
    model = engine(geometry)
    model.observe(sample(8), verified_boot=BOOT)
    for seq in (9, 10):
        n = seq - 8
        out = model.observe(sample(seq, 205 * n, 55 * n), verified_boot=BOOT)
    right, left = pi * .0646, pi * .0652 * 55 / 206
    angle = (right - left) / .290
    radius = (right + left) / (2 * angle)
    assert out.estimate.pose.x == pytest.approx(radius * sin(2 * angle))
    assert out.estimate.pose.y == pytest.approx(radius * (1 - cos(2 * angle)))
    assert out.estimate.pose.yaw == pytest.approx(2 * angle)
    assert replace(geometry, physical_right_wheel_diameter_m=.07).wheel_displacements(205, 206) == (
        geometry.wheel_displacements(205, 206)
    )


def test_near_zero_curvature_has_resolved_lateral_component():
    pose = integrate_arc(Pose(), 2.0, 1e-12)
    assert pose.x == pytest.approx(2.0)
    assert pose.y == pytest.approx(1e-12, abs=1e-24)
    assert pose.yaw == 1e-12


@pytest.mark.parametrize("values", [
    {"effective_right_wheel_diameter_m": 0.0},
    {"physical_left_wheel_diameter_m": float("nan")},
    {"track_width_m": float("inf")},
    {"right_encoder_ticks_per_rev": 205.5},
    {"left_encoder_ticks_per_rev": True},
])
def test_invalid_parameters_and_frozen_geometry(values):
    with pytest.raises(ValueError):
        WheelGeometry(**values)
    with pytest.raises(FrozenInstanceError):
        WheelGeometry().track_width_m = 1.0


def test_gap_skips_net_displacement_and_retains_pose():
    model = engine()
    model.observe(sample(1), verified_boot=BOOT)
    good = model.observe(sample(2, 10, 10), verified_boot=BOOT).estimate
    gap = model.observe(sample(4, 90, -30), verified_boot=BOOT)
    assert gap.reason == "SAMPLE_GAP" and gap.estimate is None
    assert model.pose == good.pose
    result = model.observe(sample(5, 90, -30), verified_boot=BOOT)
    assert result.estimate.pose == good.pose
    assert result.estimate.discontinuities == 1


@pytest.mark.parametrize("item,reason", [
    (sample(2, 10, 10), "DUPLICATE"),
    (sample(2, 11, 10), "CONFLICTING_DUPLICATE"),
    (sample(1), "OUT_OF_ORDER"),
    (sample(3, sample_monotonic_us=200_000), "NON_INCREASING_TIMESTAMP"),
    (sample(3, sample_monotonic_us=800_000), "DEVICE_SAMPLE_INTERVAL_EXCEEDED"),
    (sample(3, controller_state="FAULT"), "CONTROLLER_FAULT"),
    (sample(3, steered_safety_latched=True), "STEERED_SAFETY_LATCHED"),
    (sample(3, fault_latched=True), "FAULT_LATCHED"),
    (sample(3, right_counter_overflow=True), "RIGHT_COUNTER_OVERFLOW"),
    (sample(3, sequence_exhausted=True), "SEQUENCE_EXHAUSTED"),
])
def test_refusal_preserves_pose(item, reason):
    model = engine()
    model.observe(sample(1), verified_boot=BOOT)
    good = model.observe(sample(2, 10, 10), verified_boot=BOOT).estimate
    update = model.observe(item, verified_boot=BOOT)
    assert update.reason == reason and update.estimate is None
    assert model.pose == good.pose
    assert model.discontinuities == 1


def test_reboot_requires_verified_status_and_retires_prior_boot():
    model = engine()
    model.observe(sample(50, 400, -200), verified_boot=BOOT)
    good = model.observe(sample(51, 410, -190), verified_boot=BOOT).estimate
    boundary = model.observe(sample(1, 2, -3, boot_session_id=REBOOT), verified_boot=BOOT)
    assert boundary.reason == "FIRMWARE_STATUS_REQUIRED"
    assert boundary.disposition == "REBOOT_BASELINE"
    assert model.observe(sample(2, 9, 4, boot_session_id=REBOOT), verified_boot=REBOOT).reason == "RECOVERY_BASELINE"
    new = model.observe(sample(3, 9, 4, boot_session_id=REBOOT), verified_boot=REBOOT)
    assert new.estimate.pose == good.pose and new.estimate.discontinuities == 1
    assert model.observe(sample(52, 5000), verified_boot=REBOOT).reason == "RETIRED_BOOT"
    assert model.pose == good.pose


@pytest.mark.parametrize("reason", ["HOST_TELEMETRY_STALE", "ROS_CLOCK_STEP", "UART_REPLY_TIMEOUT"])
def test_time_refusal_or_timeout_requires_new_baseline(reason):
    model = engine()
    model.observe(sample(1), verified_boot=BOOT)
    good = model.observe(sample(2, 10, 10), verified_boot=BOOT).estimate
    if reason == "UART_REPLY_TIMEOUT":
        assert model.interrupt(reason).estimate is None
        next_seq = 3
    else:
        assert model.observe(sample(3, 50, 50), verified_boot=BOOT, timing_refusal=reason).estimate is None
        next_seq = 4
    assert model.observe(sample(next_seq, 70, 70), verified_boot=BOOT).reason == "RECOVERY_BASELINE"
    assert model.observe(sample(next_seq + 1, 70, 70), verified_boot=BOOT).estimate.pose == good.pose
