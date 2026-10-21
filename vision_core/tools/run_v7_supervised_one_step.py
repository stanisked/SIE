#!/usr/bin/env python3
"""One operator action with a fresh V7 diagnostic veto; not autonomous admission."""
from __future__ import annotations

import argparse
import fcntl
from copy import deepcopy
from datetime import datetime, timezone
import hashlib
import json
import math
from pathlib import Path
import secrets
import signal
import subprocess
import sys
import time

ROOT = Path(__file__).resolve().parents[2]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from firmware.esp32.tools.run_remote_stop_trial import run as stop_trial
from vision_core.person_localization.control_input_shadow import evaluate_control_input_shadow
from vision_core.person_localization.existing_static_profile_preview import (
    REVIEWED_SHA, SOURCE_HASHES, EXTRINSIC_SHA)

TOPIC = '/sie/diagnostics/ar0234_ov9281/control_preview'
SCOPE = 'OPERATOR_ONE_STEP_WITH_V7_DIAGNOSTIC_VETO'
MAX_AGE_S = .5
MODEL_SHA = '9920f0c6b74bcd5def5bfd68cfc05295557143ddc19035f9ab0ef52d00d245fc'


def finite(value):
    return type(value) in (int, float) and math.isfinite(value)


def check_observation(audit, armed_at, now):
    """Recompute checks, never accept a caller's precomputed PASS/permissions."""
    if not finite(armed_at) or not finite(now) or now < armed_at:
        raise ValueError('INVALID_LOCAL_CLOCK')
    cid = audit.get('cycle_id')
    if not isinstance(cid, str) or not cid:
        raise ValueError('MISSING_CURRENT_CYCLE_ID')
    if audit.get('schema_version') != 'sie.ar0234_ov9281.control_input_preview.v1':
        raise ValueError('UNEXPECTED_DIAGNOSTIC_SCHEMA')
    expected = {'ar0234_intrinsic_sha256': SOURCE_HASHES['ar0234_intrinsic'],
        'ov9281_stereo_sha256': SOURCE_HASHES['ov9281_stereo_calibration'],
        'ar0234_to_physical_left_extrinsic_sha256': EXTRINSIC_SHA}
    if audit.get('calibration') != expected:
        raise ValueError('CALIBRATION_BINDING_MISMATCH')
    context = audit.get('existing_static_profile_preview') or {}
    if context.get('required') is not True or context.get('profile_sha256') != REVIEWED_SHA:
        raise ValueError('STATIC_V2_REQUIRED')
    targets = audit.get('working_targets') or {}
    if (targets.get('freshness_policy_id') != 'experimental_freshness_500ms_v1'
            or targets.get('exposure_to_command_age_max_s') != MAX_AGE_S):
        raise ValueError('EXPERIMENTAL_500MS_PROFILE_REQUIRED')
    if audit.get('activated_depth_range_m') != [.5, 4.5]:
        raise ValueError('UNEXPECTED_ACTIVATED_RANGE')
    shape = audit.get('shape_diagnostic') or {}
    segmentation = audit.get('segmentation_observation') or {}
    target = audit.get('ar0234_dynamic_target') or {}
    candidate = target.get('candidate') or {}
    mask = shape.get('mask') or {}
    instances = segmentation.get('instances')
    if (target.get('current_eligible_count') != 1 or not candidate
            or type(mask.get('instance_index')) is not int
            or candidate.get('instance_index') != mask.get('instance_index')
            or not isinstance(instances, list)
            or sum(item == candidate for item in instances) != 1):
        raise ValueError('SELECTED_INSTANCE_BINDING_MISMATCH')
    if (shape.get('cycle_id') != cid or segmentation.get('cycle_id') != cid
            or shape.get('model_sha256') != MODEL_SHA
            or shape.get('model_sha256') != segmentation.get('model_sha256')
            or (shape.get('mask') or {}).get('source_rgb_arrays_sha256') != segmentation.get('arrays_sha256')):
        raise ValueError('CURRENT_RGB_FUSION_BINDING_MISMATCH')
    if (shape.get('mask_depth') or {}).get('status') != 'COHERENT_MASKED_DEPTH_DIAGNOSTIC_ONLY':
        raise ValueError('LEGACY_MASK_QUALITY_REFUSED')
    bound = evaluate_control_input_shadow(audit)
    if bound['decision_preview']['failed_checks']:
        raise ValueError('CURRENT_CHECK_FAILED:' + ','.join(bound['decision_preview']['failed_checks']))
    # Require complete current algorithmic prerequisites, while preserving all
    # physical/policy unknowns in the operator trial record.
    if bound['depth_admission_draft']['sensor_prerequisites'] != 'PRESENT_NOT_ADMITTED':
        raise ValueError('SENSOR_PREREQUISITES_UNAVAILABLE')
    frame = audit.get('frame_freshness') or {}
    if frame.get('capture_backend') != 'latest_v4l2':
        raise ValueError('DRIVER_FRAME_CLOCK_REQUIRED')
    stamps = []
    for camera in ('ar0234', 'ov9281'):
        packet = (frame.get('frames') or {}).get(camera) or {}
        stamp = packet.get('driver_timestamp_s')
        received = packet.get('host_packet_received_monotonic_s')
        flags = packet.get('flags')
        if type(flags) is not int or flags & 0xe000 != 0x2000 or flags & 0x70000 != 0x10000:
            raise ValueError('MONOTONIC_SOE_DRIVER_FLAGS_REQUIRED')
        if not finite(stamp) or not finite(received) or not armed_at <= stamp <= received <= now:
            raise ValueError('FRAME_NOT_CAPTURED_AFTER_OPERATOR_CONFIRMATION')
        stamps.append(stamp)
    age = now - min(stamps)
    if age > MAX_AGE_S:
        raise ValueError('DRIVER_AGE_AT_DISPATCH_EXCEEDS_500MS')
    return {'source_cycle_id': cid, 'oldest_driver_stamp_s': min(stamps),
        'driver_age_at_check_s': age, 'maximum_age_s': MAX_AGE_S,
        'timestamp_basis': 'DRIVER_DECLARED_MONOTONIC_SOE_TO_HOST_POST_DISPATCH',
        'esp32_acceptance_age_established': False,
        'physical_exposure_timing_verified': False,
        'diagnostic_decision': bound['decision_preview'],
        'diagnostic_is_admitted_measurement': False,
        'scope': SCOPE, 'missing_checks': bound['decision_preview']['missing_checks']}


def execute_observed_step(audit, *, armed_at, base_url, output, request=None,
                          clock=time.monotonic, sleep=time.sleep):
    # Freeze the chosen current record; subsequent frames cannot substitute its
    # identity. Recheck after status HTTP so preflight cannot age it silently.
    frozen = deepcopy(audit)
    check_observation(frozen, armed_at, clock())
    return stop_trial(base_url=base_url, output=output, execute=True,
        request=request, clock=clock, sleep=sleep,
        before_move=lambda status: check_observation(frozen, armed_at, clock()))


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--execute', action='store_true')
    p.add_argument('--base-url')
    p.add_argument('--front-clearance-m', type=float,
        help='Operator-measured horizontal clearance from cart front along the clear straight path')
    p.add_argument('--launch-cameras', action='store_true',
        help='Start existing static-v2 camera launcher after confirmation')
    args = p.parse_args(argv)
    if not args.execute:
        print('NO_EXECUTION: no cameras, ROS, network or motors opened. Use --execute for one confirmed operator trial.')
        return 0
    if not args.base_url or not finite(args.front_clearance_m) or args.front_clearance_m <= 1.1:
        p.error('--execute requires --base-url and --front-clearance-m > 1.1 (0.10 m step + 1.0 m stand-off)')
    lock_path = Path.home()/'.local/state/sie/v7_operator_one_step.lock'
    lock_path.parent.mkdir(parents=True, exist_ok=True)
    lock = lock_path.open('a')
    try:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError:
        lock.close()
        print('REFUSED: another one-step runner holds the local lock')
        return 1
    # Import before confirmation; unavailable ROS must never partially arm a run.
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
    from std_msgs.msg import String

    token = 'ONE-STEP-' + secrets.token_hex(3)
    print('One supervised straight 0.10 m request, followed by one /stop request after 0.30 s.')
    print('This is your operator action. Diagnostic Measurement admission remains blocked.')
    print('Confirm: target and cart stationary; cart faces the target; entire straight path clear; power cutoff accessible.')
    print('Unknown depth/stop bounds remain experimental. SIGKILL, power or network loss cannot guarantee remote stop.')
    if input('Type ' + token + ' to authorize this one step: ').strip() != token:
        print('NOT_CONFIRMED: nothing started')
        return 1
    armed_at = time.monotonic()
    output = Path.home()/'.local/state/sie/evidence'/('v7_operator_one_step_' +
        datetime.now().strftime('%Y%m%d_%H%M%S') + '_' + secrets.token_hex(4))
    output.mkdir(parents=True, exist_ok=False)
    summary = {'scope': SCOPE, 'operator_confirmation': token,
        'armed_at_monotonic_s': armed_at, 'operator_front_clearance_m': args.front_clearance_m,
        'clearance_source': 'OPERATOR_DECLARED_NOT_CAMERA_MEASUREMENT',
        'operator_conditions': ['STATIONARY_TARGET_AND_CART', 'FACING_TARGET', 'STRAIGHT_PATH_CLEAR', 'POWER_CUTOFF_ACCESSIBLE'],
        'canonical_action_created': False, 'measurement_admission_changed': False,
        'script_sha256': hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
        'result': 'NO_MOVEMENT', 'movement_posts': 0, 'stop_posts': 0}
    (output/'operator_authorization.json').write_text(json.dumps(summary, indent=2, allow_nan=False)+'\n')
    process = None
    node = None
    received = []
    try:
        rclpy.init()
        node = Node('sie_v7_operator_one_step_' + secrets.token_hex(3))
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.VOLATILE,
                         reliability=ReliabilityPolicy.RELIABLE)
        def callback(message):
            try:
                a = json.loads(message.data)
                evidence = check_observation(a, armed_at, time.monotonic())
            except (ValueError, TypeError, KeyError, AttributeError) as error:
                summary['last_camera_refusal'] = str(error)
                return
            received[:] = [(a, evidence)]
        subscription = node.create_subscription(String, TOPIC, callback, qos)
        if args.launch_cameras:
            process = subprocess.Popen(['bash', str(ROOT/'vision_core/tools/run_control_preview_existing_static_v2.sh')],
                cwd=ROOT, start_new_session=True)
        deadline = time.monotonic() + 45
        while not received and time.monotonic() < deadline:
            if process is not None and process.poll() is not None:
                raise RuntimeError('Camera launcher ended before a usable current observation')
            rclpy.spin_once(node, timeout_sec=.05)
        if not received:
            raise RuntimeError('No usable post-confirmation camera observation within 45 s')
        audit, evidence = received[0]
        raw = json.dumps(audit, allow_nan=False).encode()
        (output/'source_audit.json').write_bytes(raw)
        summary.update(source_cycle_id=audit['cycle_id'], source_audit_sha256=hashlib.sha256(raw).hexdigest(),
            camera_evidence=evidence)
        # This call performs fresh controller preflight, last-moment camera-age
        # veto, one move POST and one stop attempt, never movement retries.
        result = execute_observed_step(audit, armed_at=armed_at, base_url=args.base_url,
                                      output=output/'controller')
        summary['controller'] = result
        summary.update(result=result['result'], movement_posts=result['movement_posts'], stop_posts=result['stop_posts'])
    except (Exception, KeyboardInterrupt) as error:
        summary['error'] = type(error).__name__ + ': ' + str(error)
    finally:
        # Preserve the outcome even if ROS/process shutdown itself fails.
        (output/'summary.json').write_text(json.dumps(summary, indent=2, allow_nan=False)+'\n')
        try:
            if process is not None and process.poll() is None:
                import os
                os.killpg(process.pid, signal.SIGINT)
                try:
                    process.wait(timeout=12)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGTERM)
                    process.wait(timeout=3)
            if node is not None:
                node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
        except Exception as error:
            summary['cleanup_error'] = type(error).__name__ + ': ' + str(error)
        finally:
            lock.close()
        (output/'summary.json').write_text(json.dumps(summary, indent=2, allow_nan=False)+'\n')
        print('V7_ONE_STEP', summary['result'], 'movement_posts=', summary['movement_posts'], 'stop_posts=', summary['stop_posts'])
        print('Saved:', output/'summary.json')
    return 0 if summary['result'] == 'REMOTE_STOP_OBSERVED_DIAGNOSTIC_ONLY' else 1


if __name__ == '__main__':
    raise SystemExit(main())
