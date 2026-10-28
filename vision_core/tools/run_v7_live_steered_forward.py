Failed to create stream fd: Operation not permitted
Failed to create stream fd: Operation not permitted
Failed to create stream fd: Operation not permitted
#!/usr/bin/env python3
"""Operator supervised bounded forward command with live RGB steering updates."""
from __future__ import annotations

import argparse
import fcntl
from itertools import chain
import json
import math
import os
from pathlib import Path
import secrets
import signal
import subprocess
import sys
import time
from urllib.parse import urlencode, urlsplit
from urllib.error import URLError

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
from firmware.esp32.tools.run_remote_stop_trial import Http, preflight
from vision_core.tools.run_v7_supervised_one_step import (
    TOPIC, EXTRINSIC_SHA, MAX_AGE_S, MODEL_SHA, SOURCE_HASHES)
from vision_core.tools.run_v7_supervised_approach import overlap, target_bearing_deg

BUILD = 'v4_2_live_steering_v4'
DISTANCE_M = 0.10
MAX_DISTANCE_M = 0.30
MAX_BIAS_PWM = 20
STEER_GAIN_PWM_PER_DEG = 4.0
STEER_DEADBAND_DEG = 0.5
FEEDBACK_TIMEOUT_S = 0.6  # ESP32 owns the independent 700 ms deadline.
TERMINAL = {'SUCCESS', 'PARTIAL_PROGRESS', 'STOPPED', 'FAULT'}


def check_live_rgb_target(audit, armed_at, now):
    """Validate a fresh current-frame RGB person without the static depth gate.

    This is a supervised diagnostic target, never an admitted Measurement.
    It intentionally ignores static-v2 temporal stillness and mask-depth status.
    """
    if (type(armed_at) not in (int, float) or not math.isfinite(armed_at) or
            type(now) not in (int, float) or not math.isfinite(now) or now < armed_at):
        raise ValueError('INVALID_LOCAL_CLOCK')
    cid = audit.get('cycle_id')
    if not isinstance(cid, str) or not cid or \
            audit.get('schema_version') != 'sie.ar0234_ov9281.control_input_preview.v1':
        raise ValueError('INVALID_CURRENT_RGB_RECORD')
    expected = {'ar0234_intrinsic_sha256': SOURCE_HASHES['ar0234_intrinsic'],
                'ov9281_stereo_sha256': SOURCE_HASHES['ov9281_stereo_calibration'],
                'ar0234_to_physical_left_extrinsic_sha256': EXTRINSIC_SHA}
    if audit.get('calibration') != expected:
        raise ValueError('CALIBRATION_BINDING_MISMATCH')
    targets = audit.get('working_targets') or {}
    if (targets.get('freshness_policy_id') != 'experimental_freshness_500ms_v1'
            or targets.get('exposure_to_command_age_max_s') != MAX_AGE_S):
        raise ValueError('EXPERIMENTAL_500MS_PROFILE_REQUIRED')
    segmentation = audit.get('segmentation_observation') or {}
    target = audit.get('ar0234_dynamic_target') or {}
    instances = segmentation.get('instances')
    if (segmentation.get('cycle_id') != cid or
            segmentation.get('status') != 'CURRENT_INSTANCES_DIAGNOSTIC_ONLY' or
            segmentation.get('model_sha256') != MODEL_SHA or
            segmentation.get('class_name') != 'person' or
            not isinstance(segmentation.get('arrays_sha256'), str) or
            not segmentation['arrays_sha256']):
        raise ValueError('CURRENT_RGB_SEGMENTATION_BINDING_MISMATCH')
    # The existing dynamic selector rejects a person whose head touches the
    # top image edge, and the static-v2 gate rejects motion. For this RGB-only
    # diagnostic, use one current segmentation instance directly. A top-clipped
    # person is still trackable through adjacent-frame box correspondence.
    if not isinstance(instances, list) or not instances or \
            not all(isinstance(item, dict) for item in instances):
        raise ValueError('CURRENT_RGB_TARGET_UNAVAILABLE_OR_AMBIGUOUS')
    # Segmentation may include weak, side-clipped false positives while the
    # dynamic selector still has exactly one current eligible person. Do not
    # turn that harmless extra instance into a target-loss stop. A second
    # confident person remains ambiguous and is refused.
    plausible = [item for item in instances
                 if type(item.get('confidence')) in (int, float)
                 and math.isfinite(item['confidence'])
                 and item['confidence'] >= .70]
    if len(plausible) != 1:
        raise ValueError('CURRENT_RGB_TARGET_UNAVAILABLE_OR_AMBIGUOUS')
    candidate = plausible[0]
    index = candidate.get('instance_index')
    if type(index) is not int or not 0 <= index < len(instances) or \
            instances[index] != candidate:
        raise ValueError('CURRENT_RGB_INSTANCE_INDEX_MISSING')
    if target.get('disposition') == 'OBSERVED':
        if (target.get('candidate') != candidate or
                target.get('current_eligible_count') != 1):
            raise ValueError('CURRENT_RGB_SELECTOR_DISAGREES_WITH_INSTANCE')
    elif (target.get('reason') != 'NO_CURRENT_NON_EDGE_TARGET' or
          target.get('current_eligible_count') != 0):
        raise ValueError('CURRENT_RGB_TARGET_SELECTOR_REFUSED')
    box = candidate.get('bbox_xyxy_px')
    if (not isinstance(box, list) or len(box) != 4 or
            not all(type(x) in (int, float) and math.isfinite(x) for x in box) or
            not 3 < box[0] < box[2] < 1917 or
            not 0 <= box[1] < box[3] <= 1200 or
            candidate.get('truncated_left') is not False or
            candidate.get('truncated_right') is not False or
            type(candidate.get('confidence')) not in (int, float) or
            not math.isfinite(candidate['confidence']) or
            not .70 <= candidate['confidence'] <= 1.0):
        raise ValueError('CURRENT_RGB_TARGET_GEOMETRY_OR_CONFIDENCE_REFUSED')
    frame = audit.get('frame_freshness') or {}
    packet = (frame.get('frames') or {}).get('ar0234') or {}
    stamp = packet.get('driver_timestamp_s')
    received = packet.get('host_packet_received_monotonic_s')
    flags = packet.get('flags')
    if (frame.get('capture_backend') != 'latest_v4l2' or
            type(flags) is not int or flags & 0xe000 != 0x2000 or
            flags & 0x70000 != 0x10000 or
            type(stamp) not in (int, float) or not math.isfinite(stamp) or
            type(received) not in (int, float) or not math.isfinite(received) or
            not armed_at <= stamp <= received <= now):
        raise ValueError('CURRENT_RGB_DRIVER_FRAME_CLOCK_REFUSED')
    age = now - stamp
    if age > MAX_AGE_S:
        raise ValueError('DRIVER_AGE_AT_DISPATCH_EXCEEDS_500MS')
    return {'source_cycle_id': cid, 'oldest_driver_stamp_s': stamp,
            'driver_age_at_check_s': age, 'maximum_age_s': MAX_AGE_S,
            'target_source': 'CURRENT_FRAME_AR0234_PERSON_SEGMENTATION',
            'target_box_xyxy_px': box,
            'top_truncated': candidate.get('truncated_top'),
            'static_depth_gate_used': False,
            'measurement_admitted': False}


def steering_bias(bearing_deg):
    """Positive target bearing boosts the left forward wheel."""
    if not math.isfinite(bearing_deg):
        raise ValueError('INVALID_TARGET_BEARING')
    if abs(bearing_deg) <= STEER_DEADBAND_DEG:
        return 0
    return max(-MAX_BIAS_PWM, min(MAX_BIAS_PWM,
               round(STEER_GAIN_PWM_PER_DEG * bearing_deg)))


def same_target(previous_box, box):
    """Conservative adjacent-frame correspondence, including modest motion."""
    if overlap(previous_box, box) >= .20:
        return True
    width = max(previous_box[2] - previous_box[0], box[2] - box[0])
    height = max(previous_box[3] - previous_box[1], box[3] - box[1])
    px = (previous_box[0] + previous_box[2]) / 2
    py = (previous_box[1] + previous_box[3]) / 2
    x = (box[0] + box[2]) / 2
    y = (box[1] + box[3]) / 2
    return abs(x - px) <= .35 * width and abs(y - py) <= .35 * height


def valid_controller_url(base_url):
    if not isinstance(base_url, str):
        return False
    p = urlsplit(base_url)
    return (p.scheme == 'http' and bool(p.hostname) and not p.username
            and not p.password and not p.query and not p.fragment
            and p.path in ('', '/'))


def is_transport_timeout(error):
    return isinstance(error, TimeoutError) or (
        isinstance(error, URLError) and isinstance(error.reason, TimeoutError))


def preview_session(*, output, camera_factory, duration_s=10.0,
                    clock=time.monotonic):
    """Journal live camera bearings and intended PWM bias; no HTTP or motors."""
    if not .5 <= duration_s <= 60:
        raise ValueError('INVALID_PREVIEW_DURATION')
    output = Path(output)
    output.mkdir(parents=True, exist_ok=False)
    armed = clock()
    end = armed + duration_s
    previous_box = None
    rows = []
    result = 'NO_TARGET'
    losses = 0
    segment = 0
    try:
        for audit in camera_factory(armed):
            if clock() >= end:
                break
            if audit is None:
                continue
            try:
                evidence = check_live_rgb_target(audit, armed, clock())
                box = evidence['target_box_xyxy_px']
            except (ValueError, TypeError, KeyError) as error:
                target = audit.get('ar0234_dynamic_target') or {}
                candidate = target.get('candidate') or {}
                shape = audit.get('shape_diagnostic') or {}
                segmentation = audit.get('segmentation_observation') or {}
                instances = segmentation.get('instances')
                rows.append({'cycle_id': audit.get('cycle_id'),
                             'phase': 'TARGET_UNAVAILABLE',
                             'refusal': str(error),
                             'camera_status': audit.get('status'),
                             'current_eligible_count': target.get('current_eligible_count'),
                             'target_disposition': target.get('disposition'),
                             'candidate_instance_index': candidate.get('instance_index'),
                             'mask_instance_index': (shape.get('mask') or {}).get('instance_index'),
                             'segmentation_instance_count': (len(instances)
                                                             if isinstance(instances, list) else None)})
                if previous_box is not None:
                    losses += 1
                previous_box = None
                continue
            if previous_box is not None and not same_target(previous_box, box):
                rows.append({'cycle_id': audit['cycle_id'],
                             'phase': 'CORRESPONDENCE_LOST',
                             'box_xyxy_px': box,
                             'previous_box_xyxy_px': previous_box})
                losses += 1
                previous_box = None
            bearing = target_bearing_deg(box)
            if previous_box is None:
                segment += 1
            bias = 0 if previous_box is None else steering_bias(bearing)
            rows.append({'cycle_id': audit['cycle_id'],
                         'driver_stamp_s': evidence['oldest_driver_stamp_s'],
                         'box_xyxy_px': box, 'bearing_deg': bearing,
                         'intended_bias_pwm': bias,
                         'segment': segment,
                         'phase': 'ACQUIRE_STRAIGHT' if previous_box is None else 'LIVE_STEER'})
            previous_box = box
            result = ('PREVIEW_RECORDED_WITH_LOSS' if losses else
                      'PREVIEW_RECORDED')
    finally:
        if result != 'NO_TARGET' and losses:
            result = 'PREVIEW_RECORDED_WITH_LOSS'
        report = {'schema_version': 'sie.live_steering_camera_preview.v1',
                  'result': result, 'motor_posts': 0, 'network_requests': 0,
                  'target_loss_events': losses, 'acquisition_segments': segment,
                  'measurement_admission_changed': False, 'rows': rows}
        (output / 'summary.json').write_text(json.dumps(report, indent=2,
                                                       allow_nan=False) + '\n')
    return report


def run_session(*, base_url, front_clearance_m, output, distance_m=DISTANCE_M, request=None,
                clock=time.monotonic, camera_factory=None):
    """Live session; camera_factory yields successive full diagnostic records.

    The first observation starts a straight command. Only subsequent records
    can change wheel PWM. HTTP attempts and any recovery are journaled.
    """
    if not valid_controller_url(base_url) or not math.isfinite(front_clearance_m):
        raise ValueError('INVALID_OPERATOR_ARGUMENTS')
    if (type(distance_m) not in (int, float) or not math.isfinite(distance_m)
            or not .02 <= distance_m <= MAX_DISTANCE_M):
        raise ValueError('INVALID_BOUNDED_DISTANCE')
    if front_clearance_m <= 1.0 + distance_m:
        raise ValueError('INSUFFICIENT_OPERATOR_FRONT_CLEARANCE')
    output = Path(output)
    output.mkdir(parents=True, exist_ok=False)
    request = request or Http()
    if camera_factory is None:
        raise ValueError('CAMERA_FACTORY_REQUIRED')
    report = {'schema_version': 'sie.live_steered_forward_diagnostic.v1',
              'scope': 'OPERATOR_SUPERVISED_LIVE_STEERING',
              'result': 'NO_MOVEMENT', 'movement_posts': 0,
              'steer_posts': 0, 'stop_posts': 0,
              'requested_forward_m': distance_m,
              'operator_front_clearance_m': front_clearance_m,
              'measurement_admission_changed': False,
              'physical_stopping_bound_established': False,
              'camera_control_enabled': True, 'updates': [],
              'unconfirmed_steer_updates': []}
    journal = (output / 'http.jsonl').open('x')
    sent = False
    command_id = 'sie-live-' + secrets.token_hex(12)
    report['command_id'] = command_id

    def call(method, endpoint, params=None, *, timeout_s=.4):
        url = base_url.rstrip('/') + endpoint
        if params is not None:
            url += '?' + urlencode(params)
        row = {'method': method, 'endpoint': endpoint, 'query': params,
               'host_started_monotonic_s': clock()}
        try:
            code, body = request(method, url, timeout_s)
            row.update(http_code=code, response=body)
            return code, body
        except BaseException as error:
            row['error'] = type(error).__name__ + ': ' + str(error)
            raise
        finally:
            row['host_completed_monotonic_s'] = clock()
            journal.write(json.dumps(row, allow_nan=False) + '\n')
            journal.flush()

    def matching_terminal(status, boot):
        if status.get('boot_session_id') != boot:
            raise RuntimeError('BOOT_SESSION_CHANGED')
        if status.get('last_command_id') == command_id and status.get('last_command_state') in TERMINAL:
            return True
        if status.get('active_command_id') not in (None, command_id):
            raise RuntimeError('ACTIVE_COMMAND_CHANGED')
        return False

    try:
        # This read precedes every motor command. A Wi-Fi delay here is safe
        # to wait through; the tighter active-command deadlines stay intact.
        code, ready = call('GET', '/status', timeout_s=2.0)
        if code != 200:
            raise RuntimeError('CONTROLLER_STATUS_UNAVAILABLE')
        preflight(ready, accepted_builds={BUILD})
        if ready.get('steering_api_version') != 'v1_active_forward_pwm_bias':
            raise RuntimeError('STEERING_API_MISMATCH')
        if ready.get('steered_safety_latched') is not False:
            raise RuntimeError('INDEPENDENT_STEERED_SAFETY_NOT_READY')
        boot = ready['boot_session_id']
        armed = clock()
        observations = camera_factory(armed)
        first_deadline = clock() + 60.0
        first = None
        while clock() < first_deadline:
            candidate = next(observations)
            if candidate is None:
                continue
            try:
                evidence = check_live_rgb_target(candidate, armed, clock())
                first = candidate
                break
            except (ValueError, TypeError, KeyError) as error:
                report['last_camera_refusal'] = str(error)
        if first is None:
            raise RuntimeError('NO_FRESH_TARGET_WITHIN_60S')
        box = evidence['target_box_xyxy_px']
        report['first_camera_evidence'] = evidence
        report['first_box_xyxy_px'] = box
        report['first_bearing_deg'] = target_bearing_deg(box)
        # The first camera detection starts the cart straight, from anywhere
        # in the image. Steering begins with a newly captured second frame.
        check_live_rgb_target(first, armed, clock())
        sent = True  # A timed-out POST may already have started motion.
        report['movement_posts'] = 1
        move_params = {'boot_session_id': boot, 'command_id': command_id,
                       'distance_m': format(distance_m, '.3f')}
        retry_audit = None
        try:
            code, body = call('POST', '/move-forward-steered', move_params)
        except (TimeoutError, URLError) as error:
            if not is_transport_timeout(error):
                raise
            # The ESP32 remembers command IDs and reports duplicates without
            # starting a second motion. Recheck a newer RGB frame before one
            # retry of the identical request; never issue a new command ID.
            report['movement_first_response_timeout'] = True
            retry_deadline = clock() + .35
            while clock() < retry_deadline:
                candidate = next(observations)
                if candidate is None:
                    continue
                new_evidence = check_live_rgb_target(candidate, armed, clock())
                if new_evidence['oldest_driver_stamp_s'] <= evidence['oldest_driver_stamp_s']:
                    continue
                if not same_target(box, new_evidence['target_box_xyxy_px']):
                    raise RuntimeError('TARGET_CORRESPONDENCE_LOST_BEFORE_MOVE_RETRY')
                if clock() > retry_deadline:
                    raise RuntimeError('MOVE_RETRY_CAMERA_DEADLINE')
                retry_audit = candidate
                report['movement_retry_camera_evidence'] = new_evidence
                break
            if retry_audit is None:
                raise RuntimeError('NO_FRESH_TARGET_FOR_MOVE_RETRY')
            report['movement_posts'] += 1
            code, body = call('POST', '/move-forward-steered', move_params)
        duplicate = (code == 200 and body.get('duplicate') is True and
                     body.get('command_id') == command_id and
                     body.get('command_state') in TERMINAL | {'ACCEPTED', 'STARTING', 'DRIVING', 'BRAKING', 'COASTING'})
        if not ((code == 202 and body.get('accepted') is True and
                 body.get('command_id') == command_id) or duplicate):
            raise RuntimeError('MOVEMENT_ACCEPTANCE_UNKNOWN')
        last_frame_stamp = evidence['oldest_driver_stamp_s']
        last_feedback = clock()
        previous_box = box
        seq = 0
        driving_confirmed = False
        terminal_pending_deadline = None
        pending_steer_ack_deadline = None
        # camera_factory yields fresh full records while the same bounded
        # forward command is active. A missing record is a stop condition.
        last_status_poll = clock()
        for audit in chain((retry_audit,) if retry_audit is not None else (), observations):
            now = clock()
            if pending_steer_ack_deadline is not None and now > pending_steer_ack_deadline:
                raise RuntimeError('STEER_ACK_RECOVERY_DEADLINE')
            if terminal_pending_deadline is not None and now > terminal_pending_deadline:
                raise RuntimeError('TERMINAL_OBSERVATION_TIMEOUT')
            if terminal_pending_deadline is not None:
                if now - last_status_poll >= .1:
                    code, status = call('GET', '/status')
                    last_status_poll = clock()
                    if code != 200:
                        raise RuntimeError('CONTROLLER_STATUS_UNAVAILABLE_DURING_BRAKE')
                    if matching_terminal(status, boot):
                        report['terminal_status'] = status
                        break
                    if status.get('bounded_fault_latched') or status.get('state') == 'FAULT':
                        raise RuntimeError('CONTROLLER_FAULT_DURING_BRAKE:' +
                                           str(status.get('bounded_fault_reason')))
                continue
            if audit is None:
                if now - last_status_poll >= .5:
                    code, status = call('GET', '/status')
                    last_status_poll = clock()
                    if code != 200:
                        raise RuntimeError('CONTROLLER_STATUS_UNAVAILABLE_DURING_MOVE')
                    if matching_terminal(status, boot):
                        report['terminal_status'] = status
                        break
                    if status.get('bounded_fault_latched') or status.get('state') == 'FAULT':
                        raise RuntimeError('CONTROLLER_FAULT_DURING_MOVE:' +
                                           str(status.get('bounded_fault_reason')))
                    if status.get('state') in ('BRAKING', 'COASTING'):
                        terminal_pending_deadline = terminal_pending_deadline or clock() + 3.0
            if (terminal_pending_deadline is None and pending_steer_ack_deadline is None
                    and now - last_feedback > FEEDBACK_TIMEOUT_S):
                raise RuntimeError('CAMERA_FEEDBACK_TIMEOUT')
            if audit is None:
                continue
            evidence = check_live_rgb_target(audit, armed, now)
            stamp = evidence['oldest_driver_stamp_s']
            if stamp <= last_frame_stamp:
                continue
            box = evidence['target_box_xyxy_px']
            if not same_target(previous_box, box):
                raise RuntimeError('TARGET_CORRESPONDENCE_LOST')
            # A command-bound nonzero steer acknowledgement proves that the
            # ESP32 is DRIVING: the firmware rejects it during STARTING. This
            # avoids a large /status response on the time-critical first frame.
            check_live_rgb_target(audit, armed, clock())
            seq += 1
            bias = steering_bias(target_bearing_deg(box))
            drive_state = 'DRIVING' if driving_confirmed else 'UNCONFIRMED'
            report['steer_posts'] += 1
            try:
                code, reply = call('POST', '/steer-update',
                                   {'boot_session_id': boot, 'command_id': command_id,
                                    'seq': seq, 'bias_pwm': bias})
            except (TimeoutError, URLError) as error:
                if not is_transport_timeout(error) or pending_steer_ack_deadline is not None:
                    raise
                # One reply may be lost after ESP32 accepted the update. A
                # newer frame may supersede it once; ESP32 independently
                # brakes after 700 ms without an accepted fresh update.
                report['unconfirmed_steer_updates'].append({
                    'cycle_id': audit['cycle_id'], 'seq': seq,
                    'driver_stamp_s': stamp, 'bias_pwm': bias})
                pending_steer_ack_deadline = clock() + .25
                last_frame_stamp = stamp
                previous_box = box
                continue
            if (code == 409 and not driving_confirmed and bias != 0 and
                    reply.get('reason') == 'STEER_ONLY_AFTER_DRIVE_STARTED'):
                # The rejected request changed no controller state. Send one
                # zero-bias heartbeat with the same sequence while starting.
                bias = 0
                drive_state = 'STARTING'
                check_live_rgb_target(audit, armed, clock())
                report['steer_posts'] += 1
                code, reply = call('POST', '/steer-update',
                                   {'boot_session_id': boot, 'command_id': command_id,
                                    'seq': seq, 'bias_pwm': 0})
            if code != 200 or reply.get('accepted') is not True or reply.get('seq') != seq:
                if code == 409:
                    check_code, terminal_status = call('GET', '/status')
                    last_status_poll = clock()
                    if check_code == 200 and matching_terminal(terminal_status, boot):
                        pending_steer_ack_deadline = None
                        report['terminal_status'] = terminal_status
                        break
                    if (check_code == 200 and
                            (terminal_status.get('bounded_fault_latched') or
                             terminal_status.get('state') == 'FAULT')):
                        raise RuntimeError('CONTROLLER_FAULT_DURING_MOVE:' +
                                           str(terminal_status.get('bounded_fault_reason')))
                    if (check_code == 200 and
                            terminal_status.get('active_command_id') == command_id and
                            terminal_status.get('state') in ('BRAKING', 'COASTING')):
                        # The bounded command entered its normal stopping
                        # phase between the camera frame and steer request.
                        pending_steer_ack_deadline = None
                        terminal_pending_deadline = clock() + 3.0
                        continue
                raise RuntimeError('STEER_UPDATE_NOT_ACKNOWLEDGED')
            if bias != 0:
                driving_confirmed = True
                drive_state = 'DRIVING'
            pending_steer_ack_deadline = None
            report['updates'].append({'cycle_id': audit['cycle_id'],
                                      'driver_stamp_s': stamp,
                                      'bearing_deg': target_bearing_deg(box),
                                      'bias_pwm': bias, 'seq': seq,
                                      'drive_state_at_update': drive_state})
            last_frame_stamp = stamp
            previous_box = box
            last_feedback = clock()
        else:
            raise RuntimeError('CAMERA_STREAM_ENDED_BEFORE_TERMINAL')
        if report['terminal_status'].get('last_command_state') not in ('SUCCESS', 'PARTIAL_PROGRESS'):
            raise RuntimeError('FORWARD_TERMINAL_REFUSED:' +
                               str(report['terminal_status'].get('last_command_state')))
        report['result'] = 'TERMINAL_OBSERVED_DIAGNOSTIC_ONLY'
    except BaseException as error:
        report['result'] = 'STOPPED_ON_REFUSAL'
        report['error'] = type(error).__name__ + ': ' + str(error)
    finally:
        if sent:
            report['stop_posts'] = 1
            try:
                code, body = call('POST', '/stop', timeout_s=1.0)
                report['stop_response'] = {'http_code': code, 'body': body}
            except BaseException as error:
                report['stop_error'] = type(error).__name__ + ': ' + str(error)
            # A failed /stop HTTP response may still mean that ESP32 braked.
            # Observe actual READY/FAULT with zero PWM before classifying it.
            deadline = clock() + 3.0
            while clock() < deadline:
                try:
                    code, status = call('GET', '/status', timeout_s=.75)
                except BaseException as error:
                    report['last_stop_status_error'] = type(error).__name__ + ': ' + str(error)
                    time.sleep(.04)
                    continue
                if code == 200 and status.get('boot_session_id') == boot and \
                        status.get('state') in ('READY', 'FAULT') and \
                        status.get('right_pwm') == 0 and status.get('left_pwm') == 0 and \
                        status.get('remote_stop_pending') is False:
                    report['confirmed_stop_status'] = status
                    if status.get('state') == 'FAULT' or status.get('bounded_fault_latched'):
                        report['result'] = 'TERMINAL_FAULT_STOPPED'
                    break
                time.sleep(.04)
            else:
                report['result'] = 'STOP_NOT_CONFIRMED'
        journal.close()
        (output / 'summary.json').write_text(json.dumps(report, indent=2, allow_nan=False) + '\n')
    return report


def main():
    p = argparse.ArgumentParser(description=__doc__)
    mode = p.add_mutually_exclusive_group()
    mode.add_argument('--execute', action='store_true')
    mode.add_argument('--preview-only', action='store_true',
                      help='Observe target and journal intended steering, no network or motors')
    p.add_argument('--preview-duration-s', type=float, default=10.0)
    p.add_argument('--base-url')
    p.add_argument('--front-clearance-m', type=float)
    p.add_argument('--distance-m', type=float, default=DISTANCE_M,
                   help='Bounded active command length, 0.02 to 0.30 m; first trial use 0.10 m')
    args = p.parse_args()
    if not args.execute and not args.preview_only:
        print('NO_EXECUTION: no camera, network or motor opened')
        return 0
    if args.execute and (not valid_controller_url(args.base_url) or
                         args.front_clearance_m is None):
        p.error('Require --base-url and --front-clearance-m')
    lockpath = Path.home() / '.local/state/sie/v7_operator_one_step.lock'
    lockpath.parent.mkdir(parents=True, exist_ok=True)
    with lockpath.open('a') as lock:
        try:
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            raise SystemExit('Another supervised trial is running')
        import rclpy
        from rclpy.node import Node
        from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
        from std_msgs.msg import String
        root = Path.home() / '.local/state/sie/evidence' / (
            ('v7_live_steering_preview_' if args.preview_only else 'v7_live_steered_forward_')
            + str(int(time.time())) + '_' + secrets.token_hex(4))
        if args.preview_only:
            print('Camera-only preview of acquired target and intended wheel bias.')
        else:
            print('One', args.distance_m, 'm bounded forward command. Camera updates steer both wheels during that command.')
            print('Confirm clear path, measured front clearance, target visible, and accessible motor power cutoff.')
        process = None
        node = None
        holder = []
        try:
            rclpy.init()
            node = Node('sie_live_steering_' + secrets.token_hex(3))
            def callback(message):
                try:
                    holder[:] = [json.loads(message.data)]
                except (ValueError, TypeError):
                    pass
            node.create_subscription(String, TOPIC, callback,
                QoSProfile(depth=1, durability=DurabilityPolicy.VOLATILE,
                           reliability=ReliabilityPolicy.RELIABLE))
            env = os.environ.copy()
            env['SIE_PREVIEW_DURATION_S'] = '120'
            process = subprocess.Popen(
                ['bash', str(ROOT / 'vision_core/tools/run_control_preview_existing_static_v2.sh')],
                cwd=ROOT, start_new_session=True, env=env)
            def camera(armed):
                while process.poll() is None:
                    rclpy.spin_once(node, timeout_sec=.04)
                    if holder:
                        yield holder.pop()
                    else:
                        yield None
                raise RuntimeError('CAMERA_PROCESS_ENDED')
            if args.preview_only:
                report = preview_session(output=root, camera_factory=camera,
                                         duration_s=args.preview_duration_s)
                print('LIVE_STEER_PREVIEW', report['result'], 'frames=', len(report['rows']))
            else:
                report = run_session(base_url=args.base_url,
                    front_clearance_m=args.front_clearance_m,
                    distance_m=args.distance_m, output=root,
                    camera_factory=camera)
                print('LIVE_STEER', report['result'], 'movement_posts=', report['movement_posts'],
                      'steer_posts=', report['steer_posts'], 'stop_posts=', report['stop_posts'])
            print('Saved:', root / 'summary.json')
            return 0 if report['result'] in ('TERMINAL_OBSERVED_DIAGNOSTIC_ONLY',
                                             'PREVIEW_RECORDED',
                                             'PREVIEW_RECORDED_WITH_LOSS') else 1
        finally:
            if process is not None and process.poll() is None:
                os.killpg(process.pid, signal.SIGINT)
                try:
                    process.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGTERM)
            if node is not None:
                node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()


if __name__ == '__main__':
    raise SystemExit(main())

