#!/usr/bin/env python3
"""Operator supervised bounded forward command with live RGB steering updates."""
from __future__ import annotations

import argparse
import fcntl
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

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
from firmware.esp32.tools.run_remote_stop_trial import Http, preflight
from vision_core.tools.run_v7_supervised_one_step import TOPIC, check_observation
from vision_core.tools.run_v7_supervised_approach import depth_and_box, overlap, target_bearing_deg

BUILD = 'v4_2_live_steering_v1'
DISTANCE_M = 0.10
MAX_DISTANCE_M = 0.30
MAX_BIAS_PWM = 20
STEER_GAIN_PWM_PER_DEG = 4.0
STEER_DEADBAND_DEG = 0.5
FEEDBACK_TIMEOUT_S = 0.6  # ESP32 owns the independent 700 ms deadline.
TERMINAL = {'SUCCESS', 'PARTIAL_PROGRESS', 'STOPPED', 'FAULT'}


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


def preview_session(*, output, camera_factory, duration_s=10.0,
                    clock=time.monotonic):
    """Journal live camera bearings and intended PWM bias; no HTTP or motors."""
    if not .5 <= duration_s <= 60:
        raise ValueError('INVALID_PREVIEW_DURATION')
    output = Path(output)
    output.mkdir(parents=True, exist_ok=False)
    armed = clock()
    end = armed + duration_s
    first_box = previous_box = None
    rows = []
    result = 'NO_TARGET'
    try:
        for audit in camera_factory(armed):
            if clock() >= end:
                break
            if audit is None:
                continue
            try:
                evidence = check_observation(audit, armed, clock(),
                                             supervised_rgb_only=True)
                _, box = depth_and_box(audit)
            except (ValueError, TypeError, KeyError) as error:
                if previous_box is None:
                    continue
                result = 'TARGET_LOST'
                rows.append({'refusal': str(error)})
                break
            if previous_box is not None and not same_target(previous_box, box):
                result = 'TARGET_CORRESPONDENCE_LOST'
                break
            bearing = target_bearing_deg(box)
            bias = 0 if first_box is None else steering_bias(bearing)
            rows.append({'cycle_id': audit['cycle_id'],
                         'driver_stamp_s': evidence['oldest_driver_stamp_s'],
                         'box_xyxy_px': box, 'bearing_deg': bearing,
                         'intended_bias_pwm': bias,
                         'phase': 'ACQUIRE_STRAIGHT' if first_box is None else 'LIVE_STEER'})
            if first_box is None:
                first_box = box
            previous_box = box
            result = 'PREVIEW_RECORDED'
    finally:
        report = {'schema_version': 'sie.live_steering_camera_preview.v1',
                  'result': result, 'motor_posts': 0, 'network_requests': 0,
                  'measurement_admission_changed': False, 'rows': rows}
        (output / 'summary.json').write_text(json.dumps(report, indent=2,
                                                       allow_nan=False) + '\n')
    return report


def run_session(*, base_url, front_clearance_m, output, distance_m=DISTANCE_M, request=None,
                clock=time.monotonic, camera_factory=None):
    """Live session; camera_factory yields successive full diagnostic records.

    The first observation starts a straight command. Only subsequent records
    can change wheel PWM. All HTTP operations are journaled without retry.
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
              'camera_control_enabled': True, 'updates': []}
    journal = (output / 'http.jsonl').open('x')
    sent = False
    command_id = 'sie-live-' + secrets.token_hex(12)
    report['command_id'] = command_id

    def call(method, endpoint, params=None):
        url = base_url.rstrip('/') + endpoint
        if params is not None:
            url += '?' + urlencode(params)
        row = {'method': method, 'endpoint': endpoint, 'query': params,
               'host_started_monotonic_s': clock()}
        try:
            code, body = request(method, url, .4)
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
        code, ready = call('GET', '/status')
        if code != 200:
            raise RuntimeError('CONTROLLER_STATUS_UNAVAILABLE')
        preflight(ready, accepted_builds={BUILD})
        if ready.get('steering_api_version') != 'v1_active_forward_pwm_bias':
            raise RuntimeError('STEERING_API_MISMATCH')
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
                evidence = check_observation(candidate, armed, clock(),
                                             supervised_rgb_only=True)
                first = candidate
                break
            except (ValueError, TypeError, KeyError) as error:
                report['last_camera_refusal'] = str(error)
        if first is None:
            raise RuntimeError('NO_FRESH_TARGET_WITHIN_60S')
        _, box = depth_and_box(first)
        report['first_camera_evidence'] = evidence
        report['first_box_xyxy_px'] = box
        report['first_bearing_deg'] = target_bearing_deg(box)
        # The first camera detection starts the cart straight, from anywhere
        # in the image. Steering begins with a newly captured second frame.
        check_observation(first, armed, clock(), supervised_rgb_only=True)
        sent = True  # An ambiguous movement POST is never retried.
        report['movement_posts'] = 1
        code, body = call('POST', '/move-forward-steered',
                          {'boot_session_id': boot, 'command_id': command_id,
                           'distance_m': format(distance_m, '.3f')})
        if code != 202 or body.get('accepted') is not True or body.get('command_id') != command_id:
            raise RuntimeError('MOVEMENT_ACCEPTANCE_UNKNOWN')
        last_frame_stamp = evidence['oldest_driver_stamp_s']
        last_feedback = clock()
        previous_box = box
        seq = 0
        # camera_factory yields fresh full records while the same bounded
        # forward command is active. A missing record is a stop condition.
        last_status_poll = clock()
        for audit in observations:
            now = clock()
            if audit is None:
                if now - last_status_poll >= .1:
                    code, status = call('GET', '/status')
                    last_status_poll = clock()
                    if code != 200:
                        raise RuntimeError('CONTROLLER_STATUS_UNAVAILABLE_DURING_MOVE')
                    if matching_terminal(status, boot):
                        report['terminal_status'] = status
                        break
            if now - last_feedback > FEEDBACK_TIMEOUT_S:
                raise RuntimeError('CAMERA_FEEDBACK_TIMEOUT')
            if audit is None:
                continue
            evidence = check_observation(audit, armed, now, supervised_rgb_only=True)
            stamp = evidence['oldest_driver_stamp_s']
            if stamp <= last_frame_stamp:
                continue
            _, box = depth_and_box(audit)
            if not same_target(previous_box, box):
                raise RuntimeError('TARGET_CORRESPONDENCE_LOST')
            code, status = call('GET', '/status')
            last_status_poll = clock()
            if code != 200:
                raise RuntimeError('CONTROLLER_STATUS_UNAVAILABLE_DURING_MOVE')
            if matching_terminal(status, boot):
                report['terminal_status'] = status
                break
            if status.get('active_command_id') != command_id:
                raise RuntimeError('ACTIVE_COMMAND_NOT_CONFIRMED')
            # Recheck age after the status read; only this new frame may steer.
            check_observation(audit, armed, clock(), supervised_rgb_only=True)
            seq += 1
            # A fresh frame during breakaway keeps feedback alive, but only a
            # frame observed after DRIVING may request a wheel difference.
            bias = (steering_bias(target_bearing_deg(box))
                    if status.get('state') == 'DRIVING' else 0)
            report['steer_posts'] += 1
            code, reply = call('POST', '/steer-update',
                               {'boot_session_id': boot, 'command_id': command_id,
                                'seq': seq, 'bias_pwm': bias})
            if code != 200 or reply.get('accepted') is not True or reply.get('seq') != seq:
                raise RuntimeError('STEER_UPDATE_NOT_ACKNOWLEDGED')
            report['updates'].append({'cycle_id': audit['cycle_id'],
                                      'driver_stamp_s': stamp,
                                      'bearing_deg': target_bearing_deg(box),
                                      'bias_pwm': bias, 'seq': seq,
                                      'drive_state_at_update': status.get('state')})
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
                code, body = call('POST', '/stop')
                report['stop_response'] = {'http_code': code, 'body': body}
                # A stop response is only acceptance; confirm READY/zero PWM.
                deadline = clock() + 3.0
                while clock() < deadline:
                    code, status = call('GET', '/status')
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
            except BaseException as error:
                report['result'] = 'STOP_NOT_CONFIRMED'
                report['stop_error'] = type(error).__name__ + ': ' + str(error)
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
                                             'PREVIEW_RECORDED') else 1
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
