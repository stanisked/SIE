#!/usr/bin/env python3
"""One wheels-lifted ESP32 left-turn diagnosis after a latched breakaway fault."""
from __future__ import annotations

import argparse
from datetime import datetime
import json
from pathlib import Path
import sys
import time
from urllib.parse import urlencode, urlsplit
import uuid

ROOT = Path(__file__).resolve().parents[3]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from firmware.esp32.tools.run_remote_stop_trial import Http, preflight

ANGLE_DEG = 3.0
FAULT = 'BREAKAWAY_NOT_FOUND'
TERMINAL = {'SUCCESS', 'PARTIAL_PROGRESS', 'FAULT', 'STOPPED'}


def run(*, base_url, output, execute=False, wheels_lifted=False, request=None,
        clock=time.monotonic, sleep=time.sleep):
    if not execute:
        return {'result': 'NO_EXECUTION', 'turn_posts': 0, 'stop_posts': 0,
                'fault_ack_posts': 0}
    if wheels_lifted is not True:
        raise ValueError('WHEELS_LIFTED_CONFIRMATION_REQUIRED')
    parsed = urlsplit(base_url)
    if (parsed.scheme != 'http' or not parsed.hostname or parsed.username or
            parsed.password or parsed.query or parsed.fragment or
            parsed.path not in ('', '/')):
        raise ValueError('EXPLICIT_HTTP_CONTROLLER_URL_REQUIRED')
    output = Path(output)
    output.mkdir(parents=True, exist_ok=False)
    request = request or Http()
    command_id = 'stand-turn-' + uuid.uuid4().hex
    summary = {'schema_version': 'sie.wheels_lifted_turn_diagnostic.v1',
               'command_id': command_id, 'endpoint': '/turn-left',
               'angle_deg': ANGLE_DEG, 'wheels_lifted_operator_assertion': True,
               'result': 'NOT_STARTED', 'fault_ack_posts': 0,
               'turn_posts': 0, 'stop_posts': 0,
               'measurement_emission': False, 'camera_control_enabled': False}
    journal = (output / 'http.jsonl').open('x')
    sent_turn = False

    def call(method, endpoint, params=None, timeout=0.5):
        url = base_url.rstrip('/') + endpoint
        if params:
            url += '?' + urlencode(params)
        row = {'method': method, 'endpoint': endpoint, 'query': params,
               'host_started_monotonic_s': clock()}
        try:
            code, body = request(method, url, timeout)
            row.update(http_code=code, response=body)
            return code, body
        except BaseException as error:
            row['error'] = type(error).__name__ + ': ' + str(error)
            raise
        finally:
            row['host_completed_monotonic_s'] = clock()
            journal.write(json.dumps(row, allow_nan=False) + '\n')
            journal.flush()

    try:
        code, initial = call('GET', '/status', timeout=1.0)
        summary['initial_status'] = initial
        if (code != 200 or initial.get('firmware_build_id') not in {
                    'v4_2_micro_turn_watchdog_v3',
                    'v4_2_micro_turn_loaded_start_v4'}
                or initial.get('motion_api_version') != 'v2_bounded_motion_api'
                or initial.get('active_command_id') is not None
                or initial.get('remote_stop_pending') is not False
                or initial.get('right_pwm') != 0 or initial.get('left_pwm') != 0):
            raise RuntimeError('INITIAL_CONTROLLER_NOT_SAFE_FOR_BENCH')
        boot = initial.get('boot_session_id')
        if not isinstance(boot, str) or len(boot) != 16:
            raise RuntimeError('INVALID_BOOT_SESSION')
        if initial.get('bounded_fault_latched') is True:
            if (initial.get('bounded_fault_reason') != FAULT
                    or initial.get('fault') != FAULT
                    or initial.get('state') != 'FAULT'):
                raise RuntimeError('UNEXPECTED_LATCHED_FAULT')
            summary['fault_ack_posts'] = 1
            code, acknowledged = call('POST', '/ack-fault')
            summary['ack_response'] = {'http_code': code, 'body': acknowledged}
            if (code != 200 or acknowledged.get('acknowledged') is not True
                    or acknowledged.get('previous_fault_reason') != FAULT):
                raise RuntimeError('FAULT_ACK_NOT_CONFIRMED')
            code, ready = call('GET', '/status', timeout=1.0)
            if code != 200 or ready.get('boot_session_id') != boot:
                raise RuntimeError('READY_STATUS_OR_SESSION_UNAVAILABLE')
        else:
            ready = initial
        preflight(ready)
        summary['ready_status'] = ready
        sent_turn = True
        summary['turn_posts'] = 1
        code, accepted = call('POST', '/turn-left',
            {'boot_session_id': boot, 'command_id': command_id,
             'angle_deg': format(ANGLE_DEG, '.3f')})
        summary['turn_response'] = {'http_code': code, 'body': accepted}
        if (code != 202 or accepted.get('accepted') is not True or
                accepted.get('command_id') != command_id):
            raise RuntimeError('TURN_ACCEPTANCE_UNKNOWN')
        deadline = clock() + 5.0
        while clock() < deadline:
            code, status = call('GET', '/status')
            if code != 200 or status.get('boot_session_id') != boot:
                raise RuntimeError('TURN_STATUS_OR_SESSION_CHANGED')
            if (status.get('last_command_id') == command_id and
                    status.get('last_command_state') in TERMINAL):
                summary['terminal_status'] = status
                if status.get('right_pwm') != 0 or status.get('left_pwm') != 0:
                    raise RuntimeError('TERMINAL_PWM_NONZERO')
                if status.get('last_command_endpoint') != '/turn-left':
                    raise RuntimeError('TERMINAL_COMMAND_MISMATCH')
                terminal = status.get('last_command_state')
                if terminal in ('SUCCESS', 'PARTIAL_PROGRESS'):
                    if status.get('state') != 'READY' or status.get('bounded_fault_latched') is not False:
                        raise RuntimeError('TERMINAL_READY_NOT_CONFIRMED')
                    summary['result'] = 'TURN_TERMINAL_NEEDS_PHYSICAL_OBSERVATION'
                elif terminal == 'FAULT':
                    summary['result'] = 'TURN_FAULT_OBSERVED'
                else:
                    summary['result'] = 'TURN_STOPPED_NO_PROOF'
                break
            sleep(0.02)
        else:
            raise RuntimeError('TURN_TERMINAL_TIMEOUT')
    except BaseException as error:
        summary['result'] = 'INTERRUPTED' if isinstance(error, KeyboardInterrupt) else 'TRIAL_REFUSED_OR_UNKNOWN'
        summary['error'] = type(error).__name__ + ': ' + str(error)
    finally:
        if sent_turn:
            summary['stop_posts'] = 1
            try:
                code, body = call('POST', '/stop')
                summary['stop_response'] = {'http_code': code, 'body': body}
                if code not in (200, 202) or body.get('accepted') is not True:
                    raise RuntimeError('STOP_ACCEPTANCE_UNKNOWN')
                stop_deadline = clock() + 2.0
                while clock() < stop_deadline:
                    code, stopped = call('GET', '/status')
                    if (code == 200 and stopped.get('boot_session_id') == boot
                            and stopped.get('remote_stop_pending') is False
                            and stopped.get('right_pwm') == 0
                            and stopped.get('left_pwm') == 0
                            and stopped.get('active_command_id') is None):
                        summary['post_stop_status'] = stopped
                        break
                    sleep(0.02)
                else:
                    raise RuntimeError('STOP_NOT_CONFIRMED')
            except BaseException as error:
                summary['stop_error'] = type(error).__name__ + ': ' + str(error)
                summary['result'] = 'TRIAL_REFUSED_OR_UNKNOWN'
        journal.close()
        (output / 'summary.json').write_text(
            json.dumps(summary, indent=2, allow_nan=False) + '\n')
    return summary


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--execute', action='store_true')
    p.add_argument('--wheels-lifted', action='store_true',
                   help='operator confirms both driven wheels cannot contact the floor')
    p.add_argument('--base-url')
    p.add_argument('--output', type=Path)
    args = p.parse_args(argv)
    if not args.execute:
        print('NO_EXECUTION: no network or motor opened')
        return 0
    if not args.wheels_lifted or not args.base_url:
        p.error('--execute requires --wheels-lifted and --base-url')
    output = args.output or (Path.home() / '.local/state/sie/evidence' /
        ('wheels_lifted_turn_' + datetime.now().strftime('%Y%m%d_%H%M%S') +
         '_' + uuid.uuid4().hex[:8]))
    print('Wheels-lifted bench only: keep both driven wheels clear of the floor and motor power cutoff accessible.')
    result = run(base_url=args.base_url, output=output, execute=True,
                 wheels_lifted=True)
    print('STAND_TURN', result['result'], 'turn_posts=', result['turn_posts'],
          'stop_posts=', result['stop_posts'])
    print('Saved:', output / 'summary.json')
    return 0 if result['result'] == 'TURN_TERMINAL_NEEDS_PHYSICAL_OBSERVATION' else 1


if __name__ == '__main__':
    raise SystemExit(main())
