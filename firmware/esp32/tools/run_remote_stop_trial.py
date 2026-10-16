"""One supervised 0.10 m forward request, one stop, diagnostic journal. No retries."""
from __future__ import annotations
import argparse
from datetime import datetime, timezone
import hashlib
import json
import math
from pathlib import Path
import re
import time
from urllib.error import HTTPError
from urllib.parse import urlencode, urlsplit
from urllib.request import HTTPRedirectHandler, Request, build_opener
import uuid

BUILD = 'v4_2_remote_stop_observed_v1'
DISTANCE_M = 0.10
STOP_REQUEST_AFTER_S = 0.30  # host deadline from movement POST start; not a stopping bound
TERMINAL_WAIT_S = 5.0  # observation deadline, never permission for more movement
TERMINAL = {'SUCCESS', 'PARTIAL_PROGRESS', 'FAULT', 'STOPPED'}

class TrialError(RuntimeError):
    pass

class NoRedirect(HTTPRedirectHandler):
    def redirect_request(self, req, fp, code, msg, headers, newurl):
        return None

class Http:
    def __init__(self):
        self.opener = build_opener(NoRedirect())
    def __call__(self, method, url, timeout):
        req = Request(url, method=method, headers={'Accept':'application/json'})
        try:
            with self.opener.open(req, timeout=timeout) as r:
                code, body = r.status, r.read()
        except HTTPError as e:
            code, body = e.code, e.read()
        data = json.loads(body)
        if type(data) is not dict:
            raise TrialError('Response must be a JSON object')
        return code, data


def preflight(s):
    if s.get('firmware_build_id') != BUILD:
        raise TrialError('Unexpected firmware build')
    if s.get('motion_api_version') != 'v2_bounded_motion_api':
        raise TrialError('Unexpected motion API')
    if not re.fullmatch(r'[0-9A-F]{16}', str(s.get('boot_session_id', ''))):
        raise TrialError('Invalid boot session')
    if (s.get('state') != 'READY' or s.get('bounded_fault_latched') is not False or
            s.get('active_command_id') is not None or s.get('remote_stop_pending') is not False or
            s.get('square_state') not in {'IDLE', 'SUCCESS', 'FAULT'} or
            s.get('right_pwm') != 0 or s.get('left_pwm') != 0 or
            s.get('fault') not in {None, ''}):
        raise TrialError('Controller is not idle/ready')


def run(*, base_url, output, execute=False, request=None, clock=time.monotonic,
        sleep=time.sleep, timeout=0.5):
    parsed = urlsplit(base_url)
    if parsed.scheme != 'http' or not parsed.hostname or parsed.username or parsed.password or parsed.query or parsed.fragment or parsed.path not in {'','/'}:
        raise TrialError('Use an explicit http://controller-host[:port] base URL')
    output = Path(output)
    output.mkdir(parents=True, exist_ok=False)
    request = request or Http()
    command_id = 'remote-stop-' + uuid.uuid4().hex
    summary = {'schema_version':'sie.remote_stop_trial_diagnostic.v1',
               'command_id':command_id, 'requested_forward_m':DISTANCE_M,
               'stop_trigger_mode':'HOST_DEADLINE_WITHOUT_STATUS_POLL',
               'stop_request_after_move_post_s':STOP_REQUEST_AFTER_S,
               'result':'NOT_STARTED','movement_posts':0,'stop_posts':0,
               'physical_stopping_bound_established':False,
               'manual_physical_post_stop_distance_m':None,
               'manual_physical_stop_time_s':None,
               'script_sha256':hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
               'measurement_emission':False,'camera_control_enabled':False}
    journal = (output / 'http.jsonl').open('x')
    sent_move = False
    sent_stop = False
    session = None
    def call(method, endpoint, query=None):
        url = base_url.rstrip('/') + endpoint
        if query:
            url += '?' + urlencode(query)
        started = clock()
        row = {'host_started_monotonic_s':started,
               'host_started_utc':datetime.now(timezone.utc).isoformat(),
               'method':method,'endpoint':endpoint,'query':query}
        try:
            code, data = request(method, url, timeout)
            row.update(http_code=code, response=data)
            return code, data
        except BaseException as e:
            row['error'] = type(e).__name__ + ': ' + str(e)
            raise
        finally:
            row['host_completed_monotonic_s'] = clock()
            journal.write(json.dumps(row, allow_nan=False) + '\n')
            journal.flush()
    def stop_once():
        nonlocal sent_stop
        if sent_stop:
            return
        sent_stop = True  # before I/O: never repeat an ambiguous POST
        summary['stop_posts'] += 1
        summary['stop_request_host_monotonic_s'] = clock()
        code, data = call('POST', '/stop')
        summary['stop_response'] = {'http_code':code,'body':data}
        if code not in {200,202} or data.get('accepted') is not True:
            raise TrialError('Stop acknowledgement unavailable or unexpected')
    try:
        code, status = call('GET','/status')
        if code != 200:
            raise TrialError('Status request failed')
        preflight(status)
        session = status['boot_session_id']
        summary['boot_session_id'] = session
        summary['preflight_status'] = status
        if not execute:
            summary['result'] = 'PREFLIGHT_ONLY_NO_MOVEMENT'
            return summary
        sent_move = True
        summary['movement_posts'] += 1
        started = clock()
        code, response = call('POST','/move-forward',
                              {'boot_session_id':session,'command_id':command_id,'distance_m':'0.1'})
        summary['movement_response'] = {'http_code':code,'body':response}
        if code != 202 or response.get('accepted') is not True or response.get('command_id') != command_id:
            raise TrialError('Movement acceptance unknown/rejected; no retry')
        # No status request may occupy the stop dispatch window. An acceptance
        # reply later than the deadline causes an immediate stop, never a retry.
        stop_deadline = started + STOP_REQUEST_AFTER_S
        summary['move_request_host_monotonic_s'] = started
        summary['stop_dispatch_deadline_host_monotonic_s'] = stop_deadline
        remaining = stop_deadline - clock()
        if remaining > 0:
            sleep(remaining)
        stop_once()
        summary['actual_move_post_to_stop_request_s'] = summary['stop_request_host_monotonic_s'] - started
        summary['stop_dispatch_lateness_s'] = max(0.0, summary['stop_request_host_monotonic_s'] - stop_deadline)
        terminal_deadline = clock() + TERMINAL_WAIT_S
        while True:
            if clock() >= terminal_deadline:
                raise TrialError('Terminal observation deadline expired; do not start another trial')
            code, status = call('GET','/status')
            if code != 200:
                raise TrialError('Status unavailable during trial')
            if status.get('boot_session_id') != session:
                raise TrialError('Controller rebooted; stop and discard trial')
            current_terminal = (status.get('last_command_id') == command_id and
                                status.get('last_command_state') in TERMINAL)
            if current_terminal:
                summary['terminal_status'] = status
                if not sent_stop:
                    summary['result'] = 'COMPLETED_BEFORE_STOP_NO_STOP_TRIAL'
                elif status.get('last_command_state') == 'STOPPED' and status.get('remote_stop_observation_complete') is True and status.get('remote_stop_pending') is False:
                    summary['result'] = 'REMOTE_STOP_OBSERVED_DIAGNOSTIC_ONLY'
                    proxy = {}
                    for wheel, diameter, cpr in [('right',0.0646,205),('left',0.0652,206)]:
                        a = status.get(f'remote_stop_{wheel}_start_count')
                        b = status.get(f'remote_stop_{wheel}_observed_count')
                        if type(a) is not int or type(b) is not int or b < a:
                            raise TrialError('Invalid stop encoder observation')
                        proxy[wheel+'_delta_counts'] = b-a
                        proxy[wheel+'_distance_m'] = (b-a)*math.pi*diameter/cpr
                    proxy['mean_distance_m'] = (proxy['right_distance_m']+proxy['left_distance_m'])/2
                    proxy['basis'] = 'wheel encoder displacement from firmware brake onset to final braked sample; not independent physical measurement'
                    summary['encoder_stop_proxy'] = proxy
                    summary['both_wheels_moved_before_stop'] = all(
                        status.get(f'remote_stop_{wheel}_start_count', 0) > 0
                        for wheel in ('right','left'))
                    if not summary['both_wheels_moved_before_stop']:
                        summary['result'] = 'STOP_OBSERVED_WITHOUT_BOTH_WHEEL_PROGRESS'
                    summary['controller_observation_elapsed_ms'] = status.get('remote_stop_elapsed_ms')
                    summary['host_stop_request_to_terminal_observed_s'] = clock()-summary['stop_request_host_monotonic_s']
                else:
                    summary['result'] = 'REMOTE_STOP_NOT_CONFIRMED'
                    if summary.get('stop_response', {}).get('body', {}).get('basis') == 'NO_ACTIVE_MOTION':
                        summary['reason'] = 'STOP_ARRIVED_WITH_NO_ACTIVE_MOTION'
                break
            sleep(0.02)
    except BaseException as e:
        summary['result'] = 'INTERRUPTED' if isinstance(e,KeyboardInterrupt) else 'TRIAL_REFUSED_OR_UNKNOWN'
        summary['error'] = type(e).__name__ + ': ' + str(e)
        if sent_move and not sent_stop:
            try:
                stop_once()
            except BaseException as stop_error:
                summary['stop_error'] = type(stop_error).__name__ + ': ' + str(stop_error)
    finally:
        # Includes transport errors, exceptions and Ctrl+C. SIGKILL/power loss cannot run cleanup.
        journal.close()
        (output / 'summary.json').write_text(json.dumps(summary,indent=2,allow_nan=False)+'\n')
    return summary


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--base-url', default='http://192.168.0.17')
    p.add_argument('--output', type=Path)
    p.add_argument('--execute', action='store_true', help='perform one supervised 0.10 m command and stop trial')
    args=p.parse_args()
    output=args.output or Path.home()/'.local/state/sie/evidence'/('remote_stop_trial_'+datetime.now().strftime('%Y%m%d_%H%M%S')+'_'+uuid.uuid4().hex[:8])
    summary=run(base_url=args.base_url,output=output,execute=args.execute)
    print('REMOTE_STOP_TRIAL',summary['result'])
    print('movement_posts=',summary['movement_posts'],'stop_posts=',summary['stop_posts'])
    if 'encoder_stop_proxy' in summary:
        print('encoder_post_brake_mean_mm=',round(1000*summary['encoder_stop_proxy']['mean_distance_m'],2))
        print('controller_observation_elapsed_ms=',summary['controller_observation_elapsed_ms'])
    print('Saved:',output/'summary.json')
    return 0 if summary['result'] in {'PREFLIGHT_ONLY_NO_MOVEMENT','REMOTE_STOP_OBSERVED_DIAGNOSTIC_ONLY'} else 1

if __name__ == '__main__':
    raise SystemExit(main())
