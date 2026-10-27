"""Offline transport tests; no network, motor or serial operations."""
import json
from pathlib import Path
import tempfile
import unittest
from urllib.parse import parse_qs, urlsplit
from urllib.error import URLError
from firmware.esp32.tools.run_remote_stop_trial import (BUILD, REUSABLE_HISTORY_BUILD,
    TrialError, preflight, run)

class Harness:
    def __init__(self, mode='normal'):
        self.mode=mode;self.calls=[];self.time=0.;self.command=None;self.stopped=False;self.polls=0
    def clock(self):return self.time
    def sleep(self,t):
        if self.mode=='interrupt' and not self.stopped:raise KeyboardInterrupt()
        self.time+=t
    def request(self,method,url,timeout):
        self.time+=0.001
        path=urlsplit(url).path;self.calls.append((method,path))
        ready={'firmware_build_id':BUILD,'motion_api_version':'v2_bounded_motion_api',
               'boot_session_id':'0123456789ABCDEF','state':'READY','bounded_fault_latched':False,
               'active_command_id':None,'remote_stop_pending':False,'square_state':'IDLE',
               'right_pwm':0,'left_pwm':0}
        if not self.command and method=='GET':
            if self.mode=='preflight':ready['firmware_build_id']='wrong'
            if self.mode=='reusable_history_build':ready['firmware_build_id']=REUSABLE_HISTORY_BUILD
            return 200,ready
        if path=='/move-forward':
            self.command=parse_qs(urlsplit(url).query)['command_id'][0]
            if self.mode=='move_transport':raise OSError('ambiguous movement response')
            if self.mode=='slow_accept':self.time+=.45
            return 202,{'accepted':True,'command_id':self.command}
        if path=='/stop':
            self.stopped=True
            if self.mode=='stop_transport':raise OSError('stop response unknown')
            if self.mode=='natural':return 200,{'accepted':True,'stopped':True,'basis':'NO_ACTIVE_MOTION'}
            return 202,{'accepted':True,'stopped':False,'stop_pending':True}
        if self.mode=='interrupt' and not self.stopped:raise KeyboardInterrupt()
        if self.mode=='status_transport':raise OSError('status unavailable')
        if self.mode=='reboot':
            return 200,{**ready,'boot_session_id':'FEDCBA9876543210'}
        terminal={**ready,'last_command_id':self.command,'last_command_state':'STOPPED',
                  'remote_stop_observation_complete':True,'remote_stop_elapsed_ms':600,
                  'remote_stop_right_start_count':40,'remote_stop_left_start_count':40,
                  'remote_stop_right_observed_count':43,'remote_stop_left_observed_count':42}
        if self.mode=='natural':return 200,{**terminal,'last_command_state':'SUCCESS'}
        if self.mode=='no_progress':
            terminal['remote_stop_right_start_count']=0
            terminal['remote_stop_left_start_count']=0
        if self.mode=='slow_status':self.time+=.479
        if self.stopped:
            self.polls+=1
            if self.polls==1 or self.mode=='timeout':
                return 200,{**ready,'state':'BRAKING','remote_stop_pending':True,
                            'active_command_id':self.command,'last_command_id':'old','last_command_state':'STOPPED'}
            if self.mode=='fault':terminal['last_command_state']='FAULT'
            return 200,terminal
        return 200,{**ready,'state':'DRIVING','active_command_id':self.command,
                    'average_distance_m':0. if self.mode=='fallback' else .05,
                    'right_count':50,'left_count':50}

class TrialTests(unittest.TestCase):
    def test_candidate_build_requires_explicit_preflight_scope(self):
        _, ready = Harness().request('GET', 'http://127.0.0.1/status', 1.0)
        ready['firmware_build_id'] = 'v4_2_micro_turn_loaded_start_v4'
        with self.assertRaises(TrialError):
            preflight(ready)
        preflight(ready, accepted_builds={'v4_2_micro_turn_loaded_start_v4'})

    def trial(self,mode='normal',execute=True):
        h=Harness(mode)
        with tempfile.TemporaryDirectory() as tmp:
            out=Path(tmp)/'trial'
            s=run(base_url='http://127.0.0.1',output=out,execute=execute,
                  request=h.request,clock=h.clock,sleep=h.sleep)
            self.assertEqual(json.loads((out/'summary.json').read_text()),s)
            rows=[json.loads(x) for x in (out/'http.jsonl').read_text().splitlines()]
            self.assertEqual(len(rows),len(h.calls))
        return s,h
    def test_reusable_history_build_is_accepted(self):
        s,h=self.trial('reusable_history_build')
        self.assertEqual(s['result'],'REMOTE_STOP_OBSERVED_DIAGNOSTIC_ONLY')
        self.assertEqual([p for m,p in h.calls if m=='POST'],['/move-forward','/stop'])

    def test_one_move_stop_then_matching_terminal(self):
        s,h=self.trial()
        self.assertEqual(s['result'],'REMOTE_STOP_OBSERVED_DIAGNOSTIC_ONLY')
        self.assertEqual([p for m,p in h.calls if m=='POST'],['/move-forward','/stop'])
        self.assertEqual(h.polls,2)
        self.assertEqual(s['encoder_stop_proxy']['right_delta_counts'],3)
        self.assertFalse(s['physical_stopping_bound_established'])
    def test_default_is_read_only(self):
        s,h=self.trial(execute=False)
        self.assertEqual(s['result'],'PREFLIGHT_ONLY_NO_MOVEMENT')
        self.assertEqual(h.calls,[('GET','/status')])
    def test_supplied_ready_status_skips_status_http_but_is_validated(self):
        h=Harness()
        _,ready=h.request('GET','http://127.0.0.1/status',.5)
        h.calls.clear()
        ready_at=h.clock()
        with tempfile.TemporaryDirectory() as tmp:
            s=run(base_url='http://127.0.0.1',output=Path(tmp)/'trial',
                  execute=True,request=h.request,clock=h.clock,sleep=h.sleep,
                  ready_status=ready,ready_status_at=ready_at)
        self.assertEqual(s['result'],'REMOTE_STOP_OBSERVED_DIAGNOSTIC_ONLY')
        self.assertEqual(s['preflight_source'],'PROVIDED_BEFORE_FRESH_FRAME')
        self.assertEqual(h.calls[0],('POST','/move-forward'))
        self.assertEqual([p for m,p in h.calls if m=='POST'],
                         ['/move-forward','/stop'])
        bad={**ready,'state':'DRIVING'}
        h=Harness()
        with tempfile.TemporaryDirectory() as tmp:
            refused=run(base_url='http://127.0.0.1',output=Path(tmp)/'trial',
                        execute=True,request=h.request,clock=h.clock,sleep=h.sleep,
                        ready_status=bad,ready_status_at=h.clock())
        self.assertEqual(refused['movement_posts'],0)
        self.assertEqual(h.calls,[])
        h=Harness();h.time=4.3
        with tempfile.TemporaryDirectory() as tmp:
            delayed=run(base_url='http://127.0.0.1',output=Path(tmp)/'trial',
                        execute=True,request=h.request,clock=h.clock,sleep=h.sleep,
                        ready_status=ready,ready_status_at=0.0)
        self.assertEqual(delayed['result'],'REMOTE_STOP_OBSERVED_DIAGNOSTIC_ONLY')
        self.assertGreaterEqual(delayed['preflight_age_at_dispatch_s'],4.3)
        self.assertEqual(h.calls[0],('POST','/move-forward'))
        h=Harness()
        with tempfile.TemporaryDirectory() as tmp:
            future=run(base_url='http://127.0.0.1',output=Path(tmp)/'trial',
                       execute=True,request=h.request,clock=h.clock,sleep=h.sleep,
                       ready_status=ready,ready_status_at=1.0)
        self.assertEqual(future['movement_posts'],0)
        self.assertEqual(h.calls,[])

    def test_stopped_label_with_nonzero_pwm_is_not_confirmed(self):
        h=Harness(); original=h.request
        def request(method,url,timeout):
            code,data=original(method,url,timeout)
            if data.get('remote_stop_observation_complete') is True:
                data['right_pwm']=10
            return code,data
        with tempfile.TemporaryDirectory() as tmp:
            s=run(base_url='http://127.0.0.1',output=Path(tmp)/'trial',execute=True,
                request=request,clock=h.clock,sleep=h.sleep)
        self.assertEqual(s['result'],'TRIAL_REFUSED_OR_UNKNOWN')
        self.assertEqual([p for m,p in h.calls if m=='POST'],['/move-forward','/stop'])
    def test_bad_build_does_not_move(self):
        s,h=self.trial('preflight')
        self.assertEqual(s['movement_posts'],0)
        self.assertEqual(s['stop_posts'],0)
    def test_transport_errors_interrupt_reboot_send_one_stop_and_never_retry_move(self):
        for mode in ['move_transport','status_transport','interrupt','reboot','stop_transport']:
            with self.subTest(mode=mode):
                s,h=self.trial(mode)
                self.assertEqual(s['movement_posts'],1)
                self.assertEqual(s['stop_posts'],1)
                self.assertEqual([p for m,p in h.calls if m=='POST'],['/move-forward','/stop'])
                self.assertNotEqual(s['result'],'REMOTE_STOP_OBSERVED_DIAGNOSTIC_ONLY')
    def test_observation_timeout_is_not_stop_success(self):
        s,h=self.trial('timeout')
        self.assertEqual(s['result'],'TRIAL_REFUSED_OR_UNKNOWN')
        self.assertEqual(s['stop_posts'],1)
    def test_natural_completion_does_not_claim_stop_trial(self):
        s,h=self.trial('natural')
        self.assertEqual(s['result'],'REMOTE_STOP_NOT_CONFIRMED')
        self.assertEqual(s['reason'],'STOP_ARRIVED_WITH_NO_ACTIVE_MOTION')
        self.assertEqual(s['stop_posts'],1)
    def test_fault_stays_unconfirmed(self):
        s,h=self.trial('fault')
        self.assertEqual(s['result'],'REMOTE_STOP_NOT_CONFIRMED')
    def test_status_latency_cannot_delay_stop_dispatch(self):
        s,h=self.trial('slow_status')
        self.assertEqual(h.calls[:3],[('GET','/status'),('POST','/move-forward'),('POST','/stop')])
        self.assertAlmostEqual(s['actual_move_post_to_stop_request_s'],.30)
        self.assertAlmostEqual(s['stop_dispatch_lateness_s'],0.)
    def test_late_acceptance_causes_immediate_stop_without_extra_sleep(self):
        s,h=self.trial('slow_accept')
        self.assertAlmostEqual(s['actual_move_post_to_stop_request_s'],.451)
        self.assertAlmostEqual(s['stop_dispatch_lateness_s'],.151)
        self.assertEqual(h.calls[2],('POST','/stop'))
    def test_encoder_quiet_without_prior_progress_is_not_moving_stop_validation(self):
        s,h=self.trial('no_progress')
        self.assertEqual(s['result'],'STOP_OBSERVED_WITHOUT_BOTH_WHEEL_PROGRESS')
        self.assertFalse(s['both_wheels_moved_before_stop'])

    def test_timeout_after_stop_retries_only_status_and_confirms_same_command(self):
        for wrapped in (False, True):
            with self.subTest(wrapped=wrapped), tempfile.TemporaryDirectory() as tmp:
                h=Harness(); request=h.request; failures=[]
                def transport(method,url,timeout):
                    if method=='GET' and h.stopped and not failures:
                        h.calls.append((method,'/status'));h.time+=timeout;failures.append(timeout)
                        if wrapped:raise URLError(TimeoutError('read timeout'))
                        raise TimeoutError('read timeout')
                    return request(method,url,timeout)
                out=Path(tmp)/'trial'
                s=run(base_url='http://127.0.0.1',output=out,execute=True,
                    request=transport,clock=h.clock,sleep=h.sleep)
                self.assertEqual(s['result'],'REMOTE_STOP_OBSERVED_DIAGNOSTIC_ONLY')
                self.assertEqual(s['terminal_status_read_timeouts'],1)
                self.assertEqual([p for m,p in h.calls if m=='POST'],['/move-forward','/stop'])
                self.assertAlmostEqual(s['actual_move_post_to_stop_request_s'],.30)
                rows=[json.loads(x) for x in (out/'http.jsonl').read_text().splitlines()]
                self.assertEqual(len(rows),len(h.calls))
                self.assertEqual(sum('error' in r for r in rows),1)

    def test_repeated_read_timeouts_end_at_original_deadline_without_more_posts(self):
        with tempfile.TemporaryDirectory() as tmp:
            h=Harness();request=h.request;timeouts=[]
            def transport(method,url,timeout):
                if method=='GET' and h.stopped:
                    h.calls.append((method,'/status'));h.time+=timeout;timeouts.append(timeout)
                    raise TimeoutError('read timeout')
                return request(method,url,timeout)
            s=run(base_url='http://127.0.0.1',output=Path(tmp)/'trial',execute=True,
                request=transport,clock=h.clock,sleep=h.sleep)
            self.assertEqual(s['result'],'TRIAL_REFUSED_OR_UNKNOWN')
            self.assertGreater(s['terminal_status_read_timeouts'],1)
            self.assertEqual([p for m,p in h.calls if m=='POST'],['/move-forward','/stop'])
            self.assertLessEqual(h.time-s['stop_request_host_monotonic_s'],5.002)
            self.assertLess(timeouts[-1],.5)

    def test_late_success_response_cannot_extend_observation_deadline(self):
        with tempfile.TemporaryDirectory() as tmp:
            h=Harness();request=h.request
            def transport(method,url,timeout):
                result=request(method,url,timeout)
                if method=='GET' and h.stopped:h.time+=5.1
                return result
            s=run(base_url='http://127.0.0.1',output=Path(tmp)/'trial',execute=True,
                request=transport,clock=h.clock,sleep=h.sleep)
            self.assertEqual(s['result'],'TRIAL_REFUSED_OR_UNKNOWN')
            self.assertEqual(s['movement_posts'],1)
            self.assertEqual(s['stop_posts'],1)

if __name__=='__main__':unittest.main()
