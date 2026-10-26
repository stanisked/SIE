#!/usr/bin/env python3
"""Bounded supervised V7 approach: reobserve after every observed stop."""
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
import time
import sys
from urllib.parse import urlencode, urlsplit

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
from vision_core.tools.run_v7_supervised_one_step import check_observation, execute_observed_step, TOPIC
from firmware.esp32.tools.run_remote_stop_trial import Http, preflight

# V5 AR0234 intrinsic, bound by check_observation to its reviewed SHA256.
RGB_FX_PX = 2021.546012
RGB_CX_PX = 924.649158
TURN_DEADBAND_DEG = 1.5
MAX_TURN_PER_STEP_DEG = 3.0

def depth_and_box(a):
    d = (a.get('shape_diagnostic') or {}).get('mask_depth') or {}
    c = (a.get('ar0234_dynamic_target') or {}).get('candidate') or {}
    z, box = d.get('candidate_depth_m'), c.get('bbox_xyxy_px')
    if not isinstance(z, (int, float)) or not math.isfinite(z) or z <= 0:
        raise ValueError('NO_CURRENT_DEPTH')
    if not isinstance(box, list) or len(box) != 4 or not all(isinstance(x, (int,float)) and math.isfinite(x) for x in box):
        raise ValueError('NO_CURRENT_TARGET_BOX')
    return float(z), box

def overlap(a, b):
    x0,y0,x1,y1 = a
    X0,Y0,X1,Y1 = b
    inter = max(0,min(x1,X1)-max(x0,X0))*max(0,min(y1,Y1)-max(y0,Y0))
    aa=max(0,x1-x0)*max(0,y1-y0)
    bb=max(0,X1-X0)*max(0,Y1-Y0)
    return inter/(aa+bb-inter) if aa+bb>inter else 0.

def target_center_x(box):
    return (box[0] + box[2]) / 2.0

def target_bearing_deg(box):
    return math.degrees(math.atan((target_center_x(box) - RGB_CX_PX) / RGB_FX_PX))

def steering_request(bearing_deg, *, diagnostic_turn_only=False):
    if not math.isfinite(bearing_deg):
        raise ValueError('INVALID_TARGET_BEARING')
    if abs(bearing_deg) <= TURN_DEADBAND_DEG:
        return None
    # Image x increases to the right. A left turn brings a left-hand target
    # toward image center. A single correction is followed by fresh feedback.
    return {'endpoint': '/turn-right' if bearing_deg > 0 else '/turn-left',
            'angle_deg': (MAX_TURN_PER_STEP_DEG if diagnostic_turn_only else
                          min(MAX_TURN_PER_STEP_DEG, max(1.0, abs(bearing_deg) / 2.0)))}

def verify_turn_feedback(z, box, new_z, new_box):
    if overlap(box,new_box)<.7 or abs(new_z-z)>.15:
        raise RuntimeError('TURN_TARGET_CORRESPONDENCE_UNCONFIRMED')
    before=target_bearing_deg(box)
    after=target_bearing_deg(new_box)
    if abs(before)-abs(after)<.25:
        raise RuntimeError('TURN_CAMERA_ALIGNMENT_UNCONFIRMED')
    return after

def execute_bounded_turn(audit, *, armed_at, base_url, output, endpoint, angle_deg,
                         request=None, clock=time.monotonic, sleep=time.sleep,
                         ready_status=None):
    """One bounded turn; no retry. Stop once if dispatch or completion is unknown."""
    parsed = urlsplit(base_url)
    if (parsed.scheme != 'http' or not parsed.hostname or parsed.username or parsed.password
            or parsed.query or parsed.fragment or parsed.path not in ('', '/')
            or endpoint not in ('/turn-left', '/turn-right')
            or not 1.0 <= angle_deg <= MAX_TURN_PER_STEP_DEG):
        raise ValueError('INVALID_BOUNDED_TURN_REQUEST')
    check_observation(audit, armed_at, clock())
    output.mkdir(parents=True, exist_ok=False)
    request = request or Http()
    cid = 'sie-turn-' + secrets.token_hex(12)
    report = {'command_id':cid, 'endpoint':endpoint, 'angle_deg':angle_deg,
              'turn_posts':0, 'stop_posts':0, 'result':'NOT_STARTED'}
    journal = (output/'http.jsonl').open('x')
    sent = False
    def call(method, route, params=None):
        url = base_url.rstrip('/') + route
        if params: url += '?' + urlencode(params)
        row = {'method':method, 'endpoint':route, 'query':params,
               'host_started_monotonic_s':clock()}
        try:
            code, body = request(method, url, .5)
            row.update(http_code=code, response=body)
            return code, body
        except BaseException as error:
            row['error'] = type(error).__name__ + ': ' + str(error)
            raise
        finally:
            row['host_completed_monotonic_s'] = clock()
            journal.write(json.dumps(row, allow_nan=False)+'\n')
            journal.flush()
    try:
        if ready_status is None:
            code, status = call('GET','/status')
            if code != 200: raise RuntimeError('TURN_STATUS_UNAVAILABLE')
        else:
            status = ready_status
        preflight(status)
        boot = status['boot_session_id']
        check_observation(audit, armed_at, clock())
        sent = True  # An ambiguous POST is never retried.
        report['turn_posts'] = 1
        code, body = call('POST', endpoint,
                          {'boot_session_id':boot, 'command_id':cid,
                           'angle_deg':format(angle_deg,'.3f')})
        if code != 202 or body.get('accepted') is not True or body.get('command_id') != cid:
            raise RuntimeError('TURN_ACCEPTANCE_UNKNOWN')
        deadline = clock() + 5.0
        while clock() < deadline:
            code, status = call('GET','/status')
            if code != 200 or status.get('boot_session_id') != boot:
                raise RuntimeError('TURN_STATUS_OR_SESSION_CHANGED')
            if status.get('last_command_id') == cid and status.get('last_command_state') in ('SUCCESS','PARTIAL_PROGRESS','FAULT','STOPPED'):
                report['terminal_status'] = status
                if (status.get('last_command_state') not in ('SUCCESS','PARTIAL_PROGRESS')
                        or status.get('last_command_endpoint') != endpoint
                        or status.get('bounded_motion_profile') != 'BOUNDED_MICRO_TURN_V1'):
                    raise RuntimeError('TURN_TERMINAL_REFUSED')
                preflight(status)  # READY, both PWM zero, no fault latch.
                angle = status.get('bounded_turn_final_angle_rad')
                sign = 1 if endpoint == '/turn-left' else -1
                if (type(angle) not in (int,float) or not math.isfinite(angle)
                        or sign * angle <= 0 or abs(angle) > math.radians(angle_deg) + .01):
                    raise RuntimeError('TURN_ENCODER_DIRECTION_OR_BOUND_REFUSED')
                report['result'] = 'TURN_TERMINAL_REOBSERVE_REQUIRED'
                report['encoder_turn_deg'] = math.degrees(angle)
                return report
            sleep(.02)
        raise RuntimeError('TURN_TERMINAL_TIMEOUT')
    except BaseException as error:
        report['result']='TURN_REFUSED_OR_UNKNOWN'
        report['error']=type(error).__name__+': '+str(error)
        if sent:
            report['stop_posts']=1
            try:
                code, body = call('POST','/stop')
                report['stop_response']={'http_code':code,'body':body}
            except BaseException as stop_error:
                report['stop_error']=type(stop_error).__name__+': '+str(stop_error)
        return report
    finally:
        journal.close()
        (output/'summary.json').write_text(json.dumps(report,indent=2,allow_nan=False)+'\n')

def preflight_turn_before_fresh_frame(*, base_url, output, request=None,
                                      clock=time.monotonic):
    """Read controller state before selecting the frame used for the POST."""
    request = request or Http()
    output.parent.mkdir(parents=True, exist_ok=True)
    row = {'method':'GET', 'endpoint':'/status', 'attempts':[]}
    try:
        for timeout in (1.0, 2.0):
            attempt = {'host_started_monotonic_s':clock(), 'timeout_s':timeout}
            row['attempts'].append(attempt)
            try:
                code, status = request('GET',base_url.rstrip('/')+'/status',timeout)
                attempt.update(http_code=code, response=status)
            except TimeoutError as error:
                attempt['error'] = type(error).__name__+': '+str(error)
                if timeout == 2.0:
                    raise
                continue
            finally:
                attempt['host_completed_monotonic_s'] = clock()
            if code != 200: raise RuntimeError('TURN_STATUS_UNAVAILABLE')
            preflight(status)
            row.update(http_code=code, response=status)
            return status,clock()
    except BaseException as error:
        row['error'] = type(error).__name__+': '+str(error)
        raise
    finally:
        row['host_completed_monotonic_s'] = clock()
        output.write_text(json.dumps(row,allow_nan=False)+'\n')

def next_observation(node, holder, *, after, armed_at, camera_process=None, timeout_s=8):
    holder.clear()
    end=time.monotonic()+timeout_s
    last_reason='NO_FRAME_RECEIVED'
    rejected=0
    while time.monotonic()<end:
        import rclpy
        rclpy.spin_once(node,timeout_sec=.05)
        for audit in holder[:]:
            holder.clear()
            try:
                evidence=check_observation(audit,armed_at,time.monotonic())
                if evidence['oldest_driver_stamp_s'] > after:
                    return audit,evidence
                last_reason='FRAME_NOT_NEWER_THAN_LAST_STOP'
            except (ValueError,TypeError,KeyError) as error:
                last_reason=type(error).__name__+': '+str(error)
            rejected+=1
        if camera_process is not None and camera_process.poll() is not None:
            raise RuntimeError(f'CAMERA_PROCESS_ENDED:exit={camera_process.returncode} rejected={rejected} last={last_reason}')
    raise RuntimeError(f'NO_FRESH_ELIGIBLE_FRAME_AFTER_STOP:rejected={rejected} last={last_reason}')

def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--execute',action='store_true')
    p.add_argument('--base-url')
    p.add_argument('--front-clearance-m',type=float)
    p.add_argument('--max-steps',type=int,default=3)
    p.add_argument('--diagnostic-turn-only',action='store_true',
                   help='One camera-directed 3 degree turn and reobservation; never forward')
    args=p.parse_args()
    if not args.execute:
        print('NO_EXECUTION: no camera, network or motor opened')
        return 0
    if not args.base_url or not args.base_url.startswith('http://') or not args.front_clearance_m or not math.isfinite(args.front_clearance_m) or not 1 <= args.max_steps <= 10 or args.front_clearance_m <= 1.1 + .1*args.max_steps:
        p.error('Require controller URL, 1..10 steps, and measured front clearance > 1.1 m + 0.1 m per step')
    if args.diagnostic_turn_only and args.max_steps != 1:
        p.error('--diagnostic-turn-only requires --max-steps 1')
    lockpath=Path.home()/'.local/state/sie/v7_operator_one_step.lock'
    lockpath.parent.mkdir(parents=True,exist_ok=True)
    lock=lockpath.open('a')
    try: fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
    except BlockingIOError: raise SystemExit('Another V7 trial is running')
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
    from std_msgs.msg import String
    if args.diagnostic_turn_only:
        print('One camera-directed 3 degree turn followed by fresh camera feedback; no forward request.')
    else:
        print('Up to',args.max_steps,'separate 0.10 m commands, each followed by /stop and a new camera observation.')
    print('At most one 1–3 degree camera-guided turn before each forward step and after the final step; each turn requires fresh camera feedback.')
    print('This supervised experiment does not activate Measurement or autonomous motion admission.')
    print('Confirm stationary person, clear approach path and rotation space, cart facing person, and accessible motor power cutoff.')
    armed=time.monotonic()
    root=Path.home()/'.local/state/sie/evidence'/('v7_supervised_approach_'+str(int(time.time()))+'_'+secrets.token_hex(4))
    root.mkdir(parents=True,exist_ok=False)
    report={'result':'NO_MOVEMENT','movement_posts':0,'stop_posts':0,'steps':[],
            'operator_front_clearance_m':args.front_clearance_m,'max_steps':args.max_steps,
            'scope':'OPERATOR_BOUNDED_APPROACH_WITH_CAMERA_FEEDBACK_STEERING',
            'measurement_admission_changed':False, 'turn_posts':0,
            'rgb_intrinsic':{'fx_px':RGB_FX_PX,'cx_px':RGB_CX_PX},
            'turn_deadband_deg':TURN_DEADBAND_DEG,'max_turn_per_step_deg':MAX_TURN_PER_STEP_DEG,
            'diagnostic_turn_only':args.diagnostic_turn_only}
    process=None
    node=None
    try:
        rclpy.init()
        node=Node('sie_v7_approach_'+secrets.token_hex(3))
        holder=[]
        def callback(msg):
            try: holder.append(json.loads(msg.data))
            except (ValueError,TypeError): pass
            if len(holder)>1: del holder[:-1]
        node.create_subscription(String,TOPIC,callback,QoSProfile(depth=1,durability=DurabilityPolicy.VOLATILE,reliability=ReliabilityPolicy.RELIABLE))
        camera_env=os.environ.copy()
        camera_env['SIE_PREVIEW_DURATION_S']='120'
        process=subprocess.Popen(['bash',str(ROOT/'vision_core/tools/run_control_preview_existing_static_v2.sh')],cwd=ROOT,start_new_session=True,env=camera_env)
        def align_once(audit, evidence, z, box, row, *, label=None):
            request=steering_request(target_bearing_deg(box),
                                     diagnostic_turn_only=args.diagnostic_turn_only)
            row['bearing_before_turn_deg']=target_bearing_deg(box)
            if request is None:
                row['steering']='ALREADY_CENTERED'
                return audit,evidence,z,box
            turn_label=label or str(row['index']).zfill(2)
            ready_status, ready_at=preflight_turn_before_fresh_frame(
                base_url=args.base_url,output=root/('turn_'+turn_label+'_preflight.json'))
            # A slow read-only status request may age out the selected frame.
            # The command instead uses a new camera frame captured afterward.
            fresh,_=next_observation(node,holder,after=ready_at,armed_at=armed,
                                     camera_process=process)
            new_z,new_box=depth_and_box(fresh)
            if overlap(box,new_box)<.7 or abs(new_z-z)>.15:
                raise RuntimeError('TURN_PREFLIGHT_TARGET_CORRESPONDENCE_LOST')
            fresh_request=steering_request(target_bearing_deg(new_box),
                                           diagnostic_turn_only=args.diagnostic_turn_only)
            row['post_preflight_cycle_id']=fresh['cycle_id']
            row['post_preflight_bearing_deg']=target_bearing_deg(new_box)
            if fresh_request is None:
                row['steering']='ALREADY_CENTERED_AFTER_PREFLIGHT'
                return fresh,check_observation(fresh,armed,time.monotonic()),new_z,new_box
            if fresh_request['endpoint']!=request['endpoint']:
                raise RuntimeError('TURN_PREFLIGHT_BEARING_DIRECTION_CHANGED')
            audit,z,box,request=fresh,new_z,new_box,fresh_request
            row['steering_request']=request
            result=execute_bounded_turn(audit,armed_at=armed,base_url=args.base_url,
                                        output=root/('turn_'+turn_label),
                                        ready_status=ready_status,**request)
            row['turn_result']=result['result']
            row['turn_encoder_deg']=result.get('encoder_turn_deg')
            row['turn_terminal_state']=(result.get('terminal_status') or {}).get('last_command_state')
            row['turn_posts']=result['turn_posts']
            row['turn_stop_posts']=result['stop_posts']
            report['turn_posts']+=result['turn_posts']
            report['stop_posts']+=result['stop_posts']
            if result['result']!='TURN_TERMINAL_REOBSERVE_REQUIRED':
                raise RuntimeError('TURN_NOT_CONFIRMED:'+result.get('error','UNKNOWN'))
            # No straight request from pre-turn geometry. Require a new capture
            # and observed improvement of the same current-frame target.
            fresh,_=next_observation(node,holder,after=time.monotonic(),armed_at=armed,
                                     camera_process=process)
            new_z,new_box=depth_and_box(fresh)
            after_bearing=target_bearing_deg(new_box)
            row['bearing_after_turn_deg']=after_bearing
            row['post_turn_cycle_id']=fresh['cycle_id']
            row['post_turn_depth_m']=new_z
            verify_turn_feedback(z,box,new_z,new_box)
            row['steering']='CAMERA_ALIGNMENT_IMPROVED'
            return fresh,check_observation(fresh,armed,time.monotonic()),new_z,new_box
        previous_z=previous_box=initial_center_x=None
        after=armed
        for index in range(args.max_steps):
            if process.poll() is not None: raise RuntimeError('CAMERA_PROCESS_ENDED')
            audit,evidence=next_observation(node,holder,after=after,armed_at=armed,camera_process=process,
                                             timeout_s=20 if index==0 else 8)
            z,box=depth_and_box(audit)
            if initial_center_x is None:
                initial_center_x=target_center_x(box)
            center_drift_px=target_center_x(box)-initial_center_x
            if previous_z is not None:
                if overlap(previous_box,box)<.7: raise RuntimeError('TARGET_CORRESPONDENCE_LOST')
                delta=previous_z-z
                report['steps'][-1]['reobserved_depth_m']=z
                report['steps'][-1]['observed_depth_decrease_m']=delta
                report['steps'][-1]['reobserved_rgb_center_x_px']=target_center_x(box)
                report['steps'][-1]['reobserved_center_drift_px']=center_drift_px
                if not .01 <= delta <= .15: raise RuntimeError('DEPTH_PROGRESS_UNCONFIRMED')
            row={'index':index+1,'source_cycle_id':audit['cycle_id'],'depth_before_m':z,
                 'camera_evidence':evidence,'rgb_center_x_px':target_center_x(box),
                 'center_drift_from_first_px':center_drift_px}
            report['steps'].append(row)
            # Keep an operator-declared front-clearance budget separate from optical depth.
            if args.front_clearance_m - .1*(index+1) <= 1.0:
                report['result']='FRONT_CLEARANCE_BUDGET_REACHED'
                break
            audit,evidence,z,box=align_once(audit,evidence,z,box,row)
            if args.diagnostic_turn_only:
                report['result']='TURN_ONLY_REOBSERVED' if row.get('turn_posts') else 'TURN_ONLY_ALREADY_CENTERED'
                break
            row['forward_source_cycle_id']=audit['cycle_id']
            row['forward_depth_m']=z
            row['forward_rgb_center_x_px']=target_center_x(box)
            result=execute_observed_step(audit,armed_at=armed,base_url=args.base_url,
                                         output=root/('controller_'+str(index+1).zfill(2)))
            row['controller_result']=result['result']
            row['movement_posts']=result['movement_posts']
            row['stop_posts']=result['stop_posts']
            report['movement_posts']+=result['movement_posts']
            report['stop_posts']+=result['stop_posts']
            if result['result']!='REMOTE_STOP_OBSERVED_DIAGNOSTIC_ONLY':
                raise RuntimeError('CONTROLLER_STOP_NOT_CONFIRMED')
            t=result.get('terminal_status') or {}
            row['terminal_encoder_heading_rad']=t.get('heading_rad')
            row['terminal_right_count']=t.get('right_count')
            row['terminal_left_count']=t.get('left_count')
            if t.get('right_pwm')!=0 or t.get('left_pwm')!=0 or t.get('state')!='READY':
                raise RuntimeError('CONTROLLER_NOT_READY_AFTER_STOP')
            previous_z,previous_box=z,box
            after=time.monotonic()
        else:
            # Last step also needs fresh feedback before claiming progress.
            audit,_=next_observation(node,holder,after=after,armed_at=armed,camera_process=process)
            z,box=depth_and_box(audit)
            delta=previous_z-z
            report['steps'][-1]['reobserved_depth_m']=z
            report['steps'][-1]['observed_depth_decrease_m']=delta
            report['steps'][-1]['reobserved_rgb_center_x_px']=target_center_x(box)
            report['steps'][-1]['reobserved_center_drift_px']=target_center_x(box)-initial_center_x
            if overlap(previous_box,box)<.7 or not .01<=delta<=.15:
                raise RuntimeError('FINAL_DEPTH_PROGRESS_UNCONFIRMED')
            report['final_alignment']={'index':args.max_steps,'source_cycle_id':audit['cycle_id']}
            audit,_,z,box=align_once(audit,None,z,box,report['final_alignment'],label='final')
            report['final_alignment']['final_bearing_deg']=target_bearing_deg(box)
            report['result']='BOUNDED_SESSION_COMPLETE'
    except (Exception,KeyboardInterrupt) as error:
        report['result']='STOPPED_ON_REFUSAL'
        report['error']=type(error).__name__+': '+str(error)
    finally:
        (root/'summary.json').write_text(json.dumps(report,indent=2,allow_nan=False)+'\n')
        if process is not None and process.poll() is None:
            os.killpg(process.pid,signal.SIGINT)
            try: process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid,signal.SIGTERM)
        if node is not None: node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()
        lock.close()
    print('V7_APPROACH',report['result'],'movement_posts=',report['movement_posts'],'stop_posts=',report['stop_posts'])
    print('Saved:',root/'summary.json')
    return 0 if report['result'] in ('BOUNDED_SESSION_COMPLETE', 'TURN_ONLY_REOBSERVED', 'TURN_ONLY_ALREADY_CENTERED') else 1

if __name__=='__main__':
    raise SystemExit(main())

