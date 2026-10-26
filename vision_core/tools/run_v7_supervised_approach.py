#!/usr/bin/env python3
"""Bounded supervised V7 approach: reobserve after every observed stop."""
from __future__ import annotations
import argparse
import fcntl
import json
import math
from pathlib import Path
import secrets
import signal
import subprocess
import time
import sys

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
from vision_core.tools.run_v7_supervised_one_step import check_observation, execute_observed_step, TOPIC

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

def next_observation(node, holder, *, after, armed_at, timeout_s=8):
    holder.clear()
    end=time.monotonic()+timeout_s
    while time.monotonic()<end:
        import rclpy
        rclpy.spin_once(node,timeout_sec=.05)
        for audit in holder[:]:
            holder.clear()
            try:
                evidence=check_observation(audit,armed_at,time.monotonic())
                if evidence['oldest_driver_stamp_s'] > after:
                    return audit,evidence
            except (ValueError,TypeError,KeyError):
                continue
    raise RuntimeError('NO_FRESH_ELIGIBLE_FRAME_AFTER_STOP')

def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--execute',action='store_true')
    p.add_argument('--base-url')
    p.add_argument('--front-clearance-m',type=float)
    p.add_argument('--max-steps',type=int,default=3)
    args=p.parse_args()
    if not args.execute:
        print('NO_EXECUTION: no camera, network or motor opened')
        return 0
    if not args.base_url or not args.base_url.startswith('http://') or not args.front_clearance_m or not math.isfinite(args.front_clearance_m) or not 1 <= args.max_steps <= 10 or args.front_clearance_m <= 1.1 + .1*args.max_steps:
        p.error('Require controller URL, 1..10 steps, and measured front clearance > 1.1 m + 0.1 m per step')
    lockpath=Path.home()/'.local/state/sie/v7_operator_one_step.lock'
    lockpath.parent.mkdir(parents=True,exist_ok=True)
    lock=lockpath.open('a')
    try: fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
    except BlockingIOError: raise SystemExit('Another V7 trial is running')
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
    from std_msgs.msg import String
    token='APPROACH-'+secrets.token_hex(3)
    print('Up to',args.max_steps,'separate 0.10 m commands, each followed by /stop and a new camera observation.')
    print('This supervised experiment does not activate Measurement or autonomous motion admission.')
    print('Confirm stationary person, clear path both ways, cart facing person, and accessible motor power cutoff.')
    if input('Type '+token+' to authorize this bounded session: ').strip()!=token:
        print('NOT_CONFIRMED: nothing started')
        return 1
    armed=time.monotonic()
    root=Path.home()/'.local/state/sie/evidence'/('v7_supervised_approach_'+str(int(time.time()))+'_'+secrets.token_hex(4))
    root.mkdir(parents=True,exist_ok=False)
    report={'result':'NO_MOVEMENT','movement_posts':0,'stop_posts':0,'steps':[],
            'operator_front_clearance_m':args.front_clearance_m,'max_steps':args.max_steps,
            'scope':'OPERATOR_BOUNDED_APPROACH_WITH_V7_DIAGNOSTIC_VETO',
            'measurement_admission_changed':False}
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
        process=subprocess.Popen(['bash',str(ROOT/'vision_core/tools/run_control_preview_existing_static_v2.sh')],cwd=ROOT,start_new_session=True)
        previous_z=previous_box=None
        after=armed
        for index in range(args.max_steps):
            if process.poll() is not None: raise RuntimeError('CAMERA_PROCESS_ENDED')
            audit,evidence=next_observation(node,holder,after=after,armed_at=armed,
                                             timeout_s=20 if index==0 else 8)
            z,box=depth_and_box(audit)
            if previous_z is not None:
                if overlap(previous_box,box)<.7: raise RuntimeError('TARGET_CORRESPONDENCE_LOST')
                delta=previous_z-z
                report['steps'][-1]['reobserved_depth_m']=z
                report['steps'][-1]['observed_depth_decrease_m']=delta
                if not .01 <= delta <= .15: raise RuntimeError('DEPTH_PROGRESS_UNCONFIRMED')
            row={'index':index+1,'source_cycle_id':audit['cycle_id'],'depth_before_m':z,
                 'camera_evidence':evidence}
            report['steps'].append(row)
            # Keep an operator-declared front-clearance budget separate from optical depth.
            if args.front_clearance_m - .1*(index+1) <= 1.0:
                report['result']='FRONT_CLEARANCE_BUDGET_REACHED'
                break
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
            if t.get('right_pwm')!=0 or t.get('left_pwm')!=0 or t.get('state')!='READY':
                raise RuntimeError('CONTROLLER_NOT_READY_AFTER_STOP')
            previous_z,previous_box=z,box
            after=time.monotonic()
        else:
            # Last step also needs fresh feedback before claiming progress.
            audit,_=next_observation(node,holder,after=after,armed_at=armed)
            z,box=depth_and_box(audit)
            delta=previous_z-z
            report['steps'][-1]['reobserved_depth_m']=z
            report['steps'][-1]['observed_depth_decrease_m']=delta
            if overlap(previous_box,box)<.7 or not .01<=delta<=.15:
                raise RuntimeError('FINAL_DEPTH_PROGRESS_UNCONFIRMED')
            report['result']='BOUNDED_SESSION_COMPLETE'
    except (Exception,KeyboardInterrupt) as error:
        report['result']='STOPPED_ON_REFUSAL'
        report['error']=type(error).__name__+': '+str(error)
    finally:
        (root/'summary.json').write_text(json.dumps(report,indent=2,allow_nan=False)+'\n')
        if process is not None and process.poll() is None:
            import os
            os.killpg(process.pid,signal.SIGINT)
            try: process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid,signal.SIGTERM)
        if node is not None: node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()
        lock.close()
    print('V7_APPROACH',report['result'],'movement_posts=',report['movement_posts'],'stop_posts=',report['stop_posts'])
    print('Saved:',root/'summary.json')
    return 0 if report['result']=='BOUNDED_SESSION_COMPLETE' else 1

if __name__=='__main__':
    raise SystemExit(main())
