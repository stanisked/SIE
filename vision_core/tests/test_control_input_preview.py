"""Control-path scheduling, journal backpressure and task-budget preview tests."""
import ast
import contextlib
import io
from concurrent.futures import ThreadPoolExecutor
from datetime import datetime, timezone
import json
from pathlib import Path
import sys
import tempfile
import threading
import time
from types import SimpleNamespace
import unittest

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "ros2/sie_ros2/sie_ros2"))
from latest_diagnostic_writer import LatestDiagnosticWriter
from latest_v4l2_capture import freshness_report, LatestV4L2Capture, select_latest_pair, select_new_rgb, select_stereo_during_rgb
from vision_core.person_localization.control_depth_quality_shadow import assess_control_depth
from approach_control_preview import approach_control_preview
from vision_core.person_localization.diagnostic_instance_selection import select_instance


def preview_node_class(ros_ok=True):
    path = ROOT / "ros2/sie_ros2/sie_ros2/ar0234_ov9281_control_preview_node.py"
    tree = ast.parse(path.read_text())
    cls = next(x for x in tree.body if isinstance(x, ast.ClassDef))
    cls.bases = [ast.Name(id="PreviewParent", ctx=ast.Load())]
    cls.body = [n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name in ("_read", "_cycle", "_infer_current_segmentation", "_left_right_disparity", "_log_preview", "destroy_node")]
    scope = {"PreviewParent":type("PreviewParent",(),{"destroy_node":lambda self: True}),
             "rclpy": SimpleNamespace(ok=lambda: ros_ok), "time": time,
             "datetime": datetime, "timezone": timezone,
             "String": SimpleNamespace, "encode": json.dumps,
             "validate_ar0234_observation": lambda v: v,
             "ar0234_temporal_target_candidate": lambda raw, **kw: raw,
             "LatestV4L2Capture":LatestV4L2Capture,"select_latest_pair":select_latest_pair,"select_new_rgb":select_new_rgb,"select_stereo_during_rgb":select_stereo_during_rgb,"freshness_report":freshness_report, "assess_control_depth":assess_control_depth, "approach_control_preview": approach_control_preview, "select_instance":select_instance}
    exec(compile(ast.fix_missing_locations(ast.Module(body=[cls], type_ignores=[])), str(path), "exec"), scope)
    return scope[cls.name]


class ControlPreviewTests(unittest.TestCase):

    def test_original_jpeg_read_preserves_packet_without_parent_decode(self):
        node=preview_node_class()()
        cap=LatestV4L2Capture.__new__(LatestV4L2Capture)
        cap.width=1920;cap.height=1200
        node.ar_capture=cap;node._native_rgb_jpeg=True
        node._last_capture_sequences={};node._native_capture_metadata={}
        payload=b"\xff\xd8original packet"
        metadata={"sequence":17,"driver_timestamp_s":123.25}
        node._selected_native_packets={"ar0234":(payload,metadata)}
        # No cv2/np attribute: parent decoding here would fail the test.
        frame,stamp=node._read(cap)
        self.assertIs(frame,payload)
        self.assertEqual(stamp,123250000000)
        self.assertEqual(node._native_capture_metadata['ar0234'],metadata)
        self.assertFalse(node._selected_native_packets)
        node._selected_native_packets={"ar0234":(b"bad",metadata)}
        self.assertIsNone(node._read(cap)[0])
        self.assertEqual(node._native_capture_metadata['ar0234']['status'],'CAPTURE_UNAVAILABLE')

    def test_shutdown_without_ros_context_closes_all_resources_and_no_cycle_audit(self):
        node=preview_node_class(ros_ok=False)()
        calls=[]
        node._perception_worker=SimpleNamespace(shutdown=lambda **kw:calls.append("perception"))
        node._half_executor=SimpleNamespace(close=lambda:calls.append("stereo"))
        node._mask_client=SimpleNamespace(close=lambda:calls.append("mask"))
        node._writer=SimpleNamespace(close=lambda:calls.append("writer"),dropped=2,errors=0)
        def forbidden():
            raise AssertionError("ROS logger called after shutdown")
        node.get_logger=forbidden
        output=io.StringIO()
        with contextlib.redirect_stdout(output):
            self.assertTrue(node.destroy_node())
        self.assertEqual(calls,["perception","stereo","mask","writer"])
        self.assertIn("dropped=2 errors=0",output.getvalue())

    def test_slow_writer_drops_pending_record_without_blocking_producer(self):
        started, release = threading.Event(), threading.Event()
        recorded = []
        def slow_write(value):
            if not recorded:
                started.set()
                if not release.wait(3):
                    raise RuntimeError("test release timeout")
            recorded.append(value)
        with tempfile.TemporaryDirectory() as name:
            writer = LatestDiagnosticWriter(Path(name) / "records", write=slow_write)
            try:
                writer.submit({"cycle_id": "one"})
                self.assertTrue(started.wait(2))
                writer.submit({"cycle_id": "two"})
                last = {"cycle_id": "three", "value": [3]}
                writer.submit(last)
                last["value"][0] = 99
                self.assertEqual(writer.pending.qsize(), 1)
                self.assertEqual(writer.dropped, 1)
            finally:
                release.set()
                writer.close()
            self.assertEqual([r["cycle_id"] for r in recorded], ["one", "three"])
            self.assertEqual(recorded[-1]["value"], [3])
            self.assertFalse(writer.submit({"cycle_id": "after-close"}))

    def test_journal_error_is_counted_not_a_foreground_exception(self):
        with tempfile.TemporaryDirectory() as name:
            def fail(_):
                raise OSError("disk unavailable")
            writer = LatestDiagnosticWriter(Path(name) / "records", write=fail)
            writer.submit({"cycle_id": "one"})
            writer.close()
            self.assertEqual(writer.errors, 1)
            self.assertEqual(writer.last_error, "OSError")

    def test_unknown_inputs_never_become_an_approach(self):
        result = approach_control_preview()
        self.assertEqual(result["status"], "STOP")
        self.assertIn("EXPOSURE_AGE_UNKNOWN_OR_INVALID", result["reasons"])
        self.assertFalse(result["motor_command_performed"])

    def test_admitted_synthetic_input_drives_working_speed_preview_and_stale_stops(self):
        values = dict(admitted_measurement=True, measurement_id="synthetic",
                      clearance_m=1.5, clearance_error_bound_m=.1, exposure_age_s=.25,
                      closing_speed_bound_m_s=.15, target_closing_speed_bound_m_s=0.,
                      measured_stop_distance_bound_m=.025, measured_stop_time_bound_s=.5,
                      aligned_in_robot_frame=True)
        result = approach_control_preview(**values)
        self.assertEqual(result["status"], "APPROACH")
        self.assertEqual(result["recommended_speed_m_s"], .15)
        self.assertAlmostEqual(result["required_clearance_m"], 1.2625)
        self.assertFalse(result["motor_command_performed"])
        self.assertEqual(approach_control_preview(**{**values, "exposure_age_s": .251})["status"], "STOP")
        self.assertEqual(approach_control_preview(**{**values, "clearance_m": 1.26})["status"], "STOP")
        moving = approach_control_preview(**{**values, "closing_speed_bound_m_s": 1., "target_closing_speed_bound_m_s": .85})
        self.assertEqual(moving["status"], "STOP")

    def test_new_cycle_never_uses_previous_disparity_and_publish_precedes_journal(self):
        node = preview_node_class()()
        with self.assertRaises(RuntimeError):
            node._cached_stereo = None
            node._left_right_disparity(None, None)
        node.sequence, node.run_id = 0, "test"
        node.stereo_size = (1280, 800)
        node.max_pair_skew_ms = 80
        node.range_min_m, node.range_max_m, node._activated_range = .5, 4.5, [.5, 2.]
        node._confirmed_target_track = None
        node.temporal_target_track_max_age_s = 5
        node.temporal_target_minimum_iou = .2
        node.temporal_target_maximum_center_displacement_relative = .75
        node.temporal_target_minimum_match_margin = .1
        node.ar_capture, node.stereo_capture = "ar", "stereo"
        node._read = lambda cap: (np.zeros((1200,1920,3) if cap == "ar" else (800,2560,3), np.uint8), time.monotonic_ns())
        node._audit_base = lambda cid, *_: {"cycle_id": cid, "stereo_execution": {}, "pairing":{},
            "metric_measurement_authorized": False, "navigation_eligible": False, "actuator_access": False}
        stereo_started, detector_started = threading.Event(), threading.Event()
        maps = object()
        def stereo(_):
            stereo_started.set()
            if not detector_started.wait(2):
                raise RuntimeError("detector did not overlap stereo")
            return object(), object(), maps, .1
        def detect(*_, **kw):
            detector_started.set()
            if not stereo_started.wait(2):
                raise RuntimeError("stereo did not overlap detector")
            return {"disposition": "OBSERVED", "candidate": {}}
        node._compute_stereo = stereo
        node.observer = SimpleNamespace(observe=detect)
        node._publish_observation = lambda _: None
        node._dynamic_target_geometry = lambda *_: {"accepted": True}
        node._selected_observation = lambda *args: {}
        def associate(*args):
            self.assertIs(node._left_right_disparity(None, None), maps)
            return {"range_m": 2.5}, "SUCCESS", {}
        node._frustum_associate = associate
        node._confirm_current_target_track = lambda *_: None
        events, messages = [], []
        node.preview_publisher = SimpleNamespace(publish=lambda msg: (events.append("publish"), messages.append(json.loads(msg.data))))
        node._writer = SimpleNamespace(dropped=0, errors=0, submit=lambda _: events.append("journal"))
        node.get_logger = lambda: SimpleNamespace(info=lambda _: None)
        with ThreadPoolExecutor(max_workers=1) as worker:
            node._perception_worker = worker
            node._cycle()
        self.assertEqual(events, ["publish", "journal"])
        self.assertEqual(messages[0]["status"], "ASSOCIATED_DIAGNOSTIC_ONLY")
        measured=messages[0]['foreground_timing_s']
        self.assertIn('capture_ar_s',measured);self.assertIn('capture_stereo_s',measured)
        self.assertIn('stereo_wait_after_target_s',measured)
        self.assertAlmostEqual(sum(measured.values()),messages[0]['processing_timing']['host_capture_start_to_result_ready_s'],places=10)

        self.assertEqual(messages[0]["decision_preview"]["status"], "STOP")
        self.assertFalse(messages[0]["actuator_access"])
        self.assertIsNone(node._cached_stereo)
        # A global-MAD refusal must not prevent current-frame shape analysis.
        node.np = np
        node.p1 = node.p2 = np.eye(3,4)
        node.r1 = node.r_ar_to_left = node.k_ar = np.eye(3)
        node.t_ar_to_left_m = np.zeros((3,1))
        node.d_ar = np.zeros(5)
        node._ar_inner_roi = lambda _: [10,10,100,100]
        maps = (np.ones((4,4),np.float32), np.ones((4,4),bool))
        node._frustum_associate = lambda *args: (None, "DEPTH_MAD_EXCEEDED:0.06m", {})
        calls = []
        def masked(*args, **kwargs):
            calls.append((args, kwargs))
            return {"status":"SHAPE_AWARE_SHADOW_DIAGNOSTIC_ONLY"}
        node._mask_client = SimpleNamespace(evaluate=masked)
        with ThreadPoolExecutor(max_workers=1) as worker:
            node._perception_worker = worker
            node._cycle()
        self.assertEqual(len(calls),1)
        self.assertEqual(calls[0][0][0],messages[-1]["cycle_id"])
        self.assertEqual(messages[-1]["original_association"]["reason"],"DEPTH_MAD_EXCEEDED:0.06m")
        self.assertEqual(messages[-1]["shape_diagnostic"]["status"],"SHAPE_AWARE_SHADOW_DIAGNOSTIC_ONLY")
        self.assertEqual(messages[-1]["decision_preview"]["status"],"STOP")
        # Half maps must bypass full-resolution association entirely, while
        # keeping current masks and the scale explicit in the worker request.
        node._stereo_scale = .5
        node._resolution_mode = "half_640x400"
        def forbidden(*args):
            raise AssertionError("small disparity entered full-resolution association")
        node._frustum_associate = forbidden
        with ThreadPoolExecutor(max_workers=1) as worker:
            node._perception_worker = worker
            node._cycle()
        self.assertEqual(calls[-1][1], {"scale":.5})
        self.assertEqual(messages[-1]["stereo_resolution"]["matching_size_px"],[640,400])
        self.assertEqual(messages[-1]["original_association"]["reason"],"NOT_EVALUATED_IN_HALF_RESOLUTION_MODE")
        self.assertEqual(messages[-1]["status"],"SHAPE_AWARE_SHADOW_DIAGNOSTIC_ONLY")
        self.assertIsNone(messages[-1]["association"])
        self.assertEqual(messages[-1]["decision_preview"]["status"],"STOP")
        # The single-model backend must never run the old detector, its
        # observation adapter or the second segmentation/evaluate route.
        node._rgb_backend="single_segmentation"
        node.confidence_threshold=.70
        node.observer=SimpleNamespace(observe=forbidden)
        node._selected_observation=forbidden
        node._publish_observation=forbidden
        candidate={"instance_index":0,"bbox_xyxy_px":[500.,300.,850.,1000.],
                   "center_x_px":675.,"confidence":.9,
                   "truncated_left":False,"truncated_right":False,
                   "truncated_top":False,"truncated_bottom":False}
        single_calls=[]
        def infer(cid,rgb):
            single_calls.append("infer")
            return {"cycle_id":cid,"status":"CURRENT_INSTANCES_DIAGNOSTIC_ONLY",
                    "instances":[candidate],"model_sha256":"a"*64,
                    "arrays_sha256":"b"*64,"mask_s":.1,"round_trip_s":.11}
        def fuse(cid,disparity,cal,chosen,roi,interval,segmentation,**kwargs):
            single_calls.append("fuse")
            self.assertEqual(cid,segmentation["cycle_id"])
            self.assertEqual(chosen["instance_index"],0)
            self.assertEqual(kwargs,{"scale":.5})
            return {"status":"SHAPE_AWARE_SHADOW_DIAGNOSTIC_ONLY",
                    "mask_depth":{"status":"COHERENT_MASKED_DEPTH_DIAGNOSTIC_ONLY"}}
        node._mask_client=SimpleNamespace(infer=infer,evaluate_selected=fuse,evaluate=forbidden)
        with ThreadPoolExecutor(max_workers=1) as worker:
            node._perception_worker=worker
            node._cycle()
        self.assertEqual(single_calls,["infer","fuse"])
        self.assertEqual(messages[-1]["rgb_detector_s"],0.)
        self.assertEqual(messages[-1]["segmentation_observation"]["class_name"],"person")
        self.assertFalse(messages[-1]["segmentation_observation"]["canonical_observation_published"])
        self.assertEqual(messages[-1]["decision_preview"]["status"],"STOP")
        # Primary publication/journal must precede background fit submission.
        node._defer_local=True
        def deferred_fuse(*args,**kwargs):
            self.assertEqual(kwargs,{"scale":.5,"defer_local":True})
            return {"status":"MASKED_DEPTH_SHADOW_DIAGNOSTIC_ONLY",
                    "mask_depth":{"status":"DEPTH_MAD_EXCEEDED"},
                    "local_surface":{"status":"DEFERRED_NOT_YET_COMPUTED"},
                    "_surface_job":{"u":np.array([4,8])}}
        node._mask_client.evaluate_selected=deferred_fuse
        submitted=[]
        node._surface_diagnostics=SimpleNamespace(dropped=0,completed=0,errors=0,write_errors=0,
            submit=lambda *args:(events.append("local-submit"),submitted.append(args)))
        events.clear()
        with ThreadPoolExecutor(max_workers=1) as worker:
            node._perception_worker=worker
            node._cycle()
        self.assertEqual(events,["publish","journal","local-submit"])
        self.assertEqual(messages[-1]["status"],"MASKED_DEPTH_SHADOW_DIAGNOSTIC_ONLY")
        self.assertEqual(messages[-1]["shape_diagnostic"]["mask_depth"]["status"],"DEPTH_MAD_EXCEEDED")
        self.assertNotIn("_surface_job",messages[-1]["shape_diagnostic"])
        self.assertEqual(submitted[0][0],messages[-1]["cycle_id"])
        self.assertEqual(submitted[0][2]["source_cycle_id"],messages[-1]["cycle_id"])
        # Six consecutive cycles: fit only 1 and 6, depth on all six.
        node._local_every_n=5
        node.sequence=0
        sampled=[]
        def sampled_fuse(*args,**kwargs):
            scheduled=kwargs.get("prepare_local",True)
            sampled.append(scheduled)
            result={"status":"MASKED_DEPTH_SHADOW_DIAGNOSTIC_ONLY",
                    "mask_depth":{"status":"DEPTH_MAD_EXCEEDED"},
                    "local_surface":{"status":"DEFERRED_NOT_YET_COMPUTED" if scheduled else "SKIPPED_BY_DIAGNOSTIC_SAMPLING"}}
            if scheduled:result["_surface_job"]={"u":np.array([4,8])}
            return result
        node._mask_client.evaluate_selected=sampled_fuse
        submitted.clear()
        first_message=len(messages)
        with ThreadPoolExecutor(max_workers=1) as worker:
            node._perception_worker=worker
            for _ in range(6):node._cycle()
        self.assertEqual(sampled,[True,False,False,False,False,True])
        self.assertEqual([job[0][-8:] for job in submitted],["00000001","00000006"])
        new=messages[first_message:]
        self.assertEqual(len(new),6)
        self.assertTrue(all(row["shape_diagnostic"]["mask_depth"]["status"]=="DEPTH_MAD_EXCEEDED" for row in new))
        self.assertEqual([row["local_surface_async"]["status"] for row in new][1:5],["SKIPPED_BY_DIAGNOSTIC_SAMPLING"]*4)
        # Crop mode waits for THIS cycle's RGB selection and passes only its ROI.
        node._target_crop=True
        crop_calls=[]
        def crop_stereo(combined,roi):
            self.assertEqual(single_calls[-1],"infer")
            self.assertEqual(roi,(10,10,100,100))
            crop_calls.append(roi)
            return object(),object(),maps,.1
        node._compute_stereo=crop_stereo
        with ThreadPoolExecutor(max_workers=1) as worker:
            node._perception_worker=worker
            node._cycle()
        self.assertEqual(len(crop_calls),1)
        self.assertEqual(messages[-1]['crop_target_binding']['cycle_id'],messages[-1]['cycle_id'])
        self.assertEqual(messages[-1]['decision_preview']['status'],'STOP')
        node._local_enabled=False
        node._surface_diagnostics=None
        before=len(submitted)
        with ThreadPoolExecutor(max_workers=1) as worker:
            node._perception_worker=worker
            node._cycle()
        self.assertFalse(sampled[-1])
        self.assertEqual(len(submitted),before)
        self.assertEqual(messages[-1]['shape_diagnostic']['local_surface']['status'],'DISABLED_BY_EXPLICIT_PREVIEW_CONFIGURATION')
        self.assertEqual(messages[-1]['local_surface_async']['scheduled_cycle'],False)
        self.assertEqual(messages[-1]['decision_preview']['status'],'STOP')
        node._capture_backend='latest_v4l2'
        node._native_capture_sha256='c'*64
        original_read=node._read
        def native_read(cap):
            frame,_=original_read(cap)
            name='ar0234' if cap=='ar' else 'ov9281'
            stamp=time.monotonic()-.02
            node._native_capture_metadata[name]={'driver_timestamp_s':stamp,'sequence':10}
            return frame,round(stamp*1e9)
        node._read=native_read
        with ThreadPoolExecutor(max_workers=1) as worker:
            node._perception_worker=worker
            node._cycle()
        fresh=messages[-1]['frame_freshness']
        self.assertEqual(set(fresh['driver_declared_soe_to_result_s']),{'ar0234','ov9281'})
        self.assertFalse(fresh['physical_exposure_timing_independently_verified'])
        self.assertEqual(messages[-1]['decision_preview']['status'],'STOP')
        # The SAME RGB request must already be running while frozen stereo is decoded.
        node._overlap_stereo_decode=True
        inference_started=threading.Event();stereo_decoded=threading.Event();inference_done=threading.Event()
        def concurrent_infer(cid,rgb):
            inference_started.set()
            if not stereo_decoded.wait(2):raise RuntimeError('stereo decode did not overlap RGB')
            try:return infer(cid,rgb)
            finally:inference_done.set()
        def concurrent_read(cap):
            if cap=='stereo':
                if not inference_started.wait(2):raise RuntimeError('RGB not started before stereo decode')
                stereo_decoded.set()
            return native_read(cap)
        node._mask_client.infer=concurrent_infer;node._read=concurrent_read
        with ThreadPoolExecutor(max_workers=1) as worker:
            node._perception_worker=worker
            node._cycle()
        self.assertTrue(messages[-1]['rgb_decode_overlap']['enabled'])
        self.assertEqual(messages[-1]['rgb_decode_overlap']['same_cycle_id'],messages[-1]['cycle_id'])
        self.assertTrue(inference_done.is_set())
        self.assertEqual(messages[-1]['decision_preview']['status'],'STOP')
        self.assertIn('rgb_wait_after_stereo_decode_s',messages[-1]['foreground_timing_s'])
        # Invalid stereo still drains the asynchronous RGB request, with no depth/command.
        def bad_stereo(cap):
            frame,stamp=concurrent_read(cap)
            return (None if cap=='stereo' else frame),stamp
        node._read=bad_stereo;inference_done.clear()
        with ThreadPoolExecutor(max_workers=1) as worker:
            node._perception_worker=worker
            node._cycle()
        self.assertEqual(messages[-1]['status'],'CAMERA_FRAME_UNAVAILABLE')
        self.assertTrue(inference_done.is_set())
        self.assertIsNone(node._cached_stereo)
        node._read=native_read
        node._mask_client.infer=lambda cid,rgb:{"status":"CURRENT_INSTANCES_DIAGNOSTIC_ONLY","instances":[]}
        # A later no-target cycle must not reuse the previous association.
        node.observer = SimpleNamespace(observe=lambda *args, **kw: {"disposition": "UNAVAILABLE", "reason": "NO_TARGET"})
        with ThreadPoolExecutor(max_workers=1) as worker:
            node._perception_worker = worker
            node._cycle()
        self.assertEqual(len(crop_calls),4)  # no stereo job for missing current target
        self.assertEqual(messages[-1]["status"], "TARGET_UNAVAILABLE")
        self.assertIsNone(messages[-1]["association"])
        self.assertIsNone(node._cached_stereo)
        # Original JPEG uses the same selected cycle and worker-owned mask cache.
        node._native_rgb_jpeg=True
        payload=b"\xff\xd8original packet"
        def jpeg_read(cap):
            frame,stamp=native_read(cap)
            return (payload if cap=='ar' else frame),stamp
        def jpeg_infer(cid,value):
            self.assertIs(value,payload)
            return infer(cid,value)
        node._read=jpeg_read
        node._mask_client.infer_jpeg=jpeg_infer
        with ThreadPoolExecutor(max_workers=1) as worker:
            node._perception_worker=worker
            node._cycle()
        self.assertEqual(messages[-1]['status'],'MASKED_DEPTH_SHADOW_DIAGNOSTIC_ONLY')
        self.assertEqual(messages[-1]['rgb_decode']['owner'],'SEGMENTATION_WORKER')
        self.assertEqual(messages[-1]['decision_preview']['status'],'STOP')
        count=len(crop_calls)
        node._mask_client.infer_jpeg=lambda *args:{'status':'MASK_SURFACE_UNAVAILABLE'}
        with ThreadPoolExecutor(max_workers=1) as worker:
            node._perception_worker=worker
            node._cycle()
        self.assertEqual(messages[-1]['reason'],'CURRENT_SEGMENTATION_UNAVAILABLE')
        self.assertEqual(len(crop_calls),count)
        self.assertIsNone(node._cached_stereo)
        # A current RGB request runs while the fresh stereo snapshot waits.
        ar_cap=LatestV4L2Capture.__new__(LatestV4L2Capture)
        ov_cap=LatestV4L2Capture.__new__(LatestV4L2Capture)
        ar_cap.width=1920;ar_cap.height=1200
        node.ar_capture=ar_cap;node.stereo_capture=ov_cap
        node._fresh_stereo_during_rgb=True;node._last_capture_sequences={}
        node._native_rgb_trigger=True
        entered=threading.Event();arrived=threading.Event();done=threading.Event()
        mode={'value':'ok'};sequence={'ar':0,'ov':0}
        def ar_snapshot(after,**kwargs):
            sequence['ar']+=1
            return payload,{'sequence':sequence['ar'],'driver_timestamp_s':time.monotonic()-.032}
        def ov_snapshot(after,**kwargs):
            self.assertTrue(entered.wait(2))
            self.assertIn('not_before_received_s',kwargs)
            sequence['ov']+=1;arrived.set()
            if mode['value']=='timeout':raise TimeoutError('fresh stereo unavailable')
            stamp=time.monotonic()-(.2 if mode['value']=='skew' else .016)
            return b'stereo',{'sequence':sequence['ov'],'driver_timestamp_s':stamp}
        ar_cap.snapshot=ar_snapshot;ov_cap.snapshot=ov_snapshot
        def staged_infer(cid,value):
            entered.set()
            try:
                self.assertTrue(arrived.wait(2))
                return infer(cid,value)
            finally:done.set()
        def staged_read(cap):
            if cap is ar_cap:return type(node)._read(node,cap)
            _,meta=node._selected_native_packets.pop('ov9281')
            node._native_capture_metadata['ov9281']=meta
            frame,_=original_read('stereo')
            return frame,round(meta['driver_timestamp_s']*1e9)
        node._mask_client.infer_jpeg=staged_infer;node._read=staged_read
        for value in ('ok','timeout','skew'):
            mode['value']=value;entered.clear();arrived.clear();done.clear()
            with ThreadPoolExecutor(max_workers=1) as worker:
                node._perception_worker=worker
                node._cycle()
            self.assertTrue(done.is_set())
            self.assertIsNone(node._cached_stereo)
            self.assertEqual(messages[-1]['decision_preview']['status'],'STOP')
            expected={'ok':'MASKED_DEPTH_SHADOW_DIAGNOSTIC_ONLY',
                'timeout':'CAMERA_FRAME_UNAVAILABLE','skew':'PAIR_SKEW_EXCEEDED'}
            self.assertEqual(messages[-1]['status'],expected[value])
            if value=='ok':
                frames=messages[-1]['frame_freshness']['frames']
                self.assertEqual(frames['ar0234']['pair_selected_monotonic_s'],frames['ov9281']['pair_selected_monotonic_s'])
                self.assertIn('capture_stereo_select_s',messages[-1]['foreground_timing_s'])



if __name__ == "__main__":
    unittest.main()
