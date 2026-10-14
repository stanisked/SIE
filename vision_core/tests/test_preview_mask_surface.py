"""Geometry scaling, shape/MAD separation and companion cycle isolation."""
from concurrent.futures import ThreadPoolExecutor
import json
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import patch
import contextlib
import io
import cv2
import numpy as np

ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/'vision_core/tools'))
sys.path.insert(0,str(ROOT/'ros2/sie_ros2/sie_ros2'))
from preview_mask_surface import evaluate
from benchmark_stereo_resolution import scaled_configs, paired_delta
from benchmark_stereo_sgbm_parallel import runtime_parameters, compute_pair, consistency
from preview_mask_client import PreviewMaskClient
from stereo_pair_executor import StereoPairExecutor
from vision_core.person_localization.diagnostic_stereo_resolution import half_matchers, half_disparity
import ast
import time


def calibration(w=128,h=96):
    p=np.array([[100.,0,0,0],[0,100.,0,0],[0,0,1,0]])
    p2=p.copy();p2[0,3]=-75954.18816496264
    return dict(P1=p,P2=p2,R1=np.eye(3),R=np.eye(3),T=np.zeros(3),
                K=p[:,:3].copy(),D=np.zeros(5),size=(w,h))


class PreviewMaskSurfaceTests(unittest.TestCase):
    def test_scaled_constant_plane_preserves_metric_depth_and_no_duplicate_samples(self):
        cal=calibration();mask=np.ones((1200,1920),bool)
        counts=[]
        for scale in (1.,.5,.25):
            d=np.full((int(96*scale),int(128*scale)),32*scale,np.float32)
            result=evaluate(d,mask,cal,[4,4,120,88],scale=scale)
            self.assertAlmostEqual(result['mask_depth']['candidate_depth_m'],75.95418816496264/32,places=6)
            self.assertLess(result['local_surface']['pooled_held_out_axial_residual']['absolute_max_m'],1e-6)
            self.assertFalse(result['metric_measurement_authorized'])
            counts.append(result['mask_depth']['selected_sample_count'])
        # Only slight centre/ROI boundary changes, no upsampled duplicates.
        self.assertLess(max(counts)-min(counts),60)

    def test_shape_is_recorded_even_when_global_mad_refuses(self):
        cal=calibration();mask=np.ones((1200,1920),bool)
        _,u=np.indices((96,128))
        result=evaluate((30+.04*u).astype(np.float32),mask,cal,[4,4,120,88])
        self.assertEqual(result['mask_depth']['status'],'DEPTH_MAD_EXCEEDED')
        self.assertEqual(result['status'],'SHAPE_AWARE_SHADOW_DIAGNOSTIC_ONLY')
        self.assertLess(result['local_surface']['pooled_held_out_axial_residual']['absolute_p90_m'],1e-5)
        self.assertFalse(result['acceptance_rule_added'])

    def test_empty_and_invalid_depth_do_not_create_a_candidate(self):
        cal=calibration();mask=np.zeros((1200,1920),bool)
        result=evaluate(np.full((96,128),np.nan),mask,cal,[4,4,120,88])
        self.assertNotIn('candidate_depth_m',result['mask_depth'])
        self.assertEqual(result['local_surface']['sample_count'],0)
        with self.assertRaises(ValueError):
            evaluate(np.ones((48,64)),mask,cal,[4,4,120,88])

    def test_real_sgbm_scaled_search_and_lr_on_known_shift(self):
        configs,_=runtime_parameters();random=np.random.default_rng(10)
        left=random.integers(0,256,(128,384),dtype=np.uint8)
        right=np.zeros_like(left);right[:,:-24]=left[:,24:]
        old=cv2.getNumThreads()
        try:
            cv2.setNumThreads(1)
            with ThreadPoolExecutor(max_workers=2) as pool:
                for scale in (1.,.5,.25):
                    pair=[cv2.resize(a,None,fx=scale,fy=scale,interpolation=cv2.INTER_AREA) for a in (left,right)]
                    c=scaled_configs(configs,scale)
                    matchers=[cv2.StereoSGBM_create(**p) for p in c]
                    maps,_=compute_pair(matchers,*pair,executor=pool)
                    lr=consistency(maps,tolerance=1.5*scale)
                    self.assertGreater(lr.sum(),100)
                    self.assertAlmostEqual(float(np.median(maps[0][lr]/16/scale)),24,delta=.3)
        finally:
            cv2.setNumThreads(old)
        ref=np.full((96,128),32.)
        for s in (.5,.25):
            row=paired_delta(ref,np.full((int(96*s),int(128*s)),32*s),s,75.954)
            self.assertEqual(row['absolute_depth_delta_p90_m'],0)

    def test_live_half_stereo_matches_challenged_offline_and_physical_halves(self):
        # Execute the actual ROS node computation without importing ROS.
        path=ROOT/'ros2/sie_ros2/sie_ros2/ar0234_ov9281_control_preview_node.py'
        tree=ast.parse(path.read_text())
        cls=next(n for n in tree.body if isinstance(n,ast.ClassDef))
        cls.bases=[]
        cls.body=[n for n in cls.body if isinstance(n,ast.FunctionDef) and n.name=='_compute_stereo']
        scope={'time':time,'half_disparity':half_disparity}
        exec(compile(ast.fix_missing_locations(ast.Module(body=[cls],type_ignores=[])),str(path),'exec'),scope)
        node=scope[cls.name]()
        node.cv2=cv2;node.stereo_size=(1280,800);node._stereo_scale=.5
        yy,xx=np.indices((800,1280),dtype=np.float32)
        node.map_left_1=node.map_right_1=xx
        node.map_left_2=node.map_right_2=yy
        configs,_=runtime_parameters()
        parents=[cv2.StereoSGBM_create(**c) for c in configs]
        matchers,actual=half_matchers(cv2,*parents)
        expected=scaled_configs(configs,.5)
        for a,b in zip(actual,expected):
            for key,value in b.items():
                self.assertEqual(a[key],value)
        left=np.random.default_rng(14).integers(0,256,(800,1280,3),dtype=np.uint8)
        right=np.zeros_like(left);right[:,:-32]=left[:,32:]
        combined=np.concatenate((right,left),axis=1)
        old=cv2.getNumThreads();cv2.setNumThreads(1)
        node._half_executor=StereoPairExecutor(*matchers)
        try:
            l,r,(d,lr),elapsed=node._compute_stereo(combined)
            np.testing.assert_array_equal(l,left);np.testing.assert_array_equal(r,right)
            pair=[cv2.resize(cv2.cvtColor(a,cv2.COLOR_BGR2GRAY),(640,400),interpolation=cv2.INTER_AREA) for a in (left,right)]
            other=[cv2.StereoSGBM_create(**c) for c in expected]
            maps,_=compute_pair(other,*pair)
            np.testing.assert_array_equal(d,maps[0].astype(np.float32)/16.)
            np.testing.assert_array_equal(lr,consistency(maps,tolerance=.75))
            self.assertEqual(d.shape,(400,640))
        finally:
            node._half_executor.close();cv2.setNumThreads(old)

    def test_offline_cli_verifies_frozen_sources_and_reports_all_three_scales(self):
        from benchmark_stereo_resolution import main
        from benchmark_stereo_sgbm_parallel import sha256
        from analyze_ar0234_stereo_component_ambiguity import load_calibrations
        with tempfile.TemporaryDirectory() as directory:
            root=Path(directory);folder=root/'case';folder.mkdir()
            cal=calibration(1280,800)
            ar=root/'ar.json'
            ar.write_text(json.dumps({'camera_matrix':cal['K'].tolist(),'distortion_coefficients':cal['D'].tolist()}))
            stereo=root/'stereo.npz'
            np.savez(stereo,camera_1_semantics='physical_left',camera_2_semantics='physical_right',
                     P1=cal['P1'],P2=cal['P2'],R1=cal['R1'],size=cal['size'])
            ex=root/'ex.npz'
            np.savez(ex,R_ar0234_to_physical_left=cal['R'],T_ar0234_to_physical_left_mm=cal['T'],
                     source_stereo_calibration_sha256=sha256(stereo),source_ar_intrinsic_sha256=sha256(ar))
            activation=root/'activation.json'
            activation.write_text(json.dumps({'calibration_profile':{'source_sha256':{
                'ar0234_intrinsic':sha256(ar),'ov9281_stereo_calibration':sha256(stereo)},
                'ar0234_to_ov9281_physical_left_extrinsic':{'path':str(ex),'sha256':sha256(ex)}}}))
            source=folder/'ar0234_source.jpg';cv2.imwrite(str(source),np.zeros((1200,1920,3),np.uint8))
            mask=folder/'case.png';cv2.imwrite(str(mask),np.full((1200,1920),255,np.uint8))
            (folder/'case.json').write_text(json.dumps({'cycle_id':'case','reference_frame':'ar0234_image_frame',
                'source_ar0234_sha256':sha256(source),'mask_sha256':sha256(mask),'model_sha256':'model'}))
            np.save(folder/'disparity_lr_consistent_px.npy',np.full((800,1280),32,np.float32))
            audit={'cycle_id':'case','status':'ASSOCIATION_UNAVAILABLE','reason':'TEST',
                   'calibration':load_calibrations(ar,stereo,ex)['sha'],
                   'stereo_geometry':{'sample_stride_px':4,'ar_inner_roi_xyxy_px':[200,200,1100,700]},
                   'shadow_instance_mask':{'status':'MASK_SAVED_DIAGNOSTIC_ONLY','worker_model_sha256':'model'}}
            (folder/'audit.json').write_text(json.dumps(audit))
            left=np.random.default_rng(4).integers(0,256,(800,1280,3),dtype=np.uint8)
            right=np.zeros_like(left);right[:,:-32]=left[:,32:]
            for side,image in (('left',left),('right',right)):
                cv2.imwrite(str(folder/f'ov9281_physical_{side}_rectified.jpg'),image)
            output=root/'report.json'
            argv=['benchmark','--evidence-root',str(root),'--ar-intrinsic',str(ar),
                  '--stereo-calibration',str(stereo),'--activation',str(activation),'--output',str(output)]
            with patch.object(sys,'argv',argv),contextlib.redirect_stdout(io.StringIO()):
                main()
            report=json.loads(output.read_text())
            self.assertTrue(report['inputs_unchanged'])
            self.assertEqual([r['scale'] for r in report['rows']],[1,.5,.25])
            self.assertTrue(all('mask_surface' in r for r in report['rows']))

    def test_wrong_cycle_response_closes_worker_and_cleans_temporary_arrays(self):
        with tempfile.TemporaryDirectory() as folder:
            path=Path(folder);model=path/'model';model.write_bytes(b'fake')
            script=path/'worker.py'
            script.write_text('''import hashlib,json,sys
sha=hashlib.sha256(open(sys.argv[-1],'rb').read()).hexdigest()
print(json.dumps({'status':'READY','model_sha256':sha}),flush=True)
for line in sys.stdin:
 print(json.dumps({'cycle_id':'WRONG','status':'MASK_SURFACE_UNAVAILABLE'}),flush=True)
''')
            client=PreviewMaskClient(sys.executable,script,model)
            before=set(Path('/dev/shm').glob('sie-preview-*'))
            with self.assertRaisesRegex(ValueError,'cycle mismatch'):
                client.evaluate('cycle',np.zeros((2,2,3),np.uint8),np.zeros((2,2)),{}, {},[],[.5,4.5])
            self.assertIsNone(client.process)
            self.assertEqual(set(Path('/dev/shm').glob('sie-preview-*')),before)


if __name__=='__main__':
    unittest.main()
