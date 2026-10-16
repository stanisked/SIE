"""Synthetic geometry and isolated integration checks; no cameras, ROS or motion."""
import ast
import copy
import importlib.util
import math
from pathlib import Path
from types import SimpleNamespace
import unittest
import numpy as np
from vision_core.person_localization.robot_front_geometry import load_profile, forward_terms, describe_front, profile_hash
from vision_core.person_localization.instance_mask_stereo_fusion import depth_from_owned_samples

ROOT=Path(__file__).resolve().parents[2]
PROFILE=ROOT/'vision_core/config/diagnostics/robot_front_mount_operator_v1.json'
P=np.array([[100.,0,64,0],[0,100.,48,0],[0,0,1,0]])

def profile():return load_profile(PROFILE)

class FrontGeometryTests(unittest.TestCase):
    def test_axis_projection_and_unknown_datum_are_separate(self):
        p=profile();r=describe_front([64],[48],[1.5],P,np.eye(3),p)
        self.assertAlmostEqual(r['known_term_m']['median'],1.5*math.cos(math.radians(15))-.21)
        self.assertIsNone(r['distance_from_front_m'])
        self.assertIsNone(r['rim_forward_of_optical_center_m'])
        self.assertFalse(r['actuator_access']);self.assertFalse(r['measurement_authorized'])
    def test_each_point_y_changes_forward_distance_at_same_z(self):
        terms=forward_terms([64,64],[38,58],[1.5,1.5],P,np.eye(3),profile())
        self.assertAlmostEqual(terms[1]-terms[0],.3*math.sin(math.radians(15)))
    def test_rectification_is_undone_before_mount_pitch(self):
        a=.2;r1=np.array([[math.cos(a),0,math.sin(a)],[0,1,0],[-math.sin(a),0,math.cos(a)]])
        raw=np.array([[.2,.3,1.5],[-.1,-.3,2.]])
        rect=raw@r1.T
        pixels=(rect@P[:,:3].T)/rect[:,2,None]
        terms=forward_terms(pixels[:,0],pixels[:,1],rect[:,2],P,r1,profile())
        expected=raw[:,1]*math.sin(math.radians(15))+raw[:,2]*math.cos(math.radians(15))-.21
        np.testing.assert_allclose(terms,expected,atol=1e-12)
    def test_half_resolution_pixel_centres_match_original(self):
        u=np.array([60.,70.]);v=np.array([30.,80.]);z=np.array([1.,2.])
        full=forward_terms(u,v,z,P,np.eye(3),profile())
        half=forward_terms((u+.5)*.5-.5,(v+.5)*.5-.5,z,P,np.eye(3),profile(),scale=.5)
        np.testing.assert_array_equal(full,half)
    def test_invalid_or_unreviewed_geometry_refused(self):
        for r1 in (np.eye(3)*2,np.diag([1,1,-1]),np.full((3,3),np.nan)):
            with self.assertRaises(ValueError):forward_terms([1],[1],[1],P,r1,profile())
        p=profile();p['rim_forward_of_optical_center_m']=.01
        with self.assertRaises(ValueError):forward_terms([1],[1],[1],P,np.eye(3),p)
        p=profile();p['alignment_assumptions']['block_yaw_deg']=3
        with self.assertRaises(ValueError):forward_terms([1],[1],[1],P,np.eye(3),p)
    def test_empty_and_nonfinite_points(self):
        self.assertEqual(describe_front([],[],[],P,np.eye(3),profile())['status'],'NO_OWNED_POINTS')
        with self.assertRaises(ValueError):forward_terms([1],[1],[float('nan')],P,np.eye(3),profile())
    def test_input_arrays_unchanged_and_profile_hash_sensitive(self):
        arrays=[np.array([1.,2.]),np.array([3.,4.]),np.array([1.,1.]),P.copy(),np.eye(3)]
        copies=[x.copy() for x in arrays]
        forward_terms(*arrays,profile())
        for a,b in zip(arrays,copies):np.testing.assert_array_equal(a,b)
        changed=profile();changed['block_pitch_up_deg']=14
        self.assertNotEqual(profile_hash(profile()),profile_hash(changed))

class FrontIntegrationTests(unittest.TestCase):
    def client(self, corrupt=False):
        spec=importlib.util.spec_from_file_location('front_client',ROOT/'ros2/sie_ros2/sie_ros2/preview_mask_client.py')
        m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m)
        c=object.__new__(m.PreviewMaskClient);c.model_sha256='model'
        def request(cycle,arrays,req):
            self.assertIn('front_P1',arrays);self.assertIn('front_R1',arrays)
            self.assertEqual(req['front_mount_profile'],profile())
            provenance={'source_cycle_id':cycle,'mask_sha256':'mask','instance_index':0,'model_sha256':'model',
                        'rgb_arrays_sha256':'rgb','fusion_arrays_sha256':'fusion'}
            if corrupt:provenance['source_cycle_id']='old'
            return {'arrays_sha256':'fusion','mask_sha256':'mask','front_geometry':{
                'provenance':provenance,'profile_sha256':profile_hash(profile()),'distance_from_front_m':None,'actuator_access':False}}
        c._array_request=request
        return c
    def test_selected_mask_cycle_binding_is_checked(self):
        args=('cycle',np.ones((2,2)),{'P1':P,'R1':np.eye(3)},
              {'instance_index':0,'bbox_xyxy_px':[0,0,2,2]},[0,0,2,2],[.5,4.5],
              {'cycle_id':'cycle','model_sha256':'model','arrays_sha256':'rgb'})
        r=self.client().evaluate_selected(*args,scale=.5,front_profile=profile())
        self.assertIsNone(r['front_geometry']['distance_from_front_m'])
        with self.assertRaisesRegex(ValueError,'binding mismatch'):
            self.client(corrupt=True).evaluate_selected(*args,scale=.5,front_profile=profile())
    def test_optional_front_math_keeps_original_depth_and_owned_selection(self):
        # Exercise actual evaluate_prepared/diagnostic_result with mocked image ownership.
        # No OpenCV behavior is claimed by this test.
        path=ROOT/'vision_core/tools/preview_mask_surface.py';tree=ast.parse(path.read_text())
        funcs=[n for n in tree.body if isinstance(n,ast.FunctionDef) and n.name in {'evaluate_prepared','diagnostic_result'}]
        ns={'np':np,'depth_from_owned_samples':depth_from_owned_samples,
            'cv2':SimpleNamespace(erode=lambda a,*x,**k:a,BORDER_CONSTANT=0),
            'owned':lambda xy,interior,roi:np.arange(len(xy))%2==0}
        exec(compile(ast.Module(body=funcs,type_ignores=[]),str(path),'exec'),ns)
        n=240;points={'u':np.full(n,64.),'v':np.linspace(30,80,n),'d':np.full(n,30.),'z':np.full(n,2.),
                     'xy':np.zeros((n,2)),'fb':np.asarray(60.),'stride':np.asarray(4),'scale':np.asarray(1.),
                     'lower':np.asarray(.5),'upper':np.asarray(4.5)}
        mask=np.ones((1200,1920),dtype=bool)
        opts={'defer_local':True,'prepare_local':False}
        old,_=ns['evaluate_prepared'](points,mask,[0,0,10,10],**opts)
        new,_=ns['evaluate_prepared'](points,mask,[0,0,10,10],front_geometry={'profile':profile(),'P1':P,'R1':np.eye(3)},**opts)
        self.assertEqual(old['mask_depth'],new['mask_depth'])
        self.assertNotIn('front_geometry',old)
        self.assertEqual(new['front_geometry']['point_count'],120)
        self.assertFalse(new['navigation_eligible'])

if __name__=='__main__':unittest.main()
