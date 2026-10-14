"""One inference, exact box/mask ownership and current-only ambiguity gates."""
import sys
from pathlib import Path
from types import SimpleNamespace
import unittest
import os
import tempfile
from unittest.mock import patch
import numpy as np
ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/'vision_core/tools'))
sys.path.insert(0,str(ROOT/'ros2/sie_ros2/sie_ros2'))
from preview_mask_client import PreviewMaskClient
from preview_mask_surface import CurrentInstanceMasks
from vision_core.person_localization.diagnostic_instance_selection import select_instance


class Tensor:
    def __init__(self,value):self.value=np.asarray(value)
    def cpu(self):return self
    def numpy(self):return self.value


class Boxes:
    def __init__(self):
        self.xyxy=Tensor([[500.,200.,800.,1000.],[1000.,200.,1300.,1000.]])
        self.conf=Tensor([.9,.85]);self.cls=Tensor([0,0])
    def __len__(self):return 2


class SingleSegmentationTests(unittest.TestCase):
    def test_masks_come_from_one_inference_and_are_consumed_once(self):
        calls=[]
        mask=np.zeros((1200,1920),np.float32);mask[200:1000,500:800]=1
        other=np.zeros_like(mask);other[200:1000,1000:1300]=1
        def predict(**kwargs):
            calls.append(kwargs)
            return [SimpleNamespace(boxes=Boxes(),masks=SimpleNamespace(data=[Tensor(mask),Tensor(other)]))]
        cache=CurrentInstanceMasks()
        instances=cache.infer('one','rgbsha',np.zeros((1200,1920,3),np.uint8),SimpleNamespace(predict=predict))
        chosen=cache.take('one','rgbsha',1,instances[1]['bbox_xyxy_px'])
        self.assertEqual(len(calls),1)
        np.testing.assert_array_equal(chosen,other>.5)
        with self.assertRaises(ValueError):cache.take('one','rgbsha',1,instances[1]['bbox_xyxy_px'])
        cache.infer('two','sha2',np.zeros((1200,1920,3),np.uint8),SimpleNamespace(predict=predict))
        with self.assertRaises(ValueError):cache.take('one','rgbsha',0,instances[0]['bbox_xyxy_px'])
        self.assertIsNone(cache.current)

    def test_failed_inference_and_wrong_box_never_reuse_a_mask(self):
        cache=CurrentInstanceMasks();cache.current=('old','sha',[],[])
        def fail(**kwargs):raise RuntimeError('inference failed')
        with self.assertRaises(RuntimeError):
            cache.infer('new','sha2',np.zeros((1200,1920,3),np.uint8),SimpleNamespace(predict=fail))
        self.assertIsNone(cache.current)
        cache.current=('new','sha',[{'bbox_xyxy_px':[1,2,3,4]}],[np.ones((2,2),bool)])
        with self.assertRaises(ValueError):cache.take('new','sha',0,[1,2,3,5])
        self.assertIsNone(cache.current)

    def test_actual_worker_protocol_infers_once_then_fuses_cached_mask(self):
        with tempfile.TemporaryDirectory() as directory:
            folder=Path(directory);model=folder/'model.pt';model.write_bytes(b'fixture')
            # Substitute only model inference; run the production process,
            # request hashes, NumPy transport, projection and SciPy fitting.
            (folder/'ultralytics.py').write_text("""import numpy as np
from pathlib import Path
class Tensor:
 def __init__(self,a): self.a=np.asarray(a)
 def cpu(self): return self
 def numpy(self): return self.a
class Boxes:
 xyxy=Tensor([[4.,4.,120.,88.]])
 conf=Tensor([.95])
 cls=Tensor([0])
 def __len__(self): return 1
class Masks:
 data=[Tensor(np.ones((1200,1920),np.float32))]
class Result:
 boxes=Boxes()
 masks=Masks()
class YOLO:
 task='segment'
 names={0:'person'}
 def __init__(self,path,task): self.path=Path(path)
 def predict(self,**kwargs):
  with self.path.with_suffix('.calls').open('a') as f:f.write('1\\n')
  return [Result()]
""")
            env=str(folder)+os.pathsep+os.environ.get('PYTHONPATH','')
            with patch.dict(os.environ,{'PYTHONPATH':env}):
                client=PreviewMaskClient(sys.executable,ROOT/'vision_core/tools/control_preview_mask_worker.py',model)
                try:
                    segmentation=client.infer('cycle',np.zeros((1200,1920,3),np.uint8))
                    self.assertEqual(segmentation['status'],'CURRENT_INSTANCES_DIAGNOSTIC_ONLY')
                    p=np.array([[100.,0,0,0],[0,100.,0,0],[0,0,1,0]])
                    p2=p.copy();p2[0,3]=-75954.18816496264
                    cal=dict(P1=p,P2=p2,R1=np.eye(3),R=np.eye(3),T=np.zeros(3),K=p[:,:3],D=np.zeros(5),size=(128,96))
                    result=client.evaluate_selected('cycle',np.full((48,64),16,np.float32),cal,
                        segmentation['instances'][0],[4,4,120,88],[.5,4.5],segmentation,scale=.5)
                    self.assertEqual(result['status'],'SHAPE_AWARE_SHADOW_DIAGNOSTIC_ONLY')
                    self.assertEqual(result['mask_depth']['status'],'COHERENT_MASKED_DEPTH_DIAGNOSTIC_ONLY')
                    self.assertEqual(result['mask']['selection'],'EXACT_SAME_MODEL_RESULT_INDEX')
                    self.assertEqual(len(model.with_suffix('.calls').read_text().splitlines()),1)
                    again=client.evaluate_selected('cycle',np.full((48,64),16,np.float32),cal,
                        segmentation['instances'][0],[4,4,120,88],[.5,4.5],segmentation,scale=.5)
                    self.assertEqual(again['status'],'MASK_SURFACE_UNAVAILABLE')
                finally:
                    client.close()

    def test_selection_refuses_multiple_stale_ties_and_low_confidence(self):
        kwargs=dict(prior_track=None,captured_at_utc='2026-10-14T20:00:01+00:00',
                    max_age_s=5,minimum_iou=.2,maximum_center_displacement_relative=.75,
                    minimum_match_margin=.15,minimum_confidence=.7)
        first={'instance_index':0,'bbox_xyxy_px':[500,200,800,1000],'confidence':.9}
        second={'instance_index':1,'bbox_xyxy_px':[1000,200,1300,1000],'confidence':.9}
        self.assertEqual(select_instance([first],**kwargs)['disposition'],'OBSERVED')
        self.assertEqual(select_instance([],**kwargs)['disposition'],'UNAVAILABLE')
        self.assertEqual(select_instance([{**first,'confidence':.6}],**kwargs)['disposition'],'UNAVAILABLE')
        self.assertEqual(select_instance([first,second],**kwargs)['disposition'],'UNAVAILABLE')
        prior={'confirmed_at_utc':'2026-10-14T20:00:00+00:00','bbox_xyxy_px':first['bbox_xyxy_px']}
        result=select_instance([first,second],**{**kwargs,'prior_track':prior})
        self.assertEqual(result['candidate']['instance_index'],0)
        tie={**first,'instance_index':2}
        self.assertEqual(select_instance([first,tie],**{**kwargs,'prior_track':prior})['disposition'],'UNAVAILABLE')
        self.assertEqual(select_instance([first,second],**{**kwargs,'prior_track':{**prior,'confirmed_at_utc':'2026-10-14T19:00:00+00:00'}})['reason'],'TEMPORAL_TRACK_STALE')
        edge={**first,'bbox_xyxy_px':[0,200,300,1000]}
        self.assertEqual(select_instance([edge],**kwargs)['disposition'],'UNAVAILABLE')


if __name__=='__main__':unittest.main()
