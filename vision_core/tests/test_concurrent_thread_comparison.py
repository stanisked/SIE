import sys
from pathlib import Path
from types import SimpleNamespace
import unittest
from unittest.mock import patch
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'tools'))
from control_preview_mask_worker import configure_helper_threads,verify_helper_threads
from benchmark_concurrent_segmentation_threads import signature


class ThreadComparisonTests(unittest.TestCase):
    def test_helper_limit_is_explicit_and_leaves_reference_untouched(self):
        calls=[];counts={'torch':3,'cv2':1}
        def set_count(name,n):calls.append((name,n));counts[name]=n
        with patch.dict(sys.modules,{'torch':SimpleNamespace(set_num_threads=lambda n:set_count('torch',n),get_num_threads=lambda:counts['torch']),
                                    'cv2':SimpleNamespace(setNumThreads=lambda n:set_count('cv2',n),getNumThreads=lambda:counts['cv2'])}):
            configure_helper_threads(None);self.assertEqual(calls,[])
            configure_helper_threads(1);self.assertEqual(calls,[('torch',1),('cv2',1)])
            with self.assertRaises(ValueError):configure_helper_threads(8)
            counts['torch']=3
            with self.assertRaisesRegex(ValueError,'helper thread mismatch'):verify_helper_threads(1)

    def test_limit_follows_backend_initialization_and_is_checked_after_inference(self):
        import ast
        path=Path(__file__).resolve().parents[1]/'tools/control_preview_mask_worker.py'
        tree=ast.parse(path.read_text())
        calls={name:[] for name in ('OnnxSegmentationBackend','YOLO','configure_helper_threads','verify_helper_threads')}
        for n in ast.walk(tree):
            if isinstance(n,ast.Call) and isinstance(n.func,ast.Name) and n.func.id in calls:
                calls[n.func.id].append(n.lineno)
        self.assertGreater(calls['configure_helper_threads'][0],max(calls['OnnxSegmentationBackend']+calls['YOLO']))
        self.assertGreater(max(calls['verify_helper_threads']),calls['configure_helper_threads'][0])

    def test_mask_and_detection_differences_are_not_hidden(self):
        a={'status':'CURRENT_INSTANCES_DIAGNOSTIC_ONLY','instances':[{'bbox':[1,2,3,4]}],'instance_mask_sha256':['mask1']}
        b={**a,'instance_mask_sha256':['mask2']}
        self.assertNotEqual(signature(a),signature(b))
        for invalid in ({**a,'status':'UNAVAILABLE'},{**a,'instance_mask_sha256':[]}):
            with self.assertRaises(ValueError):signature(invalid)


if __name__=='__main__':unittest.main()
