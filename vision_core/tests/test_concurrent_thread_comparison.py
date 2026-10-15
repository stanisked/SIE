import sys
from pathlib import Path
from types import SimpleNamespace
import unittest
from unittest.mock import patch
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'tools'))
from control_preview_mask_worker import configure_helper_threads
from benchmark_concurrent_segmentation_threads import signature


class ThreadComparisonTests(unittest.TestCase):
    def test_helper_limit_is_explicit_and_leaves_reference_untouched(self):
        calls=[]
        with patch.dict(sys.modules,{'torch':SimpleNamespace(set_num_threads=lambda n:calls.append(('torch',n))),
                                    'cv2':SimpleNamespace(setNumThreads=lambda n:calls.append(('cv2',n)))}):
            configure_helper_threads(None);self.assertEqual(calls,[])
            configure_helper_threads(1);self.assertEqual(calls,[('torch',1),('cv2',1)])
            with self.assertRaises(ValueError):configure_helper_threads(8)

    def test_mask_and_detection_differences_are_not_hidden(self):
        a={'status':'CURRENT_INSTANCES_DIAGNOSTIC_ONLY','instances':[{'bbox':[1,2,3,4]}],'instance_mask_sha256':['mask1']}
        b={**a,'instance_mask_sha256':['mask2']}
        self.assertNotEqual(signature(a),signature(b))
        for invalid in ({**a,'status':'UNAVAILABLE'},{**a,'instance_mask_sha256':[]}):
            with self.assertRaises(ValueError):signature(invalid)


if __name__=='__main__':unittest.main()
