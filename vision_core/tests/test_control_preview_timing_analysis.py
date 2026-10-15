import sys
from pathlib import Path
import unittest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'tools'))
from analyze_control_preview_timing import timing_row,summarize


class TimingAnalysisTests(unittest.TestCase):
    def test_parallel_durations_not_summed_and_nested_deltas(self):
        a={'cycle_id':'one','processing_timing':{'host_capture_start_to_result_ready_s':.36},
           'rgb_segmentation_round_trip_s':.25,'stereo_worker_s':.21,
           'segmentation_observation':{'mask_s':.20},
           'shape_diagnostic':{'round_trip_s':.067,'mask_depth_prepare_s':.058}}
        t=timing_row(a)['timing_s']
        self.assertAlmostEqual(t['host_unallocated_proxy'],.043)
        self.assertAlmostEqual(t['rgb_rpc_outside_inference'],.05)
        self.assertAlmostEqual(t['fusion_rpc_outside_prepare'],.009)
        self.assertNotIn('capture',t)

    def test_missing_timers_remain_missing_and_negative_proxy_not_clipped(self):
        r=timing_row({'cycle_id':'one'})
        self.assertEqual(r['timing_s'],{})
        a={'cycle_id':'two','processing_timing':{'host_capture_start_to_result_ready_s':.1},
           'rgb_segmentation_round_trip_s':.2,'stereo_worker_s':.1,'shape_diagnostic':{'round_trip_s':.1}}
        self.assertLess(timing_row(a)['timing_s']['host_unallocated_proxy'],0)
        self.assertEqual(summarize([r]),{})


if __name__=='__main__':unittest.main()
