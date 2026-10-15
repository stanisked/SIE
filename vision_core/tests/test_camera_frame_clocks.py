import unittest
from vision_core.tools.probe_camera_frame_clocks import summarize

class FrameClockTests(unittest.TestCase):
    def row(self,flags=0x2000,pause=False):return {'kind':'frame','warmup':False,'after_pause':pause,
        'flags':flags,'driver_timestamp_s':10.,'host_dequeue_monotonic_s':10.25,'sequence':1}
    def test_eof_unknown_and_soe_never_promote_to_exposure_age(self):
        for flag in (0x2000,0,0x12000):
            result=summarize([self.row(flag)])
            self.assertFalse(result['exposure_age_established'])
            self.assertEqual(result['normal_driver_stamp_to_dequeue']['n'],0 if flag==0 else 1)
        self.assertEqual(summarize([self.row()])['timestamp_sources'],{'END_OF_FRAME':1})
        self.assertEqual(summarize([self.row(0x12000)])['timestamp_sources'],{'START_OF_EXPOSURE':1})
    def test_pause_and_driver_errors_are_separate(self):
        result=summarize([self.row(),self.row(0x2040,True)])
        self.assertEqual(result['buffer_error_count'],1)
        self.assertEqual(result['normal_driver_stamp_to_dequeue']['median_s'],.25)
        self.assertEqual(result['after_250ms_read_pause_driver_stamp_to_dequeue']['n'],1)
        self.assertEqual(summarize([])['status'],'NO_POST_WARMUP_FRAMES')

if __name__=='__main__':unittest.main()
