import io
import json
from pathlib import Path
import sys
import threading
import unittest
from unittest.mock import patch
sys.path.insert(0,str(Path(__file__).resolve().parents[2]/'ros2/sie_ros2/sie_ros2'))
from latest_v4l2_capture import LatestV4L2Capture,read_exact,validate_packet,freshness_report


def metadata(sequence):return {'kind':'frame','payload_bytes':3,'sequence':sequence,'flags':0x12000,
    'driver_timestamp_s':1.,'host_dequeue_monotonic_s':1.02,'warmup':False}

class HeldStream(io.BytesIO):
    def __init__(self,data):super().__init__(data);self.stop=threading.Event();self.drained=threading.Event()
    def readline(self,*args):
        value=super().readline(*args)
        if not value:self.drained.set();self.stop.wait(2)
        return value

class FakeProcess:
    def __init__(self,stream):self.stdout=stream;self.code=None
    def poll(self):return self.code
    def terminate(self):self.code=0;self.stdout.stop.set()
    def kill(self):self.terminate()
    def wait(self,timeout):return self.code

class LatestCaptureTests(unittest.TestCase):
    def test_driver_age_does_not_turn_into_physical_timing_or_command(self):
        data={'ar0234':{'driver_timestamp_s':1.},'ov9281':{'driver_timestamp_s':1.02}}
        good=freshness_report(data,1.24,'hash')
        self.assertTrue(good['within_250ms_by_driver_metadata'])
        self.assertFalse(good['physical_exposure_timing_independently_verified'])
        self.assertFalse(good['command_issued'])
        self.assertFalse(freshness_report(data,1.3,'hash')['within_250ms_by_driver_metadata'])
        self.assertFalse(freshness_report({'ar0234':data['ar0234']},1.1,'hash')['within_250ms_by_driver_metadata'])
        self.assertFalse(freshness_report(data,.9,'hash')['within_250ms_by_driver_metadata'])

    def test_latest_only_sequence_and_close(self):
        def line(value):return (json.dumps(value)+'\n').encode()
        data=line({'kind':'format','width':2,'height':2,'fourcc':1196444237})+line({'kind':'buffers','allocated':1})
        data+=line(metadata(3))+b'old'+line(metadata(5))+b'new'
        stream=HeldStream(data);process=FakeProcess(stream)
        with patch('latest_v4l2_capture.subprocess.Popen',return_value=process):
            capture=LatestV4L2Capture('fake','fake',2,2,timeout=.01)
            try:
                self.assertTrue(stream.drained.wait(1))
                payload,info=capture.snapshot(3)
                self.assertEqual(payload,b'new');self.assertEqual(info['sequence'],5)
                self.assertEqual(info['received_count'],2)
                with self.assertRaises(TimeoutError):capture.snapshot(5)
            finally:capture.release()
            self.assertFalse(capture.thread.is_alive())
            with self.assertRaises(RuntimeError):capture.snapshot()
    def test_clock_sequence_error_and_size_refuse(self):
        validate_packet(metadata(5),2,2,3)
        for changes in ({'flags':0x2000},{'flags':0x12040},{'sequence':3},{'payload_bytes':0},
                        {'driver_timestamp_s':2.}):
            with self.assertRaises(ValueError):validate_packet({**metadata(5),**changes},2,2,3)
        with self.assertRaises(EOFError):read_exact(io.BytesIO(b'ab'),3)
        self.assertEqual(read_exact(io.BytesIO(b'abc'),3),b'abc')
    def test_truncated_stream_invalidates_capture(self):
        data=(json.dumps({'kind':'format','width':2,'height':2,'fourcc':1196444237})+'\n'+
              json.dumps({'kind':'buffers','allocated':1})+'\n'+json.dumps(metadata(1))+'\n').encode()+b'x'
        stream=HeldStream(data);process=FakeProcess(stream)
        with patch('latest_v4l2_capture.subprocess.Popen',return_value=process):
            capture=LatestV4L2Capture('fake','fake',2,2,timeout=1)
            try:
                with self.assertRaisesRegex(RuntimeError,'truncated'):capture.snapshot()
            finally:capture.release()

if __name__=='__main__':unittest.main()
