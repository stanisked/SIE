"""Async queue behavior under a slow fake identity consumer. No cameras."""
import threading
import time
import unittest

from vision_core.person_localization.shadow_live_identity import LatestIdentityPreview


class FakeProcessor:
    def __init__(self,started=None,release=None):
        self.started=started
        self.release=release
        self.cycles=[]
    def process(self,event):
        cid=event["source"]["cycle_id"]
        self.cycles.append(cid)
        if self.started is not None and cid=="a":
            self.started.set()
            if not self.release.wait(2.):
                raise TimeoutError("TEST_RELEASE_NOT_SET")
        return {"cycle_id":cid,"status":"TEST_ONLY",
                "motor_authorized":False}


class LatestIdentityPreviewTests(unittest.TestCase):
    def test_latest_wins_while_slow_consumer_is_in_flight(self):
        started,release=threading.Event(),threading.Event()
        processor=FakeProcessor(started,release)
        worker=LatestIdentityPreview(lambda:processor)
        try:
            worker.submit({"cycle_id":"a"},track_id=":1",
                          captured_at_utc="test",sole_visible_person=True)
            self.assertTrue(started.wait(1.))
            worker.submit({"cycle_id":"b"},track_id=":1",
                          captured_at_utc="test",sole_visible_person=True)
            worker.submit({"cycle_id":"c"},track_id=":2",
                          captured_at_utc="test",sole_visible_person=True)
            self.assertEqual(worker.replaced,1)
            release.set()
            deadline=time.monotonic()+2.
            while worker.processed<2 and time.monotonic()<deadline:
                time.sleep(.01)
            self.assertEqual(worker.processed,2)
            self.assertEqual(processor.cycles,["a","c"])
            self.assertEqual(worker.drain()["cycle_id"],"c")
            self.assertIsNone(worker.drain())
        finally:
            release.set()
            self.assertTrue(worker.close())

    def test_processor_failure_never_promotes_identity(self):
        class Exploding:
            def process(self,_):
                raise ValueError("BAD_SOURCE")
        worker=LatestIdentityPreview(lambda:Exploding())
        try:
            worker.submit({"cycle_id":"x"},track_id=":1",
                          captured_at_utc="test",sole_visible_person=False)
            deadline=time.monotonic()+2.
            result=None
            while result is None and time.monotonic()<deadline:
                result=worker.drain()
                if result is None:time.sleep(.01)
            self.assertEqual(result["status"],"INFERENCE_REFUSED")
            self.assertIsNone(result["confirmed_entity_id"])
            self.assertFalse(result["motor_authorized"])
        finally:
            self.assertTrue(worker.close())

    def test_initialization_failure_is_visible(self):
        def fail():
            raise RuntimeError("NO_MODEL")
        worker=LatestIdentityPreview(fail)
        try:
            deadline=time.monotonic()+2.
            result=None
            while result is None and time.monotonic()<deadline:
                result=worker.drain()
                if result is None:time.sleep(.01)
            self.assertEqual(result["status"],"IDENTITY_WORKER_UNAVAILABLE")
            self.assertEqual(result["error_type"],"RuntimeError")
        finally:
            self.assertTrue(worker.close())

    def test_submissions_refused_after_close(self):
        worker=LatestIdentityPreview(lambda:FakeProcessor())
        self.assertTrue(worker.close())
        self.assertFalse(worker.submit(
            {"cycle_id":"x"},track_id=":1",
            captured_at_utc="test",sole_visible_person=True))


if __name__=="__main__":
    unittest.main()
