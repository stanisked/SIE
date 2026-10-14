"""Two independent SGBM objects owned by a persistent diagnostic worker pool."""
from concurrent.futures import ThreadPoolExecutor, wait
import time


class StereoPairExecutor:
    def __init__(self, left_matcher, right_matcher):
        if left_matcher is right_matcher:
            raise ValueError("parallel stereo requires independent matcher objects")
        self.left_matcher, self.right_matcher = left_matcher, right_matcher
        self.pool = ThreadPoolExecutor(max_workers=2, thread_name_prefix="sie-stereo")
        self.closed = False

    @staticmethod
    def _compute(matcher, first, second):
        started = time.perf_counter()
        result = matcher.compute(first, second)
        return result, time.perf_counter() - started

    def compute(self, left, right):
        if self.closed:
            raise RuntimeError("stereo executor is closed")
        first = self.pool.submit(self._compute, self.left_matcher, left, right)
        second = self.pool.submit(self._compute, self.right_matcher, right, left)
        # Drain BOTH tasks even when one raises. A next cycle must never reuse
        # an object or frame while its previous computation is still running.
        wait((first, second))
        try:
            a, left_s = first.result()
            b, right_s = second.result()
        except Exception as error:
            raise RuntimeError("parallel stereo matcher failed") from error
        return a, b, {"left_compute_s": left_s, "right_compute_s": right_s,
                      "overlap_possible": True,
                      "definition": "Concurrent worker durations; do not sum as wall time."}

    def close(self):
        if not self.closed:
            self.closed = True
            self.pool.shutdown(wait=True, cancel_futures=True)
