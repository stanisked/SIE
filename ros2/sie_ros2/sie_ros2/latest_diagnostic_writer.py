"""One pending diagnostic record, no accumulating backlog or controller wait."""
from copy import deepcopy
from pathlib import Path
import json
import queue
import threading


class LatestDiagnosticWriter:
    def __init__(self, directory, write=None):
        self.directory = Path(directory)
        self.directory.mkdir(parents=True, exist_ok=False)
        self.pending = queue.Queue(maxsize=1)
        self.dropped = 0
        self.errors = 0
        self.last_error = None
        self.closed = False
        self.write = write or self._write
        self.worker = threading.Thread(target=self._run, name="sie-diagnostic-writer", daemon=True)
        self.worker.start()

    def _write(self, value):
        # No image/array/model work on the foreground publisher.
        path = self.directory / (value["cycle_id"] + ".json")
        with path.open("x") as stream:
            json.dump(value, stream, allow_nan=False, separators=(",", ":"))
            stream.write("\n")

    def submit(self, value):
        if self.closed:
            return False
        copied = deepcopy(value)
        try:
            self.pending.put_nowait(copied)
        except queue.Full:
            try:
                self.pending.get_nowait()
                self.pending.task_done()
                self.dropped += 1
            except queue.Empty:
                pass
            self.pending.put_nowait(copied)
        return True

    def _run(self):
        while True:
            value = self.pending.get()
            try:
                if value is None:
                    return
                self.write(value)
            except Exception as error:
                self.errors += 1
                self.last_error = type(error).__name__
            finally:
                self.pending.task_done()

    def close(self):
        if not self.closed:
            self.closed = True
            self.pending.put(None)  # Shutdown only; never on the control callback.
            self.worker.join()
