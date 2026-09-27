"""Bounded, background-written native-rate G10 event windows.

Only actual decoded samples are written. Missing packets are never filled in,
and the sample timestamps are still *estimated* from UDP reception times.
"""

from collections import deque
import csv
import os
from pathlib import Path
import queue
import threading


class RawWindowRecorder:
    """Capture +/- event windows without writing 10 kHz CSV in the ROS timer."""

    COLUMNS = ("event_id", "event_command_ns", "estimated_sample_mono_ns",
               "packet_recv_mono_ns", "packet_sequence", "adc_raw",
               "force", "force_unit")

    def __init__(self, prefix, sample_period_ns, pre_sec=0.25,
                 post_sec=0.75, max_pending=3):
        if sample_period_ns <= 0 or pre_sec < 0 or post_sec <= 0:
            raise ValueError("invalid raw capture sample period/windows")
        self.pre_ns = round(pre_sec * 1e9)
        self.post_ns = round(post_sec * 1e9)
        n = max(40, int(self.pre_ns / sample_period_ns) + 80)
        self.ring = deque(maxlen=n)
        self.active = {}
        self.jobs = queue.Queue(maxsize=int(max_pending))
        self.dropped_windows = 0
        self.saved_windows = 0
        self.error = None
        self.prefix = str(prefix)
        self._thread = threading.Thread(target=self._writer, daemon=True,
                                        name="g10_raw_csv_writer")
        self._thread.start()

    def add(self, sample_ns, recv_ns, sequence, raw, force, unit):
        sample = (int(sample_ns), int(recv_ns), int(sequence),
                  float(raw), float(force), str(unit))
        self.ring.append(sample)
        for event_id, task in list(self.active.items()):
            # Samples in batches received after t0 can have estimated ADC
            # timestamps before t0; allow those into the pre-window as well.
            if task["start_ns"] <= sample_ns <= task["end_ns"]:
                task["rows"].append(sample)
            if sample_ns >= task["end_ns"]:
                self._complete(event_id)

    def trigger(self, event_id, command_ns):
        if self.active:
            # An overlapping event is still supported, but bounded to two.
            if len(self.active) >= 2:
                self.dropped_windows += 1
                return False
        event_id = int(event_id)
        command_ns = int(command_ns)
        start_ns = command_ns - self.pre_ns
        # Include prehistory already received; next UDP batch may still bring
        # several additional pre-event samples.
        rows = [s for s in self.ring if start_ns <= s[0] <= command_ns]
        self.active[event_id] = dict(
            command_ns=command_ns, start_ns=start_ns,
            end_ns=command_ns + self.post_ns, rows=rows)
        return True

    def _complete(self, event_id):
        task = self.active.pop(event_id)
        try:
            self.jobs.put_nowait((event_id, task))
        except queue.Full:
            self.dropped_windows += 1

    def abort_active(self):
        """Preserve partial windows for audit if the run stops unexpectedly."""
        for event_id in list(self.active):
            self._complete(event_id)

    def _writer(self):
        try:
            while True:
                job = self.jobs.get()
                try:
                    if job is None:
                        return
                    event_id, task = job
                    path = self.prefix + "_raw_event_%04d.csv" % event_id
                    # Sorted by estimated sample time. No synthetic gap filling.
                    rows = sorted(task["rows"], key=lambda s: s[0])
                    with open(path, "w", newline="", encoding="utf-8") as f:
                        writer = csv.writer(f)
                        writer.writerow(self.COLUMNS)
                        for sample in rows:
                            writer.writerow((event_id, task["command_ns"], *sample))
                    self.saved_windows += 1
                finally:
                    self.jobs.task_done()
        except Exception as exc:
            self.error = exc

    def close(self):
        self.abort_active()
        # Avoid an unbounded shutdown deadlock if the disk writer fails.
        try:
            self.jobs.put(None, timeout=2.0)
        except queue.Full:
            self.error = RuntimeError("G10 raw CSV queue blocked on shutdown")
        self._thread.join(timeout=20.0)
        if self._thread.is_alive() and self.error is None:
            self.error = TimeoutError("G10 raw CSV writer still running")
