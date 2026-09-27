import csv
import tempfile
import unittest
from pathlib import Path

from bidirectional_motor_test.g10_health import baseline_ready, g10_health
from bidirectional_motor_test.raw_capture import RawWindowRecorder


class HealthTests(unittest.TestCase):
    def test_udp_baseline_works_without_ros_force_topic(self):
        self.assertTrue(baseline_ready(True, "", True, 980, 1000, 100))
        self.assertFalse(baseline_ready(True, "", False, 980, 1000, 100))
        self.assertFalse(baseline_ready(True, "", True, 1200, 1000, 100))
        self.assertFalse(baseline_ready(True, "", True, 100, 1000, 100))
        self.assertFalse(baseline_ready(False, "", True, 980, 1000, 100))
        self.assertTrue(baseline_ready(False, "/force", True, 980, 1000, 100))

    def test_stream_health_requires_zero_and_fresh_packets(self):
        args = dict(now_ns=1000, timeout_ns=200, backlog=2, backlog_limit=5)
        self.assertEqual(g10_health(last_recv_ns=None, zero_ready=True, **args)[1],
                         "no_packets")
        self.assertEqual(g10_health(last_recv_ns=950, zero_ready=False, **args)[1],
                         "auto_zero_incomplete")
        self.assertEqual(g10_health(last_recv_ns=799, zero_ready=True, **args)[1],
                         "stale_packets")
        self.assertEqual(g10_health(last_recv_ns=950, zero_ready=True,
                                    error=RuntimeError(), **args)[1],
                         "receiver_error")
        self.assertEqual(g10_health(now_ns=1000, timeout_ns=200, backlog=6,
                                    backlog_limit=5, zero_ready=True,
                                    last_recv_ns=950)[1], "queue_backlog")
        self.assertEqual(g10_health(last_recv_ns=950, zero_ready=True, **args),
                         (True, "ready"))


class RawCaptureTests(unittest.TestCase):
    def test_reconstruction_contains_pre_and_post_and_actual_sequence(self):
        with tempfile.TemporaryDirectory() as d:
            rec = RawWindowRecorder(
                str(Path(d) / "run"), sample_period_ns=100_000,
                pre_sec=0.0003, post_sec=0.0002)
            try:
                for i in range(10):
                    rec.add(i * 100_000, i * 100_000 + 20_000, 17,
                            i, i + 1, "count")
                self.assertTrue(rec.trigger(1, 900_000))
                for i in range(10, 13):
                    rec.add(i * 100_000, i * 100_000 + 20_000,
                            18, i, i + 1, "count")
            finally:
                rec.close()
            files = list(Path(d).glob("*_raw_event_0001.csv"))
            self.assertEqual(len(files), 1)
            with files[0].open(newline="") as f:
                rows = list(csv.DictReader(f))
            self.assertEqual([int(x["estimated_sample_mono_ns"]) for x in rows],
                             [600000, 700000, 800000, 900000, 1000000, 1100000])
            self.assertEqual(rows[-1]["packet_sequence"], "18")
            self.assertEqual(rec.saved_windows, 1)
            self.assertIsNone(rec.error)


if __name__ == "__main__":
    unittest.main()
