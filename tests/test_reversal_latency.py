"""Reversal measurement is a causal host-observed sign change, not a spike.

The controller explicitly marks its FIRST opposite nonzero DSHOT command.
The G10 detector accepts one median per physical UDP packet; no invented
samples and no future-dated estimated ADC timestamps enter this clock.
"""
import unittest

from bidirectional_motor_test.reversal_latency import ReversalDirectionTracker

MS = 1_000_000


class ReversalDirectionTests(unittest.TestCase):
    def make(self, **kwargs):
        return ReversalDirectionTracker(
            sign_threshold=4.0, stable_sec=0.2, timeout_sec=5.0, **kwargs)

    def test_positive_to_negative_sustained_crossing_times_first_packet(self):
        tracker = self.make()
        t0 = 1_000 * MS
        self.assertEqual(tracker.start(4, t0, 1, -1, 20.0), [])
        for t in (10, 14, 18, 22, 26):
            self.assertEqual(tracker.observe_packet(t0 + t * MS, -3), [])
        self.assertEqual(tracker.observe_packet(t0 + 30 * MS, -6), [])
        detected = []
        for t in range(34, 235, 4):
            detected.extend(tracker.observe_packet(t0 + t * MS, -6))
        self.assertEqual(len(detected), 1)
        event = detected[0]
        self.assertEqual(event.metric, "force_direction_change")
        self.assertEqual(event.status, "detected")
        self.assertEqual((event.from_direction, event.to_direction), (1, -1))
        self.assertEqual(event.observed_ns, t0 + 30 * MS)
        self.assertEqual(event.confirmed_ns, t0 + 230 * MS)
        self.assertEqual(event.latency_ms, 30.0)
        self.assertEqual(tracker.observe_packet(t0 + 300 * MS, -10), [])

    def test_negative_to_positive_ignores_first_directionless_startup(self):
        tracker = self.make()
        with self.assertRaisesRegex(ValueError, "exactly"):
            tracker.start(1, 0, 0, 1, 0)
        t0 = 2_000 * MS
        tracker.start(2, t0, -1, 1, -15)
        for t in range(4, 230, 4):
            result = tracker.observe_packet(t0 + t * MS, 6)
            if result:
                self.assertEqual(result[0].latency_ms, 4.0)
                self.assertEqual(
                    (result[0].from_direction, result[0].to_direction),
                    (-1, 1))
                break
        else:
            self.fail("sustained positive thrust not detected")

    def test_single_packet_or_one_millisecond_spikes_are_not_reversal(self):
        tracker = self.make()
        t0 = 3_000 * MS
        tracker.start(3, t0, 1, -1, 16)
        for t in range(4, 150, 4):
            spike = -22 if t in (20, 64, 108) else 0
            self.assertEqual(
                tracker.observe_packet(t0 + t * MS, spike), [])
        self.assertEqual(tracker.expire(t0 + 5_000 * MS), [])
        timed_out = tracker.expire(t0 + 5_001 * MS)
        self.assertEqual(len(timed_out), 1)
        self.assertEqual(timed_out[0].status, "timeout")
        self.assertIsNone(timed_out[0].latency_ms)

    def test_missing_packets_break_confirmation_dwell(self):
        tracker = self.make()
        t0 = 1_000 * MS
        tracker.start(7, t0, -1, 1, -20)
        for t in range(10, 126, 4):
            self.assertEqual(tracker.observe_packet(t0 + t * MS, 10), [])
        self.assertEqual(tracker.observe_packet(t0 + 200 * MS, 10), [])
        results = []
        for t in range(204, 410, 4):
            results.extend(tracker.observe_packet(t0 + t * MS, 10))
        self.assertEqual(len(results), 1)
        self.assertEqual(results[0].observed_ns, t0 + 200 * MS)

    def test_no_baseline_or_already_target_does_not_fabricate_latency(self):
        tracker = self.make()
        r = tracker.start(1, 100, 1, -1, None)
        self.assertEqual(r[0].status, "no_pre_command_force")
        self.assertIsNone(r[0].observed_ns)
        self.assertIsNone(r[0].latency_ms)
        self.assertEqual(tracker.observe_packet(300, -40), [])
        r = tracker.start(2, 400, 1, -1, -8)
        self.assertEqual(r[0].status, "already_target_before_command")
        self.assertIsNone(r[0].latency_ms)

    def test_disarm_and_fault_cancel_without_detected_result(self):
        tracker = self.make()
        tracker.start(1, 100, 1, -1, 12)
        self.assertEqual(tracker.observe_packet(110, -10), [])
        rows = tracker.cancel("command_stopped")
        self.assertEqual(rows[0].status, "command_stopped")
        self.assertIsNone(rows[0].latency_ms)
        self.assertEqual(tracker.cancel(), [])
        tracker.start(2, 200, -1, 1, -10)
        rows = tracker.cancel()
        self.assertEqual(rows[0].status, "stream_fault")

    def test_superseding_a_reversal_preserves_unfinished_status(self):
        tracker = self.make()
        tracker.start(1, 100, 1, -1, 12)
        rows = tracker.start(2, 200, -1, 1, -12)
        self.assertEqual(rows[0].event_id, 1)
        self.assertEqual(rows[0].status, "superseded")
        self.assertIsNone(rows[0].latency_ms)

    def test_receive_time_not_adc_estimate_sets_measurement_clock(self):
        tracker = self.make()
        t0 = 500 * MS
        tracker.start(1, t0, 1, -1, 20)
        # Some G10 *estimated* sample timestamps are later than UDP receive;
        # this detector is passed ONLY the host receipt time. No arbitrary
        # subtraction/clock offset is performed.
        results = []
        for rx_ms in range(510, 720, 4):
            results.extend(tracker.observe_packet(rx_ms * MS, -9))
        self.assertEqual(len(results), 1)
        self.assertEqual(results[0].latency_ms, 10.0)
        self.assertEqual(results[0].command_ns, t0)


if __name__ == "__main__":
    unittest.main()
