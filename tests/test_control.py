import unittest

from bidirectional_motor_test.core import (
    ARM, DISARM, RUN, SineDshot, SwitchInterlock,
)
from bidirectional_motor_test.latency import ThrustLatency


SECOND = 1_000_000_000


class SwitchTests(unittest.TestCase):
    def test_startup_requires_disarm_then_arm(self):
        gate = SwitchInterlock()
        self.assertEqual(gate.mode(RUN), "WAIT_FOR_ARM")
        self.assertEqual(gate.mode(ARM), "WAIT_FOR_DISARM")
        self.assertEqual(gate.mode(DISARM), "DISARM")
        self.assertEqual(gate.mode(RUN), "WAIT_FOR_ARM")
        self.assertEqual(gate.mode(ARM), "ARMED")
        self.assertEqual(gate.mode(RUN), "SINE")
        self.assertEqual(gate.mode(ARM), "ARMED")
        self.assertEqual(gate.mode(RUN), "SINE")
        self.assertEqual(gate.mode(DISARM), "DISARM")
        self.assertEqual(gate.mode(RUN), "WAIT_FOR_ARM")

    def test_fault_revokes_authorization(self):
        gate = SwitchInterlock()
        gate.mode(DISARM)
        gate.mode(ARM)
        gate.fault()
        self.assertEqual(gate.mode(RUN), "WAIT_FOR_ARM")
        self.assertEqual(gate.mode(ARM), "WAIT_FOR_DISARM")
        gate.mode(DISARM)
        gate.mode(ARM)
        self.assertEqual(gate.mode(5), "INVALID_SWITCH")
        self.assertEqual(gate.mode(RUN), "WAIT_FOR_ARM")


class SineTests(unittest.TestCase):
    def test_positive_negative_and_guard_pause(self):
        wave = SineDshot(
            frequency_hz=0.05, deadband=0.08,
            positive_peak=1250, negative_peak=250,
            reversal_pause_sec=2.0)
        wave.start(0)
        self.assertEqual(wave.step(0).dshot, 0)
        pos = wave.step(SECOND)
        self.assertEqual(pos.direction, 1)
        self.assertTrue(1048 <= pos.dshot <= 1250)
        self.assertEqual(pos.event, "first_command")
        wait = wave.step(11 * SECOND)
        self.assertEqual(wait.event, "reversal_pause")
        self.assertEqual(wait.dshot, 0)
        frozen = wave.phase_rad
        self.assertEqual(wave.step(12 * SECOND).dshot, 0)
        self.assertEqual(wave.phase_rad, frozen)
        reverse = wave.step(14 * SECOND)
        self.assertEqual(reverse.event, "reversal_command")
        self.assertEqual(reverse.direction, -1)
        self.assertTrue(48 <= reverse.dshot <= 250)
        self.assertEqual(wave.phase_rad, frozen)

    def test_rpm_interlock_freezes_output(self):
        wave = SineDshot(reversal_pause_sec=1.0)
        wave.start(0)
        wave.step(SECOND)
        wave.step(11 * SECOND)
        wait = wave.step(13 * SECOND, rpm_ready=False)
        self.assertEqual(wait.dshot, 0)
        self.assertEqual(wait.event, "waiting_for_rpm")
        self.assertEqual(wave.step(20 * SECOND, rpm_ready=False).dshot, 0)
        run = wave.step(21 * SECOND, rpm_ready=True)
        self.assertEqual(run.event, "reversal_command")
        self.assertLess(run.dshot, 1048)

    def test_inversion_and_stop(self):
        wave = SineDshot(invert_direction=True)
        wave.start(0)
        command = wave.step(SECOND)
        self.assertTrue(48 <= command.dshot <= 1047)
        wave.stop()
        self.assertEqual(wave.step(2 * SECOND).dshot, 0)

    def test_reject_invalid_values(self):
        for kwargs in ({"frequency_hz": 0},
                       {"deadband": 0.9},
                       {"positive_peak": 1000},
                       {"negative_peak": 1100},
                       {"reversal_pause_sec": -1}):
            with self.subTest(kwargs=kwargs):
                with self.assertRaises(ValueError):
                    SineDshot(**kwargs)


class LatencyTests(unittest.TestCase):
    def test_detects_first_confirmed_force_change_and_sign_cross(self):
        tracker = ThrustLatency(delta_threshold=0.1, sign_threshold=0.1,
                                confirm_samples=2, timeout_sec=1)
        self.assertEqual(tracker.start(7, 0, -1, 1.0), [])
        self.assertEqual(tracker.observe(10_000_000, 0.8), [])
        result = tracker.observe(20_000_000, 0.6)
        self.assertEqual(len(result), 1)
        self.assertEqual(result[0].metric, "force_onset")
        self.assertEqual(result[0].latency_ms, 10.0)
        self.assertEqual(tracker.observe(30_000_000, -0.2), [])
        result = tracker.observe(40_000_000, -0.3)
        self.assertEqual(len(result), 1)
        self.assertEqual(result[0].metric, "target_sign")
        self.assertEqual(result[0].latency_ms, 30.0)

    def test_wrong_force_direction_and_no_feedback(self):
        tracker = ThrustLatency(confirm_samples=1)
        self.assertEqual(tracker.start(1, 0, 1, None), [])
        self.assertEqual(tracker.observe(SECOND, 20.0), [])
        tracker.start(2, SECOND, -1, 0.0)
        self.assertEqual(tracker.observe(SECOND + 1, 0.2), [])
        result = tracker.observe(SECOND + 2, -0.1)
        self.assertEqual(result[0].metric, "force_onset")

    def test_already_signed_and_timeout(self):
        tracker = ThrustLatency(confirm_samples=1, timeout_sec=0.5)
        status = tracker.start(2, 0, -1, -0.5)
        self.assertEqual(status[0].status, "already_reached")
        result = tracker.expire(SECOND)
        self.assertEqual(len(result), 1)
        self.assertEqual(result[0].metric, "force_onset")
        self.assertEqual(result[0].status, "timeout")


if __name__ == "__main__":
    unittest.main()
