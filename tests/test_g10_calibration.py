"""Calibration regression tests: stable windows, known-mass gain, persistence."""

import json
import tempfile
import unittest
from pathlib import Path

from bidirectional_motor_test.g10_calibration import (
    CalibrationError, StableADCWindow, load_scale, save_scale,
    scale_for_known_mass,
)


class CalibrationTests(unittest.TestCase):
    def test_stable_window_mean_and_reject_load_motion(self):
        window = StableADCWindow(
            sample_period_ns=100_000, window_sec=0.1, decimation=10)
        now = 1_000_000_000
        for i in range(1200):
            window.add(now - 120_000_000 + i * 100_000, 32687 + i % 3)
        mean, spread, n = window.snapshot(now, 20_000_000, 10)
        self.assertAlmostEqual(mean, 32688, places=1)
        self.assertEqual(spread, 2)
        self.assertEqual(n, 100)
        for i in range(1200, 2400):
            window.add(now - 120_000_000 + i * 100_000,
                       30000 + i % 40)
        with self.assertRaisesRegex(CalibrationError, "not stable"):
            window.snapshot(now + 120_000_000, 20_000_000, 10)

    def test_incomplete_stale_and_sample_gaps_refused(self):
        window = StableADCWindow(
            sample_period_ns=100_000, window_sec=0.1, decimation=10)
        for i in range(50):
            window.add(i * 100_000, 3)
        with self.assertRaisesRegex(CalibrationError, "need"):
            window.snapshot(50_000_000, 150_000_000, 10)
        for i in range(50, 1200):
            window.add(i * 100_000, 3)
        with self.assertRaisesRegex(CalibrationError, "stale"):
            window.snapshot(800_000_000, 50_000_000, 10)
        window.clear()
        for i in range(1000):
            ns = i * 100_000
            if i >= 500:
                ns += 40_000_000
            window.add(ns, 3)
        with self.assertRaisesRegex(CalibrationError, "timing gap"):
            window.snapshot(150_000_000, 50_000_000, 10)

    def test_sign_mass_and_minimum_delta_validation(self):
        gain, delta = scale_for_known_mass(.5, 31088, 32688, -1)
        self.assertAlmostEqual(delta, 1600)
        self.assertAlmostEqual(gain, .5 / 1600)
        with self.assertRaisesRegex(CalibrationError, "opposite"):
            scale_for_known_mass(.5, 31088, 32688, +1)
        with self.assertRaisesRegex(CalibrationError, "at least"):
            scale_for_known_mass(.5, 32678, 32688, -1)
        with self.assertRaises(CalibrationError):
            scale_for_known_mass(0, 31088, 32688, -1)

    def test_save_and_reload_gain_but_never_old_tare(self):
        with tempfile.TemporaryDirectory() as d:
            path = str(Path(d) / "g10.json")
            save_scale(path, "192.168.127.56", 6, -1,
                       .0003125, .5, 1600)
            payload = json.loads(Path(path).read_text())
            self.assertNotIn("zero_raw", payload)
            self.assertIs(payload["zero_offset_persisted"], False)
            self.assertEqual(
                load_scale(path, "192.168.127.56", 6, -1), .0003125)
            self.assertIsNone(
                load_scale(str(Path(d) / "missing.json"),
                           "192.168.127.56", 6, -1))
            with self.assertRaisesRegex(CalibrationError, "mismatch"):
                load_scale(path, "192.168.127.56", 3, -1)


if __name__ == "__main__":
    unittest.main()
