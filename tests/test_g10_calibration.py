"""Calibration regression tests: stable windows, known-mass gain, persistence."""

import json
import tempfile
import unittest
from pathlib import Path

from bidirectional_motor_test.g10_calibration import (
    CalibrationError, StableADCWindow, adc_delta, load_scale, save_scale,
    scale_for_known_mass, signed_adc16_delta,
)


class CalibrationTests(unittest.TestCase):
    def test_signed16_wrap_does_not_create_fake_65k_thrust(self):
        # Actual signed-16 wire values close to the observed G10 zero.
        self.assertEqual(signed_adc16_delta(31088, 32688), -1600)
        self.assertEqual(signed_adc16_delta(-32648, 32688), +200)
        # With the user's direction convention, push and pull have
        # opposite signs. No sign is invented from DSHOT commands.
        self.assertEqual(-adc_delta(31088, 32688), +1600)
        self.assertEqual(-adc_delta(-32648, 32688), -200)
        self.assertEqual(
            adc_delta(-32648, 32688, modulo_signed16=False), -65336)

    def test_tare_window_can_cross_signed16_boundary(self):
        window = StableADCWindow(
            sample_period_ns=100_000, window_sec=0.1, decimation=10)
        now = 1_000_000_000
        raw = [32766, 32767, -32768, -32767]
        for i in range(1200):
            window.add(now - 120_000_000 + i * 100_000,
                       raw[(i // 10) % 4])
        zero, spread, _ = window.snapshot(
            now, 20_000_000, 10, modulo_signed16=True)
        self.assertAlmostEqual(zero, 32767.5)
        self.assertEqual(spread, 3)
        with self.assertRaisesRegex(CalibrationError, "not stable"):
            window.snapshot(now, 20_000_000, 10,
                            modulo_signed16=False)

    def test_reverse_direction_known_load_calibration(self):
        # Change across signed-int16 boundary is only +200 counts.
        scale, delta = scale_for_known_mass(
            0.5, -32648, 32688, +1,
            modulo_signed16=True)
        self.assertEqual(delta, 200)
        self.assertAlmostEqual(scale, 0.5 / 200)
        with self.assertRaises(CalibrationError):
            signed_adc16_delta(float("nan"), 32688)

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
            self.assertEqual(payload["version"], 2)
            self.assertEqual(payload["adc_delta_mode"], "signed16_modulo")
            self.assertEqual(
                load_scale(path, "192.168.127.56", 6, -1), .0003125)
            self.assertIsNone(
                load_scale(str(Path(d) / "missing.json"),
                           "192.168.127.56", 6, -1))
            with self.assertRaisesRegex(CalibrationError, "mismatch"):
                load_scale(path, "192.168.127.56", 3, -1)
            with self.assertRaisesRegex(CalibrationError, "mismatch"):
                load_scale(path, "192.168.127.56", 6, -1,
                           modulo_signed16=False)
            # Legacy v1 files must not be blindly reused under the new
            # modulo-difference model.
            payload["version"] = 1
            payload.pop("adc_delta_mode")
            Path(path).write_text(json.dumps(payload))
            with self.assertRaisesRegex(CalibrationError, "mismatch"):
                load_scale(path, "192.168.127.56", 6, -1)


if __name__ == "__main__":
    unittest.main()
