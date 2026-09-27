"""Headless dashboard data tests: shared ROS CSV, no UDP socket, no Tk."""
import os
from pathlib import Path
import tempfile
import unittest

from bidirectional_motor_test.gui_data import (
    last_row, latest_session, parse_channels, parse_force,
    plot_limits, recent_rows, stream_fresh,
)


class DesktopDataTests(unittest.TestCase):
    def test_follows_newest_session_and_detects_stale_data(self):
        with tempfile.TemporaryDirectory() as folder:
            d = Path(folder)
            one = d / "test_001_force.csv"
            two = d / "test_002_force.csv"
            self.assertIsNone(latest_session(d))
            one.write_text(
                "mono_ns,raw_force,forward_positive_force,force_unit,last_dshot\n"
                "10,32688,-1,raw_count,0\n")
            two.write_text(
                "mono_ns,raw_force,forward_positive_force,force_unit,last_dshot\n"
                "11,32688,0,raw_count,0\n")
            os.utime(one, ns=(10**9, 10**9))
            os.utime(two, ns=(2 * 10**9, 2 * 10**9))
            self.assertEqual(latest_session(d), str(two)[:-10])
            self.assertFalse(stream_fresh(str(two)[:-10]))
            self.assertTrue(stream_fresh(str(two)[:-10], now=2.0))

    def test_tail_rejects_partial_rows_and_missing_files(self):
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / "force.csv"
            path.write_bytes(
                b"mono_ns,raw_force,forward_positive_force,force_unit,last_dshot\n"
                b"1,32688,-1.165,raw_count,0\n"
                b"2,31000,1686.835,raw_count,1250\n"
                b"3,3100")
            rows = recent_rows(path)
            self.assertEqual(len(rows), 2)
            self.assertEqual(
                parse_force(rows[-1]),
                (2, 1686.835, "raw_count", 31000, 1250))
            self.assertEqual(recent_rows(Path(folder) / "missing.csv"), [])
            with self.assertRaises(ValueError):
                recent_rows(path, max_bytes=12)
            path.write_text(
                "mono_ns,raw_force,forward_positive_force,force_unit,last_dshot\n"
                "3,32688,-0.1,kgf,0\n")
            self.assertEqual(parse_force(recent_rows(path)[-1])[2], "kgf")

    def test_all_eight_unmapped_adc_channels(self):
        with tempfile.TemporaryDirectory() as folder:
            prefix = str(Path(folder) / "test_x")
            channels = Path(prefix + "_g10_channels.csv")
            channels.write_text(
                "host_write_wall_ns,packet_recv_mono_ns,"
                "estimated_sample_mono_ns,packet_sequence,"
                + ",".join("adc_%d" % i for i in range(8))
                + "\n1,10,5,76,-1,-2,-3,31424,-5,-6,32688,-8\n")
            self.assertEqual(
                parse_channels(last_row(prefix, "g10_channels")),
                [-1, -2, -3, 31424, -5, -6, 32688, -8])
            self.assertIsNone(parse_channels({"adc_0": "1"}))

    def test_force_unit_validity_and_plot_zero(self):
        self.assertIsNone(parse_force({
            "mono_ns": "1", "raw_force": "1",
            "forward_positive_force": "1",
            "force_unit": "bogus", "last_dshot": "0"}))
        self.assertEqual(plot_limits([]), (-1, 1))
        low, high = plot_limits([-2.0, 5.0])
        self.assertLess(low, -2)
        self.assertGreater(high, 5)
        self.assertEqual(low, -high)
        # Positive-only readings still plot against a centered zero line.
        low, high = plot_limits([0.1, 0.2])
        self.assertEqual(low, -high)
        low, high = plot_limits([5.0, 5.0])
        self.assertLess(low, 0)
        self.assertGreater(high, 5)


if __name__ == "__main__":
    unittest.main()
