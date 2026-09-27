"""Headless dashboard data tests: shared ROS CSV, no UDP socket, no Tk."""
import os
from pathlib import Path
import tempfile
import unittest

from bidirectional_motor_test.gui_data import (
    acquisition_fresh, collector_command_fresh, last_row,
    latest_control_command, latest_session, parse_channels, parse_force,
    plot_limits, recent_rows, stream_fresh,
)


class DesktopDataTests(unittest.TestCase):
    def test_follows_newest_session_and_detects_stale_data(self):
        with tempfile.TemporaryDirectory() as folder:
            d = Path(folder)
            one = d / "g10_001_force.csv"
            two = d / "g10_002_force.csv"
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

    def test_command_heartbeat_attaches_even_when_g10_is_offline(self):
        with tempfile.TemporaryDirectory() as folder:
            prefix = str(Path(folder) / "g10_live")
            Path(prefix + "_force.csv").write_text(
                "mono_ns,raw_force,forward_positive_force,force_unit,last_dshot\n")
            quality = Path(prefix + "_g10_quality.csv")
            quality.write_text("mono_ns,stream_ready\n100,0\n")
            os.utime(quality, ns=(2 * 10**9, 2 * 10**9))
            self.assertTrue(acquisition_fresh(prefix, now=2.0))
            control = str(Path(folder) / "control_only")
            Path(control + "_command.csv").write_text("mono_ns\n10\n")
            self.assertFalse(acquisition_fresh(control, now=2.0))
            self.assertFalse(stream_fresh(prefix, now=2.0))
            self.assertFalse(acquisition_fresh(prefix, now=10.0))

    def test_live_dshot_comes_from_controller_not_g10_force_copy(self):
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            # G10's last_dshot remains zero without controller metadata.
            g10 = root / "g10_001_force.csv"
            g10.write_text(
                "mono_ns,raw_force,forward_positive_force,force_unit,last_dshot\n"
                "1,32688,0,raw_count,0\n")
            control = root / "control_002_command.csv"
            control.write_text("mono_ns,dshot,mode\n100,1250,SINE\n")
            os.utime(control, ns=(2 * 10**9, 2 * 10**9))
            self.assertEqual(
                latest_control_command(root, now=2.0)["dshot"], 1250)
            self.assertEqual(
                latest_control_command(root, now=2.0)["mode"], "SINE")
            self.assertFalse(
                collector_command_fresh(str(root / "g10_001"), now=2.0))
            self.assertIsNone(latest_control_command(root, now=6.0))
            # A malformed or stale command must not be displayed as zero.
            control.write_text("mono_ns,dshot,mode\n101,bogus,SINE\n")
            os.utime(control, ns=(2 * 10**9, 2 * 10**9))
            self.assertIsNone(latest_control_command(root, now=2.0))

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
