"""Tests the actual ZIP exporter using real SessionLogs CSV schemas."""

import csv
import json
import tempfile
from pathlib import Path
import unittest
import zipfile

from bidirectional_motor_test.session_export import (
    ExportError, DESCRIPTION, export_recording,
)
from bidirectional_motor_test.session_logs import SessionLogs


def read_zip_csv(archive, name):
    with archive.open(name) as raw:
        return list(csv.DictReader(
            line.decode("utf-8") for line in raw.readlines()))


class RecordingExportTests(unittest.TestCase):
    def setUp(self):
        self.root = tempfile.TemporaryDirectory()
        self.addCleanup(self.root.cleanup)
        self.path = Path(self.root.name)
        self.logs = SessionLogs(self.path / "continuous", "raw_count")
        self.addCleanup(self.logs.close)
        self.output = self.path / "chosen by user"
        self.output.mkdir()

    def write_command(self, t, dshot, direction=0, mode="SINE"):
        self.logs.write(
            "command", wall_ns=1000 + t, mono_ns=t, mode=mode,
            channel=1, dshot=dshot, sine=0.2 if dshot else 0,
            logical_direction=direction, phase_rad=0.2,
            last_force=0, force_unit="raw_count")

    def write_force(self, t, value):
        self.logs.write(
            "force", wall_ns=2000 + t, mono_ns=t,
            raw_force=32688 - value, forward_positive_force=value,
            force_unit="raw_count", last_dshot=1100)

    def write_event(self, t, event_id=1, detail="force baseline=0 raw_count"):
        self.logs.write(
            "event", wall_ns=3000 + t, mono_ns=t,
            event_id=event_id, kind="force_response_reference", mode="SINE",
            dshot=1100, sine=.2, detail=detail)

    def write_latency(self, event_id, metric, cmd, observed, delay,
                      status="detected"):
        self.logs.write(
            "latency", event_id=event_id, metric=metric, status=status,
            command_mono_ns=cmd,
            observed_mono_ns=observed,
            latency_ms=delay, baseline_force=0, observed_force=50,
            force_unit="raw_count")

    def test_zip_contains_bounded_raw_samples_and_real_detector(self):
        self.write_command(50, 1100, 1)
        self.write_command(100, 0)
        self.write_command(200, 1100, 1)
        self.write_command(400, 48, -1)
        self.write_command(510, 0)
        self.write_force(80, 40)
        self.write_force(120, 0)
        self.write_force(230, 50)
        self.write_force(300, 70)
        self.write_force(510, 90)
        self.write_event(200)
        self.write_event(400, event_id=2,
                         detail="NO_RECENT_FORCE: latency unavailable")
        self.write_latency(1, "force_onset", 200, 230, 30.0)
        self.write_latency(1, "target_sign", 200, 300, 100.0)
        raw_path = Path(self.logs.prefix + "_raw_event_0001.csv")
        raw_path.write_text(
            "event_id,event_command_ns,estimated_sample_mono_ns,"
            "packet_recv_mono_ns,packet_sequence,adc_raw,force,force_unit\n"
            "1,200,190,191,10,32688,0,raw_count\n"
            "1,200,310,311,11,32630,58,raw_count\n"
            "1,200,520,521,12,32610,78,raw_count\n"
        )
        self.logs.write(
            "g10_quality", mono_ns=250, wall_ns=1250,
            stream_ready=1, reason="ready",
            last_receive_age_ms=3, decoded_packets=80,
            invalid_packets=0, queue_dropped=0,
            sequence_gap_events=0, timestamp_regressions=0,
            queue_backlog=0, zero_samples=10000,
            raw_windows_dropped=0)
        self.logs.write(
            "g10_channels", host_write_wall_ns=2400,
            packet_recv_mono_ns=240, estimated_sample_mono_ns=239,
            packet_sequence=10,
            **{"adc_%d" % i: 100 + i for i in range(8)})
        result = export_recording(
            self.logs.prefix, self.output, 100, 500,
            started_wall_ns=1_790_495_000_000_000_000,
            stopped_wall_ns=1_790_495_001_000_000_000)
        path = Path(result["path"])
        self.assertEqual(path.parent, self.output)
        self.assertTrue(path.exists())
        self.assertEqual(result["counts"]["command"], 3)
        self.assertEqual(result["counts"]["force"], 3)
        self.assertEqual(result["events"], 2)
        self.assertEqual(result["detected"], 1)
        self.assertAlmostEqual(result["onset_mean_ms"], 30)
        with zipfile.ZipFile(path) as arc:
            files = set(arc.namelist())
            self.assertIn("timeline.csv", files)
            self.assertIn("event_summary.csv", files)
            self.assertIn("metadata.json", files)
            self.assertIn("raw_events/raw_event_0001.csv", files)
            self.assertIn("command.csv", files)
            self.assertIn("force.csv", files)
            self.assertEqual(
                [int(r["mono_ns"]) for r in read_zip_csv(arc, "force.csv")],
                [120, 230, 300])
            self.assertEqual(
                [int(r["mono_ns"]) for r in read_zip_csv(arc, "command.csv")],
                [100, 200, 400])
            merged = read_zip_csv(arc, "timeline.csv")
            self.assertEqual(
                [int(r["mono_ns"]) for r in merged],
                [100, 120, 200, 230, 300, 400])
            self.assertEqual(
                [r["row_type"] for r in merged],
                ["command", "force", "command", "force", "force", "command"])
            summaries = read_zip_csv(arc, "event_summary.csv")
            self.assertEqual(summaries[0]["onset_status"], "detected")
            self.assertEqual(summaries[0]["onset_delay_ms"], "30.0")
            self.assertEqual(summaries[0]["target_sign_delay_ms"], "100.0")
            self.assertEqual(summaries[1]["onset_status"],
                             "no_recent_force")
            self.assertEqual(summaries[1]["onset_delay_ms"], "")
            metadata = json.loads(arc.read("metadata.json"))
            self.assertIn("NOT a device hardware timestamp",
                          metadata["G10_sample_timestamp"])
            self.assertIn("estimated", DESCRIPTION)
            self.assertEqual(metadata["native_rate_event_files"], 1)
            self.assertEqual(
                metadata["native_rate_event_missing_or_not_finished"], [2])
            raw_rows = read_zip_csv(
                arc, "raw_events/raw_event_0001.csv")
            self.assertEqual(
                [int(r["estimated_sample_mono_ns"]) for r in raw_rows],
                [190, 310])

    def test_late_response_and_absent_response_never_invent_zero_ms(self):
        self.write_command(200, 1100, 1)
        self.write_event(200)
        self.write_force(230, 4)
        self.write_latency(1, "force_onset", 200, 800, 600)
        self.write_latency(1, "target_sign", 200, "",
                           "", status="timeout")
        result = export_recording(
            self.logs.prefix, self.output, 100, 500,
            started_wall_ns=1_790_495_000_000_000_000)
        with zipfile.ZipFile(result["path"]) as arc:
            detected = read_zip_csv(arc, "latency.csv")
            self.assertEqual(len(detected), 1)
            self.assertEqual(detected[0]["status"], "timeout")
            summary = read_zip_csv(arc, "event_summary.csv")[0]
            self.assertEqual(summary["onset_status"], "unresolved_at_stop")
            self.assertEqual(summary["onset_delay_ms"], "")
            self.assertEqual(summary["target_sign_status"], "timeout")
        self.assertIsNone(result["onset_mean_ms"])

    def test_controller_csv_recovers_commands_and_marks_missing_latency(self):
        # Reproduce an actual G10-only ZIP: force samples exist, but ROS
        # command/event/latency streams on the collector are all header-only.
        controller = SessionLogs(
            self.path / "continuous", "raw_count", prefix_tag="control")
        self.addCleanup(controller.close)
        for stamp, value, direction in (
                (120, 1100, 1), (140, 0, 0), (200, 50, -1)):
            controller.write(
                "command", wall_ns=3000 + stamp, mono_ns=stamp,
                mode="SINE", channel=1, dshot=value,
                sine=.5 if value else 0,
                logical_direction=direction, phase_rad=.5,
                last_force="", force_unit="raw_count")
        controller.write(
            "event", wall_ns=3200, mono_ns=200, event_id=3,
            kind="force_response_reference", mode="SINE",
            dshot=50, sine=-.5,
            detail="EXTERNAL_G10: collector computes latency separately")
        self.write_force(125, 4)
        self.write_force(225, -9)
        result = export_recording(
            self.logs.prefix, self.output, 100, 240,
            started_wall_ns=1_790_495_000_000_000_000)
        self.assertEqual(result["counts"]["command"], 3)
        self.assertEqual(result["events"], 1)
        self.assertEqual(result["counts"]["latency"], 0)
        self.assertEqual(result["detected"], 0)
        with zipfile.ZipFile(result["path"]) as arc:
            commands = read_zip_csv(arc, "command.csv")
            self.assertEqual(
                [int(x["mono_ns"]) for x in commands], [120, 140, 200])
            self.assertEqual(
                [int(x["dshot"]) for x in commands], [1100, 0, 50])
            self.assertEqual(
                [x["row_type"] for x in read_zip_csv(arc, "timeline.csv")],
                ["command", "force", "command", "command", "force"])
            summary = read_zip_csv(arc, "event_summary.csv")
            self.assertEqual(len(summary), 1)
            self.assertEqual(summary[0]["onset_status"],
                             "metadata_unavailable")
            self.assertEqual(summary[0]["onset_delay_ms"], "")
            self.assertEqual(summary[0]["command_mono_ns"], "200")
            meta = json.loads(arc.read("metadata.json"))
            self.assertEqual(meta["command_source"],
                             "authoritative_control_csv")
            self.assertEqual(meta["recovered_control_references"], 1)
            self.assertTrue(meta["data_integrity_warnings"])
            self.assertEqual(meta["latency_status"], "not_measured")

    def test_force_only_archive_explicitly_labels_missing_commands(self):
        self.write_force(200, 12)
        result = export_recording(
            self.logs.prefix, self.output, 100, 300,
            started_wall_ns=1_790_495_000_000_000_000)
        self.assertEqual(result["counts"]["command"], 0)
        self.assertEqual(result["metadata"]["command_source"], "none")
        self.assertTrue(any(
            "Force-only" in warning for warning in result["warnings"]))

    def test_two_overlapping_motor_controllers_are_ambiguous(self):
        for i in range(2):
            ctl = SessionLogs(
                self.path / "continuous", "raw_count",
                prefix_tag="control")
            self.addCleanup(ctl.close)
            ctl.write(
                "command", wall_ns=2000, mono_ns=200,
                mode="SINE", channel=1, dshot=1100,
                sine=0.5, logical_direction=1, phase_rad=0.5,
                last_force="", force_unit="raw_count")
        self.write_force(200, 5)
        with self.assertRaisesRegex(ExportError, "Multiple Motor Test"):
            export_recording(
                self.logs.prefix, self.output, 100, 300,
                started_wall_ns=1_790_495_000_000_000_000)

    def test_reboot_with_repeated_monotonic_uptime_is_not_a_match(self):
        # Two boots can have identical "10s after boot" mono_ns but different
        # absolute wall_ns. Never attach a previous boot's DSHOT to new G10.
        mono = 10_000_000_000
        wall = 1_790_500_000_000_000_000
        original = SessionLogs(
            self.path / "continuous", "raw_count", prefix_tag="control")
        self.addCleanup(original.close)
        original.write(
            "command", wall_ns=wall - 86_400_000_000_000,
            mono_ns=mono + 150_000_000, mode="SINE", channel=1,
            dshot=1250, sine=0.2, logical_direction=1,
            phase_rad=0.2, last_force="", force_unit="raw_count")
        self.write_force(mono + 150_000_000, 12)
        result = export_recording(
            self.logs.prefix, self.output,
            mono + 100_000_000, mono + 200_000_000,
            started_wall_ns=wall)
        self.assertEqual(result["counts"]["command"], 0)
        self.assertTrue(any(
            "previous boot" in warning for warning in result["warnings"]))
        with zipfile.ZipFile(result["path"]) as arc:
            meta = json.loads(arc.read("metadata.json"))
        self.assertEqual(meta["command_source"], "none")
        # A controller truly recorded in this boot should be recovered.
        current = SessionLogs(
            self.path / "continuous", "raw_count", prefix_tag="control")
        self.addCleanup(current.close)
        current.write(
            "command", wall_ns=wall + 150_000_000,
            mono_ns=mono + 150_000_000, mode="SINE", channel=1,
            dshot=1250, sine=0.2, logical_direction=1,
            phase_rad=0.2, last_force="", force_unit="raw_count")
        self.assertEqual(
            __import__("bidirectional_motor_test.session_export",
                       fromlist=["_select_controller"])._select_controller(
                self.logs.prefix, mono + 100_000_000,
                mono + 200_000_000, wall)[0],
            current.prefix)

    def test_timebase_regressions_reported_without_inventing_samples(self):
        self.write_command(200, 1100, 1)
        self.write_force(201, 9)
        for stamp, regressions in ((150, 123), (250, 140)):
            self.logs.write(
                "g10_quality", mono_ns=stamp, wall_ns=3000 + stamp,
                stream_ready=1, reason="ready",
                last_receive_age_ms=2, decoded_packets=1,
                invalid_packets=0, queue_dropped=0,
                sequence_gap_events=0,
                timestamp_regressions=regressions,
                queue_backlog=0, zero_samples=10000,
                raw_windows_dropped=0)
        result = export_recording(
            self.logs.prefix, self.output, 100, 300,
            started_wall_ns=1_790_495_000_000_000_000)
        self.assertEqual(result["metadata"]["g10_timebase_regressions"], 17)
        self.assertTrue(any("regressed" in w for w in result["warnings"]))
        with zipfile.ZipFile(result["path"]) as archive:
            meta = json.loads(archive.read("metadata.json"))
            self.assertEqual(meta["g10_timebase_regressions"], 17)

    def test_no_force_or_missing_folder_never_write_success_archive(self):
        self.write_command(200, 1100, 1)
        with self.assertRaisesRegex(ExportError, "No force samples"):
            export_recording(
                self.logs.prefix, self.output, 100, 500,
                started_wall_ns=1_790_495_000_000_000_000)
        self.assertFalse(list(self.output.iterdir()))
        with self.assertRaisesRegex(ExportError, "does not exist"):
            export_recording(
                self.logs.prefix, self.path / "missing", 100, 500,
                started_wall_ns=1_790_495_000_000_000_000)


if __name__ == "__main__":
    unittest.main()
