"""Collector cannot control motors, while controller timestamps correlate ADC."""
from collections import deque
import json
import tempfile
from pathlib import Path
import time
import types
import unittest
from unittest import mock

from test_motor_node_callbacks import NodeTickTests
from bidirectional_motor_test.session_logs import SessionLogs
from bidirectional_motor_test.control_recovery import recent_control_metadata


class CaptureSeparationTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        NodeTickTests.setUpClass()
        cls.module = NodeTickTests.module

    def make_capture(self):
        node = object.__new__(self.module.BidirectionalMotorTest)
        node.acquisition_only = True
        node.g10_enabled = True
        node.rows, node.stopped = [], []
        node.logs = types.SimpleNamespace(
            write=lambda record_type, **row: node.rows.append((record_type, row)),
            close=lambda: node.stopped.append("csv"))
        node.force_history = deque(maxlen=30000)
        node.force_max_age_ns = 250_000_000
        node.last_force = 0.0
        node.last_force_ns = None
        node.last_mode = None
        node.last_command = 0
        node.last_remote_command_ns = None
        node.last_remote_command_rx_ns = None
        node.command_signatures = {}
        node.seen_reference_stamps = set()
        node.metadata_source = "none"
        node.metadata_ros_commands = 0
        node.metadata_csv_commands = 0
        node.event_id = 0
        node.force_unit = "raw_count"
        node.raw_capture = None
        node.g10 = types.SimpleNamespace(
            stop=lambda: node.stopped.append("udp"))
        node.wave = types.SimpleNamespace(
            stop=lambda: node.stopped.append("wave"))
        node.interlock = types.SimpleNamespace(
            fault=lambda: node.stopped.append("fault"))
        node.latency = self.module.ThrustLatency(
            delta_threshold=10, sign_threshold=10, confirm_samples=2)
        node.get_logger = lambda: types.SimpleNamespace(
            info=lambda *a: None, warn=lambda *a: None)
        return node

    def test_collector_fault_and_shutdown_never_publish_dshot(self):
        node = self.make_capture()
        node._publish = lambda *a, **kw: self.fail("collector published DSHOT")
        node._on_g10_data_fault("stale_packets")
        with mock.patch.object(self.module.rclpy, "ok",
                               return_value=True, create=True):
            node.shutdown()
        self.assertEqual(node.stopped, ["wave", "udp", "csv"])
        self.assertIn("g10_data_fault", [
            row["kind"] for kind, row in node.rows if kind == "event"])

    def test_delayed_reference_replays_buffered_force_history(self):
        node = self.make_capture()
        t0 = time.monotonic_ns() - 20_000_000
        node.force_history.extend([
            (t0 - 1_000_000, 0.0),
            (t0 + 1_000_000, 12.0),
            (t0 + 2_000_000, 13.0),
        ])
        command = dict(
            type="command", mono_ns=t0, wall_ns=time.time_ns(),
            dshot=1100, mode="SINE", channel=1,
            sine=.5, direction=1, phase=.5)
        node._on_control_meta(types.SimpleNamespace(data=json.dumps(command)))
        reference = dict(
            type="reference", mono_ns=t0, mode="SINE",
            dshot=1100, sine=.5, direction=1, control_event_id=7)
        node._on_control_meta(types.SimpleNamespace(data=json.dumps(reference)))
        values = [row for kind, row in node.rows if kind == "latency"]
        self.assertTrue(any(
            row["metric"] == "force_onset" and row["latency_ms"] == 1.0
            for row in values), values)
        self.assertEqual([
            row["mono_ns"] for kind, row in node.rows if kind == "command"], [t0])

    def test_reference_arriving_after_newer_command_is_not_discarded(self):
        node = self.make_capture()
        t0 = time.monotonic_ns() - 40_000_000
        node.force_history.extend([
            (t0 - 1_000_000, 0.0),
            (t0 + 1_000_000, 12.0),
            (t0 + 2_000_000, 13.0),
        ])
        for stamp, value, direction in (
                (t0, 1100, 1), (t0 + 20_000_000, 0, 0)):
            node._on_control_meta(types.SimpleNamespace(data=json.dumps({
                "type": "command", "mono_ns": stamp,
                "wall_ns": time.time_ns(), "dshot": value,
                "mode": "SINE", "channel": 1,
                "sine": .4 if value else 0,
                "direction": direction, "phase": .4,
            })))
        node._on_control_meta(types.SimpleNamespace(data=json.dumps({
            "type": "reference", "mono_ns": t0,
            "mode": "SINE", "dshot": 1100, "sine": .4,
            "direction": 1, "control_event_id": 7,
        })))
        self.assertEqual(len([
            row for kind, row in node.rows
            if kind == "event" and row["kind"] ==
            "force_response_reference"]), 1)
        self.assertTrue(any(kind == "latency" for kind, _ in node.rows))
        # ROS/CSV duplicates must not create duplicate events or samples.
        node._on_control_meta(types.SimpleNamespace(data=json.dumps({
            "type": "reference", "mono_ns": t0,
            "mode": "SINE", "dshot": 1100, "sine": .4,
            "direction": 1, "control_event_id": 7,
        })))
        self.assertEqual(len([
            row for kind, row in node.rows if kind == "event"
            and row["kind"] == "force_response_reference"]), 1)

    def test_csv_fallback_restores_exact_timestamps_when_ros_is_missing(self):
        with tempfile.TemporaryDirectory() as folder:
            ctl = SessionLogs(folder, "raw_count", prefix_tag="control")
            try:
                node = self.make_capture()
                node.logs.prefix = str(Path(folder) / "g10_running")
                t0 = time.monotonic_ns() - 25_000_000
                for stamp, value, direction in (
                        (t0, 1100, 1), (t0 + 20_000_000, 0, 0)):
                    ctl.write(
                        "command", wall_ns=time.time_ns(), mono_ns=stamp,
                        mode="SINE", channel=1, dshot=value,
                        sine=0.4 if value else 0,
                        logical_direction=direction, phase_rad=0.4,
                        last_force="", force_unit="raw_count")
                ctl.write(
                    "event", wall_ns=time.time_ns(), mono_ns=t0,
                    event_id=11, kind="force_response_reference",
                    mode="SINE", dshot=1100, sine=.4,
                    detail="EXTERNAL_G10: collector computes latency")
                node.force_history.extend([
                    (t0 - 1_000_000, 0.0),
                    (t0 + 1_000_000, 12.0),
                    (t0 + 2_000_000, 13.0),
                ])
                _, messages = recent_control_metadata(
                    folder, time.monotonic_ns())
                self.assertEqual(
                    [m["type"] for m in messages],
                    ["command", "reference", "command"])
                node._recover_control_csv(time.monotonic_ns())
                self.assertEqual(node.metadata_source, "control_csv")
                self.assertEqual(node.metadata_csv_commands, 2)
                self.assertEqual(len([
                    row for kind, row in node.rows
                    if kind == "command"]), 2)
                self.assertEqual(len([
                    row for kind, row in node.rows
                    if kind == "event" and row["kind"] ==
                    "force_response_reference"]), 1)
                node._recover_control_csv(time.monotonic_ns())
                self.assertEqual(node.metadata_csv_commands, 2)
            finally:
                ctl.close()

    def test_disarm_cancels_incomplete_latency_without_motor_publish(self):
        node = self.make_capture()
        t0 = time.monotonic_ns() - 10_000_000
        node.latency.start(1, t0, 1, 0.0)
        node._publish = lambda *args: self.fail("collector emitted DSHOT")
        node._on_control_meta(types.SimpleNamespace(data=json.dumps({
            "type": "command", "mono_ns": t0,
            "wall_ns": time.time_ns(), "dshot": 0, "mode": "DISARM",
            "channel": 1, "sine": 0, "direction": 0, "phase": 0,
        })))
        self.assertIsNone(node.latency.pending)
        self.assertEqual(node.last_mode, "DISARM")

    def test_stale_or_active_command_disables_calibration(self):
        node = self.make_capture()
        node.g10_enabled = True
        node.rc_timeout_ns = 250_000_000
        node.last_mode = "DISARM"
        with self.assertRaisesRegex(
                self.module.CalibrationError, "heartbeat"):
            node._calibration_guard()
        node.last_remote_command_rx_ns = time.monotonic_ns()
        node.last_mode = "SINE"
        with self.assertRaisesRegex(
                self.module.CalibrationError, "DISARM"):
            node._calibration_guard()


if __name__ == "__main__":
    unittest.main()
