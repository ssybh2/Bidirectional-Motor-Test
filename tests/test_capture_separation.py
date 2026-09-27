"""Collector cannot control motors, while controller timestamps correlate ADC."""
from collections import deque
import json
import time
import types
import unittest
from unittest import mock

from test_motor_node_callbacks import NodeTickTests


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
