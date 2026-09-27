"""ROS callback regression tests using narrow mocks, runnable without ROS 2.

These test the *real* node _tick() code path, not a copied baseline formula.
Hardware, real DDS, UDP startup and physical timing are NOT covered.
"""

import importlib
from collections import deque
import queue
import sys
import tempfile
import time
import types
import unittest
from unittest import mock

from bidirectional_motor_test.core import SineDshot, SwitchInterlock
from bidirectional_motor_test.g10_calibration import StableADCWindow
from bidirectional_motor_test.latency import ThrustLatency


def _load_node_without_ros():
    # Keep the mock contained while importing the real production module.
    replacements = {}
    for name in ("rclpy", "rclpy.node", "rclpy.qos",
                 "std_msgs", "std_msgs.msg",
                 "custom_msgs", "custom_msgs.msg",
                 "std_srvs", "std_srvs.srv"):
        replacements[name] = types.ModuleType(name)
    replacements["rclpy"].__path__ = []
    replacements["rclpy.node"].Node = object
    replacements["rclpy.qos"].qos_profile_sensor_data = object()
    replacements["std_msgs"].__path__ = []
    replacements["std_msgs.msg"].Float64 = type("Float64", (), {})
    replacements["std_msgs.msg"].String = type("String", (), {
        "__init__": lambda self, data="": setattr(self, "data", data)})
    replacements["custom_msgs"].__path__ = []
    replacements["custom_msgs.msg"].ReadDJIRC = type("ReadDJIRC", (), {})
    replacements["custom_msgs.msg"].WriteDSHOT = type("WriteDSHOT", (), {})
    replacements["std_srvs"].__path__ = []
    replacements["std_srvs.srv"].Trigger = type("Trigger", (), {})
    with mock.patch.dict(sys.modules, replacements):
        return importlib.import_module("bidirectional_motor_test.motor_test_node")


class NodeTickTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.module = _load_node_without_ros()

    def make_node(self):
        node = object.__new__(self.module.BidirectionalMotorTest)
        node.acquisition_only = False
        node.g10_enabled = True
        node.g10_adc_modulo = True
        node.g10_require_healthy = True
        node.g10_auto_zero = True
        node.g10_zero_count = 100
        node.g10_auto_zero_samples = 100
        node.g10_last_unready = None
        node.force_history = deque()
        node.rc = types.SimpleNamespace(online=1, right_switch=1)
        node.rc_received_ns = time.monotonic_ns()
        node.rc_timeout_ns = 250_000_000
        node.interlock = SwitchInterlock()
        node.interlock.mode(2)
        node.interlock.mode(3)
        node.wave = types.SimpleNamespace(
            running=True, stop=lambda: None, step=lambda now, rpm_ready:
            types.SimpleNamespace(
                dshot=1100, sine=0.2, direction=1, phase_rad=0.2,
                event="first_command"))
        node.latency = ThrustLatency(delta_threshold=50, sign_threshold=50,
                                     confirm_samples=10)
        node.last_force = 0
        node.last_force_ns = time.monotonic_ns() - 1_000_000
        node.force_max_age_ns = 250_000_000
        node.force_topic = ""  # This was the real regression.
        node.force_unit = "raw_count"
        node.last_mode = "SINE"
        node.last_wave_stopped_ns = None
        node.event_id = 0
        node.raw_capture = None
        node._rpm_ready = lambda now: True
        node._g10_status = lambda now: (True, "ready")
        node._log_event = lambda *a, **kw: None
        node.get_logger = lambda: types.SimpleNamespace(
            info=lambda *a: None, warn=lambda *a: None)
        node.published = []
        node._publish = lambda val, *args: (
            node.published.append(val) or time.monotonic_ns())
        return node

    def test_real_tare_then_known_mass_service_and_persistence(self):
        node = self.make_node()
        node.last_mode = "DISARM"
        node.last_command = 0
        node.wave.running = False
        node.g10 = types.SimpleNamespace(error=None)
        node.g10_adc_channel = 6
        node.g10_device_ip = "192.168.127.56"
        node.g10_force_sign = -1
        node.g10_zero_raw = 32680.0
        node.g10_kgf_per_count = 0.0
        node.g10_no_packet_timeout_ns = 150_000_000
        node.g10_stability_range_counts = 10
        node.g10_raw_change_threshold = 50
        node.g10_raw_sign_threshold = 50
        node.g10_calibration_window = StableADCWindow(
            sample_period_ns=100_000, window_sec=.1)
        node.logs = types.SimpleNamespace(force_unit="raw_count")
        node.get_parameter = lambda key: types.SimpleNamespace(value=.5)
        with tempfile.TemporaryDirectory() as d:
            node.g10_calibration_file = d + "/gain.json"

            def fill_window(raw):
                now = time.monotonic_ns()
                for i in range(1200):
                    node.g10_calibration_window.add(
                        now - 120_000_000 + i * 100_000, raw)

            fill_window(32688)
            result = node._on_g10_tare(
                None, types.SimpleNamespace(success=False, message=""))
            self.assertTrue(result.success, result.message)
            self.assertAlmostEqual(node.g10_zero_raw, 32688)
            self.assertIsNone(node.last_force)

            # Must gather NEW, unloaded/loaded data after taring.
            denied = node._on_g10_calibrate(
                None, types.SimpleNamespace(success=False, message=""))
            self.assertFalse(denied.success)
            self.assertIn("need", denied.message)

            fill_window(31088)
            result = node._on_g10_calibrate(
                None, types.SimpleNamespace(success=False, message=""))
            self.assertTrue(result.success, result.message)
            self.assertAlmostEqual(node.g10_kgf_per_count, .5 / 1600)
            self.assertEqual(node.force_unit, "kgf")
            self.assertEqual(node.logs.force_unit, "kgf")
            self.assertAlmostEqual(node.latency.delta_threshold,
                                   50 * .5 / 1600)
            self.assertAlmostEqual(node.latency.sign_threshold,
                                   50 * .5 / 1600)

    def test_tare_refused_when_motor_is_armed_or_active(self):
        node = self.make_node()
        node.g10 = types.SimpleNamespace(error=None)
        node.last_mode = "ARMED"
        node.last_command = 0
        node.wave.running = False
        response = node._on_g10_tare(
            None, types.SimpleNamespace(success=False, message=""))
        self.assertFalse(response.success)
        self.assertIn("DISARM", response.message)
        node.last_mode = "DISARM"
        node.last_command = 1048
        response = node._on_g10_tare(
            None, types.SimpleNamespace(success=False, message=""))
        self.assertFalse(response.success)
        self.assertIn("stop the motor", response.message)

    def test_real_udp_decoder_to_force_handles_signed16_rollover(self):
        node = self.make_node()
        node.g10 = types.SimpleNamespace(packets=queue.Queue())
        node.g10_adc_channel = 6
        node.g10_zero_raw = 32688.0
        node.g10_kgf_per_count = 0.0
        node.g10_force_sign = -1
        node.g10_sample_period_ns = 100_000
        node.g10_arrival_bias_ns = 0
        node.g10_clock = self.module.G10SequenceClock(100_000)
        node.g10_last_sequence = None
        node.g10_sequence_gaps = 0
        node.g10_max_packets_per_poll = 6
        node.g10_channel_log_decimation = 1
        node.g10_log_decimation = 40
        node.g10_packet_count = 0
        node.g10_sample_count = 0
        node.g10_calibration_window = StableADCWindow(
            sample_period_ns=100_000, window_sec=.1)
        node.last_force = None
        node.last_force_ns = None
        node.last_command = 0
        written = []
        node.logs = types.SimpleNamespace(
            write=lambda kind, **row: written.append((kind, row)))
        now = time.monotonic_ns()

        def samples(raw):
            return tuple(
                tuple(raw if channel == 6 else 0 for channel in range(8))
                for _ in range(40))

        def feed(raw, sequence):
            packet = types.SimpleNamespace(
                sequence=sequence, samples=samples(raw),
                sample_timestamps=lambda t, period, bias:
                tuple(t - bias - 39 * period + i * period
                      for i in range(40)))
            node.g10.packets.put((now + sequence * 5_000_000, packet))
            node._drain_g10()
            return node.last_force

        # Physically one direction: raw decreases, positive in this setup.
        self.assertEqual(feed(31088, 1), +1600)
        # Opposite direction: ADC crosses the int16 representation edge.
        # Old code wrongly computed about +65,000 counts here.
        self.assertEqual(feed(-32648, 2), -200)
        adc_rows = [row for kind, row in written if kind == "force"]
        self.assertEqual(adc_rows[0]["raw_force"], 31088)
        self.assertEqual(adc_rows[-1]["raw_force"], -32648)
        self.assertEqual(adc_rows[-1]["forward_positive_force"], -200)

    def test_auto_zero_does_not_average_across_int16_gap(self):
        node = self.make_node()
        node.g10 = types.SimpleNamespace(packets=queue.Queue())
        node.g10_adc_channel = 6
        node.g10_force_sign = -1
        node.g10_auto_zero = True
        node.g10_auto_zero_samples = 40
        node.g10_zero_count = 0
        node.g10_zero_reference = None
        node.g10_zero_sum = 0.0
        node.g10_sample_period_ns = 100_000
        node.g10_arrival_bias_ns = 0
        node.g10_clock = self.module.G10SequenceClock(100_000)
        node.g10_last_sequence = None
        node.g10_sequence_gaps = 0
        node.g10_max_packets_per_poll = 6
        node.g10_channel_log_decimation = 1
        node.g10_packet_count = 0
        node.g10_calibration_window = StableADCWindow(
            sample_period_ns=100_000, window_sec=.1)
        node.logs = types.SimpleNamespace(write=lambda *a, **kw: None)
        raw_values = (32766, 32767, -32768, -32767) * 10
        packet = types.SimpleNamespace(
            sequence=1,
            samples=tuple(
                tuple(raw if ch == 6 else 0 for ch in range(8))
                for raw in raw_values),
            sample_timestamps=lambda recv, period, bias:
            tuple(recv - bias - 39 * period + i * period
                  for i in range(40)))
        node.g10.packets.put((time.monotonic_ns(), packet))
        node._drain_g10()
        self.assertEqual(node.g10_zero_count, 40)
        self.assertAlmostEqual(node.g10_zero_raw, 32767.5)

    def test_udp_event_starts_real_latency_tracker(self):
        node = self.make_node()
        node._tick()
        self.assertEqual(node.event_id, 1)
        self.assertEqual(node.latency.pending["id"], 1)
        self.assertEqual(node.latency.pending["baseline"], 0.0)
        self.assertEqual(node.published, [1100])

    def test_shutdown_sends_zero_before_any_slow_csv_close(self):
        node = self.make_node()
        order = []
        node.g10 = types.SimpleNamespace(stop=lambda: order.append("udp_stop"))
        node.raw_capture = types.SimpleNamespace(
            close=lambda: order.append("raw_close"))
        node.logs = types.SimpleNamespace(close=lambda: order.append("csv_close"))
        node._publish = lambda val, *args: order.append("zero")
        with mock.patch.object(self.module.time, "sleep", lambda _: None), \
                mock.patch.object(self.module.rclpy, "ok",
                                  return_value=True, create=True):
            node.shutdown()
        self.assertEqual(order[:10], ["zero"] * 10)
        self.assertEqual(order[10:], ["udp_stop", "raw_close", "csv_close"])

    def test_shutdown_cleans_up_if_context_already_invalid(self):
        node = self.make_node()
        order = []
        node.g10 = types.SimpleNamespace(stop=lambda: order.append("udp_stop"))
        node.raw_capture = types.SimpleNamespace(
            close=lambda: order.append("raw_close"))
        node.logs = types.SimpleNamespace(close=lambda: order.append("csv_close"))
        node._publish = lambda *a: order.append("publish")
        with mock.patch.object(self.module.rclpy, "ok",
                               return_value=False, create=True):
            node.shutdown()
        self.assertEqual(order, ["udp_stop", "raw_close", "csv_close"])

    def test_shutdown_handles_publish_failure_without_leaking_udp(self):
        node = self.make_node()
        order = []
        node.g10 = types.SimpleNamespace(stop=lambda: order.append("udp_stop"))
        node.raw_capture = types.SimpleNamespace(
            close=lambda: order.append("raw_close"))
        node.logs = types.SimpleNamespace(close=lambda: order.append("csv_close"))

        def fail_publish(*args):
            order.append("publish")
            raise RuntimeError("publisher context invalid")

        node._publish = fail_publish
        with mock.patch.object(self.module.rclpy, "ok",
                               return_value=True, create=True):
            node.shutdown()
        self.assertEqual(
            order, ["publish", "udp_stop", "raw_close", "csv_close"])

    def test_health_disables_output_for_stale_force_or_disk_failure(self):
        node = self.make_node()
        node.g10 = types.SimpleNamespace(
            error=None, packets=types.SimpleNamespace(qsize=lambda: 0))
        node.g10_last_received_ns = time.monotonic_ns()
        node.g10_no_packet_timeout_ns = 150_000_000
        node.g10_max_queue_backlog = 24
        node.force_max_age_ns = 250_000_000
        node.last_force_ns = time.monotonic_ns() - 300_000_000
        ready, reason = node.__class__._g10_status(node, time.monotonic_ns())
        self.assertFalse(ready)
        self.assertEqual(reason, "force_samples_stale")
        node.last_force_ns = time.monotonic_ns()
        node.raw_capture = types.SimpleNamespace(error=IOError("disk full"))
        ready, reason = node.__class__._g10_status(node, time.monotonic_ns())
        self.assertFalse(ready)
        self.assertEqual(reason, "receiver_error")

    def test_stream_fault_revokes_arm_and_commands_zero(self):
        node = self.make_node()
        node._g10_status = lambda now: (False, "stale_packets")
        node._tick()
        self.assertEqual(node.published[-1], 0)
        self.assertIsNone(node.latency.pending)
        self.assertFalse(node.interlock.armed)
        self.assertIn("stale_packets", node.g10_last_unready)


if __name__ == "__main__":
    unittest.main()
