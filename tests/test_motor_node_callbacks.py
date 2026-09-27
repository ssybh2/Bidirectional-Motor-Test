"""ROS callback regression tests using narrow mocks, runnable without ROS 2.

These test the *real* node _tick() code path, not a copied baseline formula.
Hardware, real DDS, UDP startup and physical timing are NOT covered.
"""

import importlib
import sys
import time
import types
import unittest
from unittest import mock

from bidirectional_motor_test.core import SineDshot, SwitchInterlock
from bidirectional_motor_test.latency import ThrustLatency


def _load_node_without_ros():
    # Keep the mock contained while importing the real production module.
    replacements = {}
    for name in ("rclpy", "rclpy.node", "rclpy.qos",
                 "std_msgs", "std_msgs.msg",
                 "custom_msgs", "custom_msgs.msg"):
        replacements[name] = types.ModuleType(name)
    replacements["rclpy"].__path__ = []
    replacements["rclpy.node"].Node = object
    replacements["rclpy.qos"].qos_profile_sensor_data = object()
    replacements["std_msgs"].__path__ = []
    replacements["std_msgs.msg"].Float64 = type("Float64", (), {})
    replacements["custom_msgs"].__path__ = []
    replacements["custom_msgs.msg"].ReadDJIRC = type("ReadDJIRC", (), {})
    replacements["custom_msgs.msg"].WriteDSHOT = type("WriteDSHOT", (), {})
    with mock.patch.dict(sys.modules, replacements):
        return importlib.import_module("bidirectional_motor_test.motor_test_node")


class NodeTickTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.module = _load_node_without_ros()

    def make_node(self):
        node = object.__new__(self.module.BidirectionalMotorTest)
        node.g10_enabled = True
        node.g10_require_healthy = True
        node.g10_auto_zero = True
        node.g10_zero_count = 100
        node.g10_auto_zero_samples = 100
        node.g10_last_unready = None
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
        with mock.patch.object(self.module.time, "sleep", lambda _: None):
            node.shutdown()
        self.assertEqual(order[:10], ["zero"] * 10)
        self.assertEqual(order[10:], ["udp_stop", "raw_close", "csv_close"])

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
