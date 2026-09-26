#!/usr/bin/env python3
"""DJIRC -> bounded sinusoidal bidirectional DSHOT, with optional thrust logging."""

from __future__ import annotations

import csv
from datetime import datetime, timezone
import math
import os
from pathlib import Path
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Float64
from custom_msgs.msg import ReadDJIRC, WriteDSHOT

from .core import SineDshot, SwitchInterlock
from .latency import ThrustLatency


class SessionLogs:
    def __init__(self, directory, force_unit):
        path = Path(os.path.expanduser(directory))
        path.mkdir(parents=True, exist_ok=True)
        stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S_%fZ")
        prefix = path / ("test_" + stamp + "_" + str(os.getpid()))
        self.force_unit = force_unit
        self.handles = []
        self.writers = {}
        schemas = {
            "command": ("wall_ns", "mono_ns", "mode", "channel", "dshot",
                        "sine", "logical_direction", "phase_rad",
                        "last_force", "force_unit"),
            "force": ("wall_ns", "mono_ns", "raw_force", "forward_positive_force",
                      "force_unit", "last_dshot"),
            "event": ("wall_ns", "mono_ns", "event_id", "kind", "mode",
                      "dshot", "sine", "detail"),
            "latency": ("event_id", "metric", "status", "command_mono_ns",
                        "observed_mono_ns", "latency_ms", "baseline_force",
                        "observed_force", "force_unit"),
        }
        for kind, columns in schemas.items():
            handle = open(str(prefix) + "_" + kind + ".csv",
                          "w", newline="", encoding="utf-8")
            self.handles.append(handle)
            writer = csv.DictWriter(handle, fieldnames=columns)
            writer.writeheader()
            self.writers[kind] = writer
        self.prefix = str(prefix)

    def write(self, kind, **row):
        self.writers[kind].writerow(row)
        # We prefer durable, immediately inspectable measurements over disk speed
        # at the default 50 Hz command rate.
        self.handles[("command", "force", "event", "latency").index(kind)].flush()

    def close(self):
        for handle in self.handles:
            handle.close()


class BidirectionalMotorTest(Node):
    def __init__(self):
        super().__init__("bidirectional_motor_test")
        defaults = {
            "input_topic": "/ecat/sn2555957/app1/read",
            "output_topic": "/ecat/sn2555957/app2/write",
            "motor_channel": 1,
            "sine_frequency_hz": 0.05,
            "sine_deadband": 0.08,
            "positive_peak_dshot": 1250,
            "negative_peak_dshot": 250,
            "invert_direction": False,
            "reversal_pause_sec": 2.0,
            "run_reentry_pause_sec": 2.0,
            "publish_rate_hz": 50.0,
            "rc_timeout_sec": 0.25,
            "rpm_topic": "",
            "require_rpm_for_reversal": False,
            "rpm_stop_threshold": 100.0,
            "rpm_stable_sec": 0.3,
            "rpm_timeout_sec": 0.25,
            "force_topic": "",
            "force_forward_sign": 1,
            "force_unit": "kgf",
            "force_sample_max_age_sec": 0.25,
            "force_change_threshold": 0.03,
            "force_sign_threshold": 0.03,
            "force_confirm_samples": 3,
            "force_latency_timeout_sec": 5.0,
            "log_directory": "~/bidirectional/measurements",
        }
        for key, value in defaults.items():
            self.declare_parameter(key, value)
        p = {key: self.get_parameter(key).value for key in defaults}

        self.channel = int(p["motor_channel"])
        self.rc_timeout_ns = int(float(p["rc_timeout_sec"]) * 1e9)
        self.force_max_age_ns = int(float(p["force_sample_max_age_sec"]) * 1e9)
        self.rpm_timeout_ns = int(float(p["rpm_timeout_sec"]) * 1e9)
        self.rpm_stable_ns = int(float(p["rpm_stable_sec"]) * 1e9)
        self.rpm_stop_threshold = float(p["rpm_stop_threshold"])
        self.run_reentry_pause_ns = round(float(
            p["run_reentry_pause_sec"]) * 1e9)
        self.force_forward_sign = int(p["force_forward_sign"])
        self.force_unit = str(p["force_unit"])
        self.force_topic = str(p["force_topic"])
        self.rpm_topic = str(p["rpm_topic"])
        self.require_rpm = bool(p["require_rpm_for_reversal"])
        rate = float(p["publish_rate_hz"])
        if self.channel not in (1, 2, 3, 4):
            raise ValueError("motor_channel must be in 1..4")
        if rate <= 0 or rate > 1000:
            raise ValueError("publish_rate_hz must be in (0, 1000]")
        if self.rc_timeout_ns <= 0 or self.force_max_age_ns <= 0:
            raise ValueError("RC and force timeouts must be positive")
        if self.rpm_timeout_ns <= 0 or self.rpm_stable_ns < 0:
            raise ValueError("RPM timeout must be positive; stable time >= 0")
        if self.rpm_stop_threshold < 0:
            raise ValueError("rpm_stop_threshold must be >= 0")
        if self.run_reentry_pause_ns < 0:
            raise ValueError("run_reentry_pause_sec must be >= 0")
        if self.force_forward_sign not in (-1, 1):
            raise ValueError("force_forward_sign must be +1 or -1")
        if self.require_rpm and not self.rpm_topic:
            raise ValueError("require_rpm_for_reversal needs a nonempty rpm_topic")

        self.wave = SineDshot(
            frequency_hz=float(p["sine_frequency_hz"]),
            deadband=float(p["sine_deadband"]),
            positive_peak=int(p["positive_peak_dshot"]),
            negative_peak=int(p["negative_peak_dshot"]),
            reversal_pause_sec=float(p["reversal_pause_sec"]),
            invert_direction=bool(p["invert_direction"]),
        )
        self.latency = ThrustLatency(
            delta_threshold=float(p["force_change_threshold"]),
            sign_threshold=float(p["force_sign_threshold"]),
            confirm_samples=int(p["force_confirm_samples"]),
            timeout_sec=float(p["force_latency_timeout_sec"]),
        )
        self.interlock = SwitchInterlock()
        self.logs = SessionLogs(str(p["log_directory"]), self.force_unit)

        self.rc = None
        self.rc_received_ns = None
        self.rpm = None
        self.rpm_received_ns = None
        self.rpm_below_since_ns = None
        self.last_force = None
        self.last_force_ns = None
        self.last_command = 0
        self.last_mode = None
        self.last_wave_stopped_ns = None
        self.event_id = 0

        self.publisher = self.create_publisher(
            WriteDSHOT, str(p["output_topic"]), qos_profile_sensor_data)
        self.rc_sub = self.create_subscription(
            ReadDJIRC, str(p["input_topic"]), self._on_rc,
            qos_profile_sensor_data)
        self.force_sub = None
        if self.force_topic:
            self.force_sub = self.create_subscription(
                Float64, self.force_topic, self._on_force,
                qos_profile_sensor_data)
        self.rpm_sub = None
        if self.rpm_topic:
            self.rpm_sub = self.create_subscription(
                Float64, self.rpm_topic, self._on_rpm,
                qos_profile_sensor_data)

        self.timer = self.create_timer(1.0 / rate, self._tick)
        self._publish(0, "STARTUP", 0.0, 0, 0.0)
        self.get_logger().info(
            "DJIRC=%s, DSHOT=%s, force=%s, rpm=%s, CSV=%s" %
            (p["input_topic"], p["output_topic"],
             self.force_topic or "(not connected)",
             self.rpm_topic or "(not connected)", self.logs.prefix))
        self.get_logger().warn(
            "3D ESC mode required. Switch 2=DISARM, 3=ARMED at zero output, "
            "1=SINE. No rotor-stop guarantee without RPM feedback. "
            "Never treat ROS alone as an emergency stop.")

    def _log_event(self, kind, mode="", dshot=0, sine=0.0, detail="",
                   event_id=0, now_ns=None):
        self.logs.write(
            "event", wall_ns=time.time_ns(),
            mono_ns=time.monotonic_ns() if now_ns is None else now_ns,
            event_id=event_id, kind=kind, mode=mode,
            dshot=dshot, sine=round(sine, 7), detail=detail)

    def _on_rc(self, msg):
        self.rc = msg
        self.rc_received_ns = time.monotonic_ns()

    def _on_rpm(self, msg):
        now = time.monotonic_ns()
        rpm = float(msg.data)
        if not math.isfinite(rpm):
            self.rpm_below_since_ns = None
            return
        self.rpm = rpm
        self.rpm_received_ns = now
        if abs(rpm) <= self.rpm_stop_threshold:
            if self.rpm_below_since_ns is None:
                self.rpm_below_since_ns = now
        else:
            self.rpm_below_since_ns = None

    def _rpm_ready(self, now_ns):
        if not self.require_rpm:
            return True
        return (self.rpm_received_ns is not None and
                now_ns - self.rpm_received_ns <= self.rpm_timeout_ns and
                self.rpm_below_since_ns is not None and
                now_ns - self.rpm_below_since_ns >= self.rpm_stable_ns)

    def _on_force(self, msg):
        force = float(msg.data)
        if not math.isfinite(force):
            return
        now = time.monotonic_ns()
        signed_force = force * self.force_forward_sign
        self.last_force = signed_force
        self.last_force_ns = now
        self.logs.write(
            "force", wall_ns=time.time_ns(), mono_ns=now,
            raw_force=force, forward_positive_force=signed_force,
            force_unit=self.force_unit, last_dshot=self.last_command)
        for result in self.latency.observe(now, signed_force):
            self._log_result(result)

    def _log_result(self, result):
        self.logs.write(
            "latency", event_id=result.event_id, metric=result.metric,
            status=result.status, command_mono_ns=result.command_ns,
            observed_mono_ns=(result.observed_ns
                              if result.observed_ns is not None else ""),
            latency_ms=(round(result.latency_ms, 3)
                        if result.latency_ms is not None else ""),
            baseline_force=result.baseline,
            observed_force=(result.observed_force
                            if result.observed_force is not None else ""),
            force_unit=self.force_unit)
        if result.latency_ms is not None:
            self.get_logger().info(
                "event %d %s: %.3f ms (ROS publish -> force receipt, %s)" %
                (result.event_id, result.metric, result.latency_ms,
                 self.force_unit))

    def _publish(self, value, mode, sine, direction, phase):
        """Stamp immediately before publishing; other channels are always zero."""
        value = int(value)
        msg = WriteDSHOT()
        msg.channel1 = msg.channel2 = msg.channel3 = msg.channel4 = 0
        setattr(msg, "channel%d" % self.channel, value)
        send_ns = time.monotonic_ns()
        self.publisher.publish(msg)
        self.last_command = value
        self.logs.write(
            "command", wall_ns=time.time_ns(), mono_ns=send_ns,
            mode=mode, channel=self.channel, dshot=value,
            sine=round(sine, 7), logical_direction=direction,
            phase_rad=round(phase, 7),
            last_force=self.last_force if self.last_force is not None else "",
            force_unit=self.force_unit)
        return send_ns

    def _tick(self):
        now = time.monotonic_ns()
        for result in self.latency.expire(now):
            self._log_result(result)

        if self.rc is None or self.rc_received_ns is None:
            mode = "WAIT_FOR_RC"
        elif now - self.rc_received_ns > self.rc_timeout_ns:
            self.interlock.fault()
            mode = "RC_TIMEOUT"
        elif int(self.rc.online) != 1:
            self.interlock.fault()
            mode = "RC_OFFLINE"
        else:
            mode = self.interlock.mode(int(self.rc.right_switch))

        if mode != self.last_mode:
            self._log_event("mode", mode=mode, detail="switch/state change",
                            now_ns=now)
            self.get_logger().info("mode -> " + mode)
            self.last_mode = mode

        if mode != "SINE":
            if self.wave.running:
                self.last_wave_stopped_ns = now
                self._log_event("sine_stopped", mode=mode, now_ns=now)
            self.wave.stop()
            self.latency.cancel()
            self._publish(0, mode, 0.0, 0, 0.0)
            return

        if not self.wave.running:
            # A quick 1 -> 3 -> 1 toggle must not bypass the previous
            # direction-change guard by resetting the sine phase.
            if (self.last_wave_stopped_ns is not None and
                    now - self.last_wave_stopped_ns <
                    self.run_reentry_pause_ns):
                self._publish(0, "RUN_REENTRY_PAUSE", 0.0, 0, 0.0)
                return
            if not self._rpm_ready(now):
                self._publish(0, "WAIT_FOR_STOPPED_RPM", 0.0, 0, 0.0)
                return
            self.wave.start(now)
            self._log_event("sine_started", mode=mode, now_ns=now)

        wave = self.wave.step(now, rpm_ready=self._rpm_ready(now))
        send_ns = self._publish(
            wave.dshot, mode, wave.sine, wave.direction, wave.phase_rad)

        if wave.event:
            self._log_event(wave.event, mode=mode, dshot=wave.dshot,
                            sine=wave.sine, now_ns=send_ns)

        if wave.event in ("first_command", "reversal_command"):
            self.event_id += 1
            event_id = self.event_id
            baseline_available = (
                self.force_topic and self.last_force_ns is not None and
                0 <= send_ns - self.last_force_ns <= self.force_max_age_ns)
            baseline = self.last_force if baseline_available else None
            self._log_event(
                "force_response_reference", mode=mode, dshot=wave.dshot,
                sine=wave.sine, event_id=event_id, now_ns=send_ns,
                detail=("force baseline=%.6g %s" % (baseline, self.force_unit))
                if baseline is not None else
                "NO_RECENT_FORCE: latency unavailable")
            if baseline is None:
                self.get_logger().warn(
                    "event %d: no recent force on %s; latency unavailable" %
                    (event_id, self.force_topic or "(force_topic is empty)"))
            else:
                # force_forward_sign calibrates forward-positive thrust.
                for result in self.latency.start(
                        event_id, send_ns, wave.direction, baseline):
                    self._log_result(result)

    def shutdown(self):
        self.wave.stop()
        # Best-effort only: a crash, OS hang, DDS loss, or slave-side latch
        # can retain the last command. A separate hardware stop is necessary.
        for _ in range(10):
            self._publish(0, "SHUTDOWN", 0.0, 0, 0.0)
            time.sleep(0.01)
        self.logs.close()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = BidirectionalMotorTest()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.shutdown()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
