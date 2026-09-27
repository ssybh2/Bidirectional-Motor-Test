#!/usr/bin/env python3
"""DJIRC -> bounded sinusoidal bidirectional DSHOT, with optional thrust logging."""

from __future__ import annotations

import math
import queue
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Float64
from std_srvs.srv import Trigger
from custom_msgs.msg import ReadDJIRC, WriteDSHOT

from .core import SineDshot, SwitchInterlock
from .g10_udp import G10UDPReceiver
from .g10_calibration import (
    CalibrationError, StableADCWindow, load_scale, save_scale,
    scale_for_known_mass,
)
from .g10_health import baseline_ready, g10_health
from .latency import ThrustLatency
from .raw_capture import RawWindowRecorder
from .session_logs import SessionLogs


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
            "g10_udp_enabled": False,
            "g10_bind_host": "0.0.0.0",
            "g10_udp_port": 4800,
            "g10_device_ip": "192.168.127.56",
            "g10_adc_channel": 6,
            "g10_sample_period_ns": 100000,
            "g10_arrival_bias_ns": 0,
            "g10_auto_zero": True,
            "g10_auto_zero_samples": 10000,
            "g10_zero_raw": 0.0,
            "g10_force_sign": -1,
            "g10_kgf_per_count": 0.0,
            "g10_load_calibration": True,
            "g10_calibration_file": "~/bidirectional/calibration/g10_channel6.json",
            "g10_calibration_mass_kg": 0.0,
            "g10_stability_range_counts": 10.0,
            "g10_calibration_window_sec": 1.0,
            "g10_raw_change_threshold": 50.0,
            "g10_raw_sign_threshold": 50.0,
            "g10_log_decimation": 40,
            "g10_require_healthy": True,
            "g10_no_packet_timeout_sec": 0.15,
            "g10_poll_rate_hz": 100.0,
            "g10_max_packets_per_poll": 6,
            "g10_max_queue_backlog": 24,
            "g10_capture_raw_events": True,
            "g10_raw_pre_sec": 0.25,
            "g10_raw_post_sec": 0.75,
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
        self.g10_enabled = bool(p["g10_udp_enabled"])
        self.g10_device_ip = str(p["g10_device_ip"])
        self.g10_adc_channel = int(p["g10_adc_channel"])
        self.g10_sample_period_ns = int(p["g10_sample_period_ns"])
        self.g10_arrival_bias_ns = int(p["g10_arrival_bias_ns"])
        self.g10_auto_zero = bool(p["g10_auto_zero"])
        self.g10_auto_zero_samples = int(p["g10_auto_zero_samples"])
        self.g10_zero_raw = float(p["g10_zero_raw"])
        self.g10_force_sign = int(p["g10_force_sign"])
        self.g10_kgf_per_count = float(p["g10_kgf_per_count"])
        self.g10_calibration_file = str(p["g10_calibration_file"])
        self.g10_load_calibration = bool(p["g10_load_calibration"])
        self.g10_stability_range_counts = float(
            p["g10_stability_range_counts"])
        self.g10_calibration_window_sec = float(
            p["g10_calibration_window_sec"])
        self.g10_raw_change_threshold = float(
            p["g10_raw_change_threshold"])
        self.g10_raw_sign_threshold = float(
            p["g10_raw_sign_threshold"])
        self.g10_log_decimation = int(p["g10_log_decimation"])
        self.g10_require_healthy = bool(p["g10_require_healthy"])
        self.g10_no_packet_timeout_ns = round(
            float(p["g10_no_packet_timeout_sec"]) * 1e9)
        self.g10_poll_rate_hz = float(p["g10_poll_rate_hz"])
        self.g10_max_packets_per_poll = int(p["g10_max_packets_per_poll"])
        self.g10_max_queue_backlog = int(p["g10_max_queue_backlog"])
        self.g10_capture_raw_events = bool(p["g10_capture_raw_events"])
        self.g10_raw_pre_sec = float(p["g10_raw_pre_sec"])
        self.g10_raw_post_sec = float(p["g10_raw_post_sec"])
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
        if self.g10_enabled and self.force_topic:
            raise ValueError("use either force_topic or direct G10 UDP, not both")
        if self.g10_adc_channel not in range(8):
            raise ValueError("g10_adc_channel must be in 0..7")
        if self.g10_sample_period_ns <= 0 or self.g10_arrival_bias_ns < 0:
            raise ValueError("G10 sample period must be positive; arrival bias >= 0")
        if self.g10_auto_zero_samples < 1:
            raise ValueError("g10_auto_zero_samples must be >= 1")
        if self.g10_force_sign not in (-1, 1):
            raise ValueError("g10_force_sign must be +1 or -1")
        if self.g10_kgf_per_count < 0 or self.g10_log_decimation < 1:
            raise ValueError("G10 scale must be >= 0 and log decimation >= 1")
        if self.g10_no_packet_timeout_ns <= 0 or self.g10_poll_rate_hz <= 0:
            raise ValueError("G10 timeout and processing rate must be > 0")
        if self.g10_max_packets_per_poll < 1 or self.g10_max_queue_backlog < 1:
            raise ValueError("G10 processing and backlog limits must be > 0")
        if self.g10_raw_pre_sec < 0 or self.g10_raw_post_sec <= 0:
            raise ValueError("G10 raw event windows must be nonnegative/positive")
        if (not math.isfinite(self.g10_stability_range_counts) or
                self.g10_stability_range_counts <= 0):
            raise ValueError("G10 static stability range must be positive")
        if (not math.isfinite(self.g10_raw_change_threshold) or
                not math.isfinite(self.g10_raw_sign_threshold) or
                self.g10_raw_change_threshold <= 0 or
                self.g10_raw_sign_threshold <= 0):
            raise ValueError("G10 raw thresholds must be positive")
        if not math.isfinite(self.g10_kgf_per_count):
            raise ValueError("G10 scale must be finite")
        if self.g10_enabled and not self.g10_calibration_file:
            raise ValueError("G10 calibration file must be configured")
        if self.require_rpm and not self.rpm_topic:
            raise ValueError("require_rpm_for_reversal needs a nonempty rpm_topic")

        if (self.g10_enabled and self.g10_load_calibration and
                self.g10_kgf_per_count == 0):
            stored = load_scale(
                self.g10_calibration_file, self.g10_device_ip,
                self.g10_adc_channel, self.g10_force_sign)
            if stored is not None:
                self.g10_kgf_per_count = stored
                self.get_logger().info(
                    "Loaded G10 gain %.10g kgf/count from %s; "
                    "a fresh zero is still required" %
                    (stored, self.g10_calibration_file))
        self.g10_calibration_window = (
            StableADCWindow(
                sample_period_ns=self.g10_sample_period_ns,
                window_sec=self.g10_calibration_window_sec)
            if self.g10_enabled else None)
        if self.g10_enabled:
            self.force_unit = ("kgf" if self.g10_kgf_per_count > 0
                               else "raw_count")

        self.wave = SineDshot(
            frequency_hz=float(p["sine_frequency_hz"]),
            deadband=float(p["sine_deadband"]),
            positive_peak=int(p["positive_peak_dshot"]),
            negative_peak=int(p["negative_peak_dshot"]),
            reversal_pause_sec=float(p["reversal_pause_sec"]),
            invert_direction=bool(p["invert_direction"]),
        )
        self.latency = ThrustLatency(
            delta_threshold=(
                self.g10_raw_change_threshold *
                (self.g10_kgf_per_count or 1.0) if self.g10_enabled
                else float(p["force_change_threshold"])),
            sign_threshold=(
                self.g10_raw_sign_threshold *
                (self.g10_kgf_per_count or 1.0) if self.g10_enabled
                else float(p["force_sign_threshold"])),
            confirm_samples=int(p["force_confirm_samples"]),
            timeout_sec=float(p["force_latency_timeout_sec"]),
        )
        self.interlock = SwitchInterlock()
        self.logs = SessionLogs(str(p["log_directory"]), self.force_unit)
        self.raw_capture = (
            RawWindowRecorder(
                self.logs.prefix, self.g10_sample_period_ns,
                self.g10_raw_pre_sec, self.g10_raw_post_sec)
            if self.g10_enabled and self.g10_capture_raw_events else None)

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
        self.g10 = None
        self.g10_zero_sum = 0.0
        self.g10_zero_count = 0
        self.g10_sample_count = 0
        self.g10_last_sequence = None
        self.g10_sequence_gaps = 0
        self.g10_error_reported = False
        self.g10_raw_error_reported = False
        self.g10_last_received_ns = None
        self.g10_last_dropped = 0
        self.g10_last_quality_ns = 0
        self.g10_last_unready = None
        self.g10_timestamp_regressions = 0

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

        self.g10_tare_service = None
        self.g10_calibrate_service = None
        self.g10_calibration_status_service = None
        if self.g10_enabled:
            self.g10_tare_service = self.create_service(
                Trigger, "~/g10_tare", self._on_g10_tare)
            self.g10_calibrate_service = self.create_service(
                Trigger, "~/g10_calibrate", self._on_g10_calibrate)
            self.g10_calibration_status_service = self.create_service(
                Trigger, "~/g10_calibration_status",
                self._on_g10_calibration_status)
            self.g10 = G10UDPReceiver(
                bind_host=str(p["g10_bind_host"]),
                port=int(p["g10_udp_port"]),
                expected_device_ip=str(p["g10_device_ip"]),
            )
            self.g10.start()
            # Never exhaust the UDP queue in the 50 Hz DSHOT timer:
            # one bounded polling callback handles at most N UDP batches.
            self.g10_timer = self.create_timer(
                1.0 / self.g10_poll_rate_hz, self._g10_poll)
        else:
            self.g10_timer = None

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
        if self.g10 is not None:
            self.get_logger().info(
                "direct G10 UDP enabled: %s:%d, ADC channel %d, %s" %
                (p["g10_bind_host"], p["g10_udp_port"], self.g10_adc_channel,
                 "auto-zeroing" if self.g10_auto_zero else
                 "zero=%.3f" % self.g10_zero_raw))

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
        self._accept_force(now, force, signed_force, log_sample=True)

    def _accept_force(self, now, raw_force, signed_force, log_sample):
        # UDP batches are host-arrival anchored; receive jitter may produce
        # overlapping sample estimates. Do not feed backward time into latency.
        if self.last_force_ns is not None and now <= self.last_force_ns:
            self.g10_timestamp_regressions += 1
            return
        self.last_force = signed_force
        self.last_force_ns = now
        if log_sample:
            wall_offset = time.time_ns() - time.monotonic_ns()
            self.logs.write(
                "force", wall_ns=now + wall_offset, mono_ns=now,
                raw_force=raw_force, forward_positive_force=signed_force,
                force_unit=self.force_unit, last_dshot=self.last_command)
        for result in self.latency.observe(now, signed_force):
            self._log_result(result)

    def _on_g10_data_fault(self, reason):
        """Immediately command zero and revoke arm authorization."""
        self.interlock.fault()
        self.latency.cancel()
        if self.wave.running:
            self.last_wave_stopped_ns = time.monotonic_ns()
        self.wave.stop()
        self._log_event("g10_data_fault", mode="G10_UNREADY",
                        detail=reason)
        self.get_logger().warn("G10 data fault: %s; output DSHOT 0" % reason)
        self._publish(0, "G10_FAULT", 0.0, 0, 0.0)

    def _g10_poll(self):
        if self.g10 is None:
            return
        try:
            self._drain_g10()
            if self.g10.error is not None and not self.g10_error_reported:
                self.g10_error_reported = True
                self._on_g10_data_fault("receiver_error: %s" % self.g10.error)
            if self.g10.dropped_packets > self.g10_last_dropped:
                diff = self.g10.dropped_packets - self.g10_last_dropped
                self.g10_last_dropped = self.g10.dropped_packets
                self._on_g10_data_fault("UDP queue dropped %d packet(s)" % diff)
            if (self.raw_capture is not None and
                    self.raw_capture.error is not None and
                    not self.g10_raw_error_reported):
                self.g10_raw_error_reported = True
                self._on_g10_data_fault(
                    "raw CSV writer error: %s" % self.raw_capture.error)
            self._log_g10_quality()
        except Exception as exc:
            self._on_g10_data_fault("G10 processing exception: %s" % exc)

    def _log_g10_quality(self):
        now = time.monotonic_ns()
        if now - self.g10_last_quality_ns < 1_000_000_000:
            return
        self.g10_last_quality_ns = now
        ready, reason = self._g10_status(now)
        age_ms = (
            (now - self.g10_last_received_ns) / 1e6
            if self.g10_last_received_ns is not None else "")
        self.logs.write(
            "g10_quality", mono_ns=now, wall_ns=time.time_ns(),
            stream_ready=int(ready), reason=reason,
            last_receive_age_ms=age_ms,
            decoded_packets=self.g10.received_packets,
            invalid_packets=self.g10.invalid_packets,
            queue_dropped=self.g10.dropped_packets,
            sequence_gap_events=self.g10_sequence_gaps,
            timestamp_regressions=self.g10_timestamp_regressions,
            queue_backlog=self.g10.packets.qsize(),
            zero_samples=self.g10_zero_count,
            raw_windows_dropped=(
                self.raw_capture.dropped_windows if self.raw_capture else 0))

    def _g10_status(self, now):
        if self.g10 is None:
            return True, "disabled"
        zero_ready = (
            not self.g10_auto_zero or
            self.g10_zero_count >= self.g10_auto_zero_samples)
        recorder_error = (
            self.raw_capture.error if self.raw_capture is not None else None)
        ready, reason = g10_health(
            now_ns=now, last_recv_ns=self.g10_last_received_ns,
            zero_ready=zero_ready, timeout_ns=self.g10_no_packet_timeout_ns,
            backlog=self.g10.packets.qsize(),
            backlog_limit=self.g10_max_queue_backlog,
            error=self.g10.error or recorder_error)
        if ready and (self.last_force_ns is None or
                      now - self.last_force_ns > self.force_max_age_ns):
            return False, "force_samples_stale"
        return ready, reason

    def _drain_g10(self):
        if self.g10 is None:
            return
        for _ in range(self.g10_max_packets_per_poll):
            try:
                received_ns, packet = self.g10.packets.get_nowait()
            except queue.Empty:
                break
            self.g10_last_received_ns = received_ns

            if self.g10_last_sequence is not None:
                expected = (self.g10_last_sequence + 1) & 0xFFFF
                if packet.sequence != expected:
                    self.g10_sequence_gaps += 1
                    self._on_g10_data_fault(
                        "UDP sequence gap %d -> %d" %
                        (self.g10_last_sequence, packet.sequence))
            self.g10_last_sequence = packet.sequence
            timestamps = packet.sample_timestamps(
                received_ns, self.g10_sample_period_ns,
                self.g10_arrival_bias_ns)
            for sample_ns, channels in zip(timestamps, packet.samples):
                raw = float(channels[self.g10_adc_channel])
                self.g10_calibration_window.add(sample_ns, raw)
                if (self.g10_auto_zero and
                        self.g10_zero_count < self.g10_auto_zero_samples):
                    self.g10_zero_sum += raw
                    self.g10_zero_count += 1
                    if self.g10_zero_count == self.g10_auto_zero_samples:
                        self.g10_zero_raw = (
                            self.g10_zero_sum / self.g10_zero_count)
                        self.get_logger().info(
                            "G10 auto-zero complete: %.6f raw counts" %
                            self.g10_zero_raw)
                        if abs(self.g10_zero_raw) > 32567:
                            self.get_logger().warn(
                                "G10 channel is near signed int16 limit: "
                                "zero=%.3f, reverse-direction headroom "
                                "must be verified before motor tests" %
                                self.g10_zero_raw)
                    continue
                scale = (self.g10_kgf_per_count
                         if self.g10_kgf_per_count > 0 else 1.0)
                force = self.g10_force_sign * (raw - self.g10_zero_raw) * scale
                self.g10_sample_count += 1
                if self.raw_capture is not None:
                    self.raw_capture.add(
                        sample_ns, received_ns, packet.sequence,
                        raw, force, self.force_unit)
                self._accept_force(
                    sample_ns, raw, force,
                    log_sample=(self.g10_sample_count %
                                self.g10_log_decimation == 0))

    def _calibration_guard(self):
        """No calibrating while spinning, armed, or while ADC stream is bad."""
        if not self.g10_enabled or self.g10 is None:
            raise CalibrationError("direct G10 UDP is disabled")
        if self.last_command != 0 or self.wave.running:
            raise CalibrationError("stop the motor first (DSHOT must be 0)")
        if self.last_mode not in ("DISARM", "WAIT_FOR_RC"):
            raise CalibrationError(
                "RC must be DISARM (switch 2), or no RC connected")
        ready, reason = self._g10_status(time.monotonic_ns())
        if not ready:
            raise CalibrationError("G10 stream not ready: " + reason)
        return self.g10_calibration_window.snapshot(
            time.monotonic_ns(), self.g10_no_packet_timeout_ns,
            self.g10_stability_range_counts)

    def _on_g10_tare(self, request, response):
        """Capture a fresh stable *unloaded* mean, without changing gain."""
        try:
            mean, spread, count = self._calibration_guard()
            self.g10_zero_raw = mean
            self.g10_calibration_window.clear()
            self.last_force = None
            self.last_force_ns = None
            self.latency.cancel()
            self._log_event(
                "g10_tare", mode=self.last_mode,
                detail="zero_raw=%.6f; range=%.3f; samples=%d" %
                (mean, spread, count))
            response.success = True
            response.message = (
                "Tare OK: zero_raw=%.6f, p2p=%.3f counts, %d samples; "
                "gain unchanged. Remove load before taring." %
                (mean, spread, count))
        except CalibrationError as exc:
            response.success = False
            response.message = "Tare refused: " + str(exc)
        return response

    def _on_g10_calibrate(self, request, response):
        """Use a stable known positive-axis mass; persist gain, not offset."""
        try:
            mean, spread, count = self._calibration_guard()
            mass_kg = float(self.get_parameter(
                "g10_calibration_mass_kg").value)
            scale, delta = scale_for_known_mass(
                mass_kg, mean, self.g10_zero_raw,
                self.g10_force_sign)
            save_scale(
                self.g10_calibration_file, self.g10_device_ip,
                self.g10_adc_channel, self.g10_force_sign,
                scale, mass_kg, delta)
            self.g10_kgf_per_count = scale
            self.force_unit = "kgf"
            self.logs.force_unit = "kgf"
            # Raw thresholds always keep their raw-count meaning, so a
            # runtime gain change cannot silently change onset sensitivity.
            self.latency.delta_threshold = (
                self.g10_raw_change_threshold * scale)
            self.latency.sign_threshold = (
                self.g10_raw_sign_threshold * scale)
            self.g10_calibration_window.clear()
            self.last_force = None
            self.last_force_ns = None
            self.latency.cancel()
            self._log_event(
                "g10_gain_calibrated", mode=self.last_mode,
                detail=(
                    "mass_kg=%.6f; delta_raw=%.6f; kgf_per_count=%.12g; "
                    "zero_raw=%.6f; p2p=%.3f; samples=%d" %
                    (mass_kg, delta, scale, self.g10_zero_raw,
                     spread, count)))
            response.success = True
            response.message = (
                "Calibration OK: %.12g kgf/count from %.6f kg load "
                "(%.3f counts); saved to %s. "
                "Zero is re-measured on next startup." %
                (scale, mass_kg, delta, self.g10_calibration_file))
        except (CalibrationError, ValueError) as exc:
            response.success = False
            response.message = "Calibration refused: " + str(exc)
        return response

    def _on_g10_calibration_status(self, request, response):
        try:
            mean, spread, count = self._calibration_guard()
            stable = ("recent stable mean=%.6f raw, p2p=%.3f, n=%d"
                      % (mean, spread, count))
        except CalibrationError as exc:
            stable = "stable-window unavailable: " + str(exc)
        response.success = True
        response.message = (
            "zero_raw=%.6f; gain=%.12g kgf/count; unit=%s; "
            "channel=%d; %s" %
            (self.g10_zero_raw, self.g10_kgf_per_count,
             self.force_unit, self.g10_adc_channel, stable))
        return response

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
                "event %d %s: %.3f ms (%s, %s)" %
                (result.event_id, result.metric, result.latency_ms,
                 "ROS publish -> estimated G10 sample (uncalibrated bias)"
                 if self.g10_enabled else
                 "ROS publish -> force ROS message receipt",
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
        # The UDP decoder is on a separate, bounded timer. This callback
        # must keep publishing DSHOT 0 even when the G10 is not streaming.
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
        elif self.g10_enabled and self.g10_require_healthy:
            ready, reason = self._g10_status(now)
            if not ready:
                if reason != self.g10_last_unready:
                    self._on_g10_data_fault(reason)
                    self.g10_last_unready = reason
                mode = "G10_UNREADY_" + reason.upper()
            else:
                self.g10_last_unready = None
                mode = self.interlock.mode(int(self.rc.right_switch))
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
            zero_ready = (
                not self.g10_enabled or not self.g10_auto_zero or
                self.g10_zero_count >= self.g10_auto_zero_samples)
            valid = baseline_ready(
                self.g10_enabled, self.force_topic, zero_ready,
                self.last_force_ns, send_ns, self.force_max_age_ns)
            baseline = self.last_force if valid else None
            if self.raw_capture is not None:
                if not self.raw_capture.trigger(event_id, send_ns):
                    self.get_logger().warn(
                        "G10 raw event buffer full for event %d" % event_id)
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
        # SAFETY: attempt zero commands BEFORE waiting for UDP/CSV I/O.
        # Closing a raw CSV writer can block for many seconds; it must never
        # precede best-effort ESC stopping.
        self.wave.stop()
        try:
            for _ in range(10):
                self._publish(0, "SHUTDOWN", 0.0, 0, 0.0)
                time.sleep(0.01)
        finally:
            if self.g10 is not None:
                self.g10.stop()
            if self.raw_capture is not None:
                self.raw_capture.close()
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
