"""Stable static G10 tare and known-mass calibration; no ROS dependencies.

This calibrates *one selected ADC channel* for one force direction only.
The device protocol does not currently provide verified hardware timestamps.
"""

from collections import deque
from datetime import datetime, timezone
import json
import math
import os
from pathlib import Path
import tempfile


class CalibrationError(ValueError):
    pass


ADC_SIGNED16_SPAN = 65536.0
ADC_HALF_SPAN = 32768.0
ADC_DELTA_MODE = "signed16_modulo"


def wrap_signed16(value):
    """Represent an unwrapped ADC baseline as a signed 16-bit coordinate."""
    value = float(value)
    if not math.isfinite(value):
        raise CalibrationError("nonfinite G10 ADC reading")
    return (value + ADC_HALF_SPAN) % ADC_SIGNED16_SPAN - ADC_HALF_SPAN


def signed_adc16_delta(raw, reference):
    """Shortest signed difference across signed int16 representation boundary.

    raw and reference must be one unsigned-16 circular coordinate represented
    as signed int16. A physical excursion >= 32768 counts is ambiguous here;
    verify this encoding with bidirectional static loads before using thrust.
    """
    raw, reference = float(raw), float(reference)
    if (not math.isfinite(raw) or not math.isfinite(reference) or
            not -ADC_HALF_SPAN <= raw <= ADC_HALF_SPAN - 1 or
            not -ADC_HALF_SPAN <= reference <= ADC_HALF_SPAN):
        raise CalibrationError("G10 ADC value outside signed int16 coordinate")
    delta = (raw - reference + ADC_HALF_SPAN) % ADC_SIGNED16_SPAN - ADC_HALF_SPAN
    if abs(abs(delta) - ADC_HALF_SPAN) < 0.5:
        raise CalibrationError("ambiguous ADC half-range displacement")
    return delta


def adc_delta(raw, reference, modulo_signed16=True):
    """Relative force coordinate; never infer force from the DSHOT direction."""
    if modulo_signed16:
        return signed_adc16_delta(raw, reference)
    result = float(raw) - float(reference)
    if not math.isfinite(result):
        raise CalibrationError("nonfinite G10 ADC difference")
    return result


class StableADCWindow:
    """Bounded decimated recent readings; never interpolate missing samples."""

    def __init__(self, sample_period_ns=100_000, window_sec=1.0,
                 decimation=10):
        if sample_period_ns <= 0 or decimation < 1:
            raise ValueError("invalid ADC sample period or decimation")
        if not 0.1 <= window_sec <= 5.0:
            raise ValueError("calibration window must be 0.1 to 5 seconds")
        self.decimation = decimation
        self.spacing_ns = sample_period_ns * decimation
        self.required = max(10, round(window_sec * 1e9 / self.spacing_ns))
        self.history = deque(maxlen=self.required)
        self.total = 0
        self.last_ns = None

    def add(self, sample_ns, raw):
        sample_ns = int(sample_ns)
        if self.last_ns is not None and sample_ns <= self.last_ns:
            return
        self.last_ns = sample_ns
        self.total += 1
        if self.total % self.decimation == 0:
            self.history.append((sample_ns, float(raw)))

    def clear(self):
        self.history.clear()
        self.total = 0
        self.last_ns = None

    def snapshot(self, now_ns, max_age_ns, max_range_counts,
                 modulo_signed16=True):
        if len(self.history) < self.required:
            raise CalibrationError(
                "need %d stable samples, currently %d; wait %.1f s" %
                (self.required, len(self.history),
                 (self.required - len(self.history)) *
                 self.spacing_ns / 1e9))
        first_ns = self.history[0][0]
        last_ns = self.history[-1][0]
        if not 0 <= now_ns - last_ns <= max_age_ns:
            raise CalibrationError("ADC readings are stale")
        if last_ns - first_ns < (self.required - 1) * self.spacing_ns * 0.8:
            raise CalibrationError("ADC window contains insufficient time coverage")
        prev_ns = first_ns
        for ns, _ in list(self.history)[1:]:
            if ns - prev_ns > self.spacing_ns * 10:
                raise CalibrationError("ADC window has a timing gap")
            prev_ns = ns
        values = [v for _, v in self.history]
        if modulo_signed16:
            reference = values[0]
            values = [reference + signed_adc16_delta(v, reference)
                      for v in values]
        spread = max(values) - min(values)
        if spread > max_range_counts:
            raise CalibrationError(
                "load is not stable: peak-to-peak %.3f counts > %.3f" %
                (spread, max_range_counts))
        mean = sum(values) / len(values)
        if modulo_signed16:
            mean = wrap_signed16(mean)
        return mean, spread, len(values)


def scale_for_known_mass(mass_kg, raw_loaded, zero_raw, force_sign,
                         min_delta_counts=100.0, modulo_signed16=True):
    """Known positive-axis mass in kg gives the equivalent numeric kgf."""
    if not math.isfinite(mass_kg) or mass_kg <= 0:
        raise CalibrationError("known mass must be positive and finite")
    if force_sign not in (-1, 1):
        raise CalibrationError("force sign must be -1 or +1")
    delta = force_sign * adc_delta(
        raw_loaded, zero_raw, modulo_signed16=modulo_signed16)
    if delta <= 0:
        raise CalibrationError(
            "known load is opposite the configured positive force direction; "
            "check g10_force_sign and loading direction")
    if delta < min_delta_counts:
        raise CalibrationError(
            "known load changes only %.3f counts; need at least %.3f" %
            (delta, min_delta_counts))
    scale = mass_kg / delta
    if not math.isfinite(scale) or scale <= 0:
        raise CalibrationError("invalid kgf/count scale")
    return scale, delta


def load_scale(path, device_ip, channel, force_sign,
               modulo_signed16=True):
    path = Path(os.path.expanduser(path))
    if not path.exists():
        return None
    try:
        data = json.loads(path.read_text(encoding="utf-8"))
        expected_mode = (
            ADC_DELTA_MODE if modulo_signed16 else "linear_signed16")
        if (data.get("version") != 2 or
                data.get("adc_delta_mode") != expected_mode or
                data.get("device_ip") != device_ip or
                data.get("adc_channel") != channel or
                data.get("force_sign") != force_sign):
            raise CalibrationError(
                "saved G10 device/channel/sign mismatch: " + str(path))
        value = float(data["kgf_per_count"])
        if not math.isfinite(value) or value <= 0:
            raise CalibrationError("saved scale must be positive and finite")
        return value
    except (ValueError, TypeError, KeyError, OSError) as exc:
        raise CalibrationError(
            "cannot load G10 calibration %s: %s" % (path, exc)) from exc


def save_scale(path, device_ip, channel, force_sign, scale, mass_kg,
               delta_counts, modulo_signed16=True):
    """Atomic sidecar write. The zero offset is intentionally not persisted."""
    path = Path(os.path.expanduser(path))
    if not path.name or str(path) == ".":
        raise CalibrationError("provide a calibration file path")
    data = {
        "version": 2,
        "adc_delta_mode": (
            ADC_DELTA_MODE if modulo_signed16 else "linear_signed16"),
        "device_ip": device_ip,
        "adc_channel": channel, "force_sign": force_sign,
        "kgf_per_count": scale, "calibration_mass_kg": mass_kg,
        "calibration_delta_counts": delta_counts,
        "timestamp_utc": datetime.now(timezone.utc).isoformat(),
        "zero_offset_persisted": False,
    }
    temp = None
    try:
        path.parent.mkdir(parents=True, exist_ok=True)
        with tempfile.NamedTemporaryFile(
                mode="w", encoding="utf-8", dir=str(path.parent),
                prefix="." + path.name + ".", suffix=".tmp",
                delete=False) as f:
            temp = f.name
            json.dump(data, f, indent=2, ensure_ascii=False)
            f.write("\n")
        os.replace(temp, path)
    except OSError as exc:
        raise CalibrationError(
            "cannot save G10 calibration: %s" % exc) from exc
    finally:
        if temp and os.path.exists(temp):
            os.unlink(temp)
