"""Read-only, ROS-independent CSV access for the Ubuntu desktop dashboard.

The GUI must never bind G10 UDP port 4800 or guess calibration units.
Every CSV row contains the fields produced by the existing ROS logger.
"""
from __future__ import annotations

import csv
import io
import math
from pathlib import Path
import time


FORCE_SUFFIX = "_force.csv"


def latest_session(directory):
    """Return newest session prefix, or None when acquisition has never run."""
    root = Path(directory).expanduser()
    if not root.is_dir():
        return None
    newest = None
    stamp = -1
    for item in root.glob("test_*_force.csv"):
        try:
            mtime = item.stat().st_mtime_ns
        except OSError:
            continue
        if mtime > stamp:
            stamp, newest = mtime, str(item)[:-len(FORCE_SUFFIX)]
    return newest


def recent_rows(path, max_bytes=16_384):
    """Read only a bounded tail; discard incomplete trailing rows.

    The header comes from the beginning of the file. Incomplete lines or
    concurrently flushed CSV rows never become telemetry samples.
    """
    if max_bytes < 512:
        raise ValueError("max_bytes must be at least 512")
    try:
        with open(path, "rb") as f:
            header = f.readline().decode("utf-8-sig", "replace").strip()
            if not header:
                return []
            columns = next(csv.reader([header]))
            f.seek(0, 2)
            end = f.tell()
            start = max(len(header.encode("utf-8")) + 1, end - max_bytes)
            f.seek(start)
            if start > len(header.encode("utf-8")) + 1:
                f.readline()  # Discard incomplete first line.
            data = f.read()
    except (OSError, UnicodeError, csv.Error):
        return []
    if not data:
        return []
    if not data.endswith(b"\n"):
        data = data.rsplit(b"\n", 1)[0] + b"\n" if b"\n" in data else b""
    if not data:
        return []
    try:
        result = []
        reader = csv.reader(io.StringIO(data.decode("utf-8", "replace")))
        for cells in reader:
            if len(cells) != len(columns) or cells == columns:
                continue
            result.append(dict(zip(columns, cells)))
        return result
    except (UnicodeError, csv.Error):
        return []


def last_row(prefix, record_type):
    rows = recent_rows(str(prefix) + "_" + record_type + ".csv", 4096)
    return rows[-1] if rows else None


def stream_fresh(prefix, max_age_sec=3.0, now=None):
    now = time.time() if now is None else now
    try:
        mtime = Path(str(prefix) + FORCE_SUFFIX).stat().st_mtime
    except OSError:
        return False
    return -1.0 <= now - mtime <= max_age_sec


def parse_force(row):
    """Return (monotonic_ns, value, unit, raw_adc, last_dshot), or None."""
    try:
        stamp = int(row["mono_ns"])
        value = float(row["forward_positive_force"])
        raw = float(row["raw_force"])
        dshot = int(row["last_dshot"])
        unit = row["force_unit"].strip()
        if unit not in ("raw_count", "kgf", "N"):
            return None
        return stamp, value, unit, raw, dshot
    except (KeyError, TypeError, ValueError, OverflowError):
        return None


def parse_channels(row):
    try:
        return [int(row["adc_%d" % channel]) for channel in range(8)]
    except (KeyError, TypeError, ValueError, OverflowError):
        return None


def plot_limits(points):
    """Symmetric autoscale about force=0, using observed samples as-is.

    Both positive and negative thrust get the same vertical scale; never
    manufacture a negative signal or hide outliers by clipping.
    """
    if not points:
        return -1.0, 1.0
    extent = max(abs(value) for value in points)
    if not math.isfinite(extent):
        return -1.0, 1.0
    half = max(1.0, extent * 1.15)
    return -half, half
