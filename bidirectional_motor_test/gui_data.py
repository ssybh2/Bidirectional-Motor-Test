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
    for item in root.glob("g10_*_force.csv"):
        try:
            mtime = item.stat().st_mtime_ns
        except OSError:
            continue
        if mtime > stamp:
            stamp, newest = mtime, str(item)[:-len(FORCE_SUFFIX)]
    return newest


def latest_control_command(directory, max_age_sec=2.0, now=None):
    """Read the command actually published by Motor Test, not G10's copy.

    A missing, unreadable or stale controller command is UNKNOWN, not zero.
    No ROS initialization or UDP socket is needed in the Tk process.
    """
    root = Path(directory).expanduser()
    if not root.is_dir():
        return None
    newest = None
    newest_mtime = -1
    for item in root.glob("control_*_command.csv"):
        try:
            mtime_ns = item.stat().st_mtime_ns
        except OSError:
            continue
        if mtime_ns > newest_mtime:
            newest, newest_mtime = item, mtime_ns
    if newest is None or not _file_fresh(newest, max_age_sec, now):
        return None
    rows = recent_rows(newest, max_bytes=8192)
    if not rows:
        return None
    row = rows[-1]
    try:
        dshot = int(row["dshot"])
        stamp = int(row["mono_ns"])
        mode = str(row["mode"]).strip()
        if dshot not in range(2048) or stamp <= 0 or not mode:
            return None
    except (KeyError, TypeError, ValueError, OverflowError):
        return None
    return {"dshot": dshot, "mode": mode, "mono_ns": stamp}


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


def _file_fresh(path, max_age_sec, now):
    now = time.time() if now is None else now
    try:
        mtime = Path(path).stat().st_mtime
    except OSError:
        return False
    return -1.0 <= now - mtime <= max_age_sec


def stream_fresh(prefix, max_age_sec=3.0, now=None):
    """Whether actual G10 force samples are fresh, not just ROS commands."""
    return _file_fresh(
        str(prefix) + FORCE_SUFFIX, max_age_sec, now)


def acquisition_fresh(prefix, max_age_sec=3.0, now=None):
    """Whether the G10 receiver is alive even during auto-zero/link loss.

    The controller's command CSV is NOT evidence of G10 collection.
    """
    return _file_fresh(
        str(prefix) + "_g10_quality.csv", max_age_sec, now)


def collector_command_fresh(prefix, max_age_sec=2.0, now=None):
    """G10 collector has recently received a real controller command."""
    path = str(prefix) + "_command.csv"
    if not _file_fresh(path, max_age_sec, now):
        return False
    row = last_row(prefix, "command")
    if row is None:
        return False
    try:
        return int(row["mono_ns"]) > 0 and 0 <= int(row["dshot"]) <= 2047
    except (KeyError, TypeError, ValueError, OverflowError):
        return False


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
