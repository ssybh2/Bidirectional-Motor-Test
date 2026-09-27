"""Read-only, bounded fallback for same-host Motor Test command CSVs.

The controller's publish timestamp is authoritative.  This module only reads
its flushed log and never publishes motor instructions or creates a ROS node.
Use alongside the ROS metadata subscriber; duplicate stamps are filtered in
the acquisition node.  No cross-host clock assumptions are made.
"""
import math
from pathlib import Path
import time

from .gui_data import recent_rows


def active_control_prefix(directory, wall_now=None, max_age_s=2.5):
    """Return the newest accessible, recently updated controller CSV prefix."""
    wall_now = time.time() if wall_now is None else wall_now
    root = Path(directory).expanduser()
    if not root.is_dir():
        return None
    candidates = []
    for item in root.glob("control_*_command.csv"):
        try:
            mtime = item.stat().st_mtime
        except OSError:
            continue
        if -1 <= wall_now - mtime <= max_age_s:
            candidates.append((mtime, item))
    if not candidates:
        return None
    candidates.sort(reverse=True)
    return str(candidates[0][1])[:-len("_command.csv")]


def recent_control_metadata(directory, now_ns, lookback_ns=2_000_000_000):
    """Recover exact timestamps of commands and reversal reference events.

    Only completed CSV rows from an active controller are considered.  Input
    validation is deliberately strict: invalid rows, missing references and
    stale/reboot-crossing monotonic clocks cannot synthesize latency events.
    """
    prefix = active_control_prefix(directory)
    if prefix is None:
        return None, []
    commands = {}
    for row in recent_rows(prefix + "_command.csv", max_bytes=65536):
        try:
            stamp = int(row["mono_ns"])
            dshot = int(row["dshot"])
            direction = int(row["logical_direction"])
            channel = int(row["channel"])
            sine = float(row["sine"])
            phase = float(row["phase_rad"])
            wall = int(row["wall_ns"])
            mode = row["mode"]
        except (KeyError, ValueError, OverflowError, TypeError):
            continue
        if not (0 <= now_ns - stamp <= lookback_ns and
                0 <= dshot <= 2047 and direction in (-1, 0, 1) and
                channel in (1, 2, 3, 4) and
                math.isfinite(sine) and math.isfinite(phase) and
                isinstance(mode, str) and mode):
            continue
        commands[stamp] = {
            "type": "command", "mono_ns": stamp,
            "wall_ns": wall, "mode": mode, "dshot": dshot,
            "channel": channel, "sine": sine, "direction": direction,
            "phase": phase, "source": "control_csv",
        }
    results = [(stamp, 0, row) for stamp, row in commands.items()]
    for row in recent_rows(prefix + "_event.csv", max_bytes=65536):
        if row.get("kind") != "force_response_reference":
            continue
        try:
            stamp = int(row["mono_ns"])
            event_id = int(row["event_id"])
        except (TypeError, ValueError, OverflowError):
            continue
        command = commands.get(stamp)
        if (command is None or event_id <= 0 or
                command["mode"] != "SINE" or command["dshot"] == 0 or
                command["direction"] not in (-1, 1)):
            continue
        results.append((stamp, 1, {
            "type": "reference", "mono_ns": stamp,
            "mode": command["mode"], "dshot": command["dshot"],
            "sine": command["sine"], "direction": command["direction"],
            "control_event_id": event_id, "source": "control_csv",
        }))
    return prefix, [item for _, _, item in sorted(results, key=lambda x: x[:2])]
