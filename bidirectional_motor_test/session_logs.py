"""ROS-independent CSV session logging; event rows contain a 'kind' column."""

import csv
from datetime import datetime, timezone
import os
from pathlib import Path


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

    def write(self, record_type, **row):
        self.writers[record_type].writerow(row)
        # We prefer durable, immediately inspectable measurements over disk speed
        # at the default 50 Hz command rate.
        self.handles[("command", "force", "event", "latency").index(record_type)].flush()

    def close(self):
        for handle in self.handles:
            handle.close()

