"""ROS-independent CSV session logging; event rows contain a 'kind' column."""

import csv
from datetime import datetime, timezone
import os
from pathlib import Path


class SessionLogs:
    def __init__(self, directory, force_unit, prefix_tag="test"):
        if prefix_tag not in ("test", "control", "g10"):
            raise ValueError("unsupported session prefix")
        path = Path(os.path.expanduser(directory))
        path.mkdir(parents=True, exist_ok=True)
        stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S_%fZ")
        prefix = path / (prefix_tag + "_" + stamp + "_" + str(os.getpid()))
        self.force_unit = force_unit
        self.handles = []
        self.writers = {}
        schemas = {
            "command": ("wall_ns", "mono_ns", "mode", "channel", "dshot",
                        "sine", "logical_direction", "phase_rad",
                        "last_force", "force_unit", "metadata_source"),
            "force": ("wall_ns", "mono_ns", "raw_force", "forward_positive_force",
                      "force_unit", "last_dshot"),
            "event": ("wall_ns", "mono_ns", "event_id", "kind", "mode",
                      "dshot", "sine", "detail"),
            "latency": ("event_id", "metric", "status", "command_mono_ns",
                        "observed_mono_ns", "latency_ms", "baseline_force",
                        "observed_force", "force_unit"),
            "g10_quality": ("mono_ns", "wall_ns", "stream_ready", "reason",
                            "last_receive_age_ms", "decoded_packets",
                            "invalid_packets", "queue_dropped",
                            "sequence_gap_events", "timestamp_regressions",
                            "queue_backlog", "zero_samples",
                            "raw_windows_dropped", "metadata_source",
                            "command_age_ms", "ros_commands",
                            "csv_recovered_commands"),
            # One eight-channel ADC snapshot per N G10 UDP packets.
            # Other channels remain raw/unidentified, NOT calibrated units.
            "g10_channels": ("host_write_wall_ns",
                              "packet_recv_mono_ns",
                              "estimated_sample_mono_ns",
                              "packet_sequence",
                              "adc_0", "adc_1", "adc_2", "adc_3",
                              "adc_4", "adc_5", "adc_6", "adc_7"),
        }
        for kind, columns in schemas.items():
            handle = open(str(prefix) + "_" + kind + ".csv",
                          "w", newline="", encoding="utf-8")
            self.handles.append(handle)
            writer = csv.DictWriter(handle, fieldnames=columns)
            writer.writeheader()
            handle.flush()  # even an untouched session has readable headers
            self.writers[kind] = writer
        self.prefix = str(prefix)

    def write(self, record_type, **row):
        self.writers[record_type].writerow(row)
        # We prefer durable, immediately inspectable measurements over disk speed
        # at the default 50 Hz command rate.
        self.handles[("command", "force", "event", "latency", "g10_quality", "g10_channels").index(record_type)].flush()

    def close(self):
        for handle in self.handles:
            handle.close()

