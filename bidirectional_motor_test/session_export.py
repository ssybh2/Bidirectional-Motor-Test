"""Export a precisely bounded GUI recording from the ROS node's original CSV.

The GUI never re-timestamps DSHOT or G10: the authoritative timestamps and
onset calculations are already recorded by motor_test_node.py. Result is one
self-contained ZIP containing raw sample CSVs, event latency and provenance.
No ROS/Tk dependencies; safe to run in a GUI background worker.
"""

import csv
from datetime import datetime, timezone
import heapq
import io
import json
import math
import os
from pathlib import Path
import tempfile
import zipfile


KINDS = {
    "command": "mono_ns",
    "force": "mono_ns",
    "event": "mono_ns",
    "latency": "command_mono_ns",
    "g10_quality": "mono_ns",
    "g10_channels": "estimated_sample_mono_ns",
}
REQUIRED = ("command", "force", "event", "latency")
LATENCY_NAMES = ("force_onset", "target_sign", "force_direction_change")
DESCRIPTION = (
    "Measured from ROS DSHOT publish monotonic timestamp to G10 estimated "
    "ADC sample time. Includes G10 sampling/filtering/buffering, network, "
    "and host scheduling bias. Not a hardware-synchronized ESC or motor delay."
)


class ExportError(RuntimeError):
    pass


def _valid_int(value):
    try:
        return int(value)
    except (ValueError, TypeError):
        return None


def _read_csv(path):
    """Discard incomplete concurrent append rows; never interpolate samples."""
    try:
        with open(path, newline="", encoding="utf-8") as stream:
            reader = csv.DictReader(stream)
            fields = reader.fieldnames
            if not fields:
                # SessionLogs can exist before its buffered header is flushed.
                # An empty event/quality/latency stream is normal at startup.
                return
            for row in reader:
                if None in row or any(value is None for value in row.values()):
                    continue
                yield fields, row
    except OSError as exc:
        raise ExportError("Cannot read measurement CSV %s: %s" %
                          (path, exc)) from exc


def _filtered(prefix, kind, start_ns, stop_ns):
    field = KINDS[kind]
    source = Path(str(prefix) + "_" + kind + ".csv")
    if not source.is_file():
        if kind in REQUIRED:
            raise ExportError("Missing ROS measurement file: " + str(source))
        return
    for fields, row in _read_csv(source):
        time_ns = _valid_int(row.get(field))
        if time_ns is None or time_ns < start_ns or time_ns > stop_ns:
            continue
        # A latency measurement observed after Stop is not a result
        # from inside the chosen recording interval.
        if kind == "latency":
            observed = _valid_int(row.get("observed_mono_ns"))
            confirmed = _valid_int(row.get("confirmed_mono_ns"))
            if ((observed is not None and observed > stop_ns) or
                    (confirmed is not None and confirmed > stop_ns)):
                continue
        yield fields, row


def _select_controller(prefix, start_ns, stop_ns, start_wall_ns=None):
    """Choose ONE same-boot controller matching BOTH monotonic and wall clocks.

    Monotonic time resets on reboot. Its numeric value alone can overlap with
    an old experiment at a similar post-boot uptime, so a matching controller
    MUST also share the same wall-minus-monotonic timebase. This is a
    consistency check, NOT hardware time synchronization.
    """
    choices = []
    inaccessible = []
    expected_offset = (
        start_wall_ns - start_ns if start_wall_ns is not None else None)
    for command_path in Path(prefix).parent.glob("control_*_command.csv"):
        clock_mismatch = False
        try:
            for _, row in _read_csv(command_path):
                stamp = _valid_int(row.get("mono_ns"))
                dshot = _valid_int(row.get("dshot"))
                if (stamp is None or not start_ns <= stamp <= stop_ns
                        or dshot is None or not 0 <= dshot <= 2047):
                    continue
                # The old ZIP and the present-day controller both use Linux
                # epoch wall_ns and boot-local mono_ns. Legacy tiny synthetic
                # test stamps are excluded from this real-clock invariant.
                source_wall_ns = _valid_int(row.get("wall_ns"))
                if (expected_offset is not None and start_ns > 1_000_000_000
                        and start_wall_ns > 1_000_000_000_000_000
                        and source_wall_ns is not None
                        and abs((source_wall_ns - stamp) -
                                expected_offset) > 5_000_000_000):
                    clock_mismatch = True
                    continue
                choices.append(str(command_path)[:-len("_command.csv")])
                break
        except ExportError as exc:
            inaccessible.append(str(exc))
        if clock_mismatch and len(inaccessible) < 10:
            inaccessible.append(
                "Rejected controller %s: wall/monotonic timebase differs "
                "(possibly from a previous boot)." % command_path.name)
    if len(choices) > 1:
        raise ExportError(
            "Multiple Motor Test command logs overlap this recording; "
            "cannot choose a safe authoritative controller: " +
            ", ".join(choices))
    return (choices[0] if choices else None), inaccessible


def _supplement_reference_events(event_csv, source_prefix, start_ns, stop_ns):
    """Include genuine control-side reversal references when G10 ROS lost them.

    Mark recovered events as lacking live G10 metadata. No fabricated
    measurement or physical motor response latency is added.
    """
    if source_prefix is None:
        return 0
    with open(event_csv, newline="", encoding="utf-8") as f:
        reader = csv.DictReader(f)
        fields = reader.fieldnames
        existing = list(reader)
    if not fields:
        return 0
    known = {
        _valid_int(row.get("mono_ns"))
        for row in existing
        if row.get("kind") == "force_response_reference"
    }
    next_id = max(
        (_valid_int(row.get("event_id")) or 0 for row in existing),
        default=0)
    found = 0
    for _, row in _filtered(source_prefix, "event", start_ns, stop_ns):
        if row.get("kind") != "force_response_reference":
            continue
        stamp = _valid_int(row.get("mono_ns"))
        if stamp is None or stamp in known:
            continue
        next_id += 1
        found += 1
        known.add(stamp)
        # Event from the controller establishes t0, not a valid force
        # baseline or observed onset on the G10 sensor.
        row = dict(row)
        row["event_id"] = str(next_id)
        row["detail"] = (
            "NO_COLLECTOR_METADATA: " + row.get("detail", ""))
        existing.append(row)
    if found:
        existing.sort(key=lambda row: _valid_int(row.get("mono_ns")) or 0)
        with open(event_csv, "w", newline="", encoding="utf-8") as f:
            writer = csv.DictWriter(f, fieldnames=fields)
            writer.writeheader()
            writer.writerows(existing)
    return found


def _write_filtered(prefix, kind, start_ns, stop_ns, file_path):
    count = 0
    header = None
    with open(file_path, "w", encoding="utf-8", newline="") as f:
        writer = None
        for fields, row in _filtered(prefix, kind, start_ns, stop_ns):
            if writer is None:
                header = fields
                writer = csv.DictWriter(f, fieldnames=fields)
                writer.writeheader()
            writer.writerow(row)
            count += 1
        if writer is None:
            source = Path(str(prefix) + "_" + kind + ".csv")
            if source.is_file():
                with open(source, newline="", encoding="utf-8") as original:
                    header = next(csv.reader(original), None)
            if not header:
                header = [KINDS[kind]]
            csv.writer(f).writerow(header)
    return count


def _read_written(path):
    with open(path, newline="", encoding="utf-8") as stream:
        yield from csv.DictReader(stream)


def _write_timeline(files, target, start_ns):
    """Sorted heterogeneous samples, not interpolated/resampled.

    Rows are tagged 'command' or 'force'; last_dshot on a force row
    is only the value logged when force was processed, not precise t0.
    """
    headers = (
        "relative_sec", "mono_ns", "row_type", "mode",
        "dshot", "logical_direction", "sine", "raw_adc",
        "signed_force", "force_unit", "source_wall_ns")
    streams = []
    try:
        # Two independent CSVs are already chronological; heap-merge.
        iterators = []
        for kind in ("command", "force"):
            stream = open(files[kind], newline="", encoding="utf-8")
            streams.append(stream)
            iterators.append((kind, csv.DictReader(stream)))
        merged = []
        for i, (kind, it) in enumerate(iterators):
            row = next(it, None)
            while row is not None and _valid_int(row.get("mono_ns")) is None:
                row = next(it, None)
            if row is not None:
                heapq.heappush(
                    merged, (_valid_int(row["mono_ns"]), i, kind, row))
        with open(target, "w", encoding="utf-8", newline="") as f:
            writer = csv.DictWriter(f, fieldnames=headers)
            writer.writeheader()
            while merged:
                stamp, i, kind, row = heapq.heappop(merged)
                writer.writerow({
                    "relative_sec": "%.9f" % ((stamp - start_ns) / 1e9),
                    "mono_ns": stamp,
                    "row_type": kind,
                    "mode": row.get("mode", "") if kind == "command" else "",
                    "dshot": row.get("dshot", "") if kind == "command" else "",
                    "logical_direction": (
                        row.get("logical_direction", "")
                        if kind == "command" else ""),
                    "sine": row.get("sine", "") if kind == "command" else "",
                    "raw_adc": row.get("raw_force", "") if kind == "force" else "",
                    "signed_force": (
                        row.get("forward_positive_force", "")
                        if kind == "force" else ""),
                    "force_unit": row.get("force_unit", ""),
                    "source_wall_ns": row.get("wall_ns", ""),
                })
                row = next(iterators[i][1], None)
                while row is not None and _valid_int(row.get("mono_ns")) is None:
                    row = next(iterators[i][1], None)
                if row is not None:
                    heapq.heappush(
                        merged, (_valid_int(row["mono_ns"]), i, kind, row))
    finally:
        for handle in streams:
            if hasattr(handle, "close"):
                handle.close()


def _event_summary(files, target, start_ns, stop_ns):
    """Use *only* latency decisions already made by the 10 kHz ROS node."""
    event_rows = {}
    for row in _read_written(files["event"]):
        if row.get("kind") != "force_response_reference":
            continue
        event_id = _valid_int(row.get("event_id"))
        ns = _valid_int(row.get("mono_ns"))
        if event_id is not None and event_id > 0 and ns is not None:
            event_rows[event_id] = row
    latency = {}
    for row in _read_written(files["latency"]):
        event_id = _valid_int(row.get("event_id"))
        if event_id in event_rows and row.get("metric") in LATENCY_NAMES:
            latency[(event_id, row["metric"])] = row

    headers = (
        "event_id", "command_mono_ns", "command_relative_sec",
        "dshot", "desired_direction", "force_unit",
        "onset_status", "onset_observed_mono_ns", "onset_delay_ms",
        "target_sign_status", "target_sign_observed_mono_ns",
        "target_sign_delay_ms",
        "reversal_from_direction", "reversal_to_direction",
        "reversal_status", "reversal_observed_mono_ns",
        "reversal_confirmed_mono_ns", "reversal_delay_ms",
        "reversal_clock_source", "note")
    summary = []
    # Build direction from the command row with exactly the same t0 stamp.
    commands = {}
    for row in _read_written(files["command"]):
        ns = _valid_int(row.get("mono_ns"))
        if ns is not None:
            commands[ns] = row
    for event_id, event in sorted(event_rows.items()):
        command_ns = int(event["mono_ns"])
        command = commands.get(command_ns, {})
        detail = event.get("detail", "")
        baseline_missing = "NO_RECENT_FORCE" in detail
        metadata_missing = "NO_COLLECTOR_METADATA" in detail
        transition = {}
        for token in detail.split():
            if token.startswith(("reversal_from=", "reversal_to=")):
                key, _, value = token.partition("=")
                try:
                    transition[key] = int(value)
                except ValueError:
                    pass
        is_reversal = (
            transition.get("reversal_from") in (-1, 1) and
            transition.get("reversal_to") ==
            -transition.get("reversal_from", 0))
        reversal_row = latency.get(
            (event_id, "force_direction_change"))
        reversal_status = (
            reversal_row["status"] if reversal_row else
            "not_reversal" if not is_reversal else
            "metadata_unavailable" if metadata_missing else
            "unresolved_at_stop")
        metrics = {}
        for name in LATENCY_NAMES:
            row = latency.get((event_id, name))
            status = row.get("status", "") if row else (
                "metadata_unavailable" if metadata_missing else
                "no_recent_force" if baseline_missing else
                "unresolved_at_stop")
            metrics[name] = (
                status,
                row.get("observed_mono_ns", "") if row else "",
                row.get("latency_ms", "") if row else "",
            )
        item = {
            "event_id": event_id,
            "command_mono_ns": command_ns,
            "command_relative_sec": "%.9f" % (
                (command_ns - start_ns) / 1e9),
            "dshot": command.get("dshot", event.get("dshot", "")),
            "desired_direction": command.get("logical_direction", ""),
            "force_unit": latency.get(
                (event_id, "force_onset"), {}).get(
                    "force_unit", command.get("force_unit", "")),
            "onset_status": metrics["force_onset"][0],
            "onset_observed_mono_ns": metrics["force_onset"][1],
            "onset_delay_ms": metrics["force_onset"][2],
            "target_sign_status": metrics["target_sign"][0],
            "target_sign_observed_mono_ns": metrics["target_sign"][1],
            "target_sign_delay_ms": metrics["target_sign"][2],
            "reversal_from_direction": (
                reversal_row.get("from_direction", "")
                if reversal_row else
                transition.get("reversal_from", "") if is_reversal else ""),
            "reversal_to_direction": (
                reversal_row.get("to_direction", "")
                if reversal_row else
                transition.get("reversal_to", "") if is_reversal else ""),
            "reversal_status": reversal_status,
            "reversal_observed_mono_ns": (
                reversal_row.get("observed_mono_ns", "")
                if reversal_row else ""),
            "reversal_confirmed_mono_ns": (
                reversal_row.get("confirmed_mono_ns", "")
                if reversal_row else ""),
            "reversal_delay_ms": (
                reversal_row.get("latency_ms", "")
                if reversal_row else ""),
            "reversal_clock_source": (
                reversal_row.get("clock_source", "")
                if reversal_row else ""),
            "note": (
                "Control timestamp recovered from separate CSV; "
                "G10 live metadata missing, latency NOT measured"
                if metadata_missing else
                "No fresh G10 force baseline" if baseline_missing
                else ("No complete onset measurement within the "
                      "selected recording" if not
                      latency.get((event_id, "force_onset")) else "")),
        }
        summary.append(item)
    with open(target, "w", encoding="utf-8", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=headers)
        writer.writeheader()
        writer.writerows(summary)
    return summary


def _write_reversal_summary(events, target):
    """Only actual first-opposite-DSHOT transitions; never startup events."""
    fields = (
        "event_id", "from_direction", "to_direction", "command_mono_ns",
        "direction_crossing_receive_ns", "confirmed_receive_ns",
        "delay_ms", "status", "force_unit", "clock_source", "note")
    count = 0
    detected = []
    with open(target, "w", encoding="utf-8", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fields)
        writer.writeheader()
        for event in events:
            if event["reversal_status"] == "not_reversal":
                continue
            writer.writerow({
                "event_id": event["event_id"],
                "from_direction": event["reversal_from_direction"],
                "to_direction": event["reversal_to_direction"],
                "command_mono_ns": event["command_mono_ns"],
                "direction_crossing_receive_ns": (
                    event["reversal_observed_mono_ns"]),
                "confirmed_receive_ns": event["reversal_confirmed_mono_ns"],
                "delay_ms": event["reversal_delay_ms"],
                "status": event["reversal_status"],
                "force_unit": event["force_unit"],
                "clock_source": event["reversal_clock_source"],
                "note": event["note"],
            })
            count += 1
            if (event["reversal_status"] == "detected" and
                    event["reversal_delay_ms"]):
                detected.append(float(event["reversal_delay_ms"]))
    return count, detected


def _write_raw_window(source, target, start_ns, stop_ns):
    """Export native-rate event samples only within the recording interval."""
    count = 0
    writer = None
    with open(target, "w", encoding="utf-8", newline="") as f:
        for fields, row in _read_csv(source):
            stamp = _valid_int(row.get("estimated_sample_mono_ns"))
            if stamp is None or not start_ns <= stamp <= stop_ns:
                continue
            if writer is None:
                writer = csv.DictWriter(f, fieldnames=fields)
                writer.writeheader()
            writer.writerow(row)
            count += 1
    return count


def export_recording(prefix, destination, start_ns, stop_ns,
                     started_wall_ns, stopped_wall_ns=None):
    """Package one recording into destination/g10_record_*.zip.

    Does not modify any source files; never invents a physical response
    delay when the ROS detector failed, timed out, or had no fresh baseline.
    """
    if not isinstance(start_ns, int) or not isinstance(stop_ns, int):
        raise ExportError("Recording bounds must be monotonic nanoseconds")
    if start_ns >= stop_ns:
        raise ExportError("Recording end must follow recording start")
    folder = Path(destination).expanduser()
    if not folder.is_dir():
        raise ExportError("Selected output folder does not exist: " + str(folder))
    prefix = Path(prefix).expanduser()
    for kind in REQUIRED:
        if not Path(str(prefix) + "_" + kind + ".csv").is_file():
            raise ExportError("Measurement source is missing: %s" % kind)
    wall_ns = (started_wall_ns if started_wall_ns is not None
               else 0)
    if not isinstance(wall_ns, int):
        raise ExportError("Expected integer start wall timestamp")
    name = ("g10_record_%s_%d.zip" %
            (datetime.fromtimestamp(
                wall_ns / 1e9, tz=timezone.utc).strftime("%Y%m%d_%H%M%S_%f"),
             os.getpid()))
    output = folder / name
    tmp_output = folder / ("." + name + ".part")
    if output.exists() or tmp_output.exists():
        raise ExportError("Recording file already exists: " + str(output))

    counts = {}
    controller, control_errors = _select_controller(
        prefix, start_ns, stop_ns, started_wall_ns)
    warnings = list(control_errors)
    try:
        with tempfile.TemporaryDirectory(prefix="g10_export_") as temp:
            temp = Path(temp)
            files = {}
            for kind in KINDS:
                files[kind] = temp / (kind + ".csv")
                # Control logs are authoritative when accessible and their
                # monotonic timestamps overlap this exact recording. They
                # remain distinct from the 10 kHz G10 sampling clock.
                origin = controller if kind == "command" and controller else prefix
                counts[kind] = _write_filtered(
                    origin, kind, start_ns, stop_ns, files[kind])
            recovered = _supplement_reference_events(
                files["event"], controller, start_ns, stop_ns)
            counts["event"] += recovered
            if recovered:
                warnings.append(
                    "%d controller reversal references recovered; G10 did "
                    "not produce corresponding live onset decisions" % recovered)
            if not counts["command"]:
                warnings.append(
                    "No Motor Test command timestamps in this interval. "
                    "Force-only recording; DSHOT latency unavailable.")
            if controller and not counts["latency"]:
                warnings.append(
                    "Motor Test timestamps included from its separate CSV, "
                    "but no verified G10 latency results were recorded. "
                    "Do not infer motor response time from this ZIP.")
            if not counts["force"]:
                raise ExportError("No force samples in the selected interval")
            timeline = temp / "timeline.csv"
            _write_timeline(files, timeline, start_ns)
            event_csv = temp / "event_summary.csv"
            events = _event_summary(
                files, event_csv, start_ns, stop_ns)
            reversal_csv = temp / "reversal_summary.csv"
            reversal_events, reversal_delays = _write_reversal_summary(
                events, reversal_csv)
            onset = [
                float(row["onset_delay_ms"])
                for row in events if row["onset_status"] == "detected"
                and row["onset_delay_ms"]]
            # Optional 10 kHz windows are created by a background writer.
            # Their absence near Stop must not be silently treated as a
            # complete native-rate record.
            raw_event_paths = []
            raw_missing = []
            for event in events:
                event_id = int(event["event_id"])
                source = Path(
                    str(prefix) + "_raw_event_%04d.csv" % event_id)
                if not source.is_file():
                    raw_missing.append(event_id)
                    continue
                target = temp / ("raw_event_%04d.csv" % event_id)
                if _write_raw_window(
                        source, target, start_ns, stop_ns):
                    raw_event_paths.append(target)
                else:
                    raw_missing.append(event_id)
            # Legacy G10 sessions independently anchored each packet at its
            # UDP receive timestamp. New sessions use a sequence-paced
            # estimated clock. Neither method is a synchronized device clock:
            # missing samples cannot be recreated after recording.
            quality = list(_read_written(files["g10_quality"]))
            def counter_delta(field):
                if len(quality) < 2:
                    return None
                first = _valid_int(quality[0].get(field))
                last = _valid_int(quality[-1].get(field))
                return max(0, last - first) if (
                    first is not None and last is not None) else None

            timebase_regressions = counter_delta("timestamp_regressions")
            udp_queue_drops = counter_delta("queue_dropped")
            udp_sequence_gaps = counter_delta("sequence_gap_events")
            def latest_quality_float(field):
                if not quality:
                    return None
                try:
                    value = float(quality[-1][field])
                    return value if math.isfinite(value) else None
                except (KeyError, ValueError, TypeError, OverflowError):
                    return None

            clock_max_residual_ms = latest_quality_float(
                "sequence_clock_max_abs_residual_ms")
            clock_sample_period_ns = latest_quality_float(
                "sample_period_ns")
            if (clock_max_residual_ms is not None and
                    clock_max_residual_ms > 10.0):
                warnings.append(
                    "G10 sequence-clock/host-receive residual reached "
                    "%.3f ms; estimated ADC timebase may drift or be "
                    "biased by network scheduling. This is NOT a "
                    "hardware-synchronized response delay." %
                    clock_max_residual_ms)
            if timebase_regressions:
                warnings.append(
                    "G10 estimated sample timestamps regressed %d times; "
                    "affected samples were rejected. Sub-millisecond "
                    "latency accuracy is NOT established."
                    % timebase_regressions)
            if udp_queue_drops or udp_sequence_gaps:
                warnings.append(
                    "G10 transport loss: queue_drops=%s, "
                    "sequence_gap_events=%s" %
                    (udp_queue_drops, udp_sequence_gaps))
            metadata = {
                "schema_version": 1,
                "source_session": prefix.name,
                "start_mono_ns": start_ns,
                "stop_mono_ns": stop_ns,
                "start_wall_ns": started_wall_ns,
                "stop_wall_ns": stopped_wall_ns,
                "duration_s": (stop_ns - start_ns) / 1e9,
                "row_counts": counts,
                "command_source": (
                    "authoritative_control_csv" if controller else
                    "collector_ros_metadata" if counts["command"] else "none"),
                "controller_session": (
                    Path(controller).name if controller else None),
                "recovered_control_references": recovered,
                "data_integrity_warnings": warnings,
                "g10_timebase_regressions": timebase_regressions,
                "g10_udp_queue_drops": udp_queue_drops,
                "g10_sequence_gap_events": udp_sequence_gaps,
                "g10_sequence_clock_max_abs_residual_ms":
                    clock_max_residual_ms,
                "g10_estimated_sample_period_ns": clock_sample_period_ns,
                "latency_status": (
                    "detected" if onset else
                    "not_measured" if not counts["latency"] else
                    "no_detected_onset"),
                "native_rate_event_files": len(raw_event_paths),
                "native_rate_event_missing_or_not_finished": raw_missing,
                "force_onset_detected": len(onset),
                "reversal_events": reversal_events,
                "reversal_detected": len(reversal_delays),
                "reversal_mean_ms": (
                    sum(reversal_delays) / len(reversal_delays)
                    if reversal_delays else None),
                "reversal_latency_definition": (
                    "FIRST opposite nonzero DSHOT ROS publish timestamp "
                    "to HOST UDP receive timestamp of FIRST G10 packet "
                    "median reaching the target thrust sign, which is "
                    "subsequently confirmed sustained for >=0.2s. "
                    "No device clock sync; includes sensor, UDP and host "
                    "latency, NOT ESC execution or rotor RPM reversal."),
                "force_onset_min_ms": min(onset) if onset else None,
                "force_onset_mean_ms": (
                    sum(onset) / len(onset)) if onset else None,
                "latency_definition": DESCRIPTION,
                "command_timestamp": "time.monotonic_ns() before ROS publish",
                "G10_sample_timestamp": (
                    "Sequence-paced estimate using configured sample period; "
                    "NOT a device hardware timestamp or synchronized clock. "
                    "force_direction_change uses causal UDP host receive "
                    "timestamps instead of future-dated sample estimates"),
                "force_unit": (
                    "Per-row: force.csv and event_summary.csv; do not "
                    "combine raw_count with kgf without calibration"),
                "notes": [
                    "timeline.csv is a time-sorted union of command and "
                    "force rows, not synchronized/interpolated readings",
                    "Missing onset status means no valid delay was "
                    "determined; do not treat it as zero milliseconds",
                    "Any motor response measured while a force already "
                    "decays may represent residual freewheel motion",
                    "No hardware-synchronized measurement or RPM verification",
                    "Optional raw_events are native-rate ~10 kHz around "
                    "commands, but may be absent if capture has not finished",
                ],
            }
            meta = temp / "metadata.json"
            meta.write_text(
                json.dumps(metadata, indent=2, ensure_ascii=False) + "\n",
                encoding="utf-8")
            with zipfile.ZipFile(
                tmp_output, mode="x", compression=zipfile.ZIP_DEFLATED,
                compresslevel=6) as archive:
                for kind in KINDS:
                    archive.write(files[kind], arcname=kind + ".csv")
                archive.write(timeline, arcname="timeline.csv")
                archive.write(event_csv, arcname="event_summary.csv")
                archive.write(reversal_csv, arcname="reversal_summary.csv")
                archive.write(meta, arcname="metadata.json")
                for source in raw_event_paths:
                    archive.write(
                        source, arcname="raw_events/" + source.name)
            os.replace(tmp_output, output)
    except Exception:
        try:
            tmp_output.unlink(missing_ok=True)
        except OSError:
            pass
        raise
    return {
        "path": str(output),
        "counts": counts,
        "events": len(events),
        "detected": len(onset),
        "onset_mean_ms": (
            sum(onset) / len(onset)) if onset else None,
        "reversal_events": reversal_events,
        "reversal_detected": len(reversal_delays),
        "reversal_mean_ms": (
            sum(reversal_delays) / len(reversal_delays)
            if reversal_delays else None),
        "metadata": metadata,
        "warnings": warnings,
    }
