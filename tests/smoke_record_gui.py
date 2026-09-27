"""Tk desktop recording integration smoke (run under xvfb-run).

The G10/ROS services are mocked; the test exercises real GUI recording
buttons and ZIP export from the actual SessionLogs schema.
"""
import csv
import json
from pathlib import Path
import tempfile
import time
import tkinter as tk
from unittest import mock
import zipfile

from bidirectional_motor_test import g10_gui
from bidirectional_motor_test.session_logs import SessionLogs


def main():
    with tempfile.TemporaryDirectory() as home:
        folder = Path(home)
        with mock.patch.object(g10_gui, "LOG_DIR", folder):
            logs = SessionLogs(folder, "raw_count", prefix_tag="g10")
            control_logs = SessionLogs(
                folder, "raw_count", prefix_tag="control")
            try:
                initial = time.monotonic_ns()
                logs.write(
                    "force", wall_ns=time.time_ns(), mono_ns=initial,
                    raw_force=32688, forward_positive_force=0,
                    force_unit="raw_count", last_dshot=0)
                logs.write(
                    "g10_quality", mono_ns=initial, wall_ns=time.time_ns(),
                    stream_ready=1, reason="ready", last_receive_age_ms=1,
                    decoded_packets=100, invalid_packets=0, queue_dropped=0,
                    sequence_gap_events=0, timestamp_regressions=0,
                    queue_backlog=0, zero_samples=10000,
                    raw_windows_dropped=0)
                root = tk.Tk()
                with mock.patch.object(
                    g10_gui.messagebox, "showinfo", lambda *a, **kw: None):
                    app = g10_gui.Dashboard(root)
                    root.update_idletasks()
                    root.update()
                    # The G10 force CSV's last_dshot is zero here, but
                    # Motor Test has issued a nonzero command independently.
                    control_logs.write(
                        "command", wall_ns=time.time_ns(),
                        mono_ns=time.monotonic_ns(), mode="SINE", channel=1,
                        dshot=1250, sine=.5, logical_direction=1,
                        phase_rad=.5, last_force="", force_unit="raw_count")
                    app._refresh()
                    assert app.dshot_value.get() == "1250"
                    assert app.mode_value.get() == "SINE"
                    assert app.btn_record_start.winfo_ismapped()
                    assert app.btn_record_stop.winfo_ismapped()
                    app.record_dir = folder / "exports"
                    app.record_folder_text.set(str(app.record_dir))
                    app._record_start()
                    assert app.recording is not None
                    assert app.btn_record_start["state"] == "disabled"

                    command_ns = time.monotonic_ns()
                    logs.write(
                        "command", wall_ns=time.time_ns(),
                        mono_ns=command_ns, mode="SINE", channel=1,
                        dshot=1100, sine=0.3, logical_direction=1,
                        phase_rad=0.3, last_force=0,
                        force_unit="raw_count")
                    logs.write(
                        "event", wall_ns=time.time_ns(),
                        mono_ns=command_ns, event_id=1,
                        kind="force_response_reference", mode="SINE",
                        dshot=1100, sine=0.3,
                        detail="force baseline=0 raw_count")
                    force_ns = time.monotonic_ns()
                    logs.write(
                        "force", wall_ns=time.time_ns(),
                        mono_ns=force_ns, raw_force=32640,
                        forward_positive_force=48.0,
                        force_unit="raw_count", last_dshot=1100)
                    logs.write(
                        "latency", event_id=1, metric="force_onset",
                        status="detected",
                        command_mono_ns=command_ns,
                        observed_mono_ns=force_ns,
                        latency_ms=(force_ns-command_ns)/1e6,
                        baseline_force=0, observed_force=48,
                        force_unit="raw_count")
                    app._record_stop()
                    assert app.record_saving
                    deadline = time.monotonic() + 5
                    while app.record_saving and time.monotonic() < deadline:
                        root.update()
                        time.sleep(0.02)
                    root.update()
                    assert not app.record_saving
                    assert app.recording is None
                    archives = list(app.record_dir.glob("g10_record_*.zip"))
                    assert len(archives) == 1, archives
                    with zipfile.ZipFile(archives[0]) as archive:
                        rows = list(csv.DictReader(
                            x.decode("utf-8") for x in
                            archive.open("event_summary.csv").readlines()))
                        assert rows[0]["onset_status"] == "detected"
                        meta = json.loads(archive.read("metadata.json"))
                        assert meta["force_onset_detected"] == 1
                        print(
                            "G10 GUI record/export smoke: passed "
                            "(one ZIP, DSHOT, force, onset)")
                    root.destroy()
            finally:
                control_logs.close()
                logs.close()


if __name__ == "__main__":
    main()
