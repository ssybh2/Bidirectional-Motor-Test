#!/usr/bin/env python3
"""Re-export an earlier GUI recording using original G10 + Motor Test CSVs.

This never modifies the original ZIP. It cannot recover missing timestamps
unless the matching control_*_command.csv still exists on the same host.
"""
import argparse
import json
from pathlib import Path
import sys
import zipfile

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from bidirectional_motor_test.session_export import (  # noqa: E402
    ExportError, export_recording,
)


def main():
    cli = argparse.ArgumentParser(description=__doc__)
    cli.add_argument("old_zip", type=Path)
    cli.add_argument(
        "--measurements", type=Path,
        default=Path.home() / "bidirectional" / "measurements",
        help="Directory containing ORIGINAL g10_* and control_* CSV files")
    cli.add_argument(
        "--output", type=Path, default=None,
        help="Existing directory for the NEW ZIP (defaults to old ZIP folder)")
    args = cli.parse_args()
    try:
        with zipfile.ZipFile(args.old_zip) as zf:
            metadata = json.loads(zf.read("metadata.json"))
        session = metadata["source_session"]
        if (not isinstance(session, str) or not session.startswith("g10_")
                or Path(session).name != session):
            raise ExportError("Invalid G10 source_session in old ZIP")
        prefix = args.measurements.expanduser() / session
        if not Path(str(prefix) + "_force.csv").is_file():
            raise ExportError(
                "Original G10 force CSV not found: %s. An old ZIP alone "
                "cannot reconstruct the controller's missing t0." % prefix)
        result = export_recording(
            prefix, (args.output or args.old_zip.parent),
            int(metadata["start_mono_ns"]), int(metadata["stop_mono_ns"]),
            int(metadata["start_wall_ns"]),
            int(metadata["stop_wall_ns"])
            if metadata.get("stop_wall_ns") is not None else None)
    except (OSError, KeyError, ValueError, zipfile.BadZipFile, ExportError) as exc:
        cli.exit(1, "Re-export failed: %s\n" % exc)
    print("New ZIP:", result["path"])
    print("Commands:", result["counts"]["command"])
    print("Force samples:", result["counts"]["force"])
    print("Live onset detections:", result["detected"])
    for warning in result.get("warnings", []):
        print("WARNING:", warning)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
