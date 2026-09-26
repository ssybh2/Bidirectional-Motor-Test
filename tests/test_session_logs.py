"""Regression: event CSV's 'kind' column must not shadow write() selector."""

import csv
import tempfile
import unittest
from pathlib import Path

from bidirectional_motor_test.session_logs import SessionLogs


class SessionLoggingTests(unittest.TestCase):
    def test_event_row_with_kind_and_other_records(self):
        with tempfile.TemporaryDirectory() as directory:
            session = SessionLogs(directory, "kgf")
            prefix = session.prefix
            try:
                # Before the fix this raised:
                # TypeError: SessionLogs.write() got multiple values for argument 'kind'
                session.write("event", wall_ns=123, mono_ns=456, event_id=7,
                              kind="mode", mode="WAIT_FOR_RC", dshot=0,
                              sine=0.0, detail="switch/state change")
                session.write("command", wall_ns=123, mono_ns=457,
                              mode="DISARM", channel=1, dshot=0,
                              sine=0.0, logical_direction=0, phase_rad=0.0,
                              last_force="", force_unit="kgf")
            finally:
                session.close()
            with open(prefix + "_event.csv", newline="", encoding="utf-8") as f:
                rows = list(csv.DictReader(f))
            self.assertEqual(len(rows), 1)
            self.assertEqual(rows[0]["kind"], "mode")
            self.assertEqual(rows[0]["mode"], "WAIT_FOR_RC")
            with open(prefix + "_command.csv", newline="", encoding="utf-8") as f:
                commands = list(csv.DictReader(f))
            self.assertEqual(commands[0]["dshot"], "0")


if __name__ == "__main__":
    unittest.main()
