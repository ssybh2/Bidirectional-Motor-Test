"""Root vs user HOME cannot change the launch-time shared directories."""
import unittest
from pathlib import Path

from bidirectional_motor_test.runtime_paths import (
    shared_ros_parameters, workspace_from_install_prefix,
)


class SharedPathsTests(unittest.TestCase):
    def test_isolated_and_merged_colcon_prefixes_share_one_workspace(self):
        for prefix in (
            "/home/hby/bidirectional/install/bidirectional_motor_test",
            "/home/hby/bidirectional/install",
        ):
            with self.subTest(prefix=prefix):
                self.assertEqual(
                    workspace_from_install_prefix(prefix),
                    Path("/home/hby/bidirectional"))
                self.assertEqual(
                    shared_ros_parameters(prefix),
                    {
                        "log_directory":
                            "/home/hby/bidirectional/measurements",
                        "g10_calibration_file":
                            "/home/hby/bidirectional/calibration/g10_channel6.json",
                    })

    def test_does_not_silently_guess_other_install_layouts(self):
        with self.assertRaises(ValueError):
            shared_ros_parameters("/opt/unrelated/pkg")


if __name__ == "__main__":
    unittest.main()
