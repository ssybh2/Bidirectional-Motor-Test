"""Independent GUI-owned G10 receiver; no DSHOT publisher."""
import os

from ament_index_python.packages import (
    get_package_prefix, get_package_share_directory,
)
from bidirectional_motor_test.runtime_paths import shared_ros_parameters
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config_file = os.path.join(
        get_package_share_directory("bidirectional_motor_test"),
        "config", "g10_capture.yaml")
    shared = shared_ros_parameters(get_package_prefix(
        "bidirectional_motor_test"))
    return LaunchDescription([
        Node(
            package="bidirectional_motor_test",
            executable="g10_acquisition",
            name="g10_acquisition",
            output="screen",
            parameters=[config_file, shared],
        ),
    ])
