import os

from ament_index_python.packages import (
    get_package_prefix, get_package_share_directory,
)
from bidirectional_motor_test.runtime_paths import shared_ros_parameters
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    package_share = get_package_share_directory('bidirectional_motor_test')
    config_file = os.path.join(package_share, 'config', 'motor_test.yaml')
    # Override the YAML's HOME-relative direct-run defaults. Resolving the
    # package install prefix also works when launch itself runs as root.
    shared_paths = shared_ros_parameters(
        get_package_prefix('bidirectional_motor_test'))

    return LaunchDescription([
        Node(
            package='bidirectional_motor_test',
            executable='motor_test',
            name='bidirectional_motor_test',
            output='screen',
            parameters=[config_file, shared_paths],
        ),
    ])
