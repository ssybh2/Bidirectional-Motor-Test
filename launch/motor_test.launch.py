import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    package_share = get_package_share_directory('bidirectional_motor_test')
    config_file = os.path.join(package_share, 'config', 'motor_test.yaml')

    return LaunchDescription([
        Node(
            package='bidirectional_motor_test',
            executable='motor_test',
            name='bidirectional_motor_test',
            output='screen',
            parameters=[config_file],
        ),
    ])
