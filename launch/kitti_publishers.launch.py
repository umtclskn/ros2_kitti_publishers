#!/usr/bin/python3

from launch import LaunchDescription
from launch.substitutions import EnvironmentVariable
from ament_index_python.packages import get_package_share_directory
import os
import launch_ros.actions
import pathlib

# Set the environment variable TZ: to set timezone for this program at Berlin,
# since KITTI data is recorded in Germany
os.environ.setdefault('TZ', ':Europe/Berlin')

def generate_launch_description():
    config1 = os.path.join(
        get_package_share_directory('ros2_kitti_publishers'),
        'directories.yaml'
        )
    config2 = os.path.join(
        get_package_share_directory('ros2_kitti_publishers'),
        'calibration.yaml'
        )
    return LaunchDescription([
        launch_ros.actions.Node(
            package='ros2_kitti_publishers',
            executable='kitti_publishers',
            output='screen',
            parameters=[
                config1,
                config2
            ],
         ),
    ])