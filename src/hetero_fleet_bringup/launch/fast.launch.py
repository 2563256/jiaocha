"""Reduce rendering load while retaining fleet visualization in RViz."""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    share = get_package_share_directory('hetero_fleet_bringup')
    return LaunchDescription([
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('scheduler', default_value='true'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(share, 'launch/bringup.launch.py')),
            launch_arguments={'headless':'true', 'rviz':LaunchConfiguration('rviz'),
                              'scheduler':LaunchConfiguration('scheduler')}.items()),
    ])
