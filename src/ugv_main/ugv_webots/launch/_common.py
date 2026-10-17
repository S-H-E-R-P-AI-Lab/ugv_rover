"""Shared pieces of the ugv_webots SLAM and navigation wrappers (not a launch file)."""

import os

from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def share(package, *path):
    return os.path.join(get_package_share_directory(package), *path)


def include(package, *path, args=None, **kwargs):
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(share(package, *path)),
        launch_arguments=(args or {}).items(), **kwargs)


def sim_arguments(default_world='apartment.wbt'):
    return [
        DeclareLaunchArgument('world', default_value=default_world,
                              description='World file in ugv_webots/worlds'),
        DeclareLaunchArgument('use_rviz', default_value='true', description='Whether to launch RViz2'),
        DeclareLaunchArgument('headless', default_value='false',
                              description='Start Webots minimized without rendering the 3D view'),
    ]


def simulation(rviz_config):
    """The Webots simulation, in place of ugv_bringup/bringup_lidar.launch.py on the robot."""
    return include('ugv_webots', 'launch', 'ugv_webots.launch.py', args={
        'world': LaunchConfiguration('world'),
        'use_rviz': LaunchConfiguration('use_rviz'),
        'rviz_config': rviz_config,
        'headless': LaunchConfiguration('headless'),
    })


def robot_pose_publisher():
    return include('robot_pose_publisher', 'launch', 'robot_pose_publisher_launch.py',
                   args={'use_sim_time': 'true'})
