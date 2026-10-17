"""2D SLAM with slam_toolbox in Webots. Mirrors ugv_slam/launch/slam_toolbox.launch.py use_sim_time:=true.

    ros2 launch ugv_webots slam_toolbox.launch.py use_slam:=sync
"""

import os
import sys

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import LaunchConfigurationEquals

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from _common import include, robot_pose_publisher, share, sim_arguments, simulation  # noqa: E402


def generate_launch_description():
    def slam(mode):
        return include('slam_toolbox', 'launch', f'online_{mode}_launch.py', args={
            'use_sim_time': 'true',
            'slam_params_file': share('ugv_slam', 'config', f'slam_toolbox_online_{mode}.yaml'),
        }, condition=LaunchConfigurationEquals('use_slam', mode))

    return LaunchDescription(sim_arguments() + [
        DeclareLaunchArgument('use_slam', default_value='sync', description='slam_toolbox mode: sync or async'),
        simulation('slam_2d'),
        robot_pose_publisher(),
        slam('sync'),
        slam('async'),
    ])
