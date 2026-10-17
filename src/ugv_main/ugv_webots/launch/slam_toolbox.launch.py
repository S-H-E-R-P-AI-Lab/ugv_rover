# Copyright 2026 S-H-E-R-P-AI-Lab
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""
2D SLAM with slam_toolbox in Webots.

Mirrors ugv_slam/launch/slam_toolbox.launch.py use_sim_time:=true.

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
        DeclareLaunchArgument('use_slam', default_value='sync',
                              description='slam_toolbox mode: sync or async'),
        simulation('slam_2d'),
        robot_pose_publisher(),
        slam('sync'),
        slam('async'),
    ])
