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
2D SLAM with gmapping in Webots. Mirrors ugv_slam/launch/gmapping.launch.py use_sim_time:=true.

    ros2 launch ugv_webots gmapping.launch.py
    ros2 run ugv_tools keyboard_ctrl        # drive around, then save the map:
    ros2 run nav2_map_server map_saver_cli -f ~/map --ros-args -p use_sim_time:=true
"""

import os
import sys

from launch import LaunchDescription

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from _common import include, robot_pose_publisher, sim_arguments, simulation  # noqa: E402


def generate_launch_description():
    return LaunchDescription(sim_arguments() + [
        simulation('slam_2d'),
        robot_pose_publisher(),
        include('slam_gmapping', 'launch', 'mapping.launch.py', args={'use_sim_time': 'true'}),
    ])
