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
Nav2 in Webots. Mirrors ugv_nav/launch/nav.launch.py use_sim_time:=true.

    ros2 launch ugv_webots nav.launch.py                       # AMCL on maps/apartment.yaml
    ros2 launch ugv_webots nav.launch.py map:=$HOME/map.yaml   # your own map
    ros2 launch ugv_webots nav.launch.py use_slam:=true        # SLAM and navigation together

Uses ugv_nav's nav2_bringup.launch.py and parameter files unchanged.
"""

import os
import sys

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from _common import include, robot_pose_publisher, share, sim_arguments, simulation  # noqa: E402

LOCAL_PLANNERS = {'dwa': 'dwa.yaml', 'teb': 'teb.yaml', 'rpp': 'rpp.yaml', 'mppi': 'mppi.yaml'}


def launch_setup(context, *args, **kwargs):
    use_localplan = context.launch_configurations['use_localplan']
    params = share('ugv_nav', 'params', LOCAL_PLANNERS.get(use_localplan, 'dwa.yaml'))
    rviz = 'nav_3d' if context.launch_configurations['use_localization'] == 'rtabmap' else 'nav_2d'
    return [
        simulation(rviz),
        robot_pose_publisher(),
        include('ugv_nav', 'launch', 'nav_bringup', 'nav2_bringup.launch.py', args={
            'map': LaunchConfiguration('map'),
            'pbstream': LaunchConfiguration('pbstream'),
            'posegraph': LaunchConfiguration('posegraph'),
            'params_file': params,
            'slam_toolbox_params_file': share('ugv_nav', 'params',
                                              'slam_toolbox_localization.yaml'),
            'use_sim_time': 'true',
            'use_localization': LaunchConfiguration('use_localization'),
            'use_slam': LaunchConfiguration('use_slam'),
            'use_keepout_zones': LaunchConfiguration('use_keepout_zones'),
            'keepout_mask': LaunchConfiguration('keepout_mask'),
        }),
    ]


def generate_launch_description(default_use_slam='false'):
    maps = share('ugv_webots', 'maps')
    return LaunchDescription(sim_arguments() + [
        DeclareLaunchArgument('use_slam', default_value=default_use_slam,
                              description='Run SLAM instead of a saved map'),
        DeclareLaunchArgument('use_localplan', default_value='dwa',
                              description='Local planner: dwa, teb, rpp, mppi'),
        DeclareLaunchArgument('use_localization', default_value='amcl',
                              description='Localization: amcl, emcl, cartographer, slam_toolbox'),
        DeclareLaunchArgument('map', default_value=os.path.join(maps, 'apartment.yaml'),
                              description='Map yaml (default: the saved apartment map)'),
        DeclareLaunchArgument('pbstream', default_value=os.path.join(maps, 'apartment.pbstream'),
                              description='Cartographer state for use_localization:=cartographer'),
        DeclareLaunchArgument('posegraph', default_value=os.path.join(maps, 'apartment'),
                              description='slam_toolbox pose graph for '
                                          'use_localization:=slam_toolbox'),
        DeclareLaunchArgument('use_keepout_zones', default_value='false',
                              description='Enable Nav2 keepout zones'),
        DeclareLaunchArgument('keepout_mask', default_value=share('ugv_nav', 'maps', 'mask.yaml'),
                              description='Keepout mask yaml'),
        OpaqueFunction(function=launch_setup),
    ])
