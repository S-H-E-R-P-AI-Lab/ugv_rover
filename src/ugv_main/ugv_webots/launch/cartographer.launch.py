"""2D SLAM with Cartographer in Webots. Mirrors ugv_slam/launch/cartographer.launch.py use_sim_time:=true.

    ros2 launch ugv_webots cartographer.launch.py
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
        include('cartographer', 'launch', 'mapping.launch.py', args={'use_sim_time': 'true'}),
    ])
