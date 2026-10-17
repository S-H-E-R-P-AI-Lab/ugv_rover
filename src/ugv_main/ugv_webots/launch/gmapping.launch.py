"""2D SLAM with gmapping in Webots. Mirrors ugv_slam/launch/gmapping.launch.py use_sim_time:=true.

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
