"""
SLAM and Nav2 together in Webots (nav.launch.py with use_slam:=true).

    ros2 launch ugv_webots slam_nav.launch.py
"""

import importlib.util
import os


def generate_launch_description():
    spec = importlib.util.spec_from_file_location(
        'ugv_webots_nav',
        os.path.join(os.path.dirname(os.path.abspath(__file__)), 'nav.launch.py'))
    nav = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(nav)
    return nav.generate_launch_description(default_use_slam='true')
