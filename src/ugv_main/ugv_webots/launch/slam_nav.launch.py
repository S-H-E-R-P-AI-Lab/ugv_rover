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
