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
Start the UGV Rover in Webots.

    ros2 launch ugv_webots ugv_webots.launch.py
    ros2 launch ugv_webots ugv_webots.launch.py world:=empty_arena.wbt use_rviz:=true
    ros2 launch ugv_webots ugv_webots.launch.py headless:=true

Starts Webots (on Windows when run from WSL, on the Mac when run from the
macOS VM), the Webots driver with ros2_control, robot_state_publisher, the
controllers, the scan normalizer and the OAK point cloud. The topics and frames
match ugv_gazebo (see docs/SIMULATOR.md).
"""

import os
import re
import subprocess
import tempfile

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, EmitEvent, LogInfo, OpaqueFunction,
                            RegisterEventHandler)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit, OnShutdown
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from webots_ros2_driver.wait_for_controller_connection import WaitForControllerConnection
from webots_ros2_driver.webots_controller import WebotsController
from webots_ros2_driver.webots_launcher import WebotsLauncher

from ugv_webots.description import webots_urdf
from ugv_webots.webots_host import patch_webots_ros2

ROBOT_NAME = 'ugv_rover'
RVIZ_CONFIGS = ['webots', 'description', 'bringup', 'slam_2d', 'slam_3d', 'nav_2d', 'nav_3d']


def rviz_config_path(name):
    """Return the RViz config path (ugv_gazebo choices plus 'webots')."""
    if name == 'webots':
        return os.path.join(get_package_share_directory('ugv_webots'), 'rviz', 'view_webots.rviz')
    choices = {
        'description': ('ugv_description', 'view_description.rviz'),
        'bringup': ('ugv_bringup', 'view_bringup.rviz'),
        'slam_2d': ('ugv_slam', 'view_slam_2d.rviz'),
        'slam_3d': ('ugv_slam', 'view_slam_3d.rviz'),
        'nav_2d': ('ugv_nav', 'view_nav_2d.rviz'),
        'nav_3d': ('ugv_nav', 'view_nav_3d.rviz'),
    }
    package, filename = choices[name]
    return os.path.join(get_package_share_directory(package), 'rviz', filename)


def stop_driver(urdf_file):
    """Send SIGTERM to the driver of this launch (matched by its unique URDF path)."""
    pattern = 'webots_ros2_driver/driver .*' + re.escape(urdf_file)
    subprocess.run(['pkill', '-TERM', '-f', pattern], check=False)
    # The driver has read the URDF by then; do not leave one file per launch in /tmp.
    if os.path.exists(urdf_file):
        os.remove(urdf_file)
    return []


def launch_setup(context, *args, **kwargs):
    package_dir = get_package_share_directory('ugv_webots')
    model = os.environ.get('UGV_MODEL', ROBOT_NAME)
    actions = []
    if model != ROBOT_NAME:
        actions.append(LogInfo(
            msg=f'UGV_MODEL={model}, but ugv_webots only models ugv_rover. Using ugv_rover.'))

    host = patch_webots_ros2()
    if host:
        actions.append(LogInfo(msg=f'Robot controller will connect to Webots at {host}:1234'))

    # The driver reads the robot description from a file.
    urdf = webots_urdf(ROBOT_NAME)
    urdf_file = os.path.join(tempfile.gettempdir(), f'ugv_webots_{os.getpid()}.urdf')
    with open(urdf_file, 'w') as f:
        f.write(urdf)

    headless = LaunchConfiguration('headless')
    webots = WebotsLauncher(
        world=PathJoinSubstitution([package_dir, 'worlds', LaunchConfiguration('world')]),
        mode=LaunchConfiguration('mode'),
        gui=PythonExpression(
            ["'false' if '", headless, "'.lower() in ('true', '1', 'yes') else 'true'"]),
        # The Ros2Supervisor publishes /clock (webots_ros2 2025.0.0); without it sim time
        # stays at 0 and ros2_control never updates.
        ros2_supervisor=True,
    )

    driver = WebotsController(
        robot_name=ROBOT_NAME,
        parameters=[
            {'robot_description': urdf_file, 'use_sim_time': True,
             'set_robot_state_publisher': False},
            os.path.join(package_dir, 'resource', 'ros2_control.yaml'),
        ],
        remappings=[
            ('/diffdrive_controller/cmd_vel_unstamped', '/cmd_vel'),
            ('/diffdrive_controller/odom', '/odom'),
            ('/pt_camera/image_raw', '/image_raw'),
            ('/pt_camera/camera_info', '/camera_info'),
        ],
        respawn=False,
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': urdf, 'use_sim_time': True}],
    )

    spawner = Node(
        package='controller_manager',
        executable='spawner',
        output='screen',
        arguments=['joint_state_broadcaster', 'diffdrive_controller',
                   'pt_joint_position_controller',
                   '--controller-manager', '/controller_manager',
                   '--controller-manager-timeout', '60'],
        parameters=[{'use_sim_time': True}],
    )

    scan_normalizer = Node(
        package='ugv_webots',
        executable='scan_normalizer',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )

    oak_points = Node(
        package='depth_image_proc',
        executable='point_cloud_xyz_node',
        name='oak_point_cloud',
        output='screen',
        parameters=[{'use_sim_time': True}],
        remappings=[
            ('image_rect', '/oak/depth/image_raw'),
            ('camera_info', '/oak/depth/camera_info'),
            ('points', '/oak/points'),
        ],
        condition=IfCondition(LaunchConfiguration('use_oak_points')),
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_path(context.launch_configurations['rviz_config'])],
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(LaunchConfiguration('use_rviz')),
    )

    actions += [
        webots,
        webots._supervisor,
        robot_state_publisher,
        driver,
        WaitForControllerConnection(target_driver=driver, nodes_to_start=[spawner]),
        scan_normalizer,
        oak_points,
        rviz,
        # Stop everything when Webots is closed.
        RegisterEventHandler(
            OnProcessExit(target_action=webots, on_exit=[EmitEvent(event=Shutdown())])),
        # webots-controller starts the driver as a child and does not pass signals on, so
        # without this the driver outlives any shutdown that does not come from Ctrl+C in a
        # terminal (closing Webots, launch_testing, kill -INT), spinning on a lost connection.
        RegisterEventHandler(OnShutdown(on_shutdown=[
            OpaqueFunction(function=lambda context: stop_driver(urdf_file))])),
    ]
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('world', default_value='apartment.wbt',
                              description='World file in ugv_webots/worlds: '
                                          'apartment.wbt or empty_arena.wbt'),
        DeclareLaunchArgument('mode', default_value='realtime',
                              description='Webots start mode: realtime, fast or pause'),
        DeclareLaunchArgument('headless', default_value='false',
                              description='Start Webots minimized without rendering the 3D view'),
        DeclareLaunchArgument('use_rviz', default_value='false',
                              description='Whether to launch RViz2'),
        DeclareLaunchArgument('rviz_config', default_value='webots', choices=RVIZ_CONFIGS,
                              description='RViz config: webots (all sensors) or an upstream one'),
        DeclareLaunchArgument('use_oak_points', default_value='true',
                              description='Publish /oak/points from the simulated OAK '
                                          'depth image'),
        OpaqueFunction(function=launch_setup),
    ])
