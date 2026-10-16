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
Smoke test of the Webots simulation (launch_testing).

Starts ugv_webots.launch.py headless in the empty arena and checks that
simulation time advances, every topic of the sensor contract publishes, the
TF tree is complete, and the robot drives and turns on /cmd_vel. After
shutdown it checks that no Webots process is left behind.

    colcon test --packages-select ugv_webots --event-handlers console_direct+

The test needs Webots. It is skipped when Webots cannot be found
(WEBOTS_HOME, /usr/local/webots or webots on the PATH). UGV_SIM_TEST=0 skips
it, UGV_SIM_TEST=1 runs it even if Webots was not found. Set
UGV_TEST_ARTIFACTS to a directory to get a JSON summary and the last camera
frames there (CI uploads them when the test fails).
"""

import json
import math
import os
import platform
import shutil
import subprocess
import time
import unittest

from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Twist
import launch
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import launch_testing
import launch_testing.actions
from nav_msgs.msg import Odometry
import pytest
import rclpy
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import CameraInfo, Image, LaserScan, PointCloud2
import tf2_ros
from tf2_msgs.msg import TFMessage
import yaml

# Wall-clock budgets. Generous because CI runners render the cameras on the CPU
# (real-time factor can drop well below 1) and the first start of a world
# downloads its textures.
STARTUP_TIMEOUT = 300.0
TOPIC_TIMEOUT = 120.0
MIN_MESSAGES = 5

# Topic contract shared with ugv_gazebo (docs/SIMULATOR.md): topic -> (type, frame_id or None).
CONTRACT = {
    '/scan': (LaserScan, 'base_lidar_link'),
    '/odom': (Odometry, 'odom'),
    '/tf': (TFMessage, None),
    '/image_raw': (Image, 'pt_camera_link'),
    '/camera_info': (CameraInfo, 'pt_camera_link'),
    '/oak/image_raw': (Image, '3d_camera_link'),
    '/oak/depth/image_raw': (Image, '3d_camera_link'),
    '/oak/points': (PointCloud2, '3d_camera_link'),
}

# Expected parent of each frame (one TF tree rooted at odom).
TF_PARENTS = {
    'base_footprint': 'odom',
    'base_link': 'base_footprint',
    'base_lidar_link': 'base_link',
    'oak_base': 'base_link',
    '3d_camera_link': 'oak_base',
    'pt_base_link': 'base_link',
    'pt_camera_link': 'pt_link2',
}

# Programs that must be gone after shutdown, matched on the executable (or the Python
# script) of each process, never on its whole command line.
LEFTOVER_PROGRAMS = {'webots-bin', 'webots-controller', 'ros2_supervisor.py'}
LEFTOVER_DRIVER = 'webots_ros2_driver/driver'


def _webots_found():
    home = os.environ.get('WEBOTS_HOME') or os.environ.get('ROS2_WEBOTS_HOME')
    if home and os.path.isdir(home):
        return True
    return os.path.exists('/usr/local/webots/webots') or shutil.which('webots') is not None


def _sim_enabled():
    flag = os.environ.get('UGV_SIM_TEST', '')
    if flag in ('0', 'false', 'no'):
        return False
    if flag in ('1', 'true', 'yes'):
        return True
    return _webots_found()


def _is_wsl():
    return 'microsoft-standard' in platform.uname().release


def _yaw(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def _wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


@pytest.mark.launch_test
@pytest.mark.skipif(not _sim_enabled(), reason='Webots not found (set UGV_SIM_TEST=1 to force)')
def generate_test_description():
    launch_file = os.path.join(
        get_package_share_directory('ugv_webots'), 'launch', 'ugv_webots.launch.py')
    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(launch_file),
        launch_arguments={
            'world': 'empty_arena.wbt',
            'headless': 'true',
            'use_rviz': 'false',
        }.items())
    return launch.LaunchDescription([sim, launch_testing.actions.ReadyToTest()])


class TestSimulation(unittest.TestCase):
    """Runs while the simulation is up. Test methods run in name order."""

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node(
            'ugv_sim_smoke_test',
            parameter_overrides=[Parameter('use_sim_time', Parameter.Type.BOOL, True)])
        cls.summary = {'counts': {}, 'rates_per_sim_s': {}, 'frames': {}}
        cls.sim_time = None
        cls.counts = {topic: 0 for topic in CONTRACT}
        cls.last = {}
        cls.subs = [cls.node.create_subscription(Clock, '/clock', cls._on_clock, 10)]
        for topic, (msg_type, _) in CONTRACT.items():
            qos = 100 if topic == '/tf' else qos_profile_sensor_data
            cls.subs.append(cls.node.create_subscription(
                msg_type, topic, lambda msg, t=topic: cls._on_msg(t, msg), qos))
        cls.truth = None
        cls.subs.append(cls.node.create_subscription(
            Odometry, '/ugv_webots/ground_truth',
            lambda msg: setattr(cls, 'truth', msg), qos_profile_sensor_data))
        cls.cmd = cls.node.create_publisher(Twist, '/cmd_vel', 10)
        cls.tf_buffer = tf2_ros.Buffer()
        cls.tf_listener = tf2_ros.TransformListener(cls.tf_buffer, cls.node)

    @classmethod
    def tearDownClass(cls):
        cls._save_artifacts()
        cls.node.destroy_node()
        rclpy.shutdown()

    @classmethod
    def _on_clock(cls, msg):
        cls.sim_time = msg.clock.sec + msg.clock.nanosec * 1e-9

    @classmethod
    def _on_msg(cls, topic, msg):
        cls.counts[topic] += 1
        cls.last[topic] = msg

    def spin_until(self, predicate, timeout, what):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.05)
            if predicate():
                return
        self.fail(f'Timed out after {timeout:.0f} s waiting for {what}')

    def drive(self, linear, angular, sim_seconds):
        """Publish a constant /cmd_vel for sim_seconds of simulation time, then stop."""
        twist = Twist()
        twist.linear.x = linear
        twist.angular.z = angular
        start = self.sim_time
        deadline = time.monotonic() + TOPIC_TIMEOUT
        while self.sim_time - start < sim_seconds:
            self.assertLess(time.monotonic(), deadline, 'simulation time stopped while driving')
            self.cmd.publish(twist)
            rclpy.spin_once(self.node, timeout_sec=0.05)
        self.cmd.publish(Twist())
        # Let the robot come to rest (acceleration limits) before measuring.
        settle = self.sim_time
        while self.sim_time - settle < 1.0:
            self.cmd.publish(Twist())
            rclpy.spin_once(self.node, timeout_sec=0.05)

    def test_1_clock_advances(self):
        self.spin_until(lambda: self.sim_time is not None, STARTUP_TIMEOUT, 'the first /clock')
        first = self.sim_time
        wall = time.monotonic()
        self.spin_until(lambda: self.sim_time - first >= 2.0, TOPIC_TIMEOUT,
                        '/clock to advance 2 s')
        rtf = (self.sim_time - first) / (time.monotonic() - wall)
        type(self).summary['real_time_factor'] = round(rtf, 3)
        print(f'/clock advances, real-time factor about {rtf:.2f}')

    def test_2_topic_contract(self):
        for topic in self.counts:
            self.counts[topic] = 0
        start = self.sim_time
        self.spin_until(lambda: min(self.counts.values()) >= MIN_MESSAGES, TOPIC_TIMEOUT,
                        f'{MIN_MESSAGES} messages on every topic, got {self.counts}')
        elapsed = max(self.sim_time - start, 1e-3)
        for topic, count in self.counts.items():
            self.summary['counts'][topic] = count
            self.summary['rates_per_sim_s'][topic] = round(count / elapsed, 1)
        print('messages per simulated second:', self.summary['rates_per_sim_s'])
        for topic, (_, frame) in CONTRACT.items():
            if frame is None:
                continue
            got = self.last[topic].header.frame_id
            self.summary['frames'][topic] = got
            self.assertEqual(got, frame, f'{topic} frame_id')
        scan = self.last['/scan']
        self.assertGreater(scan.angle_increment, 0.0, '/scan must be counter-clockwise')
        self.assertTrue(any(math.isfinite(r) for r in scan.ranges), '/scan has no finite range')

    def test_3_tf_tree(self):
        sensors = ['base_lidar_link', '3d_camera_link', 'pt_camera_link']
        self.spin_until(
            lambda: all(self.tf_buffer.can_transform('odom', f, rclpy.time.Time())
                        for f in sensors),
            TOPIC_TIMEOUT, 'odom -> sensor frames in TF')
        frames = yaml.safe_load(self.tf_buffer.all_frames_as_yaml()) or {}
        for child, parent in TF_PARENTS.items():
            self.assertIn(child, frames, f'frame {child} missing from TF')
            self.assertEqual(frames[child]['parent'], parent, f'parent of {child}')

    def test_4_drive_forward(self):
        self.spin_until(lambda: '/odom' in self.last, TOPIC_TIMEOUT, '/odom')
        before = self.last['/odom'].pose.pose
        truth_before = self.truth.pose.pose if self.truth else None
        self.drive(0.25, 0.0, 3.0)
        after = self.last['/odom'].pose.pose
        heading = _yaw(before.orientation)
        dx = after.position.x - before.position.x
        dy = after.position.y - before.position.y
        forward = dx * math.cos(heading) + dy * math.sin(heading)
        lateral = -dx * math.sin(heading) + dy * math.cos(heading)
        result = {'commanded_m': 0.75, 'odom_forward_m': round(forward, 3),
                  'odom_lateral_m': round(lateral, 3)}
        if truth_before is not None:
            t_after = self.truth.pose.pose
            result['truth_distance_m'] = round(math.hypot(
                t_after.position.x - truth_before.position.x,
                t_after.position.y - truth_before.position.y), 3)
        self.summary['forward'] = result
        print('forward:', result)
        self.assertGreater(forward, 0.5, 'odom: robot did not move forward enough')
        self.assertLess(forward, 1.0, 'odom: robot moved too far')
        self.assertLess(abs(lateral), 0.15, 'odom: robot drifted sideways')
        if 'truth_distance_m' in result:
            self.assertGreater(result['truth_distance_m'], 0.5, 'ground truth: robot did not move')

    def test_5_rotate(self):
        before = _yaw(self.last['/odom'].pose.pose.orientation)
        truth_before = _yaw(self.truth.pose.pose.orientation) if self.truth else None
        self.drive(0.0, 0.8, 2.0)
        turned = _wrap(_yaw(self.last['/odom'].pose.pose.orientation) - before)
        result = {'commanded_rad': 1.6, 'odom_rad': round(turned, 3)}
        if truth_before is not None:
            result['truth_rad'] = round(
                _wrap(_yaw(self.truth.pose.pose.orientation) - truth_before), 3)
        self.summary['rotation'] = result
        print('rotation:', result)
        self.assertGreater(turned, 0.8, 'odom: robot did not turn enough (counter-clockwise)')
        self.assertLess(turned, 2.4, 'odom: robot turned too far')
        if 'truth_rad' in result:
            self.assertGreater(result['truth_rad'], 0.8, 'ground truth: robot did not turn')

    @classmethod
    def _save_artifacts(cls):
        out = os.environ.get('UGV_TEST_ARTIFACTS')
        if not out:
            return
        os.makedirs(out, exist_ok=True)
        cls.summary['message_counts_at_end'] = dict(cls.counts)
        with open(os.path.join(out, 'sim_smoke_summary.json'), 'w') as f:
            json.dump(cls.summary, f, indent=2)
        try:
            import cv2
            import numpy as np
        except ImportError:
            return
        for topic, name in (('/image_raw', 'pt_camera'), ('/oak/image_raw', 'oak_rgb'),
                            ('/oak/depth/image_raw', 'oak_depth')):
            msg = cls.last.get(topic)
            if msg is None:
                continue
            if msg.encoding == '32FC1':
                depth = np.frombuffer(msg.data, np.float32).reshape(msg.height, msg.width)
                depth = np.nan_to_num(depth, nan=0.0, posinf=0.0, neginf=0.0)
                image = np.clip(depth / 8.0 * 255.0, 0, 255).astype(np.uint8)
            else:
                channels = 4 if msg.encoding in ('bgra8', 'rgba8') else 3
                image = np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.width, channels)
            cv2.imwrite(os.path.join(out, f'{name}.png'), image)


def _program_names(argv):
    names = [os.path.basename(argv[0])]
    if names[0].startswith('python') and len(argv) > 1:
        names.append(os.path.basename(argv[1]))
    return names


def _leftover_processes():
    found = []
    for pid in filter(str.isdigit, os.listdir('/proc')):
        try:
            with open(f'/proc/{pid}/cmdline', 'rb') as f:
                argv = [a.decode(errors='replace') for a in f.read().split(b'\0') if a]
        except OSError:
            continue
        if not argv:
            continue
        if LEFTOVER_PROGRAMS.intersection(_program_names(argv)) or \
                argv[0].endswith(LEFTOVER_DRIVER):
            found.append(f'{pid} {" ".join(argv)[:200]}')
    if _is_wsl():
        # Webots itself runs on Windows; the controllers run here.
        for image in ('webots-bin.exe', 'webots.exe'):
            out = subprocess.run(['tasklist.exe', '/FI', f'IMAGENAME eq {image}'],
                                 capture_output=True, text=True).stdout
            if image in out:
                found.append(f'{image} (Windows)')
    return found


@launch_testing.post_shutdown_test()
class TestShutdown(unittest.TestCase):

    def test_no_leftover_webots_processes(self):
        deadline = time.monotonic() + 30.0
        leftovers = _leftover_processes()
        while leftovers and time.monotonic() < deadline:
            time.sleep(1.0)
            leftovers = _leftover_processes()
        self.assertEqual(leftovers, [], 'processes still running after shutdown')
