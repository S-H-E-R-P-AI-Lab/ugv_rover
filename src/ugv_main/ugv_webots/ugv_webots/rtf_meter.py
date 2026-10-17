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
Measure the real-time factor of a running simulation from /clock.

Usage:
    ros2 run ugv_webots rtf_meter              # 20 s window
    ros2 run ugv_webots rtf_meter --ros-args -p window:=60.0

Works with any simulator that publishes /clock (Webots or Gazebo).
"""

import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rosgraph_msgs.msg import Clock


class RtfMeter(Node):

    def __init__(self):
        super().__init__('rtf_meter')
        self.window = self.declare_parameter('window', 20.0).value
        self.samples = []
        self.create_subscription(Clock, '/clock', self._on_clock, qos_profile_sensor_data)

    def _on_clock(self, msg):
        self.samples.append((time.monotonic(), msg.clock.sec + msg.clock.nanosec * 1e-9))


def main(args=None):
    rclpy.init(args=args)
    node = RtfMeter()
    node.get_logger().info(f'Measuring real-time factor over {node.window:.0f} s of wall time ...')
    start = time.monotonic()
    try:
        while rclpy.ok() and time.monotonic() - start < node.window:
            rclpy.spin_once(node, timeout_sec=0.1)
            if time.monotonic() - start > 5.0 and not node.samples:
                node.get_logger().error('No /clock messages received. Is the simulation running?')
                break
    except KeyboardInterrupt:
        pass
    if len(node.samples) > 1:
        (w0, s0), (w1, s1) = node.samples[0], node.samples[-1]
        rate = (len(node.samples) - 1) / (w1 - w0)
        node.get_logger().info(
            f'sim time {s1 - s0:.2f} s in wall time {w1 - w0:.2f} s: '
            f'real-time factor {(s1 - s0) / (w1 - w0):.3f} '
            f'(/clock at {rate:.0f} Hz)')
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    main()
