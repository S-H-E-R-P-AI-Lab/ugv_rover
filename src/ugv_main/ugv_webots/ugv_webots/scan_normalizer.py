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
Republish the Webots lidar scan in the layout of the real LD19 driver.

webots_ros2's Lidar plugin publishes a clockwise scan: angle_min = +fov/2,
angle_max = -fov/2 and a negative angle_increment. Some consumers (for example
slam_toolbox) do not accept a negative increment.

The ldlidar driver on the robot (src/ugv_else/ldlidar) publishes a
counter-clockwise scan from 0 to 2*pi in base_lidar_link, so this node
reorders the Webots scan into that layout and publishes it on /scan.
"""

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import LaserScan


class ScanNormalizer(Node):

    def __init__(self):
        super().__init__('scan_normalizer')
        self.declare_parameter('input_topic', '/ugv_webots/scan_raw')
        self.declare_parameter('output_topic', '/scan')
        in_topic = self.get_parameter('input_topic').value
        out_topic = self.get_parameter('output_topic').value

        # A reliable publisher matches both reliable and best effort subscribers.
        self._pub = self.create_publisher(
            LaserScan, out_topic, QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE))
        self._sub = self.create_subscription(
            LaserScan, in_topic, self._on_scan, qos_profile_sensor_data)

    def _on_scan(self, msg):
        n = len(msg.ranges)
        if n == 0:
            return
        full_circle = abs(abs(msg.angle_max - msg.angle_min) - 2.0 * math.pi) < 0.1
        out = LaserScan()
        out.header = msg.header
        out.range_min = msg.range_min
        out.range_max = msg.range_max
        out.scan_time = msg.scan_time
        out.time_increment = abs(msg.time_increment)

        if full_circle:
            # Webots spreads a 360 degree scan over n distinct rays, ray k pointing at
            # +pi - k * 2pi/n. Output ray j points at j * 2pi/n, i.e. input ray (n/2 - j) mod n.
            step = 2.0 * math.pi / n
            half = n // 2
            out.angle_min = 0.0
            out.angle_increment = step
            out.angle_max = step * (n - 1)
            out.ranges = [msg.ranges[(half - j) % n] for j in range(n)]
        else:
            # Partial field of view: only reverse the order to get a positive increment.
            out.angle_min = min(msg.angle_min, msg.angle_max)
            out.angle_max = max(msg.angle_min, msg.angle_max)
            out.angle_increment = abs(msg.angle_increment)
            out.ranges = list(reversed(msg.ranges))
        self._pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = ScanNormalizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
