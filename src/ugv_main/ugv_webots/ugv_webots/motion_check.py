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
Drive a fixed pattern and compare commanded motion, /odom and ground truth.

Usage (with the simulation running):
    ros2 run ugv_webots motion_check
    ros2 run ugv_webots motion_check --ros-args -p distance:=2.0 -p angular_speed:=1.0

The node sends open-loop /cmd_vel commands: ``distance`` metres forward at
``linear_speed``, then ``turns`` full turns in place at ``angular_speed``.
Durations are measured in simulation time. Because diff_drive_controller
limits acceleration symmetrically, a command held for d/v seconds moves the
robot d metres in total (the distance lost while speeding up is regained while
slowing down after the stop command).
"""

import math

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter


def _yaw(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def _wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


class PoseTracker:
    """Track position and unwrapped yaw from an Odometry stream."""

    def __init__(self):
        self.x = self.y = None
        self.yaw_unwrapped = 0.0
        self._last_yaw = None

    def update(self, msg):
        p = msg.pose.pose
        self.x, self.y = p.position.x, p.position.y
        yaw = _yaw(p.orientation)
        if self._last_yaw is not None:
            self.yaw_unwrapped += _wrap(yaw - self._last_yaw)
        self._last_yaw = yaw

    @property
    def ready(self):
        return self.x is not None

    def snapshot(self):
        return (self.x, self.y, self.yaw_unwrapped, self._last_yaw)


class MotionCheck(Node):

    def __init__(self):
        super().__init__('motion_check', parameter_overrides=[
            Parameter('use_sim_time', Parameter.Type.BOOL, True)])
        self.distance = self.declare_parameter('distance', 1.0).value
        self.linear_speed = self.declare_parameter('linear_speed', 0.2).value
        self.turns = self.declare_parameter('turns', 1.0).value
        self.angular_speed = self.declare_parameter('angular_speed', 0.6).value
        self.settle = self.declare_parameter('settle_time', 1.5).value

        self.odom = PoseTracker()
        self.truth = PoseTracker()
        self.create_subscription(Odometry, '/odom', self.odom.update, 20)
        self.create_subscription(Odometry, '/ugv_webots/ground_truth', self.truth.update, 20)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

    def now_s(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def spin_for(self, seconds, cmd=None):
        end = self.now_s() + seconds
        while rclpy.ok() and self.now_s() < end:
            if cmd is not None:
                self.cmd_pub.publish(cmd)
            rclpy.spin_once(self, timeout_sec=0.05)

    def wait_ready(self):
        self.get_logger().info('Waiting for /odom, /ugv_webots/ground_truth and /clock ...')
        while rclpy.ok() and not (self.odom.ready and self.truth.ready and self.now_s() > 0.0):
            rclpy.spin_once(self, timeout_sec=0.1)

    def run(self):
        self.wait_ready()
        stop = Twist()
        self.spin_for(self.settle, stop)

        # Forward leg
        o0, t0 = self.odom.snapshot(), self.truth.snapshot()
        cmd = Twist()
        cmd.linear.x = math.copysign(self.linear_speed, self.distance)
        self.get_logger().info(f'Driving {self.distance:.3f} m at {cmd.linear.x:.2f} m/s')
        self.spin_for(abs(self.distance / self.linear_speed), cmd)
        self.spin_for(self.settle, stop)
        o1, t1 = self.odom.snapshot(), self.truth.snapshot()

        # Rotation leg
        cmd = Twist()
        cmd.angular.z = self.angular_speed
        angle = 2.0 * math.pi * self.turns
        self.get_logger().info(
            f'Rotating {math.degrees(angle):.1f} deg at {self.angular_speed:.2f} rad/s')
        self.spin_for(angle / self.angular_speed, cmd)
        self.spin_for(self.settle, stop)
        o2, t2 = self.odom.snapshot(), self.truth.snapshot()

        def leg(a, b):
            along = (b[0] - a[0]) * math.cos(a[3]) + (b[1] - a[1]) * math.sin(a[3])
            lateral = -(b[0] - a[0]) * math.sin(a[3]) + (b[1] - a[1]) * math.cos(a[3])
            return along, lateral, math.degrees(b[2] - a[2])

        od, ol, oyaw1 = leg(o0, o1)
        td, tl, tyaw1 = leg(t0, t1)
        _, _, oyaw = leg(o1, o2)
        _, _, tyaw = leg(t1, t2)
        cmd_deg = math.degrees(angle)

        lines = [
            '',
            'Leg               commanded      /odom   ground truth   odom err   truth err',
            f'forward [m]       {self.distance:9.3f}  {od:9.3f}  {td:13.3f}  '
            f'{od - self.distance:+9.3f}  '
            f'{td - self.distance:+10.3f}',
            f'lateral [m]       {0.0:9.3f}  {ol:9.3f}  {tl:13.3f}',
            f'heading drift [deg] {0.0:7.2f}  {oyaw1:9.2f}  {tyaw1:13.2f}',
            f'rotation [deg]    {cmd_deg:9.1f}  {oyaw:9.1f}  {tyaw:13.1f}  '
            f'{oyaw - cmd_deg:+9.1f}  '
            f'{tyaw - cmd_deg:+10.1f}',
            f'odom vs truth: forward {od - td:+.3f} m, rotation {oyaw - tyaw:+.1f} deg',
        ]
        for line in lines:
            self.get_logger().info(line)


def main(args=None):
    rclpy.init(args=args)
    node = MotionCheck()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.cmd_pub.publish(Twist())
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
