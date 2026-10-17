"""webots_ros2 Python plugin publishing the true pose of the robot.

Loaded by the Webots driver from resource/webots_extras.urdf. It publishes a
nav_msgs/Odometry on /ugv_webots/ground_truth with frame_id "world" (the Webots
world origin) and child_frame_id "base_footprint" (the twist is in base_footprint,
as REP 105 defines it). It does not publish TF, so
it never conflicts with the odometry or SLAM transforms.

Use it to measure odometry drift and SLAM accuracy in simulation
(see ``ros2 run ugv_webots motion_check``). The real robot has no equivalent.
"""

import math

from nav_msgs.msg import Odometry
import rclpy
from rclpy.time import Time


def _quaternion_from_matrix(m):
    # m is a row-major 3x3 rotation matrix as a flat list of 9 values.
    r00, r01, r02, r10, r11, r12, r20, r21, r22 = m
    trace = r00 + r11 + r22
    if trace > 0.0:
        s = 0.5 / math.sqrt(trace + 1.0)
        return ((r21 - r12) * s, (r02 - r20) * s, (r10 - r01) * s, 0.25 / s)
    if r00 > r11 and r00 > r22:
        s = 2.0 * math.sqrt(1.0 + r00 - r11 - r22)
        return (0.25 * s, (r01 + r10) / s, (r02 + r20) / s, (r21 - r12) / s)
    if r11 > r22:
        s = 2.0 * math.sqrt(1.0 + r11 - r00 - r22)
        return ((r01 + r10) / s, 0.25 * s, (r12 + r21) / s, (r02 - r20) / s)
    s = 2.0 * math.sqrt(1.0 + r22 - r00 - r11)
    return ((r02 + r20) / s, (r12 + r21) / s, 0.25 * s, (r10 - r01) / s)


class GroundTruthPublisher:

    def init(self, webots_node, properties):
        self._robot = webots_node.robot
        self._self_node = self._robot.getSelf()
        if self._self_node is None:
            raise RuntimeError('GroundTruthPublisher needs "supervisor TRUE" on the robot')
        rate = float(properties.get('updateRate', '20'))
        self._period = 1.0 / rate
        self._last = -1.0

        if not rclpy.ok():
            rclpy.init(args=None)
        self._node = rclpy.create_node('ugv_webots_ground_truth')
        self._pub = self._node.create_publisher(
            Odometry, properties.get('topicName', '/ugv_webots/ground_truth'), 10)
        self._msg = Odometry()
        self._msg.header.frame_id = 'world'
        self._msg.child_frame_id = 'base_footprint'

    def step(self):
        t = self._robot.getTime()
        if t - self._last < self._period - 1e-6:
            return
        self._last = t

        position = self._self_node.getPosition()
        rotation = self._self_node.getOrientation()  # row-major, body to world
        qx, qy, qz, qw = _quaternion_from_matrix(rotation)
        # Webots gives world frame velocities; Odometry wants them in child_frame_id.
        world = self._self_node.getVelocity()  # [vx, vy, vz, wx, wy, wz]
        velocity = [sum(rotation[3 * k + i] * world[offset + k] for k in range(3))
                    for offset in (0, 3) for i in range(3)]

        msg = self._msg
        msg.header.stamp = Time(seconds=t).to_msg()
        msg.pose.pose.position.x = position[0]
        msg.pose.pose.position.y = position[1]
        msg.pose.pose.position.z = position[2]
        msg.pose.pose.orientation.x = qx
        msg.pose.pose.orientation.y = qy
        msg.pose.pose.orientation.z = qz
        msg.pose.pose.orientation.w = qw
        msg.twist.twist.linear.x = velocity[0]
        msg.twist.twist.linear.y = velocity[1]
        msg.twist.twist.linear.z = velocity[2]
        msg.twist.twist.angular.x = velocity[3]
        msg.twist.twist.angular.y = velocity[4]
        msg.twist.twist.angular.z = velocity[5]
        self._pub.publish(msg)
