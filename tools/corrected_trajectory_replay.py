#!/usr/bin/env python3
import bisect
import csv
import math
import sys

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rosgraph_msgs.msg import Clock
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster


class CorrectedTrajectoryReplay(Node):
    def __init__(self, csv_path):
        super().__init__('corrected_trajectory_replay')
        self.t, self.rows = [], []
        with open(csv_path, newline='') as stream:
            for row in csv.DictReader(stream):
                self.t.append(float(row['stamp']))
                self.rows.append(tuple(float(row[k]) for k in ('x', 'y', 'yaw', 'v', 'w')))
        self.pub = self.create_publisher(Odometry, '/odom', 20)
        self.tf = TransformBroadcaster(self)
        clock_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.create_subscription(Clock, '/clock', self.clock_cb, clock_qos)

        self.static_tf = StaticTransformBroadcaster(self)
        laser = TransformStamped()
        laser.header.frame_id = 'base_link'
        laser.child_frame_id = 'laser'
        laser.transform.translation.x = 0.27
        laser.transform.translation.z = 0.11
        laser.transform.rotation.w = 1.0
        self.static_tf.sendTransform(laser)

    def clock_cb(self, msg):
        now = msg.clock.sec + msg.clock.nanosec * 1e-9
        i = bisect.bisect_left(self.t, now)
        if i == 0 or i >= len(self.t):
            return
        t0, t1 = self.t[i - 1], self.t[i]
        a = min(1.0, max(0.0, (now - t0) / max(t1 - t0, 1e-9)))
        p0, p1 = self.rows[i - 1], self.rows[i]
        x, y, yaw, v, w = (p0[k] + a * (p1[k] - p0[k]) for k in range(5))
        half = 0.5 * yaw

        odom = Odometry()
        odom.header.stamp = msg.clock
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        odom.pose.pose.position.x = x
        odom.pose.pose.position.y = y
        odom.pose.pose.orientation.z = math.sin(half)
        odom.pose.pose.orientation.w = math.cos(half)
        odom.twist.twist.linear.x = v
        odom.twist.twist.angular.z = w
        self.pub.publish(odom)

        transform = TransformStamped()
        transform.header = odom.header
        transform.child_frame_id = 'base_link'
        transform.transform.translation.x = x
        transform.transform.translation.y = y
        transform.transform.rotation = odom.pose.pose.orientation
        self.tf.sendTransform(transform)


def main():
    if len(sys.argv) < 2:
        raise SystemExit('usage: corrected_trajectory_replay.py TRAJECTORY.csv [--ros-args ...]')
    csv_path = sys.argv[1]
    del sys.argv[1]
    rclpy.init()
    node = CorrectedTrajectoryReplay(csv_path)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()


if __name__ == '__main__':
    main()
