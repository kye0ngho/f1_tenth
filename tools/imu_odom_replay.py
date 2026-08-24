#!/usr/bin/env python3
import math

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import Imu
from tf2_ros import TransformBroadcaster
from vesc_msgs.msg import VescStateStamped


class ImuOdomReplay(Node):
    def __init__(self):
        super().__init__('imu_odom_replay')
        self.declare_parameter('speed_to_erpm_gain', 4204.3)
        self.declare_parameter('speed_to_erpm_offset', 0.0)
        self.declare_parameter('gyro_scale', math.pi / 180.0)
        self.gain = float(self.get_parameter('speed_to_erpm_gain').value)
        self.offset = float(self.get_parameter('speed_to_erpm_offset').value)
        self.gyro_scale = float(self.get_parameter('gyro_scale').value)

        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.last_t = None
        self.latest_gyro = 0.0
        self.bias_sum = 0.0
        self.bias_count = 0
        self.speed = 0.0

        self.pub = self.create_publisher(Odometry, '/odom', 20)
        self.tf = TransformBroadcaster(self)
        self.create_subscription(Imu, '/sensors/imu/raw', self.imu_cb, 50)
        self.create_subscription(VescStateStamped, '/sensors/core', self.core_cb, 50)

    @property
    def gyro_bias(self):
        return self.bias_sum / self.bias_count if self.bias_count else 0.0

    def imu_cb(self, msg):
        raw = float(msg.angular_velocity.z)
        self.latest_gyro = raw
        if abs(self.speed) < 0.03:
            self.bias_sum += raw
            self.bias_count += 1

    def core_cb(self, msg):
        stamp = msg.header.stamp
        t = stamp.sec + stamp.nanosec * 1e-9
        speed = (float(msg.state.speed) - self.offset) / self.gain
        yaw_rate = (self.latest_gyro - self.gyro_bias) * self.gyro_scale

        if self.last_t is not None:
            dt = t - self.last_t
            if 0.0 < dt < 0.2:
                mid_yaw = self.yaw + 0.5 * yaw_rate * dt
                self.x += speed * math.cos(mid_yaw) * dt
                self.y += speed * math.sin(mid_yaw) * dt
                self.yaw += yaw_rate * dt
        self.last_t = t
        self.speed = speed

        half = 0.5 * self.yaw
        qz, qw = math.sin(half), math.cos(half)

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        odom.twist.twist.linear.x = speed
        odom.twist.twist.angular.z = yaw_rate
        self.pub.publish(odom)

        tf = TransformStamped()
        tf.header = odom.header
        tf.child_frame_id = 'base_link'
        tf.transform.translation.x = self.x
        tf.transform.translation.y = self.y
        tf.transform.rotation.z = qz
        tf.transform.rotation.w = qw
        self.tf.sendTransform(tf)


def main():
    rclpy.init()
    node = ImuOdomReplay()
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
