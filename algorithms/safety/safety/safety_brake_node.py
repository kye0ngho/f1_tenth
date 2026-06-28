import math

import rclpy
from rclpy.node import Node

from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool


class SafetyBrakeNode(Node):
    def __init__(self):
        super().__init__('safety_brake_node')

        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('odom_topic', '/ego_racecar/odom')
        self.declare_parameter('drive_input_topic', '/emergency/drive')
        self.declare_parameter('drive_output_topic', '/drive')
        self.declare_parameter('brake_status_topic', '/safety/braking')
        self.declare_parameter('watchdog_alert_topic', '/watchdog/alert')

        self.declare_parameter('min_ttc', 0.5)
        self.declare_parameter('brake_distance', 0.3)
        self.declare_parameter('scan_field_deg', 60.0)

        self.scan_topic = self.get_parameter('scan_topic').value
        self.odom_topic = self.get_parameter('odom_topic').value
        self.drive_input_topic = self.get_parameter('drive_input_topic').value
        self.drive_output_topic = self.get_parameter('drive_output_topic').value
        self.brake_status_topic = self.get_parameter('brake_status_topic').value
        watchdog_alert_topic = self.get_parameter('watchdog_alert_topic').value

        self.min_ttc = float(self.get_parameter('min_ttc').value)
        self.brake_distance = float(self.get_parameter('brake_distance').value)
        self.scan_field_deg = float(self.get_parameter('scan_field_deg').value)

        self.current_drive = None
        self.current_speed = 0.0
        self.braking = False
        self.watchdog_alert = False

        self.create_subscription(LaserScan, self.scan_topic, self.scan_callback, 10)
        self.create_subscription(Odometry, self.odom_topic, self.odom_callback, 10)
        self.create_subscription(
            AckermannDriveStamped, self.drive_input_topic, self.drive_callback, 10
        )
        self.create_subscription(Bool, watchdog_alert_topic, self._cb_watchdog, 10)

        self.drive_pub = self.create_publisher(
            AckermannDriveStamped, self.drive_output_topic, 10
        )
        self.brake_status_pub = self.create_publisher(Bool, self.brake_status_topic, 10)

        self.get_logger().info('safety_brake_node started')
        self.get_logger().info(f'  scan_topic      : {self.scan_topic}')
        self.get_logger().info(f'  drive_input     : {self.drive_input_topic}')
        self.get_logger().info(f'  drive_output    : {self.drive_output_topic}')
        self.get_logger().info(f'  watchdog_alert  : {watchdog_alert_topic}')
        self.get_logger().info(f'  min_ttc         : {self.min_ttc} s')
        self.get_logger().info(f'  brake_distance  : {self.brake_distance} m')
        self.get_logger().info(f'  scan_field      : ±{self.scan_field_deg / 2:.0f}°')

    def odom_callback(self, msg):
        self.current_speed = msg.twist.twist.linear.x

    def drive_callback(self, msg):
        self.current_drive = msg

    def _cb_watchdog(self, msg):
        if msg.data and not self.watchdog_alert:
            self.get_logger().warn('[WATCHDOG] 비상정지 신호 수신')
        self.watchdog_alert = msg.data

    def scan_callback(self, msg):
        # watchdog estop이 최우선
        if self.watchdog_alert:
            self._publish_stop()
            status = Bool(); status.data = True
            self.brake_status_pub.publish(status)
            return

        min_range = self._min_front_range(msg)
        danger, reason = self._check_danger(min_range)

        if danger and not self.braking:
            self.braking = True
            self.get_logger().warn(f'[BRAKE] {reason}')
        elif not danger and self.braking:
            self.braking = False
            self.get_logger().info('[CLEAR] 장애물 해제 — 주행 재개')

        status = Bool()
        status.data = self.braking
        self.brake_status_pub.publish(status)

        if self.braking:
            self._publish_stop()
        elif self.current_drive is not None:
            self.drive_pub.publish(self.current_drive)

    def _check_danger(self, min_range):
        if min_range <= self.brake_distance:
            return True, f'근접 장애물: {min_range:.2f} m ≤ {self.brake_distance} m'

        if abs(self.current_speed) > 0.05:
            ttc = min_range / abs(self.current_speed)
            if ttc < self.min_ttc:
                return True, (
                    f'TTC: {ttc:.2f} s < {self.min_ttc} s '
                    f'(dist={min_range:.2f} m, spd={self.current_speed:.2f} m/s)'
                )

        return False, ''

    def _min_front_range(self, scan):
        half = math.radians(self.scan_field_deg / 2.0)
        min_r = float('inf')
        angle = scan.angle_min

        for r in scan.ranges:
            if -half <= angle <= half:
                if scan.range_min <= r <= scan.range_max:
                    min_r = min(min_r, r)
            angle += scan.angle_increment

        return min_r if min_r != float('inf') else scan.range_max

    def _publish_stop(self):
        msg = AckermannDriveStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.drive.speed = 0.0
        msg.drive.steering_angle = 0.0
        self.drive_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = SafetyBrakeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
