from nav_msgs.msg import Odometry, Path

import rclpy
from rclpy.node import Node

from ackermann_msgs.msg import AckermannDriveStamped
from std_msgs.msg import Bool, String


class WatchdogNode(Node):
    """
    토픽 헬스 모니터.
    감시 대상 토픽이 timeout 안에 메시지를 수신하지 못하면
    /drive에 비상정지 커맨드를 발행하고 /watchdog/alert를 True로 발행.
    """

    def __init__(self):
        super().__init__('watchdog_node')

        self.declare_parameter('drive_topic', '/drive')
        self.declare_parameter('alert_topic', '/watchdog/alert')
        self.declare_parameter('status_topic', '/watchdog/status')
        self.declare_parameter('check_rate', 10.0)
        self.declare_parameter('odom_timeout', 1.0)
        self.declare_parameter('path_timeout', 3.0)
        self.declare_parameter('drive_timeout', 1.0)

        drive_topic = self.get_parameter('drive_topic').value
        alert_topic = self.get_parameter('alert_topic').value
        status_topic = self.get_parameter('status_topic').value
        check_rate = float(self.get_parameter('check_rate').value)

        self.watched = {
            '/localization/odom': float(self.get_parameter('odom_timeout').value),
            '/planning/path':     float(self.get_parameter('path_timeout').value),
            '/control/drive':     float(self.get_parameter('drive_timeout').value),
        }
        self.last_recv = {t: None for t in self.watched}

        self.create_subscription(Odometry, '/localization/odom',
                                 lambda m: self._touch('/localization/odom'), 10)
        self.create_subscription(Path, '/planning/path',
                                 lambda m: self._touch('/planning/path'), 10)
        self.create_subscription(AckermannDriveStamped, '/control/drive',
                                 lambda m: self._touch('/control/drive'), 10)

        self.drive_pub = self.create_publisher(AckermannDriveStamped, drive_topic, 10)
        self.alert_pub = self.create_publisher(Bool, alert_topic, 10)
        self.status_pub = self.create_publisher(String, status_topic, 10)

        self.create_timer(1.0 / check_rate, self._check)

        self.get_logger().info('watchdog_node started')
        for t, to in self.watched.items():
            self.get_logger().info(f'  watching {t}  timeout={to}s')

    def _touch(self, topic):
        self.last_recv[topic] = self.get_clock().now()

    def _check(self):
        now = self.get_clock().now()
        alerts = []

        for topic, timeout in self.watched.items():
            last = self.last_recv[topic]
            if last is None:
                continue  # 첫 메시지 미수신 = 아직 시작 전
            elapsed = (now - last).nanoseconds * 1e-9
            if elapsed > timeout:
                alerts.append(f'{topic} ({elapsed:.1f}s > {timeout}s)')

        if alerts:
            self.get_logger().warn(f'[WATCHDOG] 타임아웃: {alerts}')
            self._publish_estop()

        s_msg = String()
        s_msg.data = 'OK' if not alerts else 'ALERT: ' + ' | '.join(alerts)
        self.status_pub.publish(s_msg)

        b_msg = Bool()
        b_msg.data = bool(alerts)
        self.alert_pub.publish(b_msg)

    def _publish_estop(self):
        msg = AckermannDriveStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.drive.speed = 0.0
        msg.drive.steering_angle = 0.0
        self.drive_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = WatchdogNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
