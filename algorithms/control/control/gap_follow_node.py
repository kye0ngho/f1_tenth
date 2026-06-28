import math

import rclpy
from rclpy.node import Node

from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan


class GapFollowNode(Node):
    """
    Follow the Gap Method (FGM) — LiDAR 기반 반응형 장애물 회피.

    파이프라인:
      1. 전방 scan_field_deg 범위만 추출
      2. 가장 가까운 장애물 주변에 safety_bubble 적용 (0으로 마스킹)
      3. 연속된 자유 공간(range > 0) 중 가장 넓은 gap 탐색
      4. gap 내 가장 먼 포인트 방향으로 조향
      5. 조향각 크기에 따라 속도 감속
    """

    def __init__(self):
        super().__init__('gap_follow_node')

        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('odom_topic', '/ego_racecar/odom')
        self.declare_parameter('drive_topic', '/gap_follow/drive')

        self.declare_parameter('max_range', 3.0)
        self.declare_parameter('safety_bubble', 0.3)
        self.declare_parameter('scan_field_deg', 180.0)
        self.declare_parameter('min_gap_size', 3)

        self.declare_parameter('max_speed', 1.5)
        self.declare_parameter('min_speed', 0.5)
        self.declare_parameter('corner_slowdown_gain', 1.5)
        self.declare_parameter('max_steering_angle', 0.4189)
        self.declare_parameter('wheelbase', 0.33)

        self.scan_topic = self.get_parameter('scan_topic').value
        self.odom_topic = self.get_parameter('odom_topic').value
        self.drive_topic = self.get_parameter('drive_topic').value

        self.max_range = float(self.get_parameter('max_range').value)
        self.safety_bubble = float(self.get_parameter('safety_bubble').value)
        self.scan_field_deg = float(self.get_parameter('scan_field_deg').value)
        self.min_gap_size = int(self.get_parameter('min_gap_size').value)

        self.max_speed = float(self.get_parameter('max_speed').value)
        self.min_speed = float(self.get_parameter('min_speed').value)
        self.corner_slowdown_gain = float(self.get_parameter('corner_slowdown_gain').value)
        self.max_steering_angle = float(self.get_parameter('max_steering_angle').value)
        self.wheelbase = float(self.get_parameter('wheelbase').value)

        self.current_speed = 0.0

        self.create_subscription(LaserScan, self.scan_topic, self.scan_callback, 10)
        self.create_subscription(Odometry, self.odom_topic, self.odom_callback, 10)
        self.drive_pub = self.create_publisher(AckermannDriveStamped, self.drive_topic, 10)

        self.get_logger().info('gap_follow_node started')
        self.get_logger().info(f'  scan_topic       : {self.scan_topic}')
        self.get_logger().info(f'  drive_topic      : {self.drive_topic}')
        self.get_logger().info(f'  max_range        : {self.max_range} m')
        self.get_logger().info(f'  safety_bubble    : {self.safety_bubble} m')
        self.get_logger().info(f'  scan_field       : ±{self.scan_field_deg / 2:.0f}°')
        self.get_logger().info(f'  speed range      : {self.min_speed} ~ {self.max_speed} m/s')

    def odom_callback(self, msg):
        self.current_speed = msg.twist.twist.linear.x

    def scan_callback(self, msg):
        ranges, angles = self._extract_front(msg)
        if not ranges:
            return

        ranges = self._clip(ranges, msg.range_min, self.max_range)
        ranges = self._apply_bubble(ranges, angles)

        gap = self._find_best_gap(ranges)
        if gap is None:
            self._publish(0.0, 0.0)
            return

        best_idx = self._best_point_in_gap(ranges, gap)
        target_angle = angles[best_idx]

        steering = self._compute_steering(target_angle, ranges[best_idx])
        speed = self._compute_speed(steering)
        self._publish(speed, steering)

    # ------------------------------------------------------------------ #
    # Preprocessing
    # ------------------------------------------------------------------ #
    def _extract_front(self, msg):
        half = math.radians(self.scan_field_deg / 2.0)
        ranges, angles = [], []
        angle = msg.angle_min
        for r in msg.ranges:
            if -half <= angle <= half:
                ranges.append(r)
                angles.append(angle)
            angle += msg.angle_increment
        return ranges, angles

    def _clip(self, ranges, range_min, range_max):
        return [
            r if range_min < r < range_max else range_max
            for r in ranges
        ]

    def _apply_bubble(self, ranges, angles):
        min_r = min(ranges)
        min_i = ranges.index(min_r)

        # 가장 가까운 장애물 주변을 0으로 마스킹
        result = list(ranges)
        for i, angle in enumerate(angles):
            arc = abs(angles[min_i]) if min_r > 1e-3 else 0.0
            angular_spread = math.atan2(self.safety_bubble, min_r) if min_r > 1e-3 else math.pi
            if abs(angle - angles[min_i]) <= angular_spread:
                result[i] = 0.0

        return result

    # ------------------------------------------------------------------ #
    # Gap search
    # ------------------------------------------------------------------ #
    def _find_best_gap(self, ranges):
        best_gap = None
        best_len = 0

        i = 0
        n = len(ranges)
        while i < n:
            if ranges[i] > 0.0:
                j = i
                while j < n and ranges[j] > 0.0:
                    j += 1
                length = j - i
                if length > best_len:
                    best_len = length
                    best_gap = (i, j - 1)
                i = j
            else:
                i += 1

        if best_len < self.min_gap_size:
            return None
        return best_gap

    def _best_point_in_gap(self, ranges, gap):
        start, end = gap
        # gap 내에서 가장 먼 포인트
        best_i = start
        best_r = ranges[start]
        for i in range(start, end + 1):
            if ranges[i] > best_r:
                best_r = ranges[i]
                best_i = i
        return best_i

    # ------------------------------------------------------------------ #
    # Control
    # ------------------------------------------------------------------ #
    def _compute_steering(self, target_angle, lookahead):
        if lookahead < 1e-3:
            return 0.0
        # Pure Pursuit 공식 적용
        curvature = 2.0 * math.sin(target_angle) / max(lookahead, 0.1)
        steering = math.atan(self.wheelbase * curvature)
        return max(-self.max_steering_angle, min(self.max_steering_angle, steering))

    def _compute_speed(self, steering):
        ratio = abs(steering) / max(self.max_steering_angle, 1e-6)
        speed = self.max_speed - self.corner_slowdown_gain * ratio * (self.max_speed - self.min_speed)
        return max(self.min_speed, min(self.max_speed, speed))

    def _publish(self, speed, steering):
        msg = AckermannDriveStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.drive.speed = float(speed)
        msg.drive.steering_angle = float(steering)
        self.drive_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = GapFollowNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
