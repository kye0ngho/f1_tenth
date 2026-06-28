import math

import numpy as np

import rclpy
from rclpy.node import Node

from ackermann_msgs.msg import AckermannDriveStamped
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32


class CorridorFollowNode(Node):
    """
    LiDAR 기반 회랑 중앙 추종.

    웨이포인트 없이 트랙 좌·우 벽 사이의 중앙을 추종.
    알고리즘:
      1. 좌측 섹터(45~135°) → 좌벽 거리 d_left (분위수 사용)
      2. 우측 섹터(-135~-45°) → 우벽 거리 d_right
      3. 횡방향 오차 e = (d_right - d_left) / 2
      4. 조향 = k_p · e  (비례 제어)
      5. 속도 = 전방 거리에 비례 (좁을수록 감속)

    용도:
      - 처음 주행하는 트랙에서 웨이포인트 없이 안전하게 달리기
      - behavior_selector에서 gap_drive_topic을 /corridor/drive로 교체
    """

    def __init__(self):
        super().__init__('corridor_follow_node')

        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('drive_topic', '/corridor/drive')
        self.declare_parameter('error_topic', '/corridor/error')

        self.declare_parameter('left_angle_min_deg', 45.0)
        self.declare_parameter('left_angle_max_deg', 135.0)
        self.declare_parameter('right_angle_min_deg', -135.0)
        self.declare_parameter('right_angle_max_deg', -45.0)
        self.declare_parameter('front_angle_deg', 30.0)   # 전방 속도 제어용

        self.declare_parameter('wall_percentile', 20.0)   # 벽 거리 추정 (하위 %)
        self.declare_parameter('k_p', 0.8)
        self.declare_parameter('max_steering', 0.4189)

        self.declare_parameter('max_speed', 2.0)
        self.declare_parameter('min_speed', 0.5)
        self.declare_parameter('front_target_dist', 2.0)   # 이 거리일 때 max_speed
        self.declare_parameter('front_safe_dist', 0.8)     # 이 거리일 때 min_speed

        scan_topic = self.get_parameter('scan_topic').value
        drive_topic = self.get_parameter('drive_topic').value
        error_topic = self.get_parameter('error_topic').value

        self.l_min = math.radians(self.get_parameter('left_angle_min_deg').value)
        self.l_max = math.radians(self.get_parameter('left_angle_max_deg').value)
        self.r_min = math.radians(self.get_parameter('right_angle_min_deg').value)
        self.r_max = math.radians(self.get_parameter('right_angle_max_deg').value)
        self.f_half = math.radians(self.get_parameter('front_angle_deg').value / 2)

        self.wall_pct = float(self.get_parameter('wall_percentile').value)
        self.k_p = float(self.get_parameter('k_p').value)
        self.max_steer = float(self.get_parameter('max_steering').value)

        self.max_speed = float(self.get_parameter('max_speed').value)
        self.min_speed = float(self.get_parameter('min_speed').value)
        self.front_target = float(self.get_parameter('front_target_dist').value)
        self.front_safe = float(self.get_parameter('front_safe_dist').value)

        self.create_subscription(LaserScan, scan_topic, self._cb_scan, 10)
        self.drive_pub = self.create_publisher(AckermannDriveStamped, drive_topic, 10)
        self.error_pub = self.create_publisher(Float32, error_topic, 10)

        self.get_logger().info('corridor_follow_node started')
        self.get_logger().info(f'  k_p={self.k_p}  max_steer={self.max_steer:.3f} rad')
        self.get_logger().info(f'  drive_topic : {drive_topic}')

    def _cb_scan(self, msg):
        angles = np.linspace(msg.angle_min, msg.angle_max, len(msg.ranges))
        ranges = np.array(msg.ranges, dtype=np.float32)

        # NaN/Inf 제거
        valid = np.isfinite(ranges) & (ranges > 0.05) & (ranges < msg.range_max)

        def sector_dist(a_min, a_max):
            mask = valid & (angles >= a_min) & (angles <= a_max)
            pts = ranges[mask]
            if len(pts) == 0:
                return msg.range_max
            return float(np.percentile(pts, self.wall_pct))

        d_left = sector_dist(self.l_min, self.l_max)
        d_right = sector_dist(self.r_min, self.r_max)

        # 전방 거리 (속도 계획용)
        front_mask = valid & (np.abs(angles) <= self.f_half)
        front_ranges = ranges[front_mask]
        d_front = float(np.min(front_ranges)) if len(front_ranges) > 0 else msg.range_max

        # 횡방향 오차 (양수 = 우측에 더 가까움 = 왼쪽으로 돌아야 함)
        e = (d_right - d_left) / 2.0
        steer = float(np.clip(self.k_p * e, -self.max_steer, self.max_steer))

        # 속도: 전방 거리에 따라 선형 보간
        t = (d_front - self.front_safe) / max(self.front_target - self.front_safe, 1e-6)
        t = float(np.clip(t, 0.0, 1.0))
        speed = self.min_speed + t * (self.max_speed - self.min_speed)

        out = AckermannDriveStamped()
        out.header.stamp = self.get_clock().now().to_msg()
        out.drive.speed = speed
        out.drive.steering_angle = steer
        self.drive_pub.publish(out)

        err_msg = Float32(); err_msg.data = e
        self.error_pub.publish(err_msg)


def main(args=None):
    rclpy.init(args=args)
    node = CorridorFollowNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
