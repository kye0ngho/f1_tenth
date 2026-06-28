import math

import numpy as np

import rclpy
from rclpy.node import Node

from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan


class DisparityExtenderNode(Node):
    """
    Disparity Extender 반응형 주행.

    gap_follow보다 정교한 F1TENTH 전용 알고리즘:
      1. 인접 빔 간 급격한 범위 변화(disparity) 감지
      2. 각 disparity에서 가까운 쪽 장애물의 그림자를
         차량 반폭만큼 반대편으로 확장 (사각지대 제거)
      3. 확장된 스캔에서 가장 먼 포인트 선택
      4. Pure Pursuit 공식으로 조향

    gap_follow와 비교:
      - 좁은 틈 진입 억제 (차폭 기반 확장)
      - 연속 영역이 아닌 최심점 추종 → 더 공격적인 주행 가능
    """

    def __init__(self):
        super().__init__('disparity_extender_node')

        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('odom_topic', '/ego_racecar/odom')
        self.declare_parameter('drive_topic', '/disparity_ext/drive')

        self.declare_parameter('car_width', 0.30)            # 차폭 (m)
        self.declare_parameter('max_range', 4.0)
        self.declare_parameter('disparity_threshold', 0.5)  # disparity 감지 기준 (m)
        self.declare_parameter('scan_field_deg', 180.0)

        self.declare_parameter('max_speed', 2.0)
        self.declare_parameter('min_speed', 0.5)
        self.declare_parameter('corner_gain', 1.2)           # 조향각 크기에 따른 감속
        self.declare_parameter('max_steering_angle', 0.4189)
        self.declare_parameter('wheelbase', 0.33)

        scan_topic = self.get_parameter('scan_topic').value
        odom_topic = self.get_parameter('odom_topic').value
        drive_topic = self.get_parameter('drive_topic').value

        self.car_half = float(self.get_parameter('car_width').value) / 2.0
        self.max_range = float(self.get_parameter('max_range').value)
        self.disp_thr = float(self.get_parameter('disparity_threshold').value)
        self.field_deg = float(self.get_parameter('scan_field_deg').value)

        self.max_speed = float(self.get_parameter('max_speed').value)
        self.min_speed = float(self.get_parameter('min_speed').value)
        self.corner_gain = float(self.get_parameter('corner_gain').value)
        self.max_steer = float(self.get_parameter('max_steering_angle').value)
        self.L = float(self.get_parameter('wheelbase').value)

        self.current_speed = 0.0
        self.create_subscription(LaserScan, scan_topic, self._cb_scan, 10)
        self.create_subscription(Odometry, odom_topic, self._cb_odom, 10)
        self.drive_pub = self.create_publisher(AckermannDriveStamped, drive_topic, 10)

        self.get_logger().info('disparity_extender_node started')
        self.get_logger().info(f'  car_half_width    : {self.car_half:.3f} m')
        self.get_logger().info(f'  disparity_threshold : {self.disp_thr} m')
        self.get_logger().info(f'  drive_topic       : {drive_topic}')

    def _cb_odom(self, msg):
        self.current_speed = abs(msg.twist.twist.linear.x)

    def _cb_scan(self, msg):
        ranges = np.array(msg.ranges, dtype=np.float64)
        n = len(ranges)
        angle_inc = msg.angle_increment

        # 전방 field_deg 만 사용
        half_field = math.radians(self.field_deg / 2.0)
        angles = msg.angle_min + np.arange(n) * angle_inc
        mask = np.abs(angles) <= half_field
        idx_start = int(np.argmax(mask))
        idx_end = n - int(np.argmax(mask[::-1]))

        sub = ranges[idx_start:idx_end].copy()
        sub_angles = angles[idx_start:idx_end]

        # NaN/Inf → max_range, 클리핑
        sub = np.where(np.isfinite(sub), sub, self.max_range)
        sub = np.clip(sub, 0.0, self.max_range)

        # Disparity 확장
        extended = self._extend_disparities(sub, angle_inc)

        # 최심점 선택
        best_idx = int(np.argmax(extended))
        best_angle = sub_angles[best_idx]

        # Pure Pursuit 조향
        lookahead = extended[best_idx]
        local_y = lookahead * math.sin(best_angle)
        steer = math.atan2(2.0 * self.L * local_y, lookahead ** 2)
        steer = float(np.clip(steer, -self.max_steer, self.max_steer))

        # 조향각 비례 감속
        steer_ratio = abs(steer) / self.max_steer
        speed = self.max_speed - self.corner_gain * steer_ratio * (self.max_speed - self.min_speed)
        speed = float(np.clip(speed, self.min_speed, self.max_speed))

        msg_out = AckermannDriveStamped()
        msg_out.header.stamp = self.get_clock().now().to_msg()
        msg_out.drive.speed = speed
        msg_out.drive.steering_angle = steer
        self.drive_pub.publish(msg_out)

    def _extend_disparities(self, ranges, angle_inc):
        n = len(ranges)
        extended = ranges.copy()

        for i in range(n - 1):
            diff = float(ranges[i + 1]) - float(ranges[i])
            if abs(diff) < self.disp_thr:
                continue

            if diff > 0:
                # index i 쪽이 더 가깝다 → 오른쪽(higher index)으로 확장
                near = float(ranges[i])
                if near > 0.05:
                    n_ext = int(math.ceil(self.car_half / (near * angle_inc + 1e-9)))
                    end = min(i + 1 + n_ext, n)
                    extended[i + 1:end] = np.minimum(extended[i + 1:end], near)
            else:
                # index i+1 쪽이 더 가깝다 → 왼쪽(lower index)으로 확장
                near = float(ranges[i + 1])
                if near > 0.05:
                    n_ext = int(math.ceil(self.car_half / (near * angle_inc + 1e-9)))
                    start = max(0, i + 1 - n_ext)
                    extended[start:i + 1] = np.minimum(extended[start:i + 1], near)

        return extended


def main(args=None):
    rclpy.init(args=args)
    node = DisparityExtenderNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
