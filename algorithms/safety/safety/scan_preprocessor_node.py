import math

import numpy as np
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan


class ScanPreprocessorNode(Node):
    """
    LiDAR 전처리 노드.
    - 유효 범위 필터 (range_min/max)
    - NaN/Inf 제거 → range_max로 대체
    - 미디언 필터 (스파이크 노이즈 제거)
    - 전방 scan_field_deg 외 무효화 (선택)
    원본 /scan → 전처리 후 /scan_processed 퍼블리시.
    """

    def __init__(self):
        super().__init__('scan_preprocessor_node')

        self.declare_parameter('scan_input_topic', '/scan')
        self.declare_parameter('scan_output_topic', '/scan_processed')
        self.declare_parameter('median_window', 3)
        self.declare_parameter('range_min_override', 0.05)
        self.declare_parameter('range_max_override', 20.0)
        self.declare_parameter('limit_field_deg', 0.0)   # 0 = 전체 사용

        self.input_topic = self.get_parameter('scan_input_topic').value
        self.output_topic = self.get_parameter('scan_output_topic').value
        self.median_w = int(self.get_parameter('median_window').value)
        self.range_min = float(self.get_parameter('range_min_override').value)
        self.range_max = float(self.get_parameter('range_max_override').value)
        self.limit_field = float(self.get_parameter('limit_field_deg').value)

        self.create_subscription(LaserScan, self.input_topic, self.scan_callback, 10)
        self.pub = self.create_publisher(LaserScan, self.output_topic, 10)

        self.get_logger().info('scan_preprocessor_node started')
        self.get_logger().info(f'  {self.input_topic} → {self.output_topic}')
        self.get_logger().info(f'  median_window : {self.median_w}')
        self.get_logger().info(f'  range         : {self.range_min}~{self.range_max} m')

    def scan_callback(self, msg):
        ranges = np.array(msg.ranges, dtype=np.float32)

        # NaN/Inf 제거
        ranges = np.where(np.isfinite(ranges), ranges, self.range_max)

        # 범위 클리핑
        ranges = np.clip(ranges, self.range_min, self.range_max)

        # 미디언 필터 (circular padding)
        if self.median_w > 1:
            pad = self.median_w // 2
            padded = np.concatenate([ranges[-pad:], ranges, ranges[:pad]])
            filtered = np.array([
                np.median(padded[i:i + self.median_w])
                for i in range(len(ranges))
            ], dtype=np.float32)
            ranges = filtered

        # 전방 각도 외 무효화
        if self.limit_field > 0:
            half = math.radians(self.limit_field / 2.0)
            angles = np.linspace(msg.angle_min, msg.angle_max, len(ranges))
            mask = np.abs(angles) > half
            ranges[mask] = self.range_max

        out = LaserScan()
        out.header = msg.header
        out.angle_min = msg.angle_min
        out.angle_max = msg.angle_max
        out.angle_increment = msg.angle_increment
        out.time_increment = msg.time_increment
        out.scan_time = msg.scan_time
        out.range_min = self.range_min
        out.range_max = self.range_max
        out.ranges = ranges.tolist()
        out.intensities = msg.intensities
        self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = ScanPreprocessorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
