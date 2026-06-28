import math

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, String, ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray


def _rgba(r, g, b, a=1.0):
    c = ColorRGBA()
    c.r = float(r)
    c.g = float(g)
    c.b = float(b)
    c.a = float(a)
    return c


class DebugMarkerNode(Node):
    """
    RViz 디버그 마커 종합 퍼블리셔.
    구독:
      /localization/odom   → 차량 위치 화살표
      /planning/path       → 계획 경로 (녹색 LINE_STRIP)
      /scan                → 전방 위험 구간 (빨간 POINTS)
      /behavior/mode       → 모드 TEXT
      /safety/braking      → 브레이킹 상태 TEXT
    출력: /debug/markers (MarkerArray, 20 Hz)
    """

    def __init__(self):
        super().__init__('debug_marker_node')

        self.declare_parameter('output_topic', '/debug/markers')
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('scan_frame_id', 'ego_racecar/laser')
        self.declare_parameter('publish_rate', 20.0)
        self.declare_parameter('scan_danger_range', 1.0)

        output_topic = self.get_parameter('output_topic').value
        self.frame_id = self.get_parameter('frame_id').value
        self.scan_frame = self.get_parameter('scan_frame_id').value
        rate = float(self.get_parameter('publish_rate').value)
        self.danger_range = float(self.get_parameter('scan_danger_range').value)

        # state
        self.odom = None
        self.path = None
        self.scan = None
        self.mode = 'unknown'
        self.braking = False

        self.create_subscription(Odometry, '/localization/odom', self._cb_odom, 10)
        self.create_subscription(Path, '/planning/path', self._cb_path, 10)
        self.create_subscription(LaserScan, '/scan', self._cb_scan, 10)
        self.create_subscription(String, '/behavior/mode', self._cb_mode, 10)
        self.create_subscription(Bool, '/safety/braking', self._cb_braking, 10)

        self.pub = self.create_publisher(MarkerArray, output_topic, 10)
        self.create_timer(1.0 / rate, self._publish)

        self.get_logger().info('debug_marker_node started')
        self.get_logger().info(f'  output : {output_topic}  @ {rate} Hz')

    def _cb_odom(self, msg):
        self.odom = msg

    def _cb_path(self, msg):
        self.path = msg

    def _cb_scan(self, msg):
        self.scan = msg

    def _cb_mode(self, msg):
        self.mode = msg.data

    def _cb_braking(self, msg):
        self.braking = msg.data

    def _publish(self):
        ma = MarkerArray()
        now = self.get_clock().now().to_msg()
        mid = 0

        # ── 1. 차량 위치 화살표 ──────────────────────────────────────
        if self.odom:
            m = Marker()
            m.header.stamp = now
            m.header.frame_id = self.frame_id
            m.ns = 'vehicle'
            m.id = mid; mid += 1
            m.type = Marker.ARROW
            m.action = Marker.ADD
            m.pose = self.odom.pose.pose
            m.scale.x = 0.4
            m.scale.y = 0.08
            m.scale.z = 0.08
            m.color = _rgba(0.2, 0.8, 0.2) if not self.braking else _rgba(1.0, 0.0, 0.0)
            ma.markers.append(m)

        # ── 2. 계획 경로 ─────────────────────────────────────────────
        if self.path and len(self.path.poses) > 1:
            m = Marker()
            m.header.stamp = now
            m.header.frame_id = self.frame_id
            m.ns = 'plan_path'
            m.id = mid; mid += 1
            m.type = Marker.LINE_STRIP
            m.action = Marker.ADD
            m.scale.x = 0.04
            m.color = _rgba(0.0, 1.0, 0.4, 0.8)
            for ps in self.path.poses:
                p = Point()
                p.x = ps.pose.position.x
                p.y = ps.pose.position.y
                p.z = 0.02
                m.points.append(p)
            ma.markers.append(m)

        # ── 3. LiDAR 위험 포인트 ─────────────────────────────────────
        if self.scan:
            m = Marker()
            m.header.stamp = now
            m.header.frame_id = self.scan_frame
            m.ns = 'scan_danger'
            m.id = mid; mid += 1
            m.type = Marker.POINTS
            m.action = Marker.ADD
            m.scale.x = 0.05
            m.scale.y = 0.05
            m.color = _rgba(1.0, 0.2, 0.0, 0.9)

            angle = self.scan.angle_min
            for r in self.scan.ranges:
                if math.isfinite(r) and r < self.danger_range:
                    p = Point()
                    p.x = r * math.cos(angle)
                    p.y = r * math.sin(angle)
                    p.z = 0.0
                    m.points.append(p)
                angle += self.scan.angle_increment
            ma.markers.append(m)

        # ── 4. 모드 텍스트 ───────────────────────────────────────────
        if self.odom:
            m = Marker()
            m.header.stamp = now
            m.header.frame_id = self.frame_id
            m.ns = 'mode_text'
            m.id = mid; mid += 1
            m.type = Marker.TEXT_VIEW_FACING
            m.action = Marker.ADD
            m.pose.position.x = self.odom.pose.pose.position.x
            m.pose.position.y = self.odom.pose.pose.position.y
            m.pose.position.z = self.odom.pose.pose.position.z + 0.3
            m.pose.orientation = self.odom.pose.pose.orientation
            m.scale.z = 0.18
            brake_str = ' [BRAKE]' if self.braking else ''
            m.text = f'{self.mode}{brake_str}'
            m.color = _rgba(1.0, 1.0, 0.0) if not self.braking else _rgba(1.0, 0.0, 0.0)
            ma.markers.append(m)

        self.pub.publish(ma)


def main(args=None):
    rclpy.init(args=args)
    node = DebugMarkerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
