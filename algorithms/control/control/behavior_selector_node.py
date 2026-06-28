import math

import rclpy
from rclpy.node import Node

from ackermann_msgs.msg import AckermannDriveStamped
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, String
from std_srvs.srv import SetBool


class BehaviorSelectorNode(Node):
    """
    멀티 모드 드라이브 스위처.

    모드 우선순위 (높음 → 낮음):
      OVERTAKE > GAP_FOLLOW > WAYPOINT

    자동 전환 로직:
      - /overtake/active = True  → OVERTAKE 모드
      - 전방 장애물 < obstacle_range → GAP_FOLLOW 모드
      - 그 외 → WAYPOINT 모드

    서비스:
      ~/force_waypoint (SetBool) : True=웨이포인트 강제, False=자동
      ~/set_mode (SetBool)       : True=gap_follow 강제, False=자동

    출력: /control/drive (safety_brake_node가 인터셉트)
    """

    MODES = ('WAYPOINT', 'GAP_FOLLOW', 'OVERTAKE')

    def __init__(self):
        super().__init__('behavior_selector_node')

        self.declare_parameter('waypoint_drive_topic', '/pure_pursuit/drive')
        self.declare_parameter('gap_drive_topic', '/gap_follow/drive')
        self.declare_parameter('overtake_drive_topic', '/adaptive_pp/drive')
        self.declare_parameter('output_drive_topic', '/control/drive')
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('overtake_active_topic', '/overtake/active')
        self.declare_parameter('mode_topic', '/behavior/mode')

        self.declare_parameter('obstacle_range', 1.5)
        self.declare_parameter('obstacle_clear_range', 2.0)
        self.declare_parameter('scan_field_deg', 60.0)

        waypoint_topic = self.get_parameter('waypoint_drive_topic').value
        gap_topic = self.get_parameter('gap_drive_topic').value
        overtake_topic = self.get_parameter('overtake_drive_topic').value
        output_topic = self.get_parameter('output_drive_topic').value
        scan_topic = self.get_parameter('scan_topic').value
        overtake_active_topic = self.get_parameter('overtake_active_topic').value
        mode_topic = self.get_parameter('mode_topic').value

        self.obstacle_range = float(self.get_parameter('obstacle_range').value)
        self.clear_range = float(self.get_parameter('obstacle_clear_range').value)
        self.scan_field = float(self.get_parameter('scan_field_deg').value)

        self.mode = 'WAYPOINT'
        self.force_mode = None      # None = auto

        self.waypoint_drive = None
        self.gap_drive = None
        self.overtake_drive = None
        self.overtake_active = False

        self.create_subscription(AckermannDriveStamped, waypoint_topic, self._cb_wp, 10)
        self.create_subscription(AckermannDriveStamped, gap_topic, self._cb_gap, 10)
        self.create_subscription(AckermannDriveStamped, overtake_topic, self._cb_ot, 10)
        self.create_subscription(LaserScan, scan_topic, self._cb_scan, 10)
        self.create_subscription(Bool, overtake_active_topic, self._cb_ot_active, 10)

        self.drive_pub = self.create_publisher(AckermannDriveStamped, output_topic, 10)
        self.mode_pub = self.create_publisher(String, mode_topic, 10)

        self.create_service(SetBool, '~/force_waypoint', self._srv_force_wp)
        self.create_service(SetBool, '~/set_mode', self._srv_set_mode)

        self.get_logger().info('behavior_selector_node started (멀티 모드)')
        self.get_logger().info(f'  WAYPOINT  ← {waypoint_topic}')
        self.get_logger().info(f'  GAP_FOLLOW← {gap_topic}')
        self.get_logger().info(f'  OVERTAKE  ← {overtake_topic}')
        self.get_logger().info(f'  output    → {output_topic}')
        self.get_logger().info(f'  obstacle  : {self.obstacle_range}m | clear: {self.clear_range}m')

    # ── Drive callbacks ──────────────────────────────────────────────
    def _cb_wp(self, msg):
        self.waypoint_drive = msg
        if self.mode == 'WAYPOINT':
            self._publish(msg)

    def _cb_gap(self, msg):
        self.gap_drive = msg
        if self.mode == 'GAP_FOLLOW':
            self._publish(msg)

    def _cb_ot(self, msg):
        self.overtake_drive = msg
        if self.mode == 'OVERTAKE':
            self._publish(msg)

    def _cb_ot_active(self, msg):
        self.overtake_active = msg.data

    # ── Scan → 자동 모드 결정 ────────────────────────────────────────
    def _cb_scan(self, msg):
        if self.force_mode is not None:
            new_mode = self.force_mode
        else:
            min_r = self._min_front(msg)
            if self.overtake_active:
                new_mode = 'OVERTAKE'
            elif min_r < self.obstacle_range:
                new_mode = 'GAP_FOLLOW'
            elif self.mode == 'GAP_FOLLOW' and min_r < self.clear_range:
                new_mode = 'GAP_FOLLOW'  # 히스테리시스 유지
            else:
                new_mode = 'WAYPOINT'

        if new_mode != self.mode:
            self.mode = new_mode
            self.get_logger().info(f'[BehaviorSelector] → {self.mode}')

        s = String(); s.data = self.mode
        self.mode_pub.publish(s)

    def _min_front(self, msg):
        half = math.radians(self.scan_field / 2.0)
        min_r = float('inf')
        angle = msg.angle_min
        for r in msg.ranges:
            if -half <= angle <= half and msg.range_min <= r <= msg.range_max:
                min_r = min(min_r, r)
            angle += msg.angle_increment
        return min_r if min_r != float('inf') else msg.range_max

    def _publish(self, msg):
        self.drive_pub.publish(msg)

    # ── Services ─────────────────────────────────────────────────────
    def _srv_force_wp(self, request, response):
        self.force_mode = 'WAYPOINT' if request.data else None
        response.success = True
        response.message = f'force_mode = {self.force_mode}'
        self.get_logger().info(f'[service] {response.message}')
        return response

    def _srv_set_mode(self, request, response):
        """True=gap_follow 강제, False=자동"""
        self.force_mode = 'GAP_FOLLOW' if request.data else None
        response.success = True
        response.message = f'force_mode = {self.force_mode}'
        self.get_logger().info(f'[service] {response.message}')
        return response


def main(args=None):
    rclpy.init(args=args)
    node = BehaviorSelectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
