import math

import numpy as np

import rclpy
from rclpy.node import Node

from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry, Path
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point


class AdaptivePurePursuitNode(Node):
    """
    속도 적응형 Pure Pursuit.

    고정 룩어헤드가 아니라:
      L_d = clip(k_v * v + L_min, L_min, L_max)

    또한 전방 N개 웨이포인트의 평균 곡률을 계산해
    코너 진입 전에 미리 감속 (피드포워드 속도 계획).

    출력: /adaptive_pp/drive
    behavior_selector에서 waypoint_drive_topic을 바꾸면
    pure_pursuit 대신 이 노드를 기본 컨트롤러로 쓸 수 있다.
    """

    def __init__(self):
        super().__init__('adaptive_pure_pursuit_node')

        # ── Parameters ──────────────────────────────────────────────
        self.declare_parameter('odom_topic', '/localization/odom')
        self.declare_parameter('path_topic', '/planning/path')
        self.declare_parameter('drive_topic', '/adaptive_pp/drive')
        self.declare_parameter('marker_topic', '/adaptive_pp/markers')
        self.declare_parameter('frame_id', 'map')

        self.declare_parameter('wheelbase', 0.33)
        self.declare_parameter('L_min', 0.5)
        self.declare_parameter('L_max', 3.0)
        self.declare_parameter('k_v', 0.6)              # L_d = k_v * v + L_min

        self.declare_parameter('target_speed', 1.5)
        self.declare_parameter('min_speed', 0.4)
        self.declare_parameter('max_speed', 2.5)
        self.declare_parameter('corner_speed_gain', 2.5)  # 감속 강도
        self.declare_parameter('lookahead_speed_points', 10)  # 곡률 계산 범위

        self.declare_parameter('max_steering_angle', 0.4189)

        self.declare_parameter('kp', 1.2)
        self.declare_parameter('ki', 0.0)
        self.declare_parameter('kd', 0.05)

        self.declare_parameter('control_rate', 30.0)

        # ── Get params ──────────────────────────────────────────────
        odom_topic = self.get_parameter('odom_topic').value
        path_topic = self.get_parameter('path_topic').value
        drive_topic = self.get_parameter('drive_topic').value
        marker_topic = self.get_parameter('marker_topic').value
        self.frame_id = self.get_parameter('frame_id').value

        self.L = float(self.get_parameter('wheelbase').value)
        self.L_min = float(self.get_parameter('L_min').value)
        self.L_max = float(self.get_parameter('L_max').value)
        self.k_v = float(self.get_parameter('k_v').value)

        self.target_speed = float(self.get_parameter('target_speed').value)
        self.min_speed = float(self.get_parameter('min_speed').value)
        self.max_speed = float(self.get_parameter('max_speed').value)
        self.corner_gain = float(self.get_parameter('corner_speed_gain').value)
        self.speed_pts = int(self.get_parameter('lookahead_speed_points').value)

        self.max_steer = float(self.get_parameter('max_steering_angle').value)

        self.kp = float(self.get_parameter('kp').value)
        self.ki = float(self.get_parameter('ki').value)
        self.kd = float(self.get_parameter('kd').value)

        rate = float(self.get_parameter('control_rate').value)

        # ── State ────────────────────────────────────────────────────
        self.x = self.y = self.yaw = self.speed = 0.0
        self.ref_path = None
        self.pid_integral = 0.0
        self.pid_prev_err = 0.0
        self.prev_time = None

        # ── Pub / Sub ────────────────────────────────────────────────
        self.create_subscription(Odometry, odom_topic, self._cb_odom, 10)
        self.create_subscription(Path, path_topic, self._cb_path, 10)
        self.drive_pub = self.create_publisher(AckermannDriveStamped, drive_topic, 10)
        self.marker_pub = self.create_publisher(MarkerArray, marker_topic, 10)

        self.create_timer(1.0 / rate, self._control_loop)

        self.get_logger().info('adaptive_pure_pursuit_node started')
        self.get_logger().info(f'  L_d = {self.k_v}*v + {self.L_min}  clip [{self.L_min}, {self.L_max}]')
        self.get_logger().info(f'  speed range : {self.min_speed}~{self.max_speed} m/s')

    def _cb_odom(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        self.yaw = math.atan2(2 * q.w * q.z, 1 - 2 * q.z ** 2)
        self.speed = msg.twist.twist.linear.x

    def _cb_path(self, msg):
        self.ref_path = [
            (p.pose.position.x, p.pose.position.y)
            for p in msg.poses
        ]

    def _control_loop(self):
        if self.ref_path is None or len(self.ref_path) < 2:
            return

        now = self.get_clock().now()
        dt = 0.0
        if self.prev_time is not None:
            dt = (now - self.prev_time).nanoseconds * 1e-9
        self.prev_time = now

        path = self.ref_path
        n = len(path)

        # 가장 가까운 인덱스
        dists = [math.hypot(px - self.x, py - self.y) for px, py in path]
        nearest = int(np.argmin(dists))

        # 속도 적응형 룩어헤드 거리
        L_d = float(np.clip(self.k_v * abs(self.speed) + self.L_min, self.L_min, self.L_max))

        # 룩어헤드 포인트 탐색
        lookahead = None
        for i in range(nearest, nearest + n):
            idx = i % n
            d = math.hypot(path[idx][0] - self.x, path[idx][1] - self.y)
            if d >= L_d:
                lookahead = path[idx]
                break

        if lookahead is None:
            lookahead = path[(nearest + 5) % n]

        # Pure Pursuit 조향각
        dx = lookahead[0] - self.x
        dy = lookahead[1] - self.y
        local_y = -math.sin(self.yaw) * dx + math.cos(self.yaw) * dy
        steer = math.atan2(2 * self.L * local_y, L_d ** 2)
        steer = float(np.clip(steer, -self.max_steer, self.max_steer))

        # 피드포워드 속도 (전방 곡률 기반)
        v_target = self._curvature_speed(path, nearest, n)

        # PID 속도 루프
        err = v_target - abs(self.speed)
        if dt > 0:
            self.pid_integral += err * dt
            d_err = (err - self.pid_prev_err) / dt
        else:
            d_err = 0.0
        self.pid_prev_err = err

        v_cmd = float(np.clip(
            abs(self.speed) + self.kp * err + self.ki * self.pid_integral + self.kd * d_err,
            self.min_speed, self.max_speed
        ))

        msg = AckermannDriveStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.drive.speed = v_cmd
        msg.drive.steering_angle = steer
        self.drive_pub.publish(msg)

        self._publish_marker(lookahead, L_d)

    def _curvature_speed(self, path, nearest, n):
        """전방 speed_pts 개 웨이포인트의 평균 곡률로 속도 결정."""
        kappas = []
        for i in range(1, self.speed_pts - 1):
            ia = (nearest + i - 1) % n
            ib = (nearest + i) % n
            ic = (nearest + i + 1) % n
            ax, ay = path[ia]
            bx, by = path[ib]
            cx, cy = path[ic]
            a = math.hypot(bx - ax, by - ay)
            b = math.hypot(cx - bx, cy - by)
            c = math.hypot(cx - ax, cy - ay)
            cross = abs((bx - ax) * (cy - ay) - (by - ay) * (cx - ax))
            denom = a * b * c
            if denom > 1e-6:
                kappas.append(cross / denom)

        if not kappas:
            return self.target_speed

        kappa = float(np.mean(kappas))
        v = self.target_speed - self.corner_gain * kappa * (self.target_speed - self.min_speed)
        return float(np.clip(v, self.min_speed, self.max_speed))

    def _publish_marker(self, lookahead, L_d):
        ma = MarkerArray()
        now = self.get_clock().now().to_msg()

        # 룩어헤드 포인트 구체
        m = Marker()
        m.header.stamp = now
        m.header.frame_id = self.frame_id
        m.ns = 'lookahead'
        m.id = 0
        m.type = Marker.SPHERE
        m.action = Marker.ADD
        m.pose.position.x = lookahead[0]
        m.pose.position.y = lookahead[1]
        m.pose.position.z = 0.1
        m.pose.orientation.w = 1.0
        m.scale.x = m.scale.y = m.scale.z = 0.15
        m.color.a = 1.0
        m.color.r = 0.0
        m.color.g = 0.8
        m.color.b = 1.0
        ma.markers.append(m)

        # 차량→룩어헤드 선분
        line = Marker()
        line.header = m.header
        line.ns = 'lookahead_line'
        line.id = 1
        line.type = Marker.LINE_STRIP
        line.action = Marker.ADD
        line.scale.x = 0.03
        line.color.a = 0.6
        line.color.r = 0.0
        line.color.g = 0.8
        line.color.b = 1.0
        p0 = Point(); p0.x = self.x; p0.y = self.y; p0.z = 0.05
        p1 = Point(); p1.x = lookahead[0]; p1.y = lookahead[1]; p1.z = 0.05
        line.points = [p0, p1]
        ma.markers.append(line)

        self.marker_pub.publish(ma)


def main(args=None):
    rclpy.init(args=args)
    node = AdaptivePurePursuitNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
