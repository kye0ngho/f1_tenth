import math

import numpy as np

import rclpy
from rclpy.node import Node

from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry, Path
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point


class StanleyControllerNode(Node):
    """
    Stanley 횡방향 제어기 + PID 속도 제어.

    순수 추종(Pure Pursuit)과 달리 Stanley는
    앞 차축(front axle) 기준으로 횡방향 오차를 보정:

        δ = ψ_e + atan( k · e / (v + k_soft) )

    여기서:
      ψ_e : 경로 heading과 차량 heading의 오차
      e   : 경로까지의 횡방향 거리 (부호 있음, 음수=경로 왼쪽)
      k   : 게인 (높을수록 공격적 보정)
      k_soft : 저속 안정화 소프트닝 항

    Pure Pursuit보다 저속 정밀 추종에 유리,
    고속에서는 Pure Pursuit와 유사한 성능.

    출력: /stanley/drive
    """

    def __init__(self):
        super().__init__('stanley_controller_node')

        self.declare_parameter('odom_topic', '/localization/odom')
        self.declare_parameter('path_topic', '/planning/path')
        self.declare_parameter('drive_topic', '/stanley/drive')
        self.declare_parameter('marker_topic', '/stanley/markers')
        self.declare_parameter('frame_id', 'map')

        self.declare_parameter('wheelbase', 0.33)
        self.declare_parameter('k', 1.0)          # Stanley 게인
        self.declare_parameter('k_soft', 0.3)     # 저속 소프트닝
        self.declare_parameter('max_steering_angle', 0.4189)

        self.declare_parameter('target_speed', 1.5)
        self.declare_parameter('min_speed', 0.4)
        self.declare_parameter('max_speed', 2.5)
        self.declare_parameter('speed_curvature_gain', 2.0)
        self.declare_parameter('lookahead_speed_points', 8)

        self.declare_parameter('kp', 1.0)
        self.declare_parameter('ki', 0.0)
        self.declare_parameter('kd', 0.05)
        self.declare_parameter('control_rate', 30.0)

        odom_topic = self.get_parameter('odom_topic').value
        path_topic = self.get_parameter('path_topic').value
        drive_topic = self.get_parameter('drive_topic').value
        marker_topic = self.get_parameter('marker_topic').value
        self.frame_id = self.get_parameter('frame_id').value

        self.L = float(self.get_parameter('wheelbase').value)
        self.k = float(self.get_parameter('k').value)
        self.k_soft = float(self.get_parameter('k_soft').value)
        self.max_steer = float(self.get_parameter('max_steering_angle').value)

        self.target_speed = float(self.get_parameter('target_speed').value)
        self.min_speed = float(self.get_parameter('min_speed').value)
        self.max_speed = float(self.get_parameter('max_speed').value)
        self.spd_curv_gain = float(self.get_parameter('speed_curvature_gain').value)
        self.spd_pts = int(self.get_parameter('lookahead_speed_points').value)

        self.kp = float(self.get_parameter('kp').value)
        self.ki = float(self.get_parameter('ki').value)
        self.kd = float(self.get_parameter('kd').value)
        rate = float(self.get_parameter('control_rate').value)

        # State
        self.x = self.y = self.yaw = self.speed = 0.0
        self.ref_path = None
        self.pid_i = 0.0
        self.pid_prev_e = 0.0
        self.prev_time = None

        self.create_subscription(Odometry, odom_topic, self._cb_odom, 10)
        self.create_subscription(Path, path_topic, self._cb_path, 10)
        self.drive_pub = self.create_publisher(AckermannDriveStamped, drive_topic, 10)
        self.marker_pub = self.create_publisher(MarkerArray, marker_topic, 10)
        self.create_timer(1.0 / rate, self._control_loop)

        self.get_logger().info('stanley_controller_node started')
        self.get_logger().info(f'  k={self.k}  k_soft={self.k_soft}')
        self.get_logger().info(f'  drive_topic : {drive_topic}')

    def _cb_odom(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        self.yaw = math.atan2(2 * q.w * q.z, 1 - 2 * q.z ** 2)
        self.speed = msg.twist.twist.linear.x

    def _cb_path(self, msg):
        self.ref_path = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]

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

        # 앞 차축 위치
        fx = self.x + self.L * math.cos(self.yaw)
        fy = self.y + self.L * math.sin(self.yaw)

        # 가장 가까운 경로 포인트
        dists = [math.hypot(px - fx, py - fy) for px, py in path]
        nearest = int(np.argmin(dists))

        p1 = np.array(path[nearest])
        p2 = np.array(path[(nearest + 1) % n])

        seg = p2 - p1
        seg_len = np.linalg.norm(seg)
        if seg_len < 1e-6:
            return
        seg_dir = seg / seg_len

        # 경로 heading
        path_yaw = math.atan2(seg[1], seg[0])

        # Heading error (ψ_e)
        psi_e = self._wrap(path_yaw - self.yaw)

        # 횡방향 오차 e
        to_front = np.array([fx - p1[0], fy - p1[1]])
        normal = np.array([-seg_dir[1], seg_dir[0]])
        e = float(np.dot(to_front, normal))

        # Stanley 공식
        v = max(abs(self.speed), 0.01)
        delta = psi_e + math.atan2(self.k * e, v + self.k_soft)
        delta = float(np.clip(delta, -self.max_steer, self.max_steer))

        # 속도 (전방 곡률 기반)
        v_target = self._curvature_speed(path, nearest, n)

        # PID 속도 루프
        err = v_target - abs(self.speed)
        if dt > 0:
            self.pid_i += err * dt
            d_err = (err - self.pid_prev_e) / dt
        else:
            d_err = 0.0
        self.pid_prev_e = err

        v_cmd = float(np.clip(
            abs(self.speed) + self.kp * err + self.ki * self.pid_i + self.kd * d_err,
            self.min_speed, self.max_speed
        ))

        msg = AckermannDriveStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.drive.speed = v_cmd
        msg.drive.steering_angle = delta
        self.drive_pub.publish(msg)

        self._publish_markers(fx, fy, path[nearest], path_yaw, e)

    def _curvature_speed(self, path, nearest, n):
        kappas = []
        for i in range(1, self.spd_pts - 1):
            ia = (nearest + i - 1) % n
            ib = (nearest + i) % n
            ic = (nearest + i + 1) % n
            ax, ay = path[ia]; bx, by = path[ib]; cx, cy = path[ic]
            cross = abs((bx - ax) * (cy - ay) - (by - ay) * (cx - ax))
            denom = math.hypot(bx-ax, by-ay) * math.hypot(cx-bx, cy-by) * math.hypot(cx-ax, cy-ay)
            if denom > 1e-6:
                kappas.append(cross / denom)
        if not kappas:
            return self.target_speed
        kappa = float(np.mean(kappas))
        v = self.target_speed - self.spd_curv_gain * kappa * (self.target_speed - self.min_speed)
        return float(np.clip(v, self.min_speed, self.max_speed))

    def _publish_markers(self, fx, fy, nearest_pt, path_yaw, cte):
        ma = MarkerArray()
        now = self.get_clock().now().to_msg()

        # 앞 차축 위치
        m = Marker()
        m.header.stamp = now; m.header.frame_id = self.frame_id
        m.ns = 'front_axle'; m.id = 0
        m.type = Marker.SPHERE; m.action = Marker.ADD
        m.pose.position.x = fx; m.pose.position.y = fy; m.pose.position.z = 0.1
        m.pose.orientation.w = 1.0
        m.scale.x = m.scale.y = m.scale.z = 0.12
        m.color.a = 1.0; m.color.r = 1.0; m.color.g = 0.5; m.color.b = 0.0
        ma.markers.append(m)

        # CTE 텍스트
        t = Marker()
        t.header = m.header; t.ns = 'cte_text'; t.id = 1
        t.type = Marker.TEXT_VIEW_FACING; t.action = Marker.ADD
        t.pose.position.x = fx; t.pose.position.y = fy; t.pose.position.z = 0.4
        t.pose.orientation.w = 1.0; t.scale.z = 0.12
        t.color.a = 1.0; t.color.r = t.color.g = t.color.b = 1.0
        t.text = f'CTE:{cte:.3f}m'
        ma.markers.append(t)

        self.marker_pub.publish(ma)

    @staticmethod
    def _wrap(a):
        return (a + math.pi) % (2 * math.pi) - math.pi


def main(args=None):
    rclpy.init(args=args)
    node = StanleyControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
