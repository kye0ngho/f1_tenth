import math

import numpy as np
from scipy.optimize import minimize

import rclpy
from rclpy.node import Node

from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry, Path
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point


class MPCNode(Node):
    """
    Model Predictive Control — Kinematic Bicycle Model.

    상태: [x, y, theta, v]
    제어: [delta(조향각), a(가속도)]  horizon N 스텝

    비용 함수:
      Σ w_cte*cte² + w_eth*eth² + w_v*(v-v_ref)²
      + w_delta*delta² + w_a*a²
      + w_ddelta*(Δdelta)² + w_da*(Δa)²

    scipy SLSQP로 매 scan마다 최적화 → 첫 번째 제어 입력 적용.
    """

    def __init__(self):
        super().__init__('mpc_node')

        # ── Parameters ──────────────────────────────────────────────
        self.declare_parameter('odom_topic', '/localization/odom')
        self.declare_parameter('path_topic', '/planning/path')
        self.declare_parameter('drive_topic', '/mpc/drive')
        self.declare_parameter('pred_path_topic', '/mpc/predicted_path')

        self.declare_parameter('wheelbase', 0.33)
        self.declare_parameter('dt', 0.1)
        self.declare_parameter('horizon', 10)

        self.declare_parameter('max_steering', 0.4189)
        self.declare_parameter('max_speed', 2.0)
        self.declare_parameter('min_speed', 0.3)
        self.declare_parameter('max_accel', 2.0)
        self.declare_parameter('target_speed', 1.5)

        self.declare_parameter('w_cte', 2.0)
        self.declare_parameter('w_eth', 1.5)
        self.declare_parameter('w_v', 0.5)
        self.declare_parameter('w_delta', 0.1)
        self.declare_parameter('w_a', 0.05)
        self.declare_parameter('w_ddelta', 2.0)
        self.declare_parameter('w_da', 0.5)

        self.declare_parameter('control_rate', 20.0)
        self.declare_parameter('frame_id', 'map')

        # ── Get params ──────────────────────────────────────────────
        self.odom_topic = self.get_parameter('odom_topic').value
        self.path_topic = self.get_parameter('path_topic').value
        self.drive_topic = self.get_parameter('drive_topic').value
        self.pred_topic = self.get_parameter('pred_path_topic').value
        self.frame_id = self.get_parameter('frame_id').value

        self.L = float(self.get_parameter('wheelbase').value)
        self.dt = float(self.get_parameter('dt').value)
        self.N = int(self.get_parameter('horizon').value)

        self.max_steer = float(self.get_parameter('max_steering').value)
        self.max_speed = float(self.get_parameter('max_speed').value)
        self.min_speed = float(self.get_parameter('min_speed').value)
        self.max_accel = float(self.get_parameter('max_accel').value)
        self.v_ref = float(self.get_parameter('target_speed').value)

        self.w_cte = float(self.get_parameter('w_cte').value)
        self.w_eth = float(self.get_parameter('w_eth').value)
        self.w_v = float(self.get_parameter('w_v').value)
        self.w_delta = float(self.get_parameter('w_delta').value)
        self.w_a = float(self.get_parameter('w_a').value)
        self.w_ddelta = float(self.get_parameter('w_ddelta').value)
        self.w_da = float(self.get_parameter('w_da').value)

        control_rate = float(self.get_parameter('control_rate').value)

        # ── State ───────────────────────────────────────────────────
        self.current_state = None    # [x, y, theta, v]
        self.ref_path = None         # list of (x, y, yaw)
        self.prev_delta = 0.0
        self.prev_a = 0.0

        # ── Pub / Sub ────────────────────────────────────────────────
        self.create_subscription(Odometry, self.odom_topic, self._cb_odom, 10)
        self.create_subscription(Path, self.path_topic, self._cb_path, 10)
        self.drive_pub = self.create_publisher(AckermannDriveStamped, self.drive_topic, 10)
        self.pred_pub = self.create_publisher(MarkerArray, self.pred_topic, 10)

        self.create_timer(1.0 / control_rate, self._control_loop)

        self.get_logger().info('mpc_node started')
        self.get_logger().info(f'  drive_topic  : {self.drive_topic}')
        self.get_logger().info(f'  horizon N    : {self.N}  dt={self.dt} s')
        self.get_logger().info(f'  wheelbase    : {self.L} m')
        self.get_logger().info(f'  speed range  : {self.min_speed}~{self.max_speed} m/s')
        self.get_logger().info(f'  target speed : {self.v_ref} m/s')

    # ------------------------------------------------------------------ #
    # Callbacks
    # ------------------------------------------------------------------ #
    def _cb_odom(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        theta = math.atan2(2 * q.w * q.z, 1 - 2 * q.z ** 2)
        v = msg.twist.twist.linear.x
        self.current_state = np.array([x, y, theta, v])

    def _cb_path(self, msg):
        self.ref_path = [
            (p.pose.position.x,
             p.pose.position.y,
             math.atan2(
                 2 * p.pose.orientation.w * p.pose.orientation.z,
                 1 - 2 * p.pose.orientation.z ** 2
             ))
            for p in msg.poses
        ]

    # ------------------------------------------------------------------ #
    # Control loop
    # ------------------------------------------------------------------ #
    def _control_loop(self):
        if self.current_state is None or self.ref_path is None:
            return

        ref = self._get_reference_points()
        if ref is None:
            self._publish_stop()
            return

        u_opt = self._solve_mpc(self.current_state, ref)
        delta = float(np.clip(u_opt[0], -self.max_steer, self.max_steer))
        a = float(np.clip(u_opt[1], -self.max_accel, self.max_accel))

        v_cmd = float(np.clip(
            self.current_state[3] + a * self.dt,
            self.min_speed, self.max_speed
        ))

        self.prev_delta = delta
        self.prev_a = a

        msg = AckermannDriveStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.drive.speed = v_cmd
        msg.drive.steering_angle = delta
        self.drive_pub.publish(msg)

        self._publish_prediction(self.current_state, u_opt)

    # ------------------------------------------------------------------ #
    # Reference extraction
    # ------------------------------------------------------------------ #
    def _get_reference_points(self):
        x, y, theta, v = self.current_state
        path = self.ref_path

        # 가장 가까운 웨이포인트 찾기
        dists = [math.hypot(p[0] - x, p[1] - y) for p in path]
        nearest = int(np.argmin(dists))

        # horizon 개수만큼 전방 포인트 추출
        n = len(path)
        stride = max(1, int(v * self.dt / 0.2))   # 속도에 따라 샘플 간격 조정
        ref = []
        for i in range(self.N):
            idx = (nearest + i * stride) % n
            ref.append(path[idx])

        return ref

    # ------------------------------------------------------------------ #
    # MPC optimization
    # ------------------------------------------------------------------ #
    def _solve_mpc(self, state0, ref):
        N = self.N
        # 최적화 변수: [delta_0,...,delta_{N-1}, a_0,...,a_{N-1}]
        u0 = np.zeros(2 * N)
        u0[:N] = self.prev_delta
        u0[N:] = self.prev_a

        bounds = (
            [(-self.max_steer, self.max_steer)] * N +
            [(-self.max_accel, self.max_accel)] * N
        )

        result = minimize(
            self._cost,
            u0,
            args=(state0, ref),
            method='SLSQP',
            bounds=bounds,
            options={'maxiter': 50, 'ftol': 1e-4}
        )

        u_opt = result.x
        return np.array([u_opt[0], u_opt[N]])  # 첫 번째 제어 입력

    def _cost(self, u_flat, state0, ref):
        N = self.N
        deltas = u_flat[:N]
        accels = u_flat[N:]

        state = state0.copy()
        cost = 0.0

        for k in range(N):
            rx, ry, ryaw = ref[k]
            x, y, theta, v = state

            # Cross-track error: 참조점과의 수직 거리
            cte = (y - ry) * math.cos(ryaw) - (x - rx) * math.sin(ryaw)
            # Heading error
            eth = self._wrap(theta - ryaw)

            cost += (
                self.w_cte * cte ** 2 +
                self.w_eth * eth ** 2 +
                self.w_v * (v - self.v_ref) ** 2 +
                self.w_delta * deltas[k] ** 2 +
                self.w_a * accels[k] ** 2
            )

            # 제어 변화량 페널티
            if k == 0:
                cost += self.w_ddelta * (deltas[k] - self.prev_delta) ** 2
                cost += self.w_da * (accels[k] - self.prev_a) ** 2
            else:
                cost += self.w_ddelta * (deltas[k] - deltas[k - 1]) ** 2
                cost += self.w_da * (accels[k] - accels[k - 1]) ** 2

            # 상태 전이 (Kinematic Bicycle Model)
            state = self._step(state, deltas[k], accels[k])

        return cost

    def _step(self, state, delta, a):
        x, y, theta, v = state
        v_new = float(np.clip(v + a * self.dt, self.min_speed, self.max_speed))
        x_new = x + v * math.cos(theta) * self.dt
        y_new = y + v * math.sin(theta) * self.dt
        theta_new = self._wrap(theta + v * math.tan(delta) / self.L * self.dt)
        return np.array([x_new, y_new, theta_new, v_new])

    # ------------------------------------------------------------------ #
    # Visualization
    # ------------------------------------------------------------------ #
    def _publish_prediction(self, state0, u_opt):
        N = self.N
        deltas = [u_opt[0]] * N   # 첫 번째 최적 입력만 알고 있으므로 근사
        accels = [u_opt[1]] * N

        ma = MarkerArray()
        line = Marker()
        line.header.stamp = self.get_clock().now().to_msg()
        line.header.frame_id = self.frame_id
        line.ns = 'mpc_pred'
        line.id = 0
        line.type = Marker.LINE_STRIP
        line.action = Marker.ADD
        line.scale.x = 0.06
        line.color.a = 1.0
        line.color.r = 1.0
        line.color.g = 1.0
        line.color.b = 0.0

        state = state0.copy()
        for k in range(N):
            p = Point()
            p.x = float(state[0])
            p.y = float(state[1])
            p.z = 0.1
            line.points.append(p)
            state = self._step(state, deltas[k], accels[k])

        ma.markers.append(line)
        self.pred_pub.publish(ma)

    def _publish_stop(self):
        msg = AckermannDriveStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.drive.speed = 0.0
        msg.drive.steering_angle = 0.0
        self.drive_pub.publish(msg)

    @staticmethod
    def _wrap(angle):
        return (angle + math.pi) % (2 * math.pi) - math.pi


def main(args=None):
    rclpy.init(args=args)
    node = MPCNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
