"""Nonlinear MPC raceline tracker (kinematic bicycle, scipy SLSQP).

Ported from the sibling repository's mpc_node.py:
/home/kim/f1tenth/f1tenth_gym_ros_humble/algorithms/control/control/mpc_node.py

The cost function, kinematic-bicycle step, and SLSQP solve are kept
structurally unchanged from that source -- that is the point of comparison
against this repo's cvxpy/OSQP-based linear_mpc_node.py. What changed is the
interface plumbing, to match this repo's actual contract: pose is looked up
via TF (map -> ego_racecar/base_link) instead of trusting odom pose, drive
commands go to /drive, and the node implements the same /control/enable
dry-run/safety pattern as linear_mpc_node.py and pure_pursuit_node.py.

State: [x, y, theta, v]. Input: [delta (steering), a (acceleration)].
"""

import math
import time

import numpy as np
import rclpy
from rclpy.node import Node
from scipy.optimize import minimize

from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool, Float64
from std_srvs.srv import SetBool
from tf2_ros import Buffer, TransformException, TransformListener
from rclpy.duration import Duration

from f1tenth_kkh.common.ackermann import build_ackermann, publish_stop
from f1tenth_kkh.common.pose import lookup_vehicle_pose
from f1tenth_kkh.common.raceline import (
    ClosedRaceline, angle_difference, clamp, path_msg_to_closed_points)
from f1tenth_kkh.common.ros_helpers import WarnThrottle, age_seconds


class NonlinearMpcNode(Node):

    def __init__(self):
        super().__init__('nonlinear_mpc_node')

        # -- Interface-contract parameters (shared shape with the other
        # f1tenth_kkh nodes and with algorithms/control's nodes) ----------
        self.declare_parameter('enabled', False)
        self.declare_parameter('solve_when_disabled', True)
        self.declare_parameter('global_frame_id', 'map')
        self.declare_parameter('base_frame_id', 'ego_racecar/base_link')
        self.declare_parameter('odom_topic', '/ego_racecar/odom')
        self.declare_parameter('path_topic', '/planning/path')
        self.declare_parameter('drive_topic', '/drive')
        self.declare_parameter('collision_topic', '/ego_racecar/collision')

        self.declare_parameter('wheelbase', 0.33)
        self.declare_parameter('horizon_steps', 10)
        self.declare_parameter('dt', 0.10)
        self.declare_parameter('control_rate', 10.0)

        self.declare_parameter('max_path_distance', 0.80)
        self.declare_parameter('max_heading_error', 1.0472)
        self.declare_parameter('odom_timeout', 0.50)
        self.declare_parameter('path_timeout', 2.00)
        self.declare_parameter('search_back_points', 5)
        self.declare_parameter('search_forward_points', 30)

        # -- SLSQP cost / model parameters (ported 1:1 from the source
        # mpc_node.py, renamed only where needed for cross-file consistency)
        self.declare_parameter('max_steering_angle', 0.4189)
        self.declare_parameter('max_speed', 2.0)
        self.declare_parameter('min_speed', 0.3)
        self.declare_parameter('max_acceleration', 2.0)
        self.declare_parameter('target_speed', 1.5)

        self.declare_parameter('w_cte', 2.0)
        self.declare_parameter('w_eth', 1.5)
        self.declare_parameter('w_v', 0.5)
        self.declare_parameter('w_delta', 0.1)
        self.declare_parameter('w_a', 0.05)
        self.declare_parameter('w_ddelta', 2.0)
        self.declare_parameter('w_da', 0.5)

        self.enabled = bool(self.get_parameter('enabled').value)
        self.solve_when_disabled = bool(
            self.get_parameter('solve_when_disabled').value)
        self.global_frame_id = self.get_parameter('global_frame_id').value
        self.base_frame_id = self.get_parameter('base_frame_id').value
        self.odom_topic = self.get_parameter('odom_topic').value
        self.path_topic = self.get_parameter('path_topic').value
        self.drive_topic = self.get_parameter('drive_topic').value
        self.collision_topic = self.get_parameter('collision_topic').value

        self.wheelbase = float(self.get_parameter('wheelbase').value)
        self.horizon = int(self.get_parameter('horizon_steps').value)
        self.dt = float(self.get_parameter('dt').value)
        self.control_rate = float(self.get_parameter('control_rate').value)

        self.max_path_distance = float(
            self.get_parameter('max_path_distance').value)
        self.max_heading_error = float(
            self.get_parameter('max_heading_error').value)
        self.odom_timeout = float(self.get_parameter('odom_timeout').value)
        self.path_timeout = float(self.get_parameter('path_timeout').value)
        search_back_points = int(
            self.get_parameter('search_back_points').value)
        search_forward_points = int(
            self.get_parameter('search_forward_points').value)

        self.max_steer = float(
            self.get_parameter('max_steering_angle').value)
        self.max_speed = float(self.get_parameter('max_speed').value)
        self.min_speed = float(self.get_parameter('min_speed').value)
        self.max_accel = float(
            self.get_parameter('max_acceleration').value)
        self.v_ref = float(self.get_parameter('target_speed').value)

        self.w_cte = float(self.get_parameter('w_cte').value)
        self.w_eth = float(self.get_parameter('w_eth').value)
        self.w_v = float(self.get_parameter('w_v').value)
        self.w_delta = float(self.get_parameter('w_delta').value)
        self.w_a = float(self.get_parameter('w_a').value)
        self.w_ddelta = float(self.get_parameter('w_ddelta').value)
        self.w_da = float(self.get_parameter('w_da').value)

        if self.horizon < 2:
            raise RuntimeError('horizon_steps must be at least 2')
        if self.dt <= 0.0:
            raise RuntimeError('dt must be positive')

        # -- State -----------------------------------------------------
        self.current_odom = None
        self.last_odom_time = None
        self.last_path_time = None
        self.collision = False
        self.raceline = ClosedRaceline(
            search_back_points=search_back_points,
            search_forward_points=search_forward_points)
        self.prev_delta = 0.0
        self.prev_a = 0.0
        self.last_solution_ok = False
        self.consecutive_solver_failures = 0
        self.warn_throttle = WarnThrottle(self.get_logger())

        self.tf_buffer = Buffer(cache_time=Duration(seconds=5.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # -- Pub / Sub ---------------------------------------------------
        self.create_subscription(
            Odometry, self.odom_topic, self.odom_callback, 10)
        self.create_subscription(Path, self.path_topic, self.path_callback, 10)
        self.create_subscription(
            Bool, self.collision_topic, self.collision_callback, 10)

        self.drive_pub = self.create_publisher(
            AckermannDriveStamped, self.drive_topic, 10)
        self.proposed_drive_pub = self.create_publisher(
            AckermannDriveStamped,
            '/f1tenth_kkh/nonlinear_mpc/proposed_drive', 10)
        self.predicted_path_pub = self.create_publisher(
            Path, '/f1tenth_kkh/nonlinear_mpc/predicted_path', 10)
        self.reference_path_pub = self.create_publisher(
            Path, '/f1tenth_kkh/nonlinear_mpc/reference_path', 10)
        self.solve_time_pub = self.create_publisher(
            Float64, '/f1tenth_kkh/nonlinear_mpc/solve_time_ms', 10)
        self.enable_service = self.create_service(
            SetBool, '/control/enable', self.enable_callback)

        self.timer = self.create_timer(
            1.0 / max(self.control_rate, 1.0), self.control_loop)

        self.get_logger().info(
            'Nonlinear MPC (SLSQP) ready (enabled=%s, N=%d, dt=%.2fs, '
            'target=%.2fm/s)' % (
                self.enabled, self.horizon, self.dt, self.v_ref))
        self.get_logger().info(
            'Disabled mode solves and visualizes predictions, but '
            'publishes stop')

    # ------------------------------------------------------------------ #
    # Callbacks
    # ------------------------------------------------------------------ #
    def odom_callback(self, msg):
        self.current_odom = msg
        self.last_odom_time = self.get_clock().now()

    def collision_callback(self, msg):
        self.collision = bool(msg.data)
        if self.collision and self.enabled:
            self.enabled = False
            self._publish_stop()
            self.get_logger().error(
                'Nonlinear MPC disabled: simulator collision reported')

    def path_callback(self, msg):
        points = path_msg_to_closed_points(msg, min_points=4)
        if points is None:
            return

        changed = (
            not self.raceline.ready
            or len(points) != len(self.raceline.points)
            or np.max(np.abs(points - self.raceline.points)) > 1e-6
        )
        if changed:
            try:
                self.raceline.update(points)
            except RuntimeError as error:
                self.get_logger().error(
                    'Nonlinear MPC rejected path: %s' % error)
                return
            self.get_logger().info(
                'Nonlinear MPC received closed path: %d points, %.2f m'
                % (len(points), self.raceline.length))
        self.last_path_time = self.get_clock().now()

    # ------------------------------------------------------------------ #
    # Readiness / enable
    # ------------------------------------------------------------------ #
    def readiness_problem(self):
        if not self.raceline.ready:
            return 'no global path'
        if age_seconds(self.get_clock(), self.last_path_time) \
                > self.path_timeout:
            return 'global path is stale'
        if self.current_odom is None:
            return 'no odometry'
        if age_seconds(self.get_clock(), self.last_odom_time) \
                > self.odom_timeout:
            return 'odometry is stale'
        if self.collision:
            return 'collision is active; reset simulator pose first'
        return None

    def enable_callback(self, request, response):
        if not request.data:
            self.enabled = False
            self.prev_delta = 0.0
            self.prev_a = 0.0
            self._publish_stop()
            response.success = True
            response.message = 'Nonlinear MPC stopped'
            self.get_logger().info(response.message)
            return response

        problem = self.readiness_problem()
        if problem is not None:
            response.success = False
            response.message = 'Cannot start Nonlinear MPC: ' + problem
            self.get_logger().error(response.message)
            return response
        if not self.last_solution_ok:
            response.success = False
            response.message = (
                'Cannot start Nonlinear MPC: no valid dry-run solution yet')
            self.get_logger().error(response.message)
            return response

        self.enabled = True
        self.consecutive_solver_failures = 0
        response.success = True
        response.message = 'Nonlinear MPC enabled'
        self.get_logger().info(response.message)
        return response

    # ------------------------------------------------------------------ #
    # Reference extraction (arc-length correct, unlike the source's
    # raw-index stride; see MPC_GUIDE-adjacent design notes in the plan)
    # ------------------------------------------------------------------ #
    def _build_reference(self, state0):
        x, y, theta, _v = state0
        nearest, distance, path_heading, s0 = self.raceline.nearest_state(
            x, y)
        heading_error = angle_difference(path_heading, theta)
        if distance > self.max_path_distance:
            raise RuntimeError(
                'vehicle is %.2f m from path (limit %.2f m)'
                % (distance, self.max_path_distance))
        if abs(heading_error) > self.max_heading_error:
            raise RuntimeError(
                'heading error is %.1f deg (limit %.1f deg)'
                % (math.degrees(abs(heading_error)),
                   math.degrees(self.max_heading_error)))

        sample_s = s0 + np.arange(self.horizon) * self.v_ref * self.dt
        ref_x = self.raceline.interpolate(self.raceline.points[:, 0], sample_s)
        ref_y = self.raceline.interpolate(self.raceline.points[:, 1], sample_s)
        ref_yaw = self.raceline.interpolate(
            self.raceline.yaw, sample_s, self.raceline.yaw_lap_change)
        return list(zip(ref_x.tolist(), ref_y.tolist(), ref_yaw.tolist()))

    # ------------------------------------------------------------------ #
    # MPC optimization -- ported unchanged from the source mpc_node.py
    # ------------------------------------------------------------------ #
    def _solve_mpc(self, state0, ref):
        started = time.perf_counter()
        N = self.horizon
        u0 = np.zeros(2 * N)
        u0[:N] = self.prev_delta
        u0[N:] = self.prev_a

        bounds = (
            [(-self.max_steer, self.max_steer)] * N
            + [(-self.max_accel, self.max_accel)] * N
        )

        result = minimize(
            self._cost,
            u0,
            args=(state0, ref),
            method='SLSQP',
            bounds=bounds,
            options={'maxiter': 50, 'ftol': 1e-4}
        )

        solve_ms = (time.perf_counter() - started) * 1000.0
        u_opt = result.x
        return np.array([u_opt[0], u_opt[N]]), solve_ms

    def _cost(self, u_flat, state0, ref):
        N = self.horizon
        deltas = u_flat[:N]
        accels = u_flat[N:]

        state = state0.copy()
        cost = 0.0

        for k in range(N):
            rx, ry, ryaw = ref[k]
            x, y, theta, v = state

            cte = (y - ry) * math.cos(ryaw) - (x - rx) * math.sin(ryaw)
            eth = angle_difference(theta, ryaw)

            cost += (
                self.w_cte * cte ** 2
                + self.w_eth * eth ** 2
                + self.w_v * (v - self.v_ref) ** 2
                + self.w_delta * deltas[k] ** 2
                + self.w_a * accels[k] ** 2
            )

            if k == 0:
                cost += self.w_ddelta * (deltas[k] - self.prev_delta) ** 2
                cost += self.w_da * (accels[k] - self.prev_a) ** 2
            else:
                cost += self.w_ddelta * (deltas[k] - deltas[k - 1]) ** 2
                cost += self.w_da * (accels[k] - accels[k - 1]) ** 2

            state = self._step(state, deltas[k], accels[k])

        return cost

    def _step(self, state, delta, a):
        x, y, theta, v = state
        v_new = clamp(v + a * self.dt, self.min_speed, self.max_speed)
        x_new = x + v * math.cos(theta) * self.dt
        y_new = y + v * math.sin(theta) * self.dt
        theta_new = self._wrap(
            theta + v * math.tan(delta) / self.wheelbase * self.dt)
        return np.array([x_new, y_new, theta_new, v_new])

    @staticmethod
    def _wrap(angle):
        return (angle + math.pi) % (2 * math.pi) - math.pi

    # ------------------------------------------------------------------ #
    # Publishing
    # ------------------------------------------------------------------ #
    def _publish_stop(self):
        publish_stop(self.drive_pub, self.get_clock(), self.base_frame_id)

    def _publish_prediction(self, state0, u_opt):
        # Only the first control input is known (SLSQP returns the full
        # optimized sequence internally, but _solve_mpc only surfaces the
        # first step, matching the source mpc_node.py); the rollout below
        # repeats it, exactly as the source's own visualization did.
        N = self.horizon
        deltas = [u_opt[0]] * N
        accels = [u_opt[1]] * N

        message = Path()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = self.global_frame_id
        state = state0.copy()
        for k in range(N):
            pose = PoseStamped()
            pose.header = message.header
            pose.pose.position.x = float(state[0])
            pose.pose.position.y = float(state[1])
            yaw = float(state[2])
            pose.pose.orientation.z = math.sin(yaw * 0.5)
            pose.pose.orientation.w = math.cos(yaw * 0.5)
            message.poses.append(pose)
            state = self._step(state, deltas[k], accels[k])
        self.predicted_path_pub.publish(message)

    def _publish_reference(self, ref):
        message = Path()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = self.global_frame_id
        for rx, ry, ryaw in ref:
            pose = PoseStamped()
            pose.header = message.header
            pose.pose.position.x = float(rx)
            pose.pose.position.y = float(ry)
            pose.pose.orientation.z = math.sin(ryaw * 0.5)
            pose.pose.orientation.w = math.cos(ryaw * 0.5)
            message.poses.append(pose)
        self.reference_path_pub.publish(message)

    # ------------------------------------------------------------------ #
    # Control loop
    # ------------------------------------------------------------------ #
    def control_loop(self):
        problem = self.readiness_problem()
        if problem is not None:
            self.last_solution_ok = False
            self._publish_stop()
            if self.enabled:
                self.warn_throttle.warn(
                    self.get_clock(), 'Nonlinear MPC safety stop: ' + problem)
            return
        if not self.enabled and not self.solve_when_disabled:
            self._publish_stop()
            return

        try:
            x, y, theta = lookup_vehicle_pose(
                self.tf_buffer, self.global_frame_id, self.base_frame_id)
            v = max(0.0, float(self.current_odom.twist.twist.linear.x))
            state0 = np.array([x, y, theta, v])

            ref = self._build_reference(state0)
            u_opt, solve_ms = self._solve_mpc(state0, ref)

            solve_msg = Float64()
            solve_msg.data = solve_ms
            self.solve_time_pub.publish(solve_msg)

            self._publish_prediction(state0, u_opt)
            self._publish_reference(ref)

            self.last_solution_ok = True
            self.consecutive_solver_failures = 0

            delta = clamp(float(u_opt[0]), -self.max_steer, self.max_steer)
            a = clamp(float(u_opt[1]), -self.max_accel, self.max_accel)
            v_cmd = clamp(v + a * self.dt, self.min_speed, self.max_speed)

            self.proposed_drive_pub.publish(
                build_ackermann(self.get_clock(), self.base_frame_id,
                                 v_cmd, delta))

            if not self.enabled:
                self._publish_stop()
                return

            self.prev_delta = delta
            self.prev_a = a
            self.drive_pub.publish(
                build_ackermann(self.get_clock(), self.base_frame_id,
                                 v_cmd, delta))

            if solve_ms > 1000.0 / self.control_rate:
                self.warn_throttle.warn(
                    self.get_clock(),
                    'Nonlinear MPC solve time %.1f ms exceeds %.0f Hz period'
                    % (solve_ms, self.control_rate))
        except (TransformException, RuntimeError) as error:
            self.last_solution_ok = False
            self.consecutive_solver_failures += 1
            self._publish_stop()
            self.warn_throttle.warn(
                self.get_clock(), 'Nonlinear MPC safety stop: ' + str(error))
            if self.enabled and self.consecutive_solver_failures >= 3:
                self.enabled = False
                self.get_logger().error(
                    'Nonlinear MPC disabled after 3 consecutive '
                    'solver/control failures')


def main(args=None):
    rclpy.init(args=args)
    node = NonlinearMpcNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._publish_stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
