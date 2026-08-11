"""Minimal "stock" Linear MPC raceline tracker (kinematic bicycle, cvxpy/OSQP).

Same algorithmic family as algorithms/control/control/linear_mpc_node.py
(kinematic-bicycle model, per-step linearization, cvxpy+OSQP QP), modeled
after the plain formulation taught in the official F1TENTH course labs
(github.com/f1tenth/f1tenth_lab8_template): flat target-speed reference
spacing (no curvature-based slowdown), plain Q/R/Qf cost with no
control-rate penalty or steering-rate constraint, and no persisted
failure-counter/auto-disable state. It exists as a "did the team's tuning
actually help" sanity check against the tuned linear_mpc_node.py baseline,
not as a competition candidate in its own right.

State: [x, y, v, yaw]. Input: [acceleration, steering_angle].
"""

import math
import time

import cvxpy as cp
import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node

from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import Bool, Float64
from std_srvs.srv import SetBool
from tf2_ros import Buffer, TransformException, TransformListener

from f1tenth_kkh.common.ackermann import build_ackermann, publish_stop
from f1tenth_kkh.common.pose import lookup_vehicle_pose
from f1tenth_kkh.common.raceline import (
    ClosedRaceline, angle_difference, clamp, path_msg_to_closed_points)
from f1tenth_kkh.common.ros_helpers import WarnThrottle, age_seconds


class Lab8MpcNode(Node):
    STATE_SIZE = 4  # x, y, v, yaw
    INPUT_SIZE = 2  # acceleration, steering angle

    def __init__(self):
        super().__init__('lab8_mpc_node')

        self.declare_parameter('enabled', False)
        self.declare_parameter('solve_when_disabled', True)
        self.declare_parameter('global_frame_id', 'map')
        self.declare_parameter('base_frame_id', 'ego_racecar/base_link')
        self.declare_parameter('odom_topic', '/ego_racecar/odom')
        self.declare_parameter('path_topic', '/planning/path')
        self.declare_parameter('drive_topic', '/drive')
        self.declare_parameter('collision_topic', '/ego_racecar/collision')

        self.declare_parameter('wheelbase', 0.33)
        self.declare_parameter('horizon_steps', 12)
        self.declare_parameter('dt', 0.10)
        self.declare_parameter('control_rate', 10.0)

        self.declare_parameter('target_speed', 0.55)
        self.declare_parameter('max_speed', 0.80)
        self.declare_parameter('max_acceleration', 1.50)
        self.declare_parameter('max_steering_angle', 0.4189)

        self.declare_parameter('q_x', 20.0)
        self.declare_parameter('q_y', 20.0)
        self.declare_parameter('q_speed', 3.0)
        self.declare_parameter('q_yaw', 8.0)
        self.declare_parameter('qf_scale', 1.5)
        self.declare_parameter('r_acceleration', 0.20)
        self.declare_parameter('r_steering', 1.00)

        self.declare_parameter('max_path_distance', 0.80)
        self.declare_parameter('max_heading_error', 1.0472)
        self.declare_parameter('odom_timeout', 0.50)
        self.declare_parameter('path_timeout', 2.00)
        self.declare_parameter('search_back_points', 5)
        self.declare_parameter('search_forward_points', 30)

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

        self.target_speed = float(self.get_parameter('target_speed').value)
        self.max_speed = float(self.get_parameter('max_speed').value)
        self.max_acceleration = float(
            self.get_parameter('max_acceleration').value)
        self.max_steering_angle = float(
            self.get_parameter('max_steering_angle').value)

        self.q = np.diag([
            float(self.get_parameter('q_x').value),
            float(self.get_parameter('q_y').value),
            float(self.get_parameter('q_speed').value),
            float(self.get_parameter('q_yaw').value),
        ])
        self.qf = self.q * float(self.get_parameter('qf_scale').value)
        self.r = np.diag([
            float(self.get_parameter('r_acceleration').value),
            float(self.get_parameter('r_steering').value),
        ])

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

        if self.horizon < 2:
            raise RuntimeError('horizon_steps must be at least 2')
        if self.dt <= 0.0:
            raise RuntimeError('dt must be positive')

        self.current_odom = None
        self.last_odom_time = None
        self.last_path_time = None
        self.collision = False
        self.raceline = ClosedRaceline(
            search_back_points=search_back_points,
            search_forward_points=search_forward_points)
        self.warn_throttle = WarnThrottle(self.get_logger())

        self.tf_buffer = Buffer(cache_time=Duration(seconds=5.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.create_subscription(
            Odometry, self.odom_topic, self.odom_callback, 10)
        self.create_subscription(Path, self.path_topic, self.path_callback, 10)
        self.create_subscription(
            Bool, self.collision_topic, self.collision_callback, 10)

        self.drive_pub = self.create_publisher(
            AckermannDriveStamped, self.drive_topic, 10)
        self.proposed_drive_pub = self.create_publisher(
            AckermannDriveStamped,
            '/f1tenth_kkh/lab8_mpc/proposed_drive', 10)
        self.predicted_path_pub = self.create_publisher(
            Path, '/f1tenth_kkh/lab8_mpc/predicted_path', 10)
        self.reference_path_pub = self.create_publisher(
            Path, '/f1tenth_kkh/lab8_mpc/reference_path', 10)
        self.solve_time_pub = self.create_publisher(
            Float64, '/f1tenth_kkh/lab8_mpc/solve_time_ms', 10)
        self.enable_service = self.create_service(
            SetBool, '/control/enable', self.enable_callback)

        self.build_mpc_problem()
        self.timer = self.create_timer(
            1.0 / max(self.control_rate, 1.0), self.control_loop)

        self.get_logger().info(
            'Lab8-style stock MPC ready (enabled=%s, N=%d, dt=%.2fs, '
            'target=%.2fm/s)'
            % (self.enabled, self.horizon, self.dt, self.target_speed))
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
                'Lab8 MPC disabled: simulator collision reported')

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
                    'Lab8 MPC rejected path: %s' % error)
                return
            self.get_logger().info(
                'Lab8 MPC received closed path: %d points, %.2f m'
                % (len(points), self.raceline.length))
        self.last_path_time = self.get_clock().now()

    # ------------------------------------------------------------------ #
    # Readiness / enable (minimal: no last-solution / failure-count gate)
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
            self._publish_stop()
            response.success = True
            response.message = 'Lab8 MPC stopped'
            self.get_logger().info(response.message)
            return response

        problem = self.readiness_problem()
        if problem is not None:
            response.success = False
            response.message = 'Cannot start Lab8 MPC: ' + problem
            self.get_logger().error(response.message)
            return response

        self.enabled = True
        response.success = True
        response.message = 'Lab8 MPC enabled'
        self.get_logger().info(response.message)
        return response

    # ------------------------------------------------------------------ #
    # QP problem (built once; only cp.Parameter values change per solve)
    # ------------------------------------------------------------------ #
    def build_mpc_problem(self):
        n = self.horizon
        self.state_variable = cp.Variable((self.STATE_SIZE, n + 1))
        self.input_variable = cp.Variable((self.INPUT_SIZE, n))
        self.initial_state_parameter = cp.Parameter(self.STATE_SIZE)
        self.reference_state_parameter = cp.Parameter((self.STATE_SIZE, n + 1))
        self.a_parameters = [
            cp.Parameter((self.STATE_SIZE, self.STATE_SIZE))
            for _ in range(n)
        ]
        self.b_parameters = [
            cp.Parameter((self.STATE_SIZE, self.INPUT_SIZE))
            for _ in range(n)
        ]
        self.c_parameters = [cp.Parameter(self.STATE_SIZE) for _ in range(n)]

        objective = 0.0
        constraints = [
            self.state_variable[:, 0] == self.initial_state_parameter,
            self.state_variable[2, :] >= 0.0,
            self.state_variable[2, :] <= self.max_speed,
        ]

        for step in range(n):
            state_error = (
                self.state_variable[:, step]
                - self.reference_state_parameter[:, step])
            objective += cp.quad_form(state_error, self.q)
            objective += cp.quad_form(self.input_variable[:, step], self.r)
            constraints += [
                self.state_variable[:, step + 1]
                == self.a_parameters[step] @ self.state_variable[:, step]
                + self.b_parameters[step] @ self.input_variable[:, step]
                + self.c_parameters[step],
                cp.abs(self.input_variable[0, step])
                <= self.max_acceleration,
                cp.abs(self.input_variable[1, step])
                <= self.max_steering_angle,
            ]

        terminal_error = (
            self.state_variable[:, n]
            - self.reference_state_parameter[:, n])
        objective += cp.quad_form(terminal_error, self.qf)
        self.problem = cp.Problem(cp.Minimize(objective), constraints)

    def model_matrices(self, state, control):
        _, _, speed, yaw = state
        _accel, steering = control
        dt = self.dt
        wheelbase = self.wheelbase

        a_matrix = np.eye(self.STATE_SIZE)
        a_matrix[0, 2] = dt * math.cos(yaw)
        a_matrix[0, 3] = -dt * speed * math.sin(yaw)
        a_matrix[1, 2] = dt * math.sin(yaw)
        a_matrix[1, 3] = dt * speed * math.cos(yaw)
        a_matrix[3, 2] = dt * math.tan(steering) / wheelbase

        b_matrix = np.zeros((self.STATE_SIZE, self.INPUT_SIZE))
        b_matrix[2, 0] = dt
        b_matrix[3, 1] = dt * speed / (wheelbase * math.cos(steering) ** 2)

        nonlinear_next = np.array([
            state[0] + dt * speed * math.cos(yaw),
            state[1] + dt * speed * math.sin(yaw),
            speed,
            yaw + dt * speed * math.tan(steering) / wheelbase,
        ])
        c_vector = nonlinear_next - a_matrix @ state - b_matrix @ control
        return a_matrix, b_matrix, c_vector

    def solve_mpc(self, current_state, reference):
        self.initial_state_parameter.value = current_state
        self.reference_state_parameter.value = reference

        zero_input = np.zeros(self.INPUT_SIZE)
        for step in range(self.horizon):
            matrices = self.model_matrices(reference[:, step], zero_input)
            self.a_parameters[step].value = matrices[0]
            self.b_parameters[step].value = matrices[1]
            self.c_parameters[step].value = matrices[2]

        started = time.perf_counter()
        self.problem.solve(solver=cp.OSQP, warm_start=True, verbose=False)
        solve_ms = (time.perf_counter() - started) * 1000.0

        if self.problem.status not in (cp.OPTIMAL, cp.OPTIMAL_INACCURATE):
            raise RuntimeError('solver status: ' + str(self.problem.status))
        if self.input_variable.value is None \
                or self.state_variable.value is None:
            raise RuntimeError('solver returned no solution')

        return (
            np.asarray(self.input_variable.value),
            np.asarray(self.state_variable.value),
            solve_ms,
        )

    # ------------------------------------------------------------------ #
    # Reference: flat target-speed spacing, no curvature-based slowdown
    # ------------------------------------------------------------------ #
    def build_reference(self, x, y, yaw):
        nearest, distance, path_heading, start_s = self.raceline.nearest_state(
            x, y)
        heading_error = angle_difference(path_heading, yaw)
        if distance > self.max_path_distance:
            raise RuntimeError(
                'vehicle is %.2f m from path (limit %.2f m)'
                % (distance, self.max_path_distance))
        if abs(heading_error) > self.max_heading_error:
            raise RuntimeError(
                'heading error is %.1f deg (limit %.1f deg)'
                % (math.degrees(abs(heading_error)),
                   math.degrees(self.max_heading_error)))

        sample_s = start_s + np.arange(
            self.horizon + 1) * self.target_speed * self.dt

        reference = np.zeros((self.STATE_SIZE, self.horizon + 1))
        reference[0] = self.raceline.interpolate(
            self.raceline.points[:, 0], sample_s)
        reference[1] = self.raceline.interpolate(
            self.raceline.points[:, 1], sample_s)
        reference[2] = clamp(self.target_speed, 0.0, self.max_speed)
        reference[3] = self.raceline.interpolate(
            self.raceline.yaw, sample_s, self.raceline.yaw_lap_change)
        reference[3] += 2.0 * math.pi * round(
            (yaw - reference[3, 0]) / (2.0 * math.pi))
        return reference, distance

    # ------------------------------------------------------------------ #
    # Publishing
    # ------------------------------------------------------------------ #
    def _publish_stop(self):
        publish_stop(self.drive_pub, self.get_clock(), self.base_frame_id)

    def publish_path(self, publisher, states):
        message = Path()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = self.global_frame_id
        for index in range(states.shape[1]):
            pose = PoseStamped()
            pose.header = message.header
            pose.pose.position.x = float(states[0, index])
            pose.pose.position.y = float(states[1, index])
            yaw = float(states[3, index])
            pose.pose.orientation.z = math.sin(yaw * 0.5)
            pose.pose.orientation.w = math.cos(yaw * 0.5)
            message.poses.append(pose)
        publisher.publish(message)

    # ------------------------------------------------------------------ #
    # Control loop
    # ------------------------------------------------------------------ #
    def control_loop(self):
        problem = self.readiness_problem()
        if problem is not None:
            self._publish_stop()
            if self.enabled:
                self.warn_throttle.warn(
                    self.get_clock(), 'Lab8 MPC safety stop: ' + problem)
            return
        if not self.enabled and not self.solve_when_disabled:
            self._publish_stop()
            return

        try:
            x, y, yaw = lookup_vehicle_pose(
                self.tf_buffer, self.global_frame_id, self.base_frame_id)
            speed = max(0.0, float(self.current_odom.twist.twist.linear.x))
            reference, _distance = self.build_reference(x, y, yaw)
            yaw = reference[3, 0] + angle_difference(yaw, reference[3, 0])
            current_state = np.array([x, y, speed, yaw])

            control, predicted, solve_ms = self.solve_mpc(
                current_state, reference)

            solve_msg = Float64()
            solve_msg.data = solve_ms
            self.solve_time_pub.publish(solve_msg)

            self.publish_path(self.reference_path_pub, reference)
            self.publish_path(self.predicted_path_pub, predicted)

            acceleration = float(control[0, 0])
            steering = clamp(
                float(control[1, 0]),
                -self.max_steering_angle, self.max_steering_angle)
            command_speed = clamp(
                speed + acceleration * self.dt, 0.0, self.max_speed)

            self.proposed_drive_pub.publish(
                build_ackermann(self.get_clock(), self.base_frame_id,
                                 command_speed, steering))

            if not self.enabled:
                self._publish_stop()
                return

            self.drive_pub.publish(
                build_ackermann(self.get_clock(), self.base_frame_id,
                                 command_speed, steering))

            if solve_ms > 1000.0 / self.control_rate:
                self.warn_throttle.warn(
                    self.get_clock(),
                    'Lab8 MPC solve time %.1f ms exceeds %.0f Hz period'
                    % (solve_ms, self.control_rate))
        except (TransformException, RuntimeError, cp.error.SolverError) \
                as error:
            self._publish_stop()
            self.warn_throttle.warn(
                self.get_clock(), 'Lab8 MPC safety stop: ' + str(error))


def main(args=None):
    rclpy.init(args=args)
    node = Lab8MpcNode()
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
