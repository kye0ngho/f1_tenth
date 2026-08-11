"""Model Predictive Contouring Control (MPCC) raceline tracker (cvxpy/OSQP).

New formulation (no direct code to port -- only the cost/state-augmentation
concept popularized by alexliniger/MPCC, ETH Zurich), built on top of this
repo's existing kinematic-bicycle linearization
(algorithms/control/control/linear_mpc_node.py's model_matrices/
build_reference pattern). Unlike point-tracking MPC, MPCC augments the state
with a progress variable theta (arc-length along the raceline) and rewards
progress rate directly, which tends to make it faster/more aggressive than
tracking a fixed-spacing reference -- and, because it is already
progress-parameterized, is the natural formulation to later add
opponent/obstacle keep-out constraints to for the competition's
Head-to-Head event (see the reserved, currently-inert obstacle_* parameters
and update_obstacle_constraints() below -- NOT implemented this round).

State (5): [x, y, v, yaw, theta]. Input (3): [acceleration, steering, v_theta].
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

_UNSET = -1.0  # sentinel meaning "derive default from max_speed"


class MpccNode(Node):
    STATE_SIZE = 5  # x, y, v, yaw, theta (progress)
    INPUT_SIZE = 3  # acceleration, steering, v_theta (progress rate)

    def __init__(self):
        super().__init__('mpcc_node')

        self.declare_parameter('enabled', False)
        self.declare_parameter('solve_when_disabled', True)
        self.declare_parameter('global_frame_id', 'map')
        self.declare_parameter('base_frame_id', 'ego_racecar/base_link')
        self.declare_parameter('odom_topic', '/ego_racecar/odom')
        self.declare_parameter('path_topic', '/planning/path')
        self.declare_parameter('drive_topic', '/drive')
        self.declare_parameter('collision_topic', '/ego_racecar/collision')

        self.declare_parameter('wheelbase', 0.33)
        self.declare_parameter('horizon_steps', 14)
        self.declare_parameter('dt', 0.10)
        self.declare_parameter('control_rate', 10.0)

        self.declare_parameter('target_speed', 0.55)
        self.declare_parameter('min_reference_speed', 0.30)
        self.declare_parameter('corner_slowdown_gain', 0.35)
        self.declare_parameter('max_speed', 0.80)
        self.declare_parameter('max_acceleration', 1.50)
        self.declare_parameter('max_steering_angle', 0.4189)
        self.declare_parameter('v_theta_min', 0.0)
        self.declare_parameter('v_theta_max', _UNSET)

        self.declare_parameter('q_contour', 25.0)
        self.declare_parameter('q_lag', 25.0)
        self.declare_parameter('q_speed', 3.0)
        self.declare_parameter('q_progress', 4.0)
        self.declare_parameter('qf_scale', 1.5)
        self.declare_parameter('r_acceleration', 0.20)
        self.declare_parameter('r_steering', 1.00)
        self.declare_parameter('rd_acceleration', 0.50)
        self.declare_parameter('rd_steering', 8.00)
        self.declare_parameter('rd_v_theta', 0.50)

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
        self.min_reference_speed = float(
            self.get_parameter('min_reference_speed').value)
        self.corner_slowdown_gain = float(
            self.get_parameter('corner_slowdown_gain').value)
        self.max_speed = float(self.get_parameter('max_speed').value)
        self.max_acceleration = float(
            self.get_parameter('max_acceleration').value)
        self.max_steering_angle = float(
            self.get_parameter('max_steering_angle').value)
        self.v_theta_min = float(self.get_parameter('v_theta_min').value)
        v_theta_max = float(self.get_parameter('v_theta_max').value)
        self.v_theta_max = (
            self.max_speed if v_theta_max == _UNSET else v_theta_max)

        self.q_contour = float(self.get_parameter('q_contour').value)
        self.q_lag = float(self.get_parameter('q_lag').value)
        self.q_speed = float(self.get_parameter('q_speed').value)
        self.q_progress = float(self.get_parameter('q_progress').value)
        self.qf_scale = float(self.get_parameter('qf_scale').value)
        self.r_acceleration = float(
            self.get_parameter('r_acceleration').value)
        self.r_steering = float(self.get_parameter('r_steering').value)
        self.rd_acceleration = float(
            self.get_parameter('rd_acceleration').value)
        self.rd_steering = float(self.get_parameter('rd_steering').value)
        self.rd_v_theta = float(self.get_parameter('rd_v_theta').value)

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
        self.last_solution_ok = False
        self.consecutive_solver_failures = 0
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
            AckermannDriveStamped, '/f1tenth_kkh/mpcc/proposed_drive', 10)
        self.predicted_path_pub = self.create_publisher(
            Path, '/f1tenth_kkh/mpcc/predicted_path', 10)
        self.reference_path_pub = self.create_publisher(
            Path, '/f1tenth_kkh/mpcc/reference_path', 10)
        self.solve_time_pub = self.create_publisher(
            Float64, '/f1tenth_kkh/mpcc/solve_time_ms', 10)
        self.progress_pub = self.create_publisher(
            Float64, '/f1tenth_kkh/mpcc/progress', 10)
        self.enable_service = self.create_service(
            SetBool, '/control/enable', self.enable_callback)

        self.build_mpc_problem()
        self.timer = self.create_timer(
            1.0 / max(self.control_rate, 1.0), self.control_loop)

        self.get_logger().info(
            'MPCC ready (enabled=%s, N=%d, dt=%.2fs, target=%.2fm/s)'
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
                'MPCC disabled: simulator collision reported')

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
                self.get_logger().error('MPCC rejected path: %s' % error)
                return
            self.get_logger().info(
                'MPCC received closed path: %d points, %.2f m'
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
            self._publish_stop()
            response.success = True
            response.message = 'MPCC stopped'
            self.get_logger().info(response.message)
            return response

        problem = self.readiness_problem()
        if problem is not None:
            response.success = False
            response.message = 'Cannot start MPCC: ' + problem
            self.get_logger().error(response.message)
            return response
        if not self.last_solution_ok:
            response.success = False
            response.message = (
                'Cannot start MPCC: no valid dry-run solution yet')
            self.get_logger().error(response.message)
            return response

        self.enabled = True
        self.consecutive_solver_failures = 0
        response.success = True
        response.message = 'MPCC enabled'
        self.get_logger().info(response.message)
        return response

    # ------------------------------------------------------------------ #
    # QP problem
    # ------------------------------------------------------------------ #
    def build_mpc_problem(self):
        n = self.horizon
        self.state_variable = cp.Variable((self.STATE_SIZE, n + 1))
        self.input_variable = cp.Variable((self.INPUT_SIZE, n))
        self.initial_state_parameter = cp.Parameter(self.STATE_SIZE)

        self.a_parameters = [
            cp.Parameter((self.STATE_SIZE, self.STATE_SIZE))
            for _ in range(n)
        ]
        self.b_parameters = [
            cp.Parameter((self.STATE_SIZE, self.INPUT_SIZE))
            for _ in range(n)
        ]
        self.c_parameters = [cp.Parameter(self.STATE_SIZE) for _ in range(n)]

        # Contouring/lag linearization references (path point + tangent at
        # theta_bar_k); parameterized so a full solve only updates values.
        self.cos_phi_parameters = [cp.Parameter() for _ in range(n + 1)]
        self.sin_phi_parameters = [cp.Parameter() for _ in range(n + 1)]
        self.xp_parameters = [cp.Parameter() for _ in range(n + 1)]
        self.yp_parameters = [cp.Parameter() for _ in range(n + 1)]

        # Obstacle-avoidance extension point: reserved per-step halfspace
        # constraints, inert by default (a=0, b=large). NOT used this round
        # -- see update_obstacle_constraints(). A future opponent/obstacle
        # keep-out constraint can tighten these without restructuring the
        # QP or losing OSQP warm-start.
        self.obstacle_a_parameters = [cp.Parameter(2) for _ in range(n + 1)]
        self.obstacle_b_parameters = [cp.Parameter() for _ in range(n + 1)]

        objective = 0.0
        constraints = [
            self.state_variable[:, 0] == self.initial_state_parameter,
            self.state_variable[2, :] >= 0.0,
            self.state_variable[2, :] <= self.max_speed,
            self.input_variable[2, :] >= self.v_theta_min,
            self.input_variable[2, :] <= self.v_theta_max,
        ]

        for k in range(n + 1):
            dx = self.state_variable[0, k] - self.xp_parameters[k]
            dy = self.state_variable[1, k] - self.yp_parameters[k]
            e_c = (-self.sin_phi_parameters[k] * dx
                   + self.cos_phi_parameters[k] * dy)
            e_l = (-self.cos_phi_parameters[k] * dx
                   - self.sin_phi_parameters[k] * dy)

            constraints.append(
                self.obstacle_a_parameters[k] @ self.state_variable[0:2, k]
                <= self.obstacle_b_parameters[k])

            if k < n:
                objective += self.q_contour * cp.square(e_c)
                objective += self.q_lag * cp.square(e_l)
                objective += self.q_speed * cp.square(
                    self.state_variable[2, k] - self.target_speed)
                objective += -self.q_progress * self.input_variable[2, k]
                objective += self.r_acceleration * cp.square(
                    self.input_variable[0, k])
                objective += self.r_steering * cp.square(
                    self.input_variable[1, k])

                constraints += [
                    self.state_variable[:, k + 1]
                    == self.a_parameters[k] @ self.state_variable[:, k]
                    + self.b_parameters[k] @ self.input_variable[:, k]
                    + self.c_parameters[k],
                    cp.abs(self.input_variable[0, k])
                    <= self.max_acceleration,
                    cp.abs(self.input_variable[1, k])
                    <= self.max_steering_angle,
                ]
                if k > 0:
                    objective += self.rd_acceleration * cp.square(
                        self.input_variable[0, k]
                        - self.input_variable[0, k - 1])
                    objective += self.rd_steering * cp.square(
                        self.input_variable[1, k]
                        - self.input_variable[1, k - 1])
                    objective += self.rd_v_theta * cp.square(
                        self.input_variable[2, k]
                        - self.input_variable[2, k - 1])
            else:
                objective += self.qf_scale * (
                    self.q_contour * cp.square(e_c)
                    + self.q_lag * cp.square(e_l))

        self.problem = cp.Problem(cp.Minimize(objective), constraints)

    @staticmethod
    def _kinematic_matrices(state4, control2, dt, wheelbase):
        """4x4 kinematic-bicycle linearization, [x,y,v,yaw]/[accel,steer].

        Ported from LinearMpcNode.model_matrices in
        algorithms/control/control/linear_mpc_node.py.
        """
        _, _, speed, yaw = state4
        acceleration, steering = control2

        a_matrix = np.eye(4)
        a_matrix[0, 2] = dt * math.cos(yaw)
        a_matrix[0, 3] = -dt * speed * math.sin(yaw)
        a_matrix[1, 2] = dt * math.sin(yaw)
        a_matrix[1, 3] = dt * speed * math.cos(yaw)
        a_matrix[3, 2] = dt * math.tan(steering) / wheelbase

        b_matrix = np.zeros((4, 2))
        b_matrix[2, 0] = dt
        b_matrix[3, 1] = dt * speed / (wheelbase * math.cos(steering) ** 2)

        nonlinear_next = np.array([
            state4[0] + dt * speed * math.cos(yaw),
            state4[1] + dt * speed * math.sin(yaw),
            speed + dt * acceleration,
            yaw + dt * speed * math.tan(steering) / wheelbase,
        ])
        c_vector = nonlinear_next - a_matrix @ state4 - b_matrix @ control2
        return a_matrix, b_matrix, c_vector

    def update_obstacle_constraints(self, obstacles):
        """Reserved hook for opponent/obstacle keep-out constraints.

        Not implemented this round -- obstacle_a/b_parameters stay inert
        (see solve_mpc). A future implementation would, for each step k,
        tighten obstacle_a_parameters[k]/obstacle_b_parameters[k] to a
        halfspace excluding the opponent/obstacle's predicted footprint at
        theta_bar_k, without needing to rebuild the QP.
        """
        pass

    # ------------------------------------------------------------------ #
    # Progress-parameterized reference (theta_bar_k), curvature-adjusted
    # exactly like LinearMpcNode.build_reference's spacing, reused here as
    # both the linearization reference and the contouring-error tangent.
    # ------------------------------------------------------------------ #
    def _build_progress_reference(self, s0):
        n = self.horizon
        sample_s = np.empty(n + 1)
        sample_s[0] = s0
        for step in range(n):
            step_curvature = float(self.raceline.interpolate(
                self.raceline.curvature, np.asarray([sample_s[step]]))[0])
            step_speed = clamp(
                self.target_speed
                / (1.0 + self.corner_slowdown_gain * abs(step_curvature)),
                self.min_reference_speed, self.max_speed)
            sample_s[step + 1] = sample_s[step] + step_speed * self.dt

        xp = self.raceline.interpolate(self.raceline.points[:, 0], sample_s)
        yp = self.raceline.interpolate(self.raceline.points[:, 1], sample_s)
        phi = self.raceline.interpolate(
            self.raceline.yaw, sample_s, self.raceline.yaw_lap_change)
        curvature = self.raceline.interpolate(
            self.raceline.curvature, sample_s)
        speed_profile = np.clip(
            self.target_speed
            / (1.0 + self.corner_slowdown_gain * np.abs(curvature)),
            self.min_reference_speed, self.max_speed)

        reference_state = np.zeros((4, n + 1))
        reference_state[0] = xp
        reference_state[1] = yp
        reference_state[2] = speed_profile
        reference_state[3] = phi

        reference_input = np.zeros((2, n))
        reference_input[1] = np.clip(
            np.arctan(self.wheelbase * curvature[:-1]),
            -self.max_steering_angle, self.max_steering_angle)

        return reference_state, reference_input, sample_s

    # ------------------------------------------------------------------ #
    # Solve
    # ------------------------------------------------------------------ #
    def solve_mpc(self, initial_state, reference_state, reference_input):
        n = self.horizon
        self.initial_state_parameter.value = initial_state

        for k in range(n + 1):
            phi_k = float(reference_state[3, k])
            self.cos_phi_parameters[k].value = math.cos(phi_k)
            self.sin_phi_parameters[k].value = math.sin(phi_k)
            self.xp_parameters[k].value = float(reference_state[0, k])
            self.yp_parameters[k].value = float(reference_state[1, k])
            # Inert by default -- see update_obstacle_constraints().
            self.obstacle_a_parameters[k].value = np.zeros(2)
            self.obstacle_b_parameters[k].value = 1.0e6

        for step in range(n):
            a4, b4, c4 = self._kinematic_matrices(
                reference_state[:, step], reference_input[:, step],
                self.dt, self.wheelbase)
            a_matrix = np.zeros((self.STATE_SIZE, self.STATE_SIZE))
            a_matrix[0:4, 0:4] = a4
            a_matrix[4, 4] = 1.0
            b_matrix = np.zeros((self.STATE_SIZE, self.INPUT_SIZE))
            b_matrix[0:4, 0:2] = b4
            b_matrix[4, 2] = self.dt
            c_vector = np.zeros(self.STATE_SIZE)
            c_vector[0:4] = c4

            self.a_parameters[step].value = a_matrix
            self.b_parameters[step].value = b_matrix
            self.c_parameters[step].value = c_vector

        started = time.perf_counter()
        self.problem.solve(
            solver=cp.OSQP,
            warm_start=True,
            verbose=False,
            eps_abs=1e-3,
            eps_rel=1e-3,
            max_iter=4000,
        )
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
    # Publishing
    # ------------------------------------------------------------------ #
    def _publish_stop(self):
        publish_stop(self.drive_pub, self.get_clock(), self.base_frame_id)

    def _publish_path(self, publisher, x_row, y_row, yaw_row):
        message = Path()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = self.global_frame_id
        for index in range(len(x_row)):
            pose = PoseStamped()
            pose.header = message.header
            pose.pose.position.x = float(x_row[index])
            pose.pose.position.y = float(y_row[index])
            yaw = float(yaw_row[index])
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
            self.last_solution_ok = False
            self._publish_stop()
            if self.enabled:
                self.warn_throttle.warn(
                    self.get_clock(), 'MPCC safety stop: ' + problem)
            return
        if not self.enabled and not self.solve_when_disabled:
            self._publish_stop()
            return

        try:
            x, y, yaw = lookup_vehicle_pose(
                self.tf_buffer, self.global_frame_id, self.base_frame_id)
            v = max(0.0, float(self.current_odom.twist.twist.linear.x))

            nearest, distance, path_heading, s0 = self.raceline.nearest_state(
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

            reference_state, reference_input, _sample_s = (
                self._build_progress_reference(s0))
            yaw_unwrapped = reference_state[3, 0] + angle_difference(
                yaw, reference_state[3, 0])
            initial_state = np.array(
                [x, y, v, yaw_unwrapped, s0])

            control, predicted, solve_ms = self.solve_mpc(
                initial_state, reference_state, reference_input)

            solve_msg = Float64()
            solve_msg.data = solve_ms
            self.solve_time_pub.publish(solve_msg)

            progress_msg = Float64()
            progress_msg.data = (
                s0 / self.raceline.length if self.raceline.length else 0.0)
            self.progress_pub.publish(progress_msg)

            self._publish_path(
                self.predicted_path_pub,
                predicted[0], predicted[1], predicted[3])
            self._publish_path(
                self.reference_path_pub,
                reference_state[0], reference_state[1], reference_state[3])

            self.last_solution_ok = True
            self.consecutive_solver_failures = 0

            acceleration = float(control[0, 0])
            steering = clamp(
                float(control[1, 0]),
                -self.max_steering_angle, self.max_steering_angle)
            command_speed = clamp(
                v + acceleration * self.dt, 0.0, self.max_speed)

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
                    'MPCC solve time %.1f ms exceeds %.0f Hz period'
                    % (solve_ms, self.control_rate))
        except (TransformException, RuntimeError, cp.error.SolverError) \
                as error:
            self.last_solution_ok = False
            self.consecutive_solver_failures += 1
            self._publish_stop()
            self.warn_throttle.warn(
                self.get_clock(), 'MPCC safety stop: ' + str(error))
            if self.enabled and self.consecutive_solver_failures >= 3:
                self.enabled = False
                self.get_logger().error(
                    'MPCC disabled after 3 consecutive solver/control '
                    'failures')


def main(args=None):
    rclpy.init(args=args)
    node = MpccNode()
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
