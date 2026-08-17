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
Head-to-Head event -- see the obstacle_* parameters in build_mpc_problem
and update_obstacle_constraints() below. Obstacle handling implemented
this round covers static obstacles only (single nearest one ahead,
detected via LaserScan clustering + track-corridor filtering in
scan_callback); a moving opponent is out of scope until its pose/velocity
is tracked separately.

State (5): [x, y, v, yaw, theta]. Input (3): [acceleration, steering, v_theta].
"""

import math
import time

import cvxpy as cp
import numpy as np
try:
    import osqp
    from scipy import sparse
except ImportError:  # native OSQP is optional; cvxpy backend remains usable.
    osqp = None
    sparse = None
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node

from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import Pose, PoseArray, PoseStamped
from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, Float32MultiArray, Float64, String
from std_srvs.srv import SetBool
from tf2_ros import Buffer, TransformException, TransformListener

from f1tenth_kkh.common.ackermann import build_ackermann, publish_stop
from f1tenth_kkh.common.obstacle_detection import cluster_scan
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
        self.declare_parameter('speed_profile_topic', '/planning/speed_profile')
        self.declare_parameter('speed_profile_enabled', True)
        self.declare_parameter('speed_profile_timeout', 2.0)
        self.declare_parameter('replan_state_topic', '/planning/replan_state')
        self.declare_parameter('replan_state_timeout', 0.50)
        self.declare_parameter('avoidance_speed_cap', 1.80)
        self.declare_parameter('blocked_speed_cap', 0.0)
        self.declare_parameter('drive_topic', '/drive')
        self.declare_parameter('collision_topic', '/ego_racecar/collision')

        self.declare_parameter('wheelbase', 0.33)
        self.declare_parameter('horizon_steps', 14)
        self.declare_parameter('dt', 0.10)
        self.declare_parameter('control_rate', 10.0)

        self.declare_parameter('target_speed', 0.55)
        self.declare_parameter('min_reference_speed', 0.30)
        self.declare_parameter('corner_slowdown_gain', 0.35)
        self.declare_parameter('max_lateral_accel', 0.65)
        self.declare_parameter('max_speed', 0.80)
        self.declare_parameter('max_acceleration', 1.50)
        self.declare_parameter('max_steering_angle', 0.4189)
        self.declare_parameter('max_steering_rate', 1.60)
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

        self.declare_parameter('solver_backend', 'osqp_native')
        self.declare_parameter('solver_eps_abs', 2.0e-3)
        self.declare_parameter('solver_eps_rel', 2.0e-3)
        self.declare_parameter('solver_max_iter', 600)
        self.declare_parameter('solver_polish', False)
        self.declare_parameter('solver_verbose', False)
        self.declare_parameter('solver_ignore_dpp', True)
        self.declare_parameter('solver_warm_start', True)
        self.declare_parameter('publish_visualization_rate', 5.0)

        # Static-obstacle avoidance (Head-to-Head prep). See the module
        # docstring and update_obstacle_constraints() for the algorithm;
        # defaults are justified in mpcc_params.yaml next to the values.
        self.declare_parameter('obstacle_enabled', True)
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('laser_frame_id', 'ego_racecar/laser')
        self.declare_parameter('obstacle_cluster_jump_m', 0.15)
        self.declare_parameter('obstacle_min_points', 3)
        self.declare_parameter('obstacle_max_width_m', 0.60)
        self.declare_parameter('obstacle_max_range_m', 4.00)
        self.declare_parameter('obstacle_corridor_m', 0.50)
        self.declare_parameter('obstacle_relevance_ahead_m', 3.00)
        self.declare_parameter('obstacle_relevance_behind_m', 0.30)
        self.declare_parameter('obstacle_longitudinal_margin_m', 0.30)
        self.declare_parameter('obstacle_keepout_radius_m', 0.20)
        self.declare_parameter('obstacle_slowdown_gain', 6.00)
        self.declare_parameter('obstacle_stale_timeout_s', 0.50)

        self.enabled = bool(self.get_parameter('enabled').value)
        self.solve_when_disabled = bool(
            self.get_parameter('solve_when_disabled').value)
        self.global_frame_id = self.get_parameter('global_frame_id').value
        self.base_frame_id = self.get_parameter('base_frame_id').value
        self.odom_topic = self.get_parameter('odom_topic').value
        self.path_topic = self.get_parameter('path_topic').value
        self.speed_profile_topic = self.get_parameter(
            'speed_profile_topic').value
        self.speed_profile_enabled = bool(
            self.get_parameter('speed_profile_enabled').value)
        self.speed_profile_timeout = float(
            self.get_parameter('speed_profile_timeout').value)
        self.replan_state_topic = self.get_parameter(
            'replan_state_topic').value
        self.replan_state_timeout = float(
            self.get_parameter('replan_state_timeout').value)
        self.avoidance_speed_cap = float(
            self.get_parameter('avoidance_speed_cap').value)
        self.blocked_speed_cap = float(
            self.get_parameter('blocked_speed_cap').value)
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
        self.max_lateral_accel = float(
            self.get_parameter('max_lateral_accel').value)
        self.max_speed = float(self.get_parameter('max_speed').value)
        self.max_acceleration = float(
            self.get_parameter('max_acceleration').value)
        self.max_steering_angle = float(
            self.get_parameter('max_steering_angle').value)
        self.max_steering_rate = float(
            self.get_parameter('max_steering_rate').value)
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

        self.solver_backend = str(
            self.get_parameter('solver_backend').value).strip().lower()
        self.solver_eps_abs = float(
            self.get_parameter('solver_eps_abs').value)
        self.solver_eps_rel = float(
            self.get_parameter('solver_eps_rel').value)
        self.solver_max_iter = int(
            self.get_parameter('solver_max_iter').value)
        self.solver_polish = bool(
            self.get_parameter('solver_polish').value)
        self.solver_verbose = bool(
            self.get_parameter('solver_verbose').value)
        self.solver_ignore_dpp = bool(
            self.get_parameter('solver_ignore_dpp').value)
        self.solver_warm_start = bool(
            self.get_parameter('solver_warm_start').value)
        self.publish_visualization_rate = float(
            self.get_parameter('publish_visualization_rate').value)

        self.obstacle_enabled = bool(
            self.get_parameter('obstacle_enabled').value)
        self.scan_topic = self.get_parameter('scan_topic').value
        self.laser_frame_id = self.get_parameter('laser_frame_id').value
        self.obstacle_cluster_jump_m = float(
            self.get_parameter('obstacle_cluster_jump_m').value)
        self.obstacle_min_points = int(
            self.get_parameter('obstacle_min_points').value)
        self.obstacle_max_width_m = float(
            self.get_parameter('obstacle_max_width_m').value)
        self.obstacle_max_range_m = float(
            self.get_parameter('obstacle_max_range_m').value)
        self.obstacle_corridor_m = float(
            self.get_parameter('obstacle_corridor_m').value)
        self.obstacle_relevance_ahead_m = float(
            self.get_parameter('obstacle_relevance_ahead_m').value)
        self.obstacle_relevance_behind_m = float(
            self.get_parameter('obstacle_relevance_behind_m').value)
        self.obstacle_longitudinal_margin_m = float(
            self.get_parameter('obstacle_longitudinal_margin_m').value)
        self.obstacle_keepout_radius_m = float(
            self.get_parameter('obstacle_keepout_radius_m').value)
        self.obstacle_slowdown_gain = float(
            self.get_parameter('obstacle_slowdown_gain').value)
        self.obstacle_stale_timeout_s = float(
            self.get_parameter('obstacle_stale_timeout_s').value)

        if self.solver_backend not in ('osqp_native', 'cvxpy'):
            raise RuntimeError(
                'solver_backend must be osqp_native or cvxpy')
        if self.solver_backend == 'osqp_native' and osqp is None:
            self.get_logger().warn(
                'osqp Python package is unavailable; falling back to cvxpy')
            self.solver_backend = 'cvxpy'
        if self.solver_backend == 'osqp_native' and self.obstacle_enabled:
            self.get_logger().warn(
                'native OSQP backend currently supports obstacle_enabled=false; '
                'falling back to cvxpy for obstacle constraints')
            self.solver_backend = 'cvxpy'

        if self.horizon < 2:
            raise RuntimeError('horizon_steps must be at least 2')
        if self.dt <= 0.0:
            raise RuntimeError('dt must be positive')

        self.current_odom = None
        self.last_odom_time = None
        self.last_path_time = None
        self.last_scan_time = None
        self.last_speed_profile_time = None
        self.speed_profile_s = None
        self.speed_profile_v = None
        self.last_replan_state_time = None
        self.replan_state = 'GLOBAL'
        self.detected_obstacles = []  # [(map_x, map_y, raceline_s), ...]
        self.collision = False
        self.raceline = ClosedRaceline(
            search_back_points=search_back_points,
            search_forward_points=search_forward_points)
        self.last_solution_ok = False
        self.consecutive_solver_failures = 0
        self.previous_steering = 0.0
        self.last_control_solution = None
        self.last_state_solution = None
        self.last_visualization_time = None
        self.warn_throttle = WarnThrottle(self.get_logger())

        self.tf_buffer = Buffer(cache_time=Duration(seconds=5.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.create_subscription(
            Odometry, self.odom_topic, self.odom_callback, 10)
        self.create_subscription(Path, self.path_topic, self.path_callback, 10)
        self.create_subscription(
            String, self.replan_state_topic, self.replan_state_callback, 10)
        if self.speed_profile_enabled:
            self.create_subscription(
                Float32MultiArray, self.speed_profile_topic,
                self.speed_profile_callback, 10)
        self.create_subscription(
            Bool, self.collision_topic, self.collision_callback, 10)
        if self.obstacle_enabled:
            self.create_subscription(
                LaserScan, self.scan_topic, self.scan_callback, 10)

        self.obstacle_pub = self.create_publisher(
            PoseArray, '/f1tenth_kkh/mpcc/detected_obstacles', 10)
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
        self.osqp_solve_time_pub = self.create_publisher(
            Float64, '/f1tenth_kkh/mpcc/osqp_solve_time_ms', 10)
        self.solver_iterations_pub = self.create_publisher(
            Float64, '/f1tenth_kkh/mpcc/solver_iterations', 10)
        self.progress_pub = self.create_publisher(
            Float64, '/f1tenth_kkh/mpcc/progress', 10)
        self.enable_service = self.create_service(
            SetBool, '/control/enable', self.enable_callback)

        self.build_mpc_problem()
        self.timer = self.create_timer(
            1.0 / max(self.control_rate, 1.0), self.control_loop)

        self.get_logger().info(
            'MPCC ready (enabled=%s, backend=%s, N=%d, dt=%.2fs, '
            'target=%.2fm/s, eps=%.1e/%.1e, max_iter=%d)'
            % (self.enabled, self.solver_backend, self.horizon, self.dt,
               self.target_speed, self.solver_eps_abs, self.solver_eps_rel,
               self.solver_max_iter))
        self.get_logger().info(
            'Disabled mode solves and visualizes predictions, but '
            'publishes stop')
        if self.speed_profile_enabled:
            self.get_logger().info(
                'MPCC will use external speed profile: %s'
                % self.speed_profile_topic)

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
            self.previous_steering = 0.0
            self._publish_stop()
            self.get_logger().error(
                'MPCC disabled: simulator collision reported')

    def scan_callback(self, msg):
        self.last_scan_time = self.get_clock().now()
        if not self.obstacle_enabled or not self.raceline.ready:
            self.detected_obstacles = []
            return

        try:
            laser_x, laser_y, laser_yaw = lookup_vehicle_pose(
                self.tf_buffer, self.global_frame_id, self.laser_frame_id)
        except TransformException:
            return

        clusters = cluster_scan(
            msg.ranges, msg.angle_min, msg.angle_increment,
            max(msg.range_min, 0.05),
            min(msg.range_max, self.obstacle_max_range_m),
            self.obstacle_cluster_jump_m, self.obstacle_min_points)

        cos_yaw = math.cos(laser_yaw)
        sin_yaw = math.sin(laser_yaw)
        obstacles = []
        for cluster in clusters:
            # Wide/long clusters are wall segments, not a small obstacle.
            if cluster.width > self.obstacle_max_width_m:
                continue
            map_x = laser_x + cluster.x * cos_yaw - cluster.y * sin_yaw
            map_y = laser_y + cluster.x * sin_yaw + cluster.y * cos_yaw
            s_obs, lateral = self.raceline.nearest_arc_length_stateless(
                map_x, map_y)
            # Track walls sit outside the corridor; only accept clusters
            # close enough to the raceline to plausibly be on the track.
            if lateral > self.obstacle_corridor_m:
                continue
            obstacles.append((map_x, map_y, s_obs))

        self.detected_obstacles = obstacles
        self._publish_obstacles(obstacles)

    def _publish_obstacles(self, obstacles):
        message = PoseArray()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = self.global_frame_id
        for map_x, map_y, _s_obs in obstacles:
            pose = Pose()
            pose.position.x = float(map_x)
            pose.position.y = float(map_y)
            pose.orientation.w = 1.0
            message.poses.append(pose)
        self.obstacle_pub.publish(message)

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

    def replan_state_callback(self, msg):
        self.replan_state = str(msg.data)
        self.last_replan_state_time = self.get_clock().now()

    def _active_speed_cap(self):
        cap = self.max_speed
        if (self.last_replan_state_time is not None
                and age_seconds(self.get_clock(), self.last_replan_state_time)
                <= self.replan_state_timeout
                and self.replan_state == 'BLOCKED'):
            cap = min(cap, self.blocked_speed_cap)
        elif (self.last_replan_state_time is not None
                and age_seconds(self.get_clock(), self.last_replan_state_time)
                <= self.replan_state_timeout
                and self.replan_state.startswith('LOCAL_AVOIDANCE')):
            cap = min(cap, self.avoidance_speed_cap)
        return cap

    def speed_profile_callback(self, msg):
        data = np.asarray(msg.data, dtype=float)
        if data.size < 4 or data.size % 2 != 0:
            self.warn_throttle.warn(
                self.get_clock(), 'Ignoring malformed speed profile')
            return
        pairs = data.reshape((-1, 2))
        s_values = pairs[:, 0]
        speeds = pairs[:, 1]
        if not np.all(np.isfinite(s_values)) or not np.all(np.isfinite(speeds)):
            self.warn_throttle.warn(
                self.get_clock(), 'Ignoring non-finite speed profile')
            return
        order = np.argsort(s_values)
        s_values = s_values[order]
        speeds = speeds[order]
        keep = np.concatenate(([True], np.diff(s_values) > 1.0e-5))
        s_values = s_values[keep]
        speeds = speeds[keep]
        if len(s_values) < 2:
            self.warn_throttle.warn(
                self.get_clock(), 'Ignoring too-short speed profile')
            return
        s_values = s_values - s_values[0]
        self.speed_profile_s = s_values
        self.speed_profile_v = np.clip(
            speeds, self.min_reference_speed, self.max_speed)
        self.last_speed_profile_time = self.get_clock().now()

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
            self.previous_steering = 0.0
            self._publish_stop()
            self._clear_visualization()
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
        self.speed_reference_parameters = [cp.Parameter() for _ in range(n)]

        # Obstacle-avoidance extension point: reserved per-step halfspace
        # constraints, inert by default (a=0, b=large). NOT used this round
        # -- see update_obstacle_constraints(). A future opponent/obstacle
        # keep-out constraint can tighten these without restructuring the
        # QP or losing OSQP warm-start.
        if self.obstacle_enabled:
            self.obstacle_a_parameters = [
                cp.Parameter(2) for _ in range(n + 1)]
            self.obstacle_b_parameters = [
                cp.Parameter() for _ in range(n + 1)]
        else:
            self.obstacle_a_parameters = []
            self.obstacle_b_parameters = []

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

            if self.obstacle_enabled:
                constraints.append(
                    self.obstacle_a_parameters[k]
                    @ self.state_variable[0:2, k]
                    <= self.obstacle_b_parameters[k])

            if k < n:
                objective += self.q_contour * cp.square(e_c)
                objective += self.q_lag * cp.square(e_l)
                objective += self.q_speed * cp.square(
                    self.state_variable[2, k]
                    - self.speed_reference_parameters[k])
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

    @staticmethod
    def _wrap_delta(a, b, length):
        """Signed shortest arc-length from b to a on a closed loop of the
        given length, in (-length/2, length/2]."""
        return (a - b + length / 2.0) % length - length / 2.0

    def _reference_speed(self, curvature, obstacle_proximity=0.0):
        curvature = abs(float(curvature))
        shaped_speed = (
            self.target_speed
            / (1.0 + self.corner_slowdown_gain * curvature
               + self.obstacle_slowdown_gain * obstacle_proximity))
        if self.max_lateral_accel > 0.0 and curvature > 1.0e-4:
            curve_speed = math.sqrt(self.max_lateral_accel / curvature)
            shaped_speed = min(shaped_speed, curve_speed)
        return clamp(shaped_speed, self.min_reference_speed, self._active_speed_cap())

    def _has_fresh_speed_profile(self):
        return (
            self.speed_profile_enabled
            and self.speed_profile_s is not None
            and self.speed_profile_v is not None
            and age_seconds(self.get_clock(), self.last_speed_profile_time)
            <= self.speed_profile_timeout)

    def _profile_speed(self, sample_s, obstacle_proximity=0.0):
        if not self._has_fresh_speed_profile():
            return None
        if self.raceline.length is None or self.raceline.length <= 0.0:
            return None

        wrapped = float(sample_s) % self.raceline.length
        s_values = self.speed_profile_s
        speeds = self.speed_profile_v
        closed_s = np.concatenate((s_values, [self.raceline.length]))
        closed_v = np.concatenate((speeds, [speeds[0]]))
        speed = float(np.interp(wrapped, closed_s, closed_v))
        if obstacle_proximity > 0.0:
            speed /= 1.0 + self.obstacle_slowdown_gain * obstacle_proximity
        return clamp(speed, self.min_reference_speed, self._active_speed_cap())

    def _reference_speed_at(self, sample_s, curvature, obstacle_proximity=0.0):
        profile_speed = self._profile_speed(sample_s, obstacle_proximity)
        if profile_speed is not None:
            return profile_speed
        return self._reference_speed(curvature, obstacle_proximity)

    def _nearest_relevant_obstacle(self, s0):
        """Return (obstacle_x, obstacle_y, s_obs, ds) for the detected
        obstacle nearest ahead of the vehicle's progress s0 -- ds is its
        wrap-aware signed arc-length offset from s0, positive means
        ahead -- restricted to
        [-obstacle_relevance_behind_m, +obstacle_relevance_ahead_m].
        Returns None if no detected obstacle qualifies. Shared by
        _build_progress_reference (proximity slowdown) and
        update_obstacle_constraints (the keep-out halfspace) so both
        react to the same single obstacle.
        """
        length = self.raceline.length
        nearest = None
        nearest_ds = None
        for obstacle_x, obstacle_y, s_obs in self.detected_obstacles:
            ds = self._wrap_delta(s_obs, s0, length)
            if -self.obstacle_relevance_behind_m <= ds \
                    <= self.obstacle_relevance_ahead_m:
                if nearest_ds is None or ds < nearest_ds:
                    nearest_ds = ds
                    nearest = (obstacle_x, obstacle_y, s_obs, ds)
        return nearest

    def update_obstacle_constraints(self, ego_x, ego_y, sample_s,
                                    nearest_obstacle):
        """Tighten obstacle_a/b_parameters to keep out nearest_obstacle
        (as returned by _nearest_relevant_obstacle), or leave every step
        inert if it is None.

        Builds one separating halfspace -- a @ [x, y] <= b, tangent to a
        disk of radius obstacle_keepout_radius_m around the obstacle.

        The halfspace normal is the raceline's LATERAL (contouring) axis
        at the obstacle's arc-length, not the raw ego->obstacle direction:
        a plane perpendicular to the direction of travel would just wall
        off the whole corridor and stop the car dead in front of the
        obstacle (the "emergency stop, not avoidance" case called out in
        MPC_GUIDE.md), whereas the lateral-axis plane only blocks the
        obstacle's side of the corridor and leaves the rest free to drive
        past. Which side to pass on is decided by which side of the
        centerline the obstacle already sits on (falling back to the
        vehicle's own current side if the obstacle is ~on the
        centerline, so the choice doesn't flip-flop solve to solve).

        The halfspace is only applied to horizon steps k whose reference
        progress sample_s[k] is within obstacle_longitudinal_margin_m of
        the obstacle's arc-length (other steps stay inert). Without this
        gating, a uniform constraint across the whole horizon would also
        bind at k=0 -- whose position is pinned to the vehicle's *actual*
        current pose by a hard equality constraint -- making the QP
        infeasible as soon as an obstacle enters the relevance window,
        long before the vehicle is anywhere near it. Because the obstacle
        is static, no per-step future-position prediction is needed
        beyond this arc-length gating (a moving opponent would need one,
        out of scope this round). Recomputing this every control cycle
        from the current ego position/progress is what gives replanning
        behaviour: as the vehicle advances, which steps are gated and
        where the plane sits both update every solve.

        If the tightened constraint still makes the QP infeasible
        (obstacle blocks the whole corridor), solve_mpc raises and
        control_loop's existing safety-stop/auto-disable path handles
        it -- no separate obstacle-specific safety logic is needed.
        """
        n = self.horizon
        length = self.raceline.length

        if not self.obstacle_enabled:
            return

        if nearest_obstacle is None:
            for k in range(n + 1):
                self.obstacle_a_parameters[k].value = np.zeros(2)
                self.obstacle_b_parameters[k].value = 1.0e6
            return

        obstacle_x, obstacle_y, s_obs, _ds = nearest_obstacle
        sample = np.asarray([s_obs])
        path_x = float(self.raceline.interpolate(
            self.raceline.points[:, 0], sample)[0])
        path_y = float(self.raceline.interpolate(
            self.raceline.points[:, 1], sample)[0])
        path_yaw = float(self.raceline.interpolate(
            self.raceline.yaw, sample, self.raceline.yaw_lap_change)[0])
        normal = np.array([-math.sin(path_yaw), math.cos(path_yaw)])
        obstacle_point = np.array([obstacle_x, obstacle_y])
        path_point = np.array([path_x, path_y])

        obstacle_lateral = float(normal @ (obstacle_point - path_point))
        if abs(obstacle_lateral) < 1.0e-3:
            ego_lateral = float(
                normal @ (np.array([ego_x, ego_y]) - path_point))
            side = -1.0 if ego_lateral >= 0.0 else 1.0
        else:
            side = 1.0 if obstacle_lateral >= 0.0 else -1.0

        a_value = side * normal
        b_value = float(
            a_value @ obstacle_point - self.obstacle_keepout_radius_m)

        for k in range(n + 1):
            ds_k = self._wrap_delta(float(sample_s[k]), s_obs, length)
            if abs(ds_k) <= self.obstacle_longitudinal_margin_m:
                self.obstacle_a_parameters[k].value = a_value
                self.obstacle_b_parameters[k].value = b_value
            else:
                self.obstacle_a_parameters[k].value = np.zeros(2)
                self.obstacle_b_parameters[k].value = 1.0e6

    # ------------------------------------------------------------------ #
    # Progress-parameterized reference (theta_bar_k), curvature-adjusted
    # exactly like LinearMpcNode.build_reference's spacing, reused here as
    # both the linearization reference and the contouring-error tangent.
    # ------------------------------------------------------------------ #
    def _build_progress_reference(self, s0, nearest_obstacle=None):
        """Build the horizon reference. nearest_obstacle (as returned by
        _nearest_relevant_obstacle), if given, additionally ramps the
        reference speed down on approach -- see the "obstacle proximity
        slowdown" note below for why this is needed for the avoidance
        maneuver in update_obstacle_constraints to be kinematically
        achievable at all, not just a cosmetic touch.
        """
        n = self.horizon
        obstacle_ds = (
            nearest_obstacle[3] if nearest_obstacle is not None else None)

        def obstacle_proximity(distance_travelled):
            # obstacle proximity slowdown: ramps 0 -> 1 as the reference
            # point closes in on the obstacle from
            # obstacle_relevance_ahead_m away to right on top of it, 0
            # once past it. The MPC horizon only reaches
            # target_speed * horizon_steps * dt ahead of the vehicle
            # (well under 1m at competition-tuned speeds) -- an obstacle
            # detected only once it enters that reach leaves too few
            # horizon steps to complete a keepout_radius-sized swerve
            # within the steering-rate/curvature limits (empirically
            # confirmed: at target_speed 0.85 m/s / horizon_steps 10, an
            # obstacle first seen at the edge of horizon reach makes the
            # QP infeasible for any keepout_radius_m above ~0.15m).
            # Slowing down on approach -- exactly like corner_slowdown_gain
            # already does for curvature -- buys more elapsed time (hence
            # more steering-authority) to cover the same remaining
            # distance, so the horizon has room to actually swerve rather
            # than just emergency-stop in front of the obstacle.
            # Symmetric around the obstacle (rather than 0 <= remaining
            # only) so the reference doesn't snap back to full speed the
            # instant the 1-D progress schedule numerically passes
            # s_obs, even though update_obstacle_constraints' gating
            # window (obstacle_longitudinal_margin_m) means the real
            # avoidance maneuver is still in progress around there.
            if obstacle_ds is None:
                return 0.0
            distance = abs(obstacle_ds - distance_travelled)
            if distance <= self.obstacle_relevance_ahead_m:
                return 1.0 - distance / self.obstacle_relevance_ahead_m
            return 0.0

        sample_s = np.empty(n + 1)
        sample_s[0] = s0
        for step in range(n):
            step_curvature = float(self.raceline.interpolate(
                self.raceline.curvature, np.asarray([sample_s[step]]))[0])
            proximity = obstacle_proximity(sample_s[step] - s0)
            step_speed = self._reference_speed_at(
                sample_s[step], step_curvature, proximity)
            sample_s[step + 1] = sample_s[step] + step_speed * self.dt

        xp = self.raceline.interpolate(self.raceline.points[:, 0], sample_s)
        yp = self.raceline.interpolate(self.raceline.points[:, 1], sample_s)
        phi = self.raceline.interpolate(
            self.raceline.yaw, sample_s, self.raceline.yaw_lap_change)
        curvature = self.raceline.interpolate(
            self.raceline.curvature, sample_s)
        proximity_profile = np.array(
            [obstacle_proximity(s - s0) for s in sample_s])
        speed_profile = np.array([
            self._reference_speed_at(s, kappa, proximity)
            for s, kappa, proximity in zip(
                sample_s, curvature, proximity_profile)
        ])

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
    def _seed_warm_start(self, initial_state, reference_state,
                         reference_input, sample_s):
        if not self.solver_warm_start:
            return

        n = self.horizon
        if self.last_state_solution is not None \
                and self.last_state_solution.shape == (self.STATE_SIZE, n + 1):
            state_seed = np.empty_like(self.last_state_solution)
            state_seed[:, :-1] = self.last_state_solution[:, 1:]
            state_seed[:, -1] = self.last_state_solution[:, -1]
            state_seed[:, 0] = initial_state
        else:
            state_seed = np.zeros((self.STATE_SIZE, n + 1))
            state_seed[0:4, :] = reference_state
            if sample_s is not None:
                state_seed[4, :] = sample_s
            else:
                state_seed[4, :] = (
                    initial_state[4]
                    + np.arange(n + 1) * self.target_speed * self.dt)
            state_seed[:, 0] = initial_state
        self.state_variable.value = state_seed

        if self.last_control_solution is not None \
                and self.last_control_solution.shape == (self.INPUT_SIZE, n):
            control_seed = np.empty_like(self.last_control_solution)
            control_seed[:, :-1] = self.last_control_solution[:, 1:]
            control_seed[:, -1] = self.last_control_solution[:, -1]
        else:
            control_seed = np.zeros((self.INPUT_SIZE, n))
            control_seed[1, :] = reference_input[1, :]
            control_seed[2, :] = reference_state[2, :-1]
        self.input_variable.value = control_seed

    def _state_index(self, step, component):
        return step * self.STATE_SIZE + component

    def _input_index(self, step, component):
        return self.STATE_SIZE * (self.horizon + 1) + step * self.INPUT_SIZE + component

    @staticmethod
    def _add_quadratic(rows, cols, data, q_vector, indices, coeffs, weight,
                       offset=0.0):
        # OSQP uses 0.5*x'Px + q'x, while these costs are
        # weight*(a'x + offset)^2.
        for row_index, row_coeff in zip(indices, coeffs):
            q_vector[row_index] += 2.0 * weight * offset * row_coeff
            for col_index, col_coeff in zip(indices, coeffs):
                rows.append(row_index)
                cols.append(col_index)
                data.append(2.0 * weight * row_coeff * col_coeff)

    @staticmethod
    def _add_linear_constraint(rows, cols, data, row, terms):
        for index, value in terms:
            if value != 0.0:
                rows.append(row)
                cols.append(index)
                data.append(float(value))

    def solve_mpc_native(self, initial_state, reference_state, reference_input,
                         sample_s):
        if osqp is None or sparse is None:
            raise RuntimeError('native OSQP backend is unavailable')
        if self.obstacle_enabled:
            raise RuntimeError(
                'native OSQP backend does not handle obstacle constraints yet')

        n = self.horizon
        nx = self.STATE_SIZE
        nu = self.INPUT_SIZE
        total_variables = nx * (n + 1) + nu * n

        p_rows = []
        p_cols = []
        p_data = []
        q_vector = np.zeros(total_variables)

        for k in range(n + 1):
            x_idx = self._state_index(k, 0)
            y_idx = self._state_index(k, 1)
            v_idx = self._state_index(k, 2)
            phi_k = float(reference_state[3, k])
            sin_phi = math.sin(phi_k)
            cos_phi = math.cos(phi_k)
            xp = float(reference_state[0, k])
            yp = float(reference_state[1, k])

            contour_offset = sin_phi * xp - cos_phi * yp
            lag_offset = cos_phi * xp + sin_phi * yp
            contour_weight = self.q_contour if k < n else self.qf_scale * self.q_contour
            lag_weight = self.q_lag if k < n else self.qf_scale * self.q_lag
            self._add_quadratic(
                p_rows, p_cols, p_data, q_vector,
                [x_idx, y_idx], [-sin_phi, cos_phi],
                contour_weight, contour_offset)
            self._add_quadratic(
                p_rows, p_cols, p_data, q_vector,
                [x_idx, y_idx], [-cos_phi, -sin_phi],
                lag_weight, lag_offset)

            if k < n:
                self._add_quadratic(
                    p_rows, p_cols, p_data, q_vector,
                    [v_idx], [1.0], self.q_speed,
                    -float(reference_state[2, k]))
                accel_idx = self._input_index(k, 0)
                steer_idx = self._input_index(k, 1)
                vtheta_idx = self._input_index(k, 2)
                self._add_quadratic(
                    p_rows, p_cols, p_data, q_vector,
                    [accel_idx], [1.0], self.r_acceleration)
                self._add_quadratic(
                    p_rows, p_cols, p_data, q_vector,
                    [steer_idx], [1.0], self.r_steering)
                q_vector[vtheta_idx] += -self.q_progress

                if k > 0:
                    prev_accel_idx = self._input_index(k - 1, 0)
                    prev_steer_idx = self._input_index(k - 1, 1)
                    prev_vtheta_idx = self._input_index(k - 1, 2)
                    self._add_quadratic(
                        p_rows, p_cols, p_data, q_vector,
                        [accel_idx, prev_accel_idx], [1.0, -1.0],
                        self.rd_acceleration)
                    self._add_quadratic(
                        p_rows, p_cols, p_data, q_vector,
                        [steer_idx, prev_steer_idx], [1.0, -1.0],
                        self.rd_steering)
                    self._add_quadratic(
                        p_rows, p_cols, p_data, q_vector,
                        [vtheta_idx, prev_vtheta_idx], [1.0, -1.0],
                        self.rd_v_theta)

        p_matrix = sparse.csc_matrix(
            (p_data, (p_rows, p_cols)),
            shape=(total_variables, total_variables))

        a_rows = []
        a_cols = []
        a_data = []
        lower = []
        upper = []
        row = 0

        # Initial state equality.
        for component in range(nx):
            self._add_linear_constraint(
                a_rows, a_cols, a_data, row,
                [(self._state_index(0, component), 1.0)])
            value = float(initial_state[component])
            lower.append(value)
            upper.append(value)
            row += 1

        # Linearized dynamics equality.
        for step in range(n):
            a4, b4, c4 = self._kinematic_matrices(
                reference_state[:, step], reference_input[:, step],
                self.dt, self.wheelbase)
            a_matrix = np.zeros((nx, nx))
            a_matrix[0:4, 0:4] = a4
            a_matrix[4, 4] = 1.0
            b_matrix = np.zeros((nx, nu))
            b_matrix[0:4, 0:2] = b4
            b_matrix[4, 2] = self.dt
            c_vector = np.zeros(nx)
            c_vector[0:4] = c4

            for component in range(nx):
                terms = [(self._state_index(step + 1, component), 1.0)]
                for state_component in range(nx):
                    value = -a_matrix[component, state_component]
                    terms.append((
                        self._state_index(step, state_component), value))
                for input_component in range(nu):
                    value = -b_matrix[component, input_component]
                    terms.append((
                        self._input_index(step, input_component), value))
                self._add_linear_constraint(
                    a_rows, a_cols, a_data, row, terms)
                value = float(c_vector[component])
                lower.append(value)
                upper.append(value)
                row += 1

        # Variable bounds as identity rows. Keeping them in A makes setup
        # simple and still avoids cvxpy's canonicalization cost.
        for step in range(n + 1):
            v_idx = self._state_index(step, 2)
            self._add_linear_constraint(
                a_rows, a_cols, a_data, row, [(v_idx, 1.0)])
            lower.append(0.0)
            upper.append(self.max_speed)
            row += 1

        for step in range(n):
            accel_idx = self._input_index(step, 0)
            steer_idx = self._input_index(step, 1)
            vtheta_idx = self._input_index(step, 2)
            for index, lo, hi in (
                    (accel_idx, -self.max_acceleration,
                     self.max_acceleration),
                    (steer_idx, -self.max_steering_angle,
                     self.max_steering_angle),
                    (vtheta_idx, self.v_theta_min, self.v_theta_max)):
                self._add_linear_constraint(
                    a_rows, a_cols, a_data, row, [(index, 1.0)])
                lower.append(lo)
                upper.append(hi)
                row += 1

        a_matrix = sparse.csc_matrix(
            (a_data, (a_rows, a_cols)),
            shape=(row, total_variables))
        lower = np.asarray(lower, dtype=float)
        upper = np.asarray(upper, dtype=float)

        solver = osqp.OSQP()
        setup_kwargs = {
            'verbose': self.solver_verbose,
            'warm_start': self.solver_warm_start,
            'eps_abs': self.solver_eps_abs,
            'eps_rel': self.solver_eps_rel,
            'max_iter': self.solver_max_iter,
            'polish': self.solver_polish,
        }

        started = time.perf_counter()
        solver.setup(P=p_matrix, q=q_vector, A=a_matrix, l=lower, u=upper,
                     **setup_kwargs)
        if self.solver_warm_start:
            seed = None
            if self.last_state_solution is not None \
                    and self.last_control_solution is not None:
                seed = np.zeros(total_variables)
                seed[:nx * (n + 1)] = self.last_state_solution.reshape(-1, order='F')
                seed[nx * (n + 1):] = self.last_control_solution.reshape(-1, order='F')
                seed[0:nx] = initial_state
            elif sample_s is not None:
                seed = np.zeros(total_variables)
                state_seed = np.zeros((nx, n + 1))
                state_seed[0:4, :] = reference_state
                state_seed[4, :] = sample_s
                state_seed[:, 0] = initial_state
                control_seed = np.zeros((nu, n))
                control_seed[1, :] = reference_input[1, :]
                control_seed[2, :] = reference_state[2, :-1]
                seed[:nx * (n + 1)] = state_seed.reshape(-1, order='F')
                seed[nx * (n + 1):] = control_seed.reshape(-1, order='F')
            if seed is not None:
                solver.warm_start(x=seed)

        result = solver.solve()
        solve_ms = (time.perf_counter() - started) * 1000.0

        status = str(result.info.status).lower()
        if 'solved' not in status:
            raise RuntimeError('solver status: ' + str(result.info.status))
        if result.x is None:
            raise RuntimeError('solver returned no solution')

        solution = np.asarray(result.x)
        state_solution = solution[:nx * (n + 1)].reshape((nx, n + 1), order='F')
        control_solution = solution[nx * (n + 1):].reshape((nu, n), order='F')
        self.last_control_solution = control_solution.copy()
        self.last_state_solution = state_solution.copy()

        return (
            control_solution,
            state_solution,
            solve_ms,
            1000.0 * float(result.info.run_time),
            float(result.info.iter),
        )

    def solve_mpc(self, initial_state, reference_state, reference_input,
                  sample_s):
        if self.solver_backend == 'osqp_native':
            return self.solve_mpc_native(
                initial_state, reference_state, reference_input, sample_s)
        return self.solve_mpc_cvxpy(
            initial_state, reference_state, reference_input, sample_s)

    def solve_mpc_cvxpy(self, initial_state, reference_state, reference_input,
                         sample_s):
        n = self.horizon
        self.initial_state_parameter.value = initial_state
        self._seed_warm_start(
            initial_state, reference_state, reference_input, sample_s)

        for k in range(n + 1):
            phi_k = float(reference_state[3, k])
            self.cos_phi_parameters[k].value = math.cos(phi_k)
            self.sin_phi_parameters[k].value = math.sin(phi_k)
            self.xp_parameters[k].value = float(reference_state[0, k])
            self.yp_parameters[k].value = float(reference_state[1, k])
            # obstacle_a/b_parameters[k], when enabled, are set by
            # update_obstacle_constraints() before solve_mpc.

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
            self.speed_reference_parameters[step].value = float(
                reference_state[2, step])

        solve_kwargs = {
            'solver': cp.OSQP,
            'warm_start': self.solver_warm_start,
            'verbose': self.solver_verbose,
            'eps_abs': self.solver_eps_abs,
            'eps_rel': self.solver_eps_rel,
            'max_iter': self.solver_max_iter,
            'polish': self.solver_polish,
        }
        if self.solver_ignore_dpp:
            solve_kwargs['ignore_dpp'] = True

        started = time.perf_counter()
        self.problem.solve(**solve_kwargs)
        solve_ms = (time.perf_counter() - started) * 1000.0
        solver_stats = self.problem.solver_stats
        osqp_solve_ms = (
            1000.0 * float(solver_stats.solve_time)
            if solver_stats.solve_time is not None else -1.0)
        solver_iterations = float(
            solver_stats.num_iters
            if solver_stats.num_iters is not None else -1.0)

        if self.problem.status not in (cp.OPTIMAL, cp.OPTIMAL_INACCURATE):
            raise RuntimeError('solver status: ' + str(self.problem.status))
        if self.input_variable.value is None \
                or self.state_variable.value is None:
            raise RuntimeError('solver returned no solution')

        control_solution = np.asarray(self.input_variable.value)
        state_solution = np.asarray(self.state_variable.value)
        self.last_control_solution = control_solution.copy()
        self.last_state_solution = state_solution.copy()

        return (
            control_solution,
            state_solution,
            solve_ms,
            osqp_solve_ms,
            solver_iterations,
        )

    # ------------------------------------------------------------------ #
    # Publishing
    # ------------------------------------------------------------------ #
    def _publish_stop(self):
        publish_stop(self.drive_pub, self.get_clock(), self.base_frame_id)

    def _clear_visualization(self):
        stamp = self.get_clock().now().to_msg()
        for publisher in (self.predicted_path_pub, self.reference_path_pub):
            message = Path()
            message.header.stamp = stamp
            message.header.frame_id = self.global_frame_id
            publisher.publish(message)

    def _should_publish_visualization(self):
        if self.publish_visualization_rate <= 0.0:
            return False
        now = self.get_clock().now()
        if self.last_visualization_time is None:
            self.last_visualization_time = now
            return True
        if age_seconds(self.get_clock(), self.last_visualization_time) \
                >= 1.0 / self.publish_visualization_rate:
            self.last_visualization_time = now
            return True
        return False

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
            self._clear_visualization()
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

            nearest_obstacle = None
            if self.obstacle_enabled:
                if age_seconds(self.get_clock(), self.last_scan_time) \
                        > self.obstacle_stale_timeout_s:
                    # Fail safe: if the scan feed drops, stop trusting the
                    # last-known obstacle position rather than keep steering
                    # around a possibly-stale one.
                    self.detected_obstacles = []
                nearest_obstacle = self._nearest_relevant_obstacle(s0)

            reference_state, reference_input, sample_s = (
                self._build_progress_reference(s0, nearest_obstacle))
            yaw_unwrapped = reference_state[3, 0] + angle_difference(
                yaw, reference_state[3, 0])
            initial_state = np.array(
                [x, y, v, yaw_unwrapped, s0])

            if self.obstacle_enabled:
                self.update_obstacle_constraints(
                    x, y, sample_s, nearest_obstacle)

            control, predicted, solve_ms, osqp_solve_ms, solver_iterations = (
                self.solve_mpc(
                    initial_state, reference_state, reference_input, sample_s))

            solve_msg = Float64()
            solve_msg.data = solve_ms
            self.solve_time_pub.publish(solve_msg)

            osqp_solve_msg = Float64()
            osqp_solve_msg.data = osqp_solve_ms
            self.osqp_solve_time_pub.publish(osqp_solve_msg)

            iteration_msg = Float64()
            iteration_msg.data = solver_iterations
            self.solver_iterations_pub.publish(iteration_msg)

            progress_msg = Float64()
            progress_msg.data = (
                s0 / self.raceline.length if self.raceline.length else 0.0)
            self.progress_pub.publish(progress_msg)

            if self._should_publish_visualization():
                self._publish_path(
                    self.predicted_path_pub,
                    predicted[0], predicted[1], predicted[3])
                self._publish_path(
                    self.reference_path_pub,
                    reference_state[0], reference_state[1],
                    reference_state[3])

            self.last_solution_ok = True
            self.consecutive_solver_failures = 0

            acceleration = float(control[0, 0])
            raw_steering = clamp(
                float(control[1, 0]),
                -self.max_steering_angle, self.max_steering_angle)
            max_steering_step = self.max_steering_rate * self.dt
            steering = clamp(
                raw_steering,
                self.previous_steering - max_steering_step,
                self.previous_steering + max_steering_step)
            active_speed_cap = self._active_speed_cap()
            command_speed = clamp(
                v + acceleration * self.dt, 0.0, active_speed_cap)

            self.proposed_drive_pub.publish(
                build_ackermann(self.get_clock(), self.base_frame_id,
                                 command_speed, steering))

            if not self.enabled:
                self._publish_stop()
                return

            self.drive_pub.publish(
                build_ackermann(self.get_clock(), self.base_frame_id,
                                 command_speed, steering))
            self.previous_steering = steering

            if solve_ms > 1000.0 / self.control_rate:
                self.warn_throttle.warn(
                    self.get_clock(),
                    'MPCC solve time %.1f ms exceeds %.0f Hz period'
                    % (solve_ms, self.control_rate))
        except Exception as error:
            # Broad on purpose: this loop drives a physical vehicle, and
            # an unhandled exception here would kill the whole node
            # (losing the safety-stop it was about to publish) instead of
            # degrading to a stop. Concretely hit in testing: cvxpy/OSQP
            # can raise a plain ValueError ("Upper bound update error!")
            # from warm-started solver internals on some parameter
            # transitions, which is not wrapped in cp.error.SolverError
            # and was previously uncaught here.
            self.last_solution_ok = False
            self.consecutive_solver_failures += 1
            self.previous_steering = 0.0
            self._publish_stop()
            self._clear_visualization()
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
        if rclpy.ok():
            node._publish_stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
