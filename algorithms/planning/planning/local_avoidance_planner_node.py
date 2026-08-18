import math

import numpy as np
import rclpy
from geometry_msgs.msg import Point, PoseArray
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy)
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray

from planning.path_utils import ClosedPath, build_path_msg, wrap_delta


def clamp(value, lo, hi):
    return max(lo, min(hi, value))


class LocalAvoidancePlannerNode(Node):
    def __init__(self):
        super().__init__('local_avoidance_planner_node')

        self.declare_parameter('global_path_topic', '/planning/global_path')
        self.declare_parameter('obstacle_topic', '/planning/detected_obstacles')
        self.declare_parameter('odom_topic', '/car_state/odom')
        self.declare_parameter('map_topic', '/map')
        self.declare_parameter('output_path_topic', '/planning/path')
        self.declare_parameter('marker_topic', '/planning/local_replan_markers')
        self.declare_parameter('state_topic', '/planning/replan_state')
        self.declare_parameter('publish_rate', 20.0)
        self.declare_parameter('interest_horizon_m', 5.5)
        self.declare_parameter('obstacle_stale_timeout_s', 0.50)
        self.declare_parameter('obstacle_corridor_m', 0.55)
        self.declare_parameter('lane_offset_m', 0.32)
        self.declare_parameter('max_lane_offset_m', 0.55)
        self.declare_parameter('vehicle_width_m', 0.31)
        self.declare_parameter('obstacle_radius_m', 0.18)
        self.declare_parameter('safety_margin_m', 0.08)
        self.declare_parameter('ramp_in_m', 1.20)
        self.declare_parameter('ramp_out_m', 1.60)
        self.declare_parameter('obstacle_half_length_m', 0.35)
        self.declare_parameter('center_deadband_m', 0.05)
        self.declare_parameter('obstacle_filter_alpha', 0.25)
        self.declare_parameter('obstacle_update_epsilon_m', 0.08)
        self.declare_parameter('obstacle_reset_distance_m', 0.80)
        self.declare_parameter('map_clearance_check_radius_m', 0.55)
        self.declare_parameter('min_wall_clearance_m', 0.16)
        self.declare_parameter('obstacle_path_clearance_m', 0.0)
        self.declare_parameter('side_switch_margin_m', 0.08)
        self.declare_parameter('state_switch_hold_s', 0.30)
        self.declare_parameter('side_lock_ego_offset_m', 0.25)
        self.declare_parameter('target_lateral_rate_limit_mps', 1.50)
        self.declare_parameter('corner_lookahead_m', 2.00)
        self.declare_parameter('corner_curvature_threshold', 0.15)
        self.declare_parameter('lateral_candidate_step_m', 0.05)
        self.declare_parameter('wall_clearance_weight', 1.00)
        self.declare_parameter('obstacle_clearance_weight', 0.35)
        self.declare_parameter('smoothness_weight', 0.20)
        self.declare_parameter('ego_lateral_weight', 0.10)
        self.declare_parameter('target_continuity_weight', 0.80)
        self.declare_parameter('default_side', 'right')

        self.path = ClosedPath()
        self.current_odom = None
        self.last_obstacle_time = None
        self.obstacles = []
        self.committed_side = None
        self.filtered_obstacle = None
        self.smoothed_target_lateral = None
        self.smoothed_target_time = None
        self.committed_state = 'GLOBAL'
        self.committed_points = None
        self.committed_side_out = 0.0
        self.pending_label = None
        self.pending_since = None
        self.pending_points = None
        self.pending_side = 0.0
        self.map_grid = None
        self.map_resolution = None
        self.map_origin_x = 0.0
        self.map_origin_y = 0.0
        self.map_width = 0
        self.map_height = 0

        self.interest_horizon = float(self.get_parameter('interest_horizon_m').value)
        self.obstacle_timeout = float(self.get_parameter('obstacle_stale_timeout_s').value)
        self.corridor = float(self.get_parameter('obstacle_corridor_m').value)
        self.lane_offset = abs(float(self.get_parameter('lane_offset_m').value))
        self.max_lane_offset = max(
            self.lane_offset,
            abs(float(self.get_parameter('max_lane_offset_m').value)))
        self.vehicle_width = float(self.get_parameter('vehicle_width_m').value)
        self.obstacle_radius = float(self.get_parameter('obstacle_radius_m').value)
        self.safety_margin = float(self.get_parameter('safety_margin_m').value)
        self.ramp_in = float(self.get_parameter('ramp_in_m').value)
        self.ramp_out = float(self.get_parameter('ramp_out_m').value)
        self.obstacle_half_length = float(
            self.get_parameter('obstacle_half_length_m').value)
        self.center_deadband = float(self.get_parameter('center_deadband_m').value)
        self.obstacle_filter_alpha = float(
            self.get_parameter('obstacle_filter_alpha').value)
        self.obstacle_update_epsilon = float(
            self.get_parameter('obstacle_update_epsilon_m').value)
        self.obstacle_reset_distance = float(
            self.get_parameter('obstacle_reset_distance_m').value)
        self.clearance_check_radius = float(
            self.get_parameter('map_clearance_check_radius_m').value)
        self.min_wall_clearance = float(
            self.get_parameter('min_wall_clearance_m').value)
        configured_clearance = float(
            self.get_parameter('obstacle_path_clearance_m').value)
        physical_clearance = (
            0.5 * self.vehicle_width + self.obstacle_radius
            + self.safety_margin)
        self.obstacle_path_clearance = max(
            configured_clearance, physical_clearance)
        self.side_switch_margin = float(
            self.get_parameter('side_switch_margin_m').value)
        self.state_switch_hold = float(
            self.get_parameter('state_switch_hold_s').value)
        self.side_lock_ego_offset = float(
            self.get_parameter('side_lock_ego_offset_m').value)
        self.target_lateral_rate_limit = max(
            0.01, float(self.get_parameter('target_lateral_rate_limit_mps').value))
        self.corner_lookahead = float(
            self.get_parameter('corner_lookahead_m').value)
        self.corner_curvature_threshold = float(
            self.get_parameter('corner_curvature_threshold').value)
        self.lateral_candidate_step = max(
            0.01, float(self.get_parameter('lateral_candidate_step_m').value))
        self.wall_clearance_weight = float(
            self.get_parameter('wall_clearance_weight').value)
        self.obstacle_clearance_weight = float(
            self.get_parameter('obstacle_clearance_weight').value)
        self.smoothness_weight = float(
            self.get_parameter('smoothness_weight').value)
        self.ego_lateral_weight = float(
            self.get_parameter('ego_lateral_weight').value)
        self.target_continuity_weight = float(
            self.get_parameter('target_continuity_weight').value)
        default_side = str(self.get_parameter('default_side').value).lower()
        self.default_side = 1.0 if default_side == 'left' else -1.0

        map_qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(
            OccupancyGrid, self.get_parameter('map_topic').value,
            self.map_callback, map_qos)
        self.create_subscription(
            Path, self.get_parameter('global_path_topic').value,
            self.global_path_callback, 10)
        self.create_subscription(
            PoseArray, self.get_parameter('obstacle_topic').value,
            self.obstacle_callback, 10)
        self.create_subscription(
            Odometry, self.get_parameter('odom_topic').value, self.odom_callback, 10)

        self.path_pub = self.create_publisher(
            Path, self.get_parameter('output_path_topic').value, 10)
        self.marker_pub = self.create_publisher(
            MarkerArray, self.get_parameter('marker_topic').value, 10)
        self.state_pub = self.create_publisher(
            String, self.get_parameter('state_topic').value, 10)

        self.timer = self.create_timer(
            1.0 / max(float(self.get_parameter('publish_rate').value), 0.1),
            self.publish)
        self.get_logger().info('local_avoidance_planner_node started')

    def map_callback(self, msg):
        self.map_resolution = float(msg.info.resolution)
        self.map_origin_x = float(msg.info.origin.position.x)
        self.map_origin_y = float(msg.info.origin.position.y)
        self.map_width = int(msg.info.width)
        self.map_height = int(msg.info.height)
        self.map_grid = np.asarray(msg.data, dtype=np.int16).reshape(
            self.map_height, self.map_width)

    def global_path_callback(self, msg):
        if self.path.update_from_path(msg):
            return
        self.get_logger().warn('Ignoring invalid global path', throttle_duration_sec=1.0)

    def obstacle_callback(self, msg):
        self.last_obstacle_time = self.get_clock().now()
        self.obstacles = [(pose.position.x, pose.position.y) for pose in msg.poses]

    def odom_callback(self, msg):
        self.current_odom = msg

    def _obstacles_are_fresh(self):
        if self.last_obstacle_time is None:
            return False
        age = (self.get_clock().now() - self.last_obstacle_time).nanoseconds * 1.0e-9
        return age <= self.obstacle_timeout

    def _select_obstacle(self, ego_s):
        if not self._obstacles_are_fresh():
            return None
        candidates = []
        for obs_x, obs_y in self.obstacles:
            obs_s, lateral, distance, path_yaw = self.path.nearest(obs_x, obs_y)
            ds = (obs_s - ego_s) % self.path.length
            if ds <= self.interest_horizon and distance <= self.corridor:
                candidates.append((ds, obs_x, obs_y, obs_s, lateral, path_yaw))
        if not candidates:
            return None
        candidates.sort(key=lambda item: (abs(item[4]), item[0]))
        return candidates[0]

    def _sample_path_pose(self, s_value):
        s_value = s_value % self.path.length
        index = int(np.searchsorted(self.path.cumulative, s_value, side='right') - 1)
        index = max(0, min(index, len(self.path.points) - 1))
        segment_length = max(float(self.path.segment_lengths[index]), 1.0e-6)
        fraction = (s_value - float(self.path.cumulative[index])) / segment_length
        start = self.path.points[index]
        end = self.path.points[(index + 1) % len(self.path.points)]
        point = start + max(0.0, min(1.0, fraction)) * (end - start)
        return point, float(self.path.yaw[index])

    def _stabilize_obstacle(self, obstacle):
        ds, _obs_x, _obs_y, obs_s, lateral, _path_yaw = obstacle
        if self.filtered_obstacle is None:
            filtered_s = obs_s
            filtered_lateral = lateral
        else:
            prev_s, prev_lateral = self.filtered_obstacle
            s_delta = wrap_delta(obs_s, prev_s, self.path.length)
            lateral_delta = lateral - prev_lateral
            motion = math.hypot(s_delta, lateral_delta)
            if abs(s_delta) > self.obstacle_reset_distance:
                filtered_s = obs_s
                filtered_lateral = lateral
            elif motion < self.obstacle_update_epsilon:
                filtered_s = prev_s
                filtered_lateral = prev_lateral
            else:
                alpha = max(0.0, min(1.0, self.obstacle_filter_alpha))
                filtered_s = (prev_s + alpha * s_delta) % self.path.length
                filtered_lateral = prev_lateral + alpha * lateral_delta

        self.filtered_obstacle = (filtered_s, filtered_lateral)
        center, yaw = self._sample_path_pose(filtered_s)
        normal = np.array([-math.sin(yaw), math.cos(yaw)])
        position = center + normal * filtered_lateral
        return (ds, float(position[0]), float(position[1]),
                filtered_s, filtered_lateral, yaw)

    def _corner_outside_side(self, obs_s):
        """Side that is on the outside of any corner starting near the
        obstacle, using path curvature ahead. Positive (mean) curvature is
        a left turn, whose outside is to the right, and vice versa. Returns
        None when the road ahead is close enough to straight that this
        shouldn't override anything."""
        mean_kappa = self.path.mean_curvature_ahead(
            obs_s, self.corner_lookahead)
        if abs(mean_kappa) < self.corner_curvature_threshold:
            return None
        return -1.0 if mean_kappa > 0.0 else 1.0

    def _preferred_side(self, obstacle_lateral, obs_s):
        if obstacle_lateral > self.center_deadband:
            return -1.0
        if obstacle_lateral < -self.center_deadband:
            return 1.0
        corner_side = self._corner_outside_side(obs_s)
        if corner_side is not None:
            return corner_side
        if self.committed_side is not None:
            return self.committed_side
        return self.default_side

    def _world_to_grid(self, x, y):
        if self.map_grid is None or self.map_resolution is None:
            return None
        gx = int((x - self.map_origin_x) / self.map_resolution)
        gy = int((y - self.map_origin_y) / self.map_resolution)
        if gx < 0 or gy < 0 or gx >= self.map_width or gy >= self.map_height:
            return None
        return gx, gy

    def _cell_is_occupied(self, gx, gy):
        if gx < 0 or gy < 0 or gx >= self.map_width or gy >= self.map_height:
            return True
        value = int(self.map_grid[gy, gx])
        return value < 0 or value >= 50

    def _point_wall_clearance(self, x, y):
        grid = self._world_to_grid(x, y)
        if grid is None:
            return -1.0
        gx, gy = grid
        max_cells = max(1, int(math.ceil(
            self.clearance_check_radius / self.map_resolution)))
        for radius in range(max_cells + 1):
            x0 = gx - radius
            x1 = gx + radius
            y0 = gy - radius
            y1 = gy + radius
            for cx in range(x0, x1 + 1):
                if self._cell_is_occupied(cx, y0) or self._cell_is_occupied(cx, y1):
                    return max(0.0, (radius - 1) * self.map_resolution)
            for cy in range(y0 + 1, y1):
                if self._cell_is_occupied(x0, cy) or self._cell_is_occupied(x1, cy):
                    return max(0.0, (radius - 1) * self.map_resolution)
        return self.clearance_check_radius

    def _candidate_laterals_for_side(self, obstacle_lateral, side):
        min_target = obstacle_lateral + side * self.obstacle_path_clearance
        if abs(min_target) < self.lane_offset:
            min_target = side * self.lane_offset
        min_target = clamp(min_target, -self.max_lane_offset,
                           self.max_lane_offset)

        signed_limit = side * self.max_lane_offset
        if side > 0.0:
            values = np.arange(
                min_target,
                signed_limit + 0.5 * self.lateral_candidate_step,
                self.lateral_candidate_step)
        else:
            values = np.arange(
                min_target,
                signed_limit - 0.5 * self.lateral_candidate_step,
                -self.lateral_candidate_step)

        candidates = [
            float(clamp(value, -self.max_lane_offset, self.max_lane_offset))
            for value in values]
        candidates.append(float(signed_limit))

        unique = []
        for value in candidates:
            if not unique or abs(value - unique[-1]) > 1.0e-4:
                unique.append(value)
        return unique

    def _build_points_for_target(self, obstacle, target_lateral):
        _ds, _obs_x, _obs_y, obs_s, _lateral, _path_yaw = obstacle
        shifted = np.array(self.path.points, copy=True)
        active_indices = []

        for index, base in enumerate(self.path.points):
            delta = wrap_delta(
                float(self.path.cumulative[index]), obs_s, self.path.length)
            if (delta < -self.ramp_in
                    or delta > self.obstacle_half_length + self.ramp_out):
                weight = 0.0
            elif delta < 0.0:
                weight = 0.5 * (
                    1.0 - math.cos(
                        math.pi * (delta + self.ramp_in) / self.ramp_in))
            elif delta <= self.obstacle_half_length:
                weight = 1.0
            else:
                weight = 0.5 * (
                    1.0 + math.cos(
                        math.pi * (delta - self.obstacle_half_length)
                        / self.ramp_out))

            if weight <= 0.0:
                continue
            yaw = float(self.path.yaw[index])
            normal = np.array([-math.sin(yaw), math.cos(yaw)])
            shifted[index] = base + normal * (target_lateral * weight)
            active_indices.append(index)
        return shifted, active_indices, target_lateral

    def _smoothness_cost(self, points, active_indices):
        if len(active_indices) < 3:
            return 0.0
        total = 0.0
        count = 0
        path_count = len(points)
        for index in active_indices:
            previous_point = points[(index - 1) % path_count]
            point = points[index]
            next_point = points[(index + 1) % path_count]
            first = point - previous_point
            second = next_point - point
            first_norm = float(np.linalg.norm(first))
            second_norm = float(np.linalg.norm(second))
            if first_norm <= 1.0e-6 or second_norm <= 1.0e-6:
                continue
            cos_angle = clamp(
                float(np.dot(first, second) / (first_norm * second_norm)),
                -1.0, 1.0)
            total += abs(math.acos(cos_angle))
            count += 1
        return total / max(count, 1)

    def _score_candidate(self, points, active_indices, obstacle,
                         target_lateral, ego_lateral, side):
        if not active_indices:
            return {
                'feasible': False,
                'score': -float('inf'),
                'min_wall_clearance': -1.0,
                'min_obstacle_distance': 0.0,
                'obstacle_margin': -float('inf'),
                'smoothness_cost': float('inf'),
                'continuity_cost': float('inf'),
            }
        _ds, obs_x, obs_y, _obs_s, _lateral, _path_yaw = obstacle
        min_wall_clearance = self.clearance_check_radius
        min_obstacle_distance = float('inf')
        for index in active_indices:
            point = points[index]
            wall_clearance = self._point_wall_clearance(
                float(point[0]), float(point[1]))
            min_wall_clearance = min(min_wall_clearance, wall_clearance)
            min_obstacle_distance = min(
                min_obstacle_distance,
                math.hypot(float(point[0]) - obs_x, float(point[1]) - obs_y))

        obstacle_margin = min_obstacle_distance - self.obstacle_path_clearance
        smoothness_cost = self._smoothness_cost(points, active_indices)
        continuity_cost = 0.0
        if self.smoothed_target_lateral is not None:
            continuity_cost = abs(
                target_lateral - self.smoothed_target_lateral)
        feasible = (
            obstacle_margin >= 0.0
            and min_wall_clearance >= self.min_wall_clearance)
        if not feasible:
            score = min(obstacle_margin,
                        min_wall_clearance - self.min_wall_clearance)
        else:
            score = (
                self.wall_clearance_weight * min_wall_clearance
                + self.obstacle_clearance_weight
                * min(obstacle_margin, self.clearance_check_radius)
                - self.smoothness_weight * smoothness_cost
                - self.ego_lateral_weight * abs(target_lateral - ego_lateral))
            # Map-cell quantization can make near-equivalent candidates
            # alternate between ticks. Stabilize the argmax itself before
            # the separate output slew limiter is applied.
            score -= self.target_continuity_weight * continuity_cost
            if self.committed_side is not None and side == self.committed_side:
                score += self.side_switch_margin

        return {
            'feasible': feasible,
            'score': score,
            'min_wall_clearance': min_wall_clearance,
            'min_obstacle_distance': min_obstacle_distance,
            'obstacle_margin': obstacle_margin,
            'smoothness_cost': smoothness_cost,
            'continuity_cost': continuity_cost,
        }

    def _slew_target_lateral(self, goal):
        """Rate-limit the published target_lateral toward `goal` instead of
        snapping to it every tick. _build_replanned_points' argmax over a
        discretized candidate set is not smooth even for a stationary
        obstacle (wall clearance is grid-quantized, candidates are spaced
        lateral_candidate_step_m apart) -- confirmed live to swing the
        selected target by up to ~0.25m tick to tick, and occasionally flip
        sides outright, which is what continuously feeding into mpcc_node's
        raceline eventually turned into a real collision. This smooths the
        selection itself, not just the input obstacle position (which
        _stabilize_obstacle already does but that wasn't enough)."""
        now = self.get_clock().now()
        if self.smoothed_target_lateral is None or self.smoothed_target_time is None:
            self.smoothed_target_lateral = goal
            self.smoothed_target_time = now
            return goal
        dt = (now - self.smoothed_target_time).nanoseconds * 1.0e-9
        self.smoothed_target_time = now
        max_step = self.target_lateral_rate_limit * max(dt, 0.0)
        delta = clamp(goal - self.smoothed_target_lateral, -max_step, max_step)
        self.smoothed_target_lateral += delta
        return self.smoothed_target_lateral

    def _build_replanned_points(self, obstacle, ego_lateral):
        preferred = self._preferred_side(obstacle[4], obstacle[3])
        if (self.committed_side is not None
                and abs(ego_lateral) > self.side_lock_ego_offset
                and preferred != self.committed_side):
            # Mid-maneuver and meaningfully off centerline: don't even
            # consider flipping sides, only re-evaluate the committed one.
            # Flipping while already committed to a lateral offset is a
            # much sharper, more dangerous correction than flipping from
            # near-centerline.
            candidate_sides = [self.committed_side]
        else:
            candidate_sides = [preferred, -preferred]
        candidates = []

        for side in candidate_sides:
            for target_lateral in self._candidate_laterals_for_side(
                    obstacle[4], side):
                points, active_indices, target_lateral = (
                    self._build_points_for_target(obstacle, target_lateral))
                metrics = self._score_candidate(
                    points, active_indices, obstacle, target_lateral,
                    ego_lateral, side)
                candidates.append((metrics['score'], metrics, side,
                                   points, target_lateral))

        if not candidates:
            self.committed_side = None
            return np.array(self.path.points, copy=True), 0.0, True

        left_best = max(
            (score for score, _metrics, side, _points, _target in candidates
             if side > 0.0),
            default=-float('inf'))
        right_best = max(
            (score for score, _metrics, side, _points, _target in candidates
             if side < 0.0),
            default=-float('inf'))
        candidates.sort(
            key=lambda item: (
                item[0],
                item[1]['feasible'],
                -item[1]['continuity_cost'],
                1 if self.committed_side is not None
                and item[2] == self.committed_side else 0),
            reverse=True)
        feasible_candidates = [
            item for item in candidates if item[1]['feasible']]
        best = feasible_candidates[0] if feasible_candidates else candidates[0]
        best_score, best_metrics, best_side, best_points, best_target = best

        self.get_logger().info(
            ('avoidance candidates: left=%.2f, right=%.2f, selected=%s '
             'd=%.2f wall=%.2f obs=%.2f score=%.2f')
            % (
                left_best,
                right_best,
                'left' if best_side > 0.0 else 'right',
                best_target,
                best_metrics['min_wall_clearance'],
                best_metrics['min_obstacle_distance'],
                best_score),
            throttle_duration_sec=1.0)

        if not best_metrics['feasible']:
            self.get_logger().warn(
                ('local avoidance blocked: wall=%.2f m req=%.2f m, '
                 'obstacle_margin=%.2f m')
                % (
                    best_metrics['min_wall_clearance'],
                    self.min_wall_clearance,
                    best_metrics['obstacle_margin']),
                throttle_duration_sec=1.0)
            self.committed_side = None
            self.smoothed_target_lateral = None
            self.smoothed_target_time = None
            return np.array(self.path.points, copy=True), 0.0, True

        smoothed_target = self._slew_target_lateral(best_target)
        smoothed_points, _active_indices, smoothed_target = (
            self._build_points_for_target(obstacle, smoothed_target))

        self.get_logger().info(
            'avoidance selected %s target_d=%.2f m (goal %.2f m) clearance_req=%.2f m'
            % ('left' if best_side > 0.0 else 'right',
               smoothed_target, best_target, self.obstacle_path_clearance),
            throttle_duration_sec=1.0)

        self.committed_side = best_side
        return smoothed_points, best_side, False

    def _publish_markers(self, active, obstacle=None, side=0.0):
        markers = MarkerArray()
        header_stamp = self.get_clock().now().to_msg()
        clear = Marker()
        clear.header.stamp = header_stamp
        clear.header.frame_id = self.path.frame_id if self.path.ready else 'map'
        clear.ns = 'local_replan'
        clear.action = Marker.DELETEALL
        markers.markers.append(clear)

        if active and obstacle is not None:
            _ds, obs_x, obs_y, _obs_s, _lat, _yaw = obstacle
            marker = Marker()
            marker.header = clear.header
            marker.ns = 'local_replan'
            marker.id = 1
            marker.type = Marker.ARROW
            marker.action = Marker.ADD
            marker.scale.x = 0.45
            marker.scale.y = 0.08
            marker.scale.z = 0.08
            marker.color.r = 0.1
            marker.color.g = 0.7
            marker.color.b = 1.0
            marker.color.a = 0.95
            p0 = Point()
            p0.x = float(obs_x)
            p0.y = float(obs_y)
            p0.z = 0.20
            p1 = Point()
            p1.x = float(obs_x)
            p1.y = float(obs_y + side * 0.45)
            p1.z = 0.20
            marker.points = [p0, p1]
            markers.markers.append(marker)
        self.marker_pub.publish(markers)

    def _commit(self, state, points, side):
        self.committed_state = state
        self.committed_points = points
        self.committed_side_out = side
        self.pending_label = None
        self.pending_since = None
        self.pending_points = None

    def _debounce(self, state, points, side):
        """Only let the published state/path change after the newly
        computed candidate wins for state_switch_hold_s straight, instead
        of on every 8Hz tick -- a single noisy tick (borderline wall/
        obstacle clearance, a jittery LiDAR cluster) must not flip
        /planning/replan_state and re-route MPCC."""
        if self.committed_points is None:
            self._commit(state, points, side)
            return self.committed_state, self.committed_points, self.committed_side_out

        if state == self.committed_state:
            # Only the state *label* is debounced -- geometry stays frozen
            # at the commit snapshot. Tried refreshing every tick twice
            # today (once raw, once with _slew_target_lateral rate-
            # limiting the candidate selection): both caused a real
            # collision in extended live stress testing (confirmed via
            # `MPCC disabled: simulator collision reported` + the vehicle
            # ending up ~170 deg off heading afterward). The rate limiter
            # reduced how often it happened but did not make it safe.
            # Freezing at commit is the proven-safe choice -- see docs
            # worklog 2026-08-18 for both incidents. This does mean a
            # genuinely moving obstacle won't get a re-shaped avoidance
            # path mid-maneuver, but there is no obstacle-motion tracking
            # in this stack at all yet, so that's not a live gap. Do not
            # re-enable this without first fixing why
            # _build_replanned_points' argmax is unstable in the first
            # place (grid-quantized wall clearance, discretized candidate
            # sweep) rather than just smoothing its output harder.
            self.pending_label = None
            self.pending_since = None
        else:
            now = self.get_clock().now()
            if state != self.pending_label:
                self.pending_label = state
                self.pending_since = now
            self.pending_points = points
            self.pending_side = side
            elapsed = (now - self.pending_since).nanoseconds * 1.0e-9
            if elapsed >= self.state_switch_hold:
                self._commit(state, points, side)

        return self.committed_state, self.committed_points, self.committed_side_out

    def publish(self):
        if not self.path.ready:
            return

        active = False
        obstacle = None
        side = 0.0
        points = self.path.points
        state = 'GLOBAL'

        if self.current_odom is not None:
            x = self.current_odom.pose.pose.position.x
            y = self.current_odom.pose.pose.position.y
            ego_s, ego_lateral, _dist, _yaw = self.path.nearest(x, y)
            obstacle = self._select_obstacle(ego_s)
            if obstacle is not None:
                obstacle = self._stabilize_obstacle(obstacle)
                points, side, blocked = self._build_replanned_points(
                    obstacle, ego_lateral)
                if blocked:
                    state = 'BLOCKED'
                else:
                    if side > 0.0:
                        state = 'LOCAL_AVOIDANCE_LEFT'
                    else:
                        state = 'LOCAL_AVOIDANCE_RIGHT'
            else:
                self.committed_side = None
                self.filtered_obstacle = None
                self.smoothed_target_lateral = None
                self.smoothed_target_time = None

        state, points, side = self._debounce(state, points, side)
        active = state != 'GLOBAL'

        self.path_pub.publish(build_path_msg(self, points, self.path.frame_id))
        self._publish_markers(active, obstacle, side)
        self.state_pub.publish(String(data=state))


def main(args=None):
    rclpy.init(args=args)
    node = LocalAvoidancePlannerNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
