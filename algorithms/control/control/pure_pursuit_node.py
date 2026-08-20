import bisect
import math

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time

from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import Bool, Float32MultiArray, Float64, String
from std_srvs.srv import SetBool
from tf2_ros import Buffer, TransformException, TransformListener


class PurePursuitNode(Node):
    def __init__(self):
        super().__init__('pure_pursuit_node')

        self.declare_parameter('drive_mode', 'sim')
        self.declare_parameter('enabled', False)
        self.declare_parameter('auto_enable', False)
        self.declare_parameter('auto_enable_retry_period', 0.5)

        self.declare_parameter('global_frame_id', 'map')
        self.declare_parameter('base_frame_id', 'ego_racecar/base_link')
        self.declare_parameter('odom_topic', '/ego_racecar/odom')
        self.declare_parameter('path_topic', '/planning/path')
        self.declare_parameter('speed_profile_topic', '/planning/speed_profile')
        self.declare_parameter('use_speed_profile', True)
        self.declare_parameter('speed_profile_timeout', 2.00)
        self.declare_parameter('sim_drive_topic', '/drive')
        self.declare_parameter('real_speed_topic', '/commands/motor/speed')
        self.declare_parameter('real_servo_topic', '/commands/servo/position')

        self.declare_parameter('wheelbase', 0.33)
        self.declare_parameter('lookahead_distance', 0.70)
        self.declare_parameter('max_steering_angle', 0.4189)
        self.declare_parameter('max_path_distance', 1.00)
        self.declare_parameter('max_heading_error', 1.0472)
        self.declare_parameter('search_back_points', 8)
        self.declare_parameter('search_forward_points', 30)

        self.declare_parameter('target_speed', 0.60)
        self.declare_parameter('min_speed', 0.25)
        self.declare_parameter('max_speed', 0.80)
        self.declare_parameter('corner_slowdown_gain', 0.55)

        # Ported from f1tenth_kkh/mpcc_node.py's _active_speed_cap (same
        # defaults) -- pure_pursuit had zero awareness of avoidance state
        # until now, so a BLOCKED replan (raw unshifted path, i.e. driving
        # straight at the obstacle -- confirmed live 2026-08-20/21) never
        # slowed down. mpcc's own max_speed is only 1.0 m/s so
        # avoidance_speed_cap=1.80 was never exercised above that in
        # mpcc's real usage -- treat these as a starting point pending
        # empirical re-tuning at pure_pursuit's actual speed regime.
        self.declare_parameter('replan_state_topic', '/planning/replan_state')
        self.declare_parameter('replan_state_timeout', 0.50)
        self.declare_parameter('avoidance_speed_cap', 1.80)
        self.declare_parameter('blocked_speed_cap', 0.0)
        # 2026-08-21: mpcc's 1.50/0.80 ramp rates were sized for its own
        # max_speed of 1.0 m/s -- confirmed live at pure_pursuit's actual
        # 8.0 m/s max_speed, 0.80 m/s^2 meant a full cap recovery back to
        # max_speed took up to (8.0-0.0)/0.80 = 10 seconds, so the car
        # stayed artificially slow long after clearing an obstacle and
        # returning to GLOBAL. Scaled up to be comparable to
        # speed_profile_node's own accel/decel limits instead of mpcc's
        # unrelated speed range.
        self.declare_parameter('avoidance_cap_decel_mps2', 6.00)
        self.declare_parameter('avoidance_cap_accel_mps2', 5.00)

        # Self-contained stall/no-progress watchdog -- commanding a
        # meaningful speed but not actually moving (e.g. wedged against
        # an obstacle after a bad avoidance decision) auto-disables
        # instead of continuing to command torque indefinitely. Starting
        # points pending empirical tuning.
        self.declare_parameter('stall_watchdog_enabled', True)
        self.declare_parameter('stall_speed_threshold_mps', 0.15)
        self.declare_parameter('stall_command_speed_threshold_mps', 0.30)
        self.declare_parameter('stall_timeout_s', 1.00)
        # 2026-08-21: found live -- an external kill-switch/safety stop
        # (vehicle_interface_node's /safety/stop_required, see
        # AVOIDANCE_TUNING.md) legitimately zeros real vehicle speed
        # while pure_pursuit itself is unaware and keeps computing a
        # nonzero commanded_speed -- without this, the stall watchdog
        # misreads that as "stuck" and auto-disables after
        # stall_timeout_s, so releasing the kill switch does NOT resume
        # driving (needs a manual /control/enable call), which directly
        # fights the "Fully Autonomous" competition requirement. Same
        # default topic as vehicle_interface_node's safety_stop_topic so
        # this works out of the box without extra launch wiring.
        self.declare_parameter('safety_stop_topic', '/safety/stop_required')

        self.declare_parameter('speed_to_erpm_gain', 3000.0)
        self.declare_parameter('speed_to_erpm_offset', 0.0)
        self.declare_parameter('servo_center', 0.5)
        self.declare_parameter('servo_gain', 1.0)
        self.declare_parameter('servo_min', 0.0)
        self.declare_parameter('servo_max', 1.0)

        self.declare_parameter('control_rate', 30.0)
        self.declare_parameter('odom_timeout', 0.50)
        self.declare_parameter('path_timeout', 2.00)

        self.drive_mode = self.get_parameter('drive_mode').value
        self.enabled = bool(self.get_parameter('enabled').value)
        self.auto_enable = bool(self.get_parameter('auto_enable').value)
        self.auto_enable_retry_period = max(
            0.05, float(self.get_parameter('auto_enable_retry_period').value))

        self.global_frame_id = self.get_parameter('global_frame_id').value
        self.base_frame_id = self.get_parameter('base_frame_id').value
        self.odom_topic = self.get_parameter('odom_topic').value
        self.path_topic = self.get_parameter('path_topic').value
        self.speed_profile_topic = self.get_parameter('speed_profile_topic').value
        self.use_speed_profile = bool(
            self.get_parameter('use_speed_profile').value)
        self.speed_profile_timeout = float(
            self.get_parameter('speed_profile_timeout').value)
        self.sim_drive_topic = self.get_parameter('sim_drive_topic').value
        self.real_speed_topic = self.get_parameter('real_speed_topic').value
        self.real_servo_topic = self.get_parameter('real_servo_topic').value

        self.wheelbase = float(self.get_parameter('wheelbase').value)
        self.lookahead_distance = float(
            self.get_parameter('lookahead_distance').value)
        self.max_steering_angle = float(
            self.get_parameter('max_steering_angle').value)
        self.max_path_distance = float(
            self.get_parameter('max_path_distance').value)
        self.max_heading_error = float(
            self.get_parameter('max_heading_error').value)
        self.search_back_points = int(
            self.get_parameter('search_back_points').value)
        self.search_forward_points = int(
            self.get_parameter('search_forward_points').value)

        self.target_speed = float(self.get_parameter('target_speed').value)
        self.min_speed = float(self.get_parameter('min_speed').value)
        self.max_speed = float(self.get_parameter('max_speed').value)
        self.corner_slowdown_gain = float(
            self.get_parameter('corner_slowdown_gain').value)

        self.replan_state_topic = self.get_parameter('replan_state_topic').value
        self.replan_state_timeout = float(
            self.get_parameter('replan_state_timeout').value)
        self.avoidance_speed_cap = float(
            self.get_parameter('avoidance_speed_cap').value)
        self.blocked_speed_cap = float(
            self.get_parameter('blocked_speed_cap').value)
        self.avoidance_cap_decel = max(0.01, float(
            self.get_parameter('avoidance_cap_decel_mps2').value))
        self.avoidance_cap_accel = max(0.01, float(
            self.get_parameter('avoidance_cap_accel_mps2').value))

        self.stall_watchdog_enabled = bool(
            self.get_parameter('stall_watchdog_enabled').value)
        self.stall_speed_threshold = float(
            self.get_parameter('stall_speed_threshold_mps').value)
        self.stall_command_speed_threshold = float(
            self.get_parameter('stall_command_speed_threshold_mps').value)
        self.stall_timeout = float(
            self.get_parameter('stall_timeout_s').value)
        self.safety_stop_topic = self.get_parameter('safety_stop_topic').value

        self.speed_to_erpm_gain = float(
            self.get_parameter('speed_to_erpm_gain').value)
        self.speed_to_erpm_offset = float(
            self.get_parameter('speed_to_erpm_offset').value)
        self.servo_center = float(self.get_parameter('servo_center').value)
        self.servo_gain = float(self.get_parameter('servo_gain').value)
        self.servo_min = float(self.get_parameter('servo_min').value)
        self.servo_max = float(self.get_parameter('servo_max').value)

        self.odom_timeout = float(self.get_parameter('odom_timeout').value)
        self.path_timeout = float(self.get_parameter('path_timeout').value)
        control_rate = float(self.get_parameter('control_rate').value)

        if self.drive_mode not in ('sim', 'real'):
            raise RuntimeError("drive_mode must be 'sim' or 'real'")

        self.current_odom = None
        self.current_path = None
        self.last_odom_time = None
        self.last_path_time = None
        self.nearest_index = None
        self.path_cumulative_s = []
        self.path_length = 0.0
        self.profile_s = []
        self.profile_speed = []
        self.last_profile_time = None
        self.last_status_message = None
        self.last_status_time = None

        self.replan_state = 'GLOBAL'
        self.last_replan_state_time = None
        self._ramped_speed_cap = self.max_speed
        self._last_speed_cap_update = None

        self.stall_condition_since = None
        self.external_safety_stop_active = False

        self.tf_buffer = Buffer(cache_time=Duration(seconds=5.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.create_subscription(
            Odometry, self.odom_topic, self.odom_callback, 10)
        self.create_subscription(
            Path, self.path_topic, self.path_callback, 10)
        self.create_subscription(
            Float32MultiArray, self.speed_profile_topic,
            self.speed_profile_callback, 10)
        self.create_subscription(
            String, self.replan_state_topic, self.replan_state_callback, 10)
        self.create_subscription(
            Bool, self.safety_stop_topic, self.safety_stop_callback, 10)

        self.sim_drive_pub = self.create_publisher(
            AckermannDriveStamped, self.sim_drive_topic, 10)
        self.real_speed_pub = self.create_publisher(
            Float64, self.real_speed_topic, 10)
        self.real_servo_pub = self.create_publisher(
            Float64, self.real_servo_topic, 10)

        self.enable_service = self.create_service(
            SetBool, '/control/enable', self.enable_callback)
        self.timer = self.create_timer(
            1.0 / max(control_rate, 1.0), self.control_loop)
        self.auto_enable_timer = None
        if self.auto_enable:
            self.auto_enable_timer = self.create_timer(
                self.auto_enable_retry_period, self.auto_enable_tick)

        self.get_logger().info(
            'Pure Pursuit ready (enabled=%s, pose=%s -> %s, path=%s, '
            'drive=%s)' % (
                self.enabled,
                self.global_frame_id,
                self.base_frame_id,
                self.path_topic,
                self.sim_drive_topic if self.drive_mode == 'sim'
                else self.real_speed_topic,
            ))
        self.get_logger().info(
            'Start/stop: ros2 service call /control/enable '
            'std_srvs/srv/SetBool "{data: true|false}"')

    def odom_callback(self, msg):
        self.current_odom = msg
        self.last_odom_time = self.get_clock().now()

    def path_callback(self, msg):
        if not msg.poses:
            return
        if self.current_path is None or len(self.current_path.poses) != len(msg.poses):
            self.nearest_index = None
            self.rebuild_path_arc_lengths(msg.poses)
        self.current_path = msg
        self.last_path_time = self.get_clock().now()

    def rebuild_path_arc_lengths(self, poses):
        cumulative = [0.0]
        for i in range(1, len(poses)):
            p0 = poses[i - 1].pose.position
            p1 = poses[i].pose.position
            cumulative.append(
                cumulative[-1] + math.hypot(p1.x - p0.x, p1.y - p0.y))
        self.path_cumulative_s = cumulative
        if len(poses) >= 2:
            first = poses[0].pose.position
            last = poses[-1].pose.position
            self.path_length = cumulative[-1] + math.hypot(
                last.x - first.x, last.y - first.y)
        else:
            self.path_length = 0.0

    def speed_profile_callback(self, msg):
        data = msg.data
        if len(data) < 4:
            return
        self.profile_s = [float(data[i]) for i in range(0, len(data), 2)]
        self.profile_speed = [float(data[i + 1]) for i in range(0, len(data), 2)]
        self.last_profile_time = self.get_clock().now()

    def interpolate_profile_speed(self, s):
        if len(self.profile_s) < 2:
            return None
        idx = bisect.bisect_right(self.profile_s, s) - 1
        idx = max(0, min(idx, len(self.profile_s) - 2))
        s0, s1 = self.profile_s[idx], self.profile_s[idx + 1]
        v0, v1 = self.profile_speed[idx], self.profile_speed[idx + 1]
        if s1 <= s0:
            return v0
        t = self.clamp((s - s0) / (s1 - s0), 0.0, 1.0)
        return v0 + t * (v1 - v0)

    def profile_target_speed(self):
        if (not self.use_speed_profile or self.nearest_index is None
                or not self.path_cumulative_s
                or self.nearest_index >= len(self.path_cumulative_s)):
            return None
        if self.age_seconds(self.last_profile_time) > self.speed_profile_timeout:
            return None
        s = self.path_cumulative_s[self.nearest_index]
        if self.path_length > 0.0:
            s = s % self.path_length
        return self.interpolate_profile_speed(s)

    def _speed_cap_category(self, state):
        if state == 'BLOCKED':
            return 'BLOCKED'
        if state.startswith('LOCAL_AVOIDANCE'):
            return 'AVOIDANCE'
        return 'GLOBAL'

    def replan_state_callback(self, msg):
        new_state = str(msg.data)
        previous_category = self._speed_cap_category(self.replan_state)
        new_category = self._speed_cap_category(new_state)
        if new_category == 'AVOIDANCE' and previous_category != 'AVOIDANCE':
            # Hold the current speed cap for exactly the next control tick
            # instead of ramping immediately -- see _active_avoidance_speed_cap.
            self._last_speed_cap_update = None
        self.replan_state = new_state
        self.last_replan_state_time = self.get_clock().now()

    def _active_avoidance_speed_cap(self):
        """Ported from f1tenth_kkh/mpcc_node.py's _active_speed_cap --
        BLOCKED snaps immediately to blocked_speed_cap (an emergency-stop
        condition, not ramped); LOCAL_AVOIDANCE_* ramps toward
        avoidance_speed_cap; otherwise ramps back toward max_speed."""
        target_cap = self.max_speed
        if (self.last_replan_state_time is not None
                and self.age_seconds(self.last_replan_state_time)
                <= self.replan_state_timeout
                and self.replan_state == 'BLOCKED'):
            self._ramped_speed_cap = min(
                self.max_speed, self.blocked_speed_cap)
            self._last_speed_cap_update = self.get_clock().now()
            return self._ramped_speed_cap
        elif (self.last_replan_state_time is not None
                and self.age_seconds(self.last_replan_state_time)
                <= self.replan_state_timeout
                and self.replan_state.startswith('LOCAL_AVOIDANCE')):
            target_cap = min(self.max_speed, self.avoidance_speed_cap)

        now = self.get_clock().now()
        if self._last_speed_cap_update is None:
            self._last_speed_cap_update = now
            return self._ramped_speed_cap

        dt = max(0.0, (now - self._last_speed_cap_update).nanoseconds * 1.0e-9)
        if target_cap < self._ramped_speed_cap:
            self._ramped_speed_cap = max(
                target_cap,
                self._ramped_speed_cap - self.avoidance_cap_decel * dt)
        else:
            self._ramped_speed_cap = min(
                target_cap,
                self._ramped_speed_cap + self.avoidance_cap_accel * dt)
        self._last_speed_cap_update = now
        return self._ramped_speed_cap

    def safety_stop_callback(self, msg):
        self.external_safety_stop_active = bool(msg.data)

    def _check_stall(self, commanded_speed):
        if self.external_safety_stop_active:
            # A kill switch/external safety stop legitimately zeroes real
            # vehicle speed -- not a stall. Without this guard, releasing
            # the kill switch would not resume driving (see the
            # 2026-08-21 comment on safety_stop_topic's declare_parameter).
            self.stall_condition_since = None
            return

        measured_speed = 0.0
        if self.current_odom is not None:
            twist = self.current_odom.twist.twist.linear
            measured_speed = math.hypot(twist.x, twist.y)

        stalled = (commanded_speed >= self.stall_command_speed_threshold
                   and measured_speed < self.stall_speed_threshold)
        now = self.get_clock().now()
        if not stalled:
            self.stall_condition_since = None
            return
        if self.stall_condition_since is None:
            self.stall_condition_since = now
            return
        elapsed = (now - self.stall_condition_since).nanoseconds * 1e-9
        if elapsed >= self.stall_timeout:
            self.get_logger().error(
                'Stall watchdog: commanding %.2f m/s but measured %.2f m/s '
                'for %.2fs -- auto-disabling (same effect as '
                '/control/enable false)'
                % (commanded_speed, measured_speed, elapsed))
            self.enabled = False
            self.nearest_index = None
            self.publish_stop()
            self.stall_condition_since = None

    def enable_callback(self, request, response):
        if not request.data:
            self.enabled = False
            self.nearest_index = None
            self.publish_stop()
            response.success = True
            response.message = 'Pure Pursuit stopped'
            self.get_logger().info(response.message)
            return response

        response.success, response.message = self._try_enable()
        if response.success:
            self.get_logger().info(response.message)
        else:
            self.get_logger().error(response.message)
        return response

    def _try_enable(self):
        """Shared by the /control/enable service and auto_enable_tick --
        both need identical readiness checks (path/odom freshness, TF,
        cross-track/heading error) before flipping self.enabled."""
        problem = self.readiness_problem()
        if problem is not None:
            self.enabled = False
            self.publish_stop()
            return False, 'Cannot start: ' + problem

        try:
            x, y, yaw = self.lookup_vehicle_pose()
        except TransformException as error:
            self.enabled = False
            self.publish_stop()
            return False, 'Cannot start: TF unavailable: ' + str(error)

        self.nearest_index = None
        _, path_distance, path_heading = self.nearest_path_state(x, y)
        heading_error = math.atan2(
            math.sin(path_heading - yaw), math.cos(path_heading - yaw))
        if path_distance > self.max_path_distance:
            self.enabled = False
            self.publish_stop()
            return False, (
                'Cannot start: vehicle is %.2f m from path (limit %.2f m)'
                % (path_distance, self.max_path_distance))
        if abs(heading_error) > self.max_heading_error:
            self.enabled = False
            self.publish_stop()
            return False, (
                'Cannot start: heading error is %.1f deg (limit %.1f deg)'
                % (math.degrees(abs(heading_error)),
                   math.degrees(self.max_heading_error)))

        self.enabled = True
        return True, 'Pure Pursuit enabled'

    def auto_enable_tick(self):
        """Retries _try_enable on a timer instead of the launch file's old
        one-shot `ros2 service call` at a fixed delay -- that raced
        AMCL/localized-odom startup (2026-08-20: fired at t=7s while odom
        was still None, failed permanently with no retry). Self-contained
        in the node so it behaves the same in sim and on the real car."""
        if self.enabled:
            self.auto_enable_timer.cancel()
            return
        success, message = self._try_enable()
        if success:
            self.get_logger().info('Auto-enable: ' + message)
            self.auto_enable_timer.cancel()
        else:
            self.warn_throttled('Auto-enable waiting: ' + message)

    @staticmethod
    def quaternion_to_yaw(q):
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    @staticmethod
    def clamp(value, minimum, maximum):
        return max(minimum, min(value, maximum))

    def lookup_vehicle_pose(self):
        transform = self.tf_buffer.lookup_transform(
            self.global_frame_id,
            self.base_frame_id,
            Time(),
            timeout=Duration(seconds=0.03),
        )
        translation = transform.transform.translation
        yaw = self.quaternion_to_yaw(transform.transform.rotation)
        return translation.x, translation.y, yaw

    def age_seconds(self, stamp):
        if stamp is None:
            return float('inf')
        return (self.get_clock().now() - stamp).nanoseconds * 1e-9

    def readiness_problem(self):
        if self.current_path is None or not self.current_path.poses:
            return 'no global path'
        if self.age_seconds(self.last_path_time) > self.path_timeout:
            return 'global path is stale'
        if self.current_odom is None:
            return 'no odometry'
        if self.age_seconds(self.last_odom_time) > self.odom_timeout:
            return 'odometry is stale'
        return None

    def candidate_indices(self, count):
        if self.nearest_index is None:
            return range(count)
        return [
            (self.nearest_index + offset) % count
            for offset in range(-self.search_back_points,
                                self.search_forward_points + 1)
        ]

    def nearest_path_state(self, x, y):
        poses = self.current_path.poses
        count = len(poses)
        nearest_idx = min(
            self.candidate_indices(count),
            key=lambda idx: math.hypot(
                poses[idx].pose.position.x - x,
                poses[idx].pose.position.y - y,
            ),
        )
        nearest_dist = math.hypot(
            poses[nearest_idx].pose.position.x - x,
            poses[nearest_idx].pose.position.y - y,
        )
        previous = poses[(nearest_idx - 1) % count].pose.position
        following = poses[(nearest_idx + 1) % count].pose.position
        path_heading = math.atan2(
            following.y - previous.y, following.x - previous.x)
        return nearest_idx, nearest_dist, path_heading

    def find_lookahead_point(self, x, y, yaw):
        poses = self.current_path.poses
        count = len(poses)
        if count < 2:
            return None

        nearest_idx, nearest_dist, path_heading = self.nearest_path_state(x, y)
        self.nearest_index = nearest_idx

        if nearest_dist > self.max_path_distance:
            return None
        heading_error = math.atan2(
            math.sin(path_heading - yaw), math.cos(path_heading - yaw))
        if abs(heading_error) > self.max_heading_error:
            return None

        travelled = 0.0
        previous = poses[nearest_idx].pose.position
        for offset in range(1, count + 1):
            idx = (nearest_idx + offset) % count
            point = poses[idx].pose.position
            travelled += math.hypot(point.x - previous.x, point.y - previous.y)
            previous = point

            if travelled < self.lookahead_distance:
                continue

            dx = point.x - x
            dy = point.y - y
            x_car = math.cos(yaw) * dx + math.sin(yaw) * dy
            y_car = -math.sin(yaw) * dx + math.cos(yaw) * dy
            if x_car > 0.0:
                return x_car, y_car, math.hypot(dx, dy), nearest_dist

        return None

    def compute_steering(self, x_car, y_car, lookahead_dist):
        if lookahead_dist < 1e-6:
            return 0.0
        curvature = 2.0 * y_car / (lookahead_dist ** 2)
        steering = math.atan(self.wheelbase * curvature)
        return self.clamp(
            steering, -self.max_steering_angle, self.max_steering_angle)

    def compute_speed(self, steering, profile_target=None):
        if profile_target is not None:
            base_speed = profile_target
        else:
            steer_ratio = abs(steering) / max(self.max_steering_angle, 1e-6)
            base_speed = self.target_speed * (
                1.0 - self.corner_slowdown_gain * steer_ratio)
        capped_speed = min(base_speed, self._active_avoidance_speed_cap())
        return self.clamp(capped_speed, self.min_speed, self.max_speed)

    def publish_drive(self, speed, steering):
        if self.drive_mode == 'sim':
            msg = AckermannDriveStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = self.base_frame_id
            msg.drive.speed = float(speed)
            msg.drive.steering_angle = float(steering)
            self.sim_drive_pub.publish(msg)
            return

        speed_msg = Float64()
        servo_msg = Float64()
        speed_msg.data = (
            self.speed_to_erpm_gain * speed + self.speed_to_erpm_offset)
        servo_msg.data = self.clamp(
            self.servo_center + self.servo_gain * steering,
            self.servo_min,
            self.servo_max,
        )
        self.real_speed_pub.publish(speed_msg)
        self.real_servo_pub.publish(servo_msg)

    def publish_stop(self):
        if self.drive_mode == 'sim':
            msg = AckermannDriveStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = self.base_frame_id
            msg.drive.speed = 0.0
            msg.drive.steering_angle = 0.0
            self.sim_drive_pub.publish(msg)
            return

        speed_msg = Float64()
        servo_msg = Float64()
        speed_msg.data = 0.0
        servo_msg.data = self.servo_center
        self.real_speed_pub.publish(speed_msg)
        self.real_servo_pub.publish(servo_msg)

    def warn_throttled(self, message):
        now = self.get_clock().now()
        if (message != self.last_status_message or
                self.last_status_time is None or
                (now - self.last_status_time).nanoseconds > 2_000_000_000):
            self.get_logger().warn(message)
            self.last_status_message = message
            self.last_status_time = now

    def control_loop(self):
        if not self.enabled:
            self.publish_stop()
            return

        problem = self.readiness_problem()
        if problem is not None:
            self.publish_stop()
            self.stall_condition_since = None
            self.warn_throttled('Safety stop: ' + problem)
            return

        try:
            x, y, yaw = self.lookup_vehicle_pose()
        except TransformException as error:
            self.publish_stop()
            self.stall_condition_since = None
            self.warn_throttled('Safety stop: TF unavailable: ' + str(error))
            return

        lookahead = self.find_lookahead_point(x, y, yaw)
        if lookahead is None:
            self.publish_stop()
            self.stall_condition_since = None
            self.warn_throttled(
                'Safety stop: no valid lookahead point or vehicle too far from path')
            return

        x_car, y_car, lookahead_dist, _ = lookahead
        steering = self.compute_steering(x_car, y_car, lookahead_dist)
        profile_target = self.profile_target_speed()
        commanded_speed = self.compute_speed(steering, profile_target)
        if self.stall_watchdog_enabled:
            self._check_stall(commanded_speed)
            if not self.enabled:
                return
        self.publish_drive(commanded_speed, steering)


def main(args=None):
    rclpy.init(args=args)
    node = PurePursuitNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.publish_stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
