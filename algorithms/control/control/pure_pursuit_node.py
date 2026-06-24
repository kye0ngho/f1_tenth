import math

import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry, Path
from ackermann_msgs.msg import AckermannDriveStamped
from std_msgs.msg import Float64


class PurePursuitNode(Node):
    def __init__(self):
        super().__init__('pure_pursuit_node')

        # Mode
        self.declare_parameter('drive_mode', 'sim')  # sim | real

        # Topics
        self.declare_parameter('odom_topic', '/localization/odom')
        self.declare_parameter('path_topic', '/planning/path')
        self.declare_parameter('sim_drive_topic', '/drive')
        self.declare_parameter('real_speed_topic', '/commands/motor/speed')
        self.declare_parameter('real_servo_topic', '/commands/servo/position')

        # Pure Pursuit params
        self.declare_parameter('wheelbase', 0.33)
        self.declare_parameter('lookahead_distance', 1.0)
        self.declare_parameter('max_steering_angle', 0.4189)  # about 24 deg

        # Speed params
        self.declare_parameter('target_speed', 1.0)
        self.declare_parameter('min_speed', 0.4)
        self.declare_parameter('max_speed', 2.0)
        self.declare_parameter('corner_slowdown_gain', 0.5)

        # PID for speed tracking
        self.declare_parameter('kp', 1.0)
        self.declare_parameter('ki', 0.0)
        self.declare_parameter('kd', 0.05)

        # Real car conversion params
        self.declare_parameter('speed_to_erpm_gain', 3000.0)
        self.declare_parameter('speed_to_erpm_offset', 0.0)
        self.declare_parameter('servo_center', 0.5)
        self.declare_parameter('servo_gain', 1.0)
        self.declare_parameter('servo_min', 0.0)
        self.declare_parameter('servo_max', 1.0)

        # Loop
        self.declare_parameter('control_rate', 30.0)

        self.drive_mode = self.get_parameter('drive_mode').value

        self.odom_topic = self.get_parameter('odom_topic').value
        self.path_topic = self.get_parameter('path_topic').value
        self.sim_drive_topic = self.get_parameter('sim_drive_topic').value
        self.real_speed_topic = self.get_parameter('real_speed_topic').value
        self.real_servo_topic = self.get_parameter('real_servo_topic').value

        self.wheelbase = float(self.get_parameter('wheelbase').value)
        self.lookahead_distance = float(self.get_parameter('lookahead_distance').value)
        self.max_steering_angle = float(self.get_parameter('max_steering_angle').value)

        self.target_speed = float(self.get_parameter('target_speed').value)
        self.min_speed = float(self.get_parameter('min_speed').value)
        self.max_speed = float(self.get_parameter('max_speed').value)
        self.corner_slowdown_gain = float(self.get_parameter('corner_slowdown_gain').value)

        self.kp = float(self.get_parameter('kp').value)
        self.ki = float(self.get_parameter('ki').value)
        self.kd = float(self.get_parameter('kd').value)

        self.speed_to_erpm_gain = float(self.get_parameter('speed_to_erpm_gain').value)
        self.speed_to_erpm_offset = float(self.get_parameter('speed_to_erpm_offset').value)
        self.servo_center = float(self.get_parameter('servo_center').value)
        self.servo_gain = float(self.get_parameter('servo_gain').value)
        self.servo_min = float(self.get_parameter('servo_min').value)
        self.servo_max = float(self.get_parameter('servo_max').value)

        control_rate = float(self.get_parameter('control_rate').value)

        self.current_odom = None
        self.current_path = None

        self.integral_error = 0.0
        self.prev_error = 0.0
        self.prev_time = None

        self.odom_sub = self.create_subscription(
            Odometry,
            self.odom_topic,
            self.odom_callback,
            10
        )

        self.path_sub = self.create_subscription(
            Path,
            self.path_topic,
            self.path_callback,
            10
        )

        self.sim_drive_pub = self.create_publisher(
            AckermannDriveStamped,
            self.sim_drive_topic,
            10
        )

        self.real_speed_pub = self.create_publisher(
            Float64,
            self.real_speed_topic,
            10
        )

        self.real_servo_pub = self.create_publisher(
            Float64,
            self.real_servo_topic,
            10
        )

        self.timer = self.create_timer(
            1.0 / max(control_rate, 1.0),
            self.control_loop
        )

        self.get_logger().info('pure_pursuit_node started')
        self.get_logger().info(f'drive_mode       : {self.drive_mode}')
        self.get_logger().info(f'odom_topic       : {self.odom_topic}')
        self.get_logger().info(f'path_topic       : {self.path_topic}')
        self.get_logger().info(f'sim_drive_topic  : {self.sim_drive_topic}')
        self.get_logger().info(f'real_speed_topic : {self.real_speed_topic}')
        self.get_logger().info(f'real_servo_topic : {self.real_servo_topic}')

        if self.drive_mode not in ['sim', 'real']:
            raise RuntimeError("drive_mode must be 'sim' or 'real'")

    def odom_callback(self, msg):
        self.current_odom = msg

    def path_callback(self, msg):
        self.current_path = msg

    def quaternion_to_yaw(self, q):
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def clamp(self, value, min_value, max_value):
        return max(min_value, min(value, max_value))

    def find_lookahead_point(self, x, y, yaw):
        poses = self.current_path.poses

        if len(poses) == 0:
            return None

        # nearest waypoint index
        nearest_idx = 0
        nearest_dist = float('inf')

        for i, pose_stamped in enumerate(poses):
            px = pose_stamped.pose.position.x
            py = pose_stamped.pose.position.y
            dist = math.hypot(px - x, py - y)

            if dist < nearest_dist:
                nearest_dist = dist
                nearest_idx = i

        # search forward from nearest point
        n = len(poses)

        for offset in range(n):
            idx = (nearest_idx + offset) % n
            pose_stamped = poses[idx]

            px = pose_stamped.pose.position.x
            py = pose_stamped.pose.position.y

            dx = px - x
            dy = py - y

            # Transform map point to vehicle frame
            x_car = math.cos(yaw) * dx + math.sin(yaw) * dy
            y_car = -math.sin(yaw) * dx + math.cos(yaw) * dy

            dist = math.hypot(dx, dy)

            if x_car > 0.0 and dist >= self.lookahead_distance:
                return x_car, y_car, dist

        return None

    def compute_steering(self, x_car, y_car, lookahead_dist):
        if lookahead_dist < 1e-6:
            return 0.0

        curvature = 2.0 * y_car / (lookahead_dist ** 2)
        steering = math.atan(self.wheelbase * curvature)

        return self.clamp(
            steering,
            -self.max_steering_angle,
            self.max_steering_angle
        )

    def compute_pid_speed(self, steering):
        now = self.get_clock().now().nanoseconds * 1e-9

        current_speed = 0.0
        if self.current_odom is not None:
            current_speed = self.current_odom.twist.twist.linear.x

        steer_ratio = abs(steering) / max(self.max_steering_angle, 1e-6)
        desired_speed = self.target_speed * (1.0 - self.corner_slowdown_gain * steer_ratio)
        desired_speed = self.clamp(desired_speed, self.min_speed, self.max_speed)

        error = desired_speed - current_speed

        if self.prev_time is None:
            dt = 0.0
        else:
            dt = max(now - self.prev_time, 1e-6)

        if dt > 0.0:
            self.integral_error += error * dt
            derivative = (error - self.prev_error) / dt
        else:
            derivative = 0.0

        pid_output = (
            self.kp * error +
            self.ki * self.integral_error +
            self.kd * derivative
        )

        cmd_speed = current_speed + pid_output
        cmd_speed = self.clamp(cmd_speed, self.min_speed, self.max_speed)

        self.prev_error = error
        self.prev_time = now

        return cmd_speed

    def publish_drive(self, speed, steering):
        if self.drive_mode == 'sim':
            msg = AckermannDriveStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'base_link'
            msg.drive.speed = float(speed)
            msg.drive.steering_angle = float(steering)
            self.sim_drive_pub.publish(msg)

        else:
            speed_msg = Float64()
            servo_msg = Float64()

            speed_msg.data = self.speed_to_erpm_gain * speed + self.speed_to_erpm_offset

            servo = self.servo_center + self.servo_gain * steering
            servo_msg.data = self.clamp(servo, self.servo_min, self.servo_max)

            self.real_speed_pub.publish(speed_msg)
            self.real_servo_pub.publish(servo_msg)

    def publish_stop(self):
        if self.drive_mode == 'sim':
            msg = AckermannDriveStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.drive.speed = 0.0
            msg.drive.steering_angle = 0.0
            self.sim_drive_pub.publish(msg)
        else:
            speed_msg = Float64()
            servo_msg = Float64()
            speed_msg.data = 0.0
            servo_msg.data = self.servo_center
            self.real_speed_pub.publish(speed_msg)
            self.real_servo_pub.publish(servo_msg)

    def control_loop(self):
        if self.current_odom is None or self.current_path is None:
            return

        pose = self.current_odom.pose.pose
        x = pose.position.x
        y = pose.position.y
        yaw = self.quaternion_to_yaw(pose.orientation)

        lookahead = self.find_lookahead_point(x, y, yaw)

        if lookahead is None:
            self.publish_stop()
            return

        x_car, y_car, lookahead_dist = lookahead

        steering = self.compute_steering(x_car, y_car, lookahead_dist)
        speed = self.compute_pid_speed(steering)

        self.publish_drive(speed, steering)


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
