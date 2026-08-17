import csv
import math

import numpy as np
import rclpy
from rclpy.node import Node

from std_msgs.msg import Float32MultiArray
from visualization_msgs.msg import Marker, MarkerArray


def clamp(value, lo, hi):
    return max(lo, min(hi, value))


def _column(header, names):
    return next((header.index(name) for name in names if name in header), None)


class SpeedProfileNode(Node):
    """Publish a closed-loop velocity profile as [s0, v0, s1, v1, ...].

    This is intentionally message-light: nav_msgs/Path keeps carrying geometry,
    while this topic carries the UNICORN-style per-waypoint velocity plan.
    """

    def __init__(self):
        super().__init__('speed_profile_node')

        self.declare_parameter('waypoint_csv', '/sim_ws/src/planning/waypoints/waypoints.csv')
        self.declare_parameter('speed_profile_topic', '/planning/speed_profile')
        self.declare_parameter('marker_topic', '/planning/speed_profile_markers')
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('publish_rate', 2.0)

        self.declare_parameter('max_speed', 4.0)
        self.declare_parameter('min_speed', 0.8)
        self.declare_parameter('lateral_accel_limit', 5.3)
        self.declare_parameter('accel_limit', 7.0)
        self.declare_parameter('decel_limit', 8.0)
        self.declare_parameter('corner_slowdown_gain', 0.10)
        self.declare_parameter('csv_speed_scale', 1.0)
        self.declare_parameter('use_csv_speed_limit', False)
        self.declare_parameter('smoothing_passes', 2)
        self.declare_parameter('closed_loop_passes', 4)

        self.waypoint_csv = self.get_parameter('waypoint_csv').value
        self.speed_profile_topic = self.get_parameter('speed_profile_topic').value
        self.marker_topic = self.get_parameter('marker_topic').value
        self.frame_id = self.get_parameter('frame_id').value
        self.publish_rate = float(self.get_parameter('publish_rate').value)

        self.max_speed = float(self.get_parameter('max_speed').value)
        self.min_speed = float(self.get_parameter('min_speed').value)
        self.lateral_accel_limit = float(
            self.get_parameter('lateral_accel_limit').value)
        self.accel_limit = float(self.get_parameter('accel_limit').value)
        self.decel_limit = float(self.get_parameter('decel_limit').value)
        self.corner_slowdown_gain = float(
            self.get_parameter('corner_slowdown_gain').value)
        self.csv_speed_scale = float(self.get_parameter('csv_speed_scale').value)
        self.use_csv_speed_limit = bool(
            self.get_parameter('use_csv_speed_limit').value)
        self.smoothing_passes = int(self.get_parameter('smoothing_passes').value)
        self.closed_loop_passes = int(
            self.get_parameter('closed_loop_passes').value)

        self.s_values, self.points, self.curvature, self.csv_speed = (
            self.load_waypoints(self.waypoint_csv))
        self.profile = self.build_velocity_profile()

        self.profile_pub = self.create_publisher(
            Float32MultiArray, self.speed_profile_topic, 10)
        self.marker_pub = self.create_publisher(
            MarkerArray, self.marker_topic, 10)

        self.timer = self.create_timer(
            1.0 / max(self.publish_rate, 0.1), self.publish)

        self.get_logger().info(
            'speed_profile_node started: %d points, v %.2f..%.2f m/s'
            % (len(self.profile), float(np.min(self.profile)),
               float(np.max(self.profile))))

    def load_waypoints(self, csv_path):
        with open(csv_path, 'r') as handle:
            rows = [row for row in csv.reader(handle)
                    if row and not row[0].strip().startswith('#')]
        if not rows:
            raise RuntimeError('Waypoint CSV is empty')

        header = [cell.strip().lower() for cell in rows[0]]
        x_idx = _column(header, ['x', 'x_m'])
        y_idx = _column(header, ['y', 'y_m'])
        s_idx = _column(header, ['s', 's_m'])
        kappa_idx = _column(header, ['kappa', 'kappa_radpm', 'curvature'])
        speed_idx = _column(header, ['speed', 'velocity', 'vx', 'vx_mps'])

        if x_idx is None or y_idx is None:
            x_idx = 0
            y_idx = 1
            s_idx = None
            kappa_idx = None
            speed_idx = 2 if len(rows[0]) > 2 else None
            data_rows = rows
        else:
            data_rows = rows[1:]

        points = []
        csv_speed = []
        s_values = []
        kappa_values = []
        for row in data_rows:
            if len(row) <= max(x_idx, y_idx):
                continue
            points.append([float(row[x_idx]), float(row[y_idx])])
            if s_idx is not None and len(row) > s_idx:
                s_values.append(float(row[s_idx]))
            if kappa_idx is not None and len(row) > kappa_idx:
                kappa_values.append(float(row[kappa_idx]))
            if speed_idx is not None and len(row) > speed_idx:
                csv_speed.append(float(row[speed_idx]))
            else:
                csv_speed.append(self.max_speed)

        points = np.asarray(points, dtype=float)
        if len(points) > 2 and np.linalg.norm(points[0] - points[-1]) < 1e-4:
            points = points[:-1]
            csv_speed = csv_speed[:-1]
            if len(s_values) > len(points):
                s_values = s_values[:-1]
            if len(kappa_values) > len(points):
                kappa_values = kappa_values[:-1]
        if len(points) < 4:
            raise RuntimeError('Waypoint CSV must contain at least 4 points')

        s_array = self._arc_lengths(points)
        if len(s_values) == len(points):
            supplied_s = np.asarray(s_values, dtype=float)
            if np.all(np.diff(supplied_s) > 0.0):
                s_array = supplied_s - supplied_s[0]

        if len(kappa_values) == len(points):
            curvature = np.asarray(kappa_values, dtype=float)
        else:
            curvature = self._curvature(points)

        return s_array, points, curvature, np.asarray(csv_speed, dtype=float)

    @staticmethod
    def _arc_lengths(points):
        segments = np.roll(points, -1, axis=0) - points
        lengths = np.linalg.norm(segments, axis=1)
        return np.concatenate(([0.0], np.cumsum(lengths[:-1])))

    @staticmethod
    def _curvature(points):
        segments = np.roll(points, -1, axis=0) - points
        lengths = np.linalg.norm(segments, axis=1)
        yaw = np.unwrap(np.arctan2(segments[:, 1], segments[:, 0]))
        previous_yaw = np.roll(yaw, 1)
        next_yaw = np.roll(yaw, -1)
        span = np.roll(lengths, 1) + lengths
        return (next_yaw - previous_yaw) / np.maximum(span, 1e-6)

    def build_velocity_profile(self):
        curvature = np.abs(self.curvature)
        allowed = np.full(len(curvature), self.max_speed, dtype=float)

        if self.use_csv_speed_limit:
            allowed = np.minimum(allowed, self.csv_speed_scale * self.csv_speed)

        if self.corner_slowdown_gain > 0.0:
            allowed = np.minimum(
                allowed,
                self.max_speed / (1.0 + self.corner_slowdown_gain * curvature))

        if self.lateral_accel_limit > 0.0:
            curve_speed = np.sqrt(np.divide(
                self.lateral_accel_limit, curvature,
                out=np.full_like(curvature, self.max_speed),
                where=curvature > 1.0e-5))
            allowed = np.minimum(allowed, curve_speed)

        profile = np.clip(allowed, self.min_speed, self.max_speed)
        segment_lengths = np.linalg.norm(
            np.roll(self.points, -1, axis=0) - self.points, axis=1)

        for _ in range(max(self.closed_loop_passes, 1)):
            for i in range(1, len(profile)):
                ds = max(float(segment_lengths[i - 1]), 1.0e-4)
                profile[i] = min(
                    profile[i],
                    math.sqrt(profile[i - 1] ** 2 + 2.0 * self.accel_limit * ds))
            for i in range(len(profile) - 2, -1, -1):
                ds = max(float(segment_lengths[i]), 1.0e-4)
                profile[i] = min(
                    profile[i],
                    math.sqrt(profile[i + 1] ** 2 + 2.0 * self.decel_limit * ds))

        for _ in range(max(self.smoothing_passes, 0)):
            profile = 0.25 * np.roll(profile, 1) + 0.5 * profile + 0.25 * np.roll(profile, -1)
            profile = np.minimum(profile, allowed)
            profile = np.clip(profile, self.min_speed, self.max_speed)

        return profile

    def build_profile_msg(self):
        msg = Float32MultiArray()
        data = []
        for s_value, speed in zip(self.s_values, self.profile):
            data.extend([float(s_value), float(speed)])
        msg.data = data
        return msg

    def build_marker_msg(self):
        markers = MarkerArray()
        max_speed = max(float(np.max(self.profile)), self.min_speed + 1.0e-3)
        for i, ((x, y), speed) in enumerate(zip(self.points, self.profile)):
            marker = Marker()
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.header.frame_id = self.frame_id
            marker.ns = 'speed_profile'
            marker.id = i
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position.x = float(x)
            marker.pose.position.y = float(y)
            marker.pose.position.z = 0.08
            marker.pose.orientation.w = 1.0
            scale = 0.045 + 0.035 * speed / max_speed
            marker.scale.x = scale
            marker.scale.y = scale
            marker.scale.z = scale
            ratio = clamp(speed / max_speed, 0.0, 1.0)
            marker.color.a = 0.9
            marker.color.r = 1.0 - ratio
            marker.color.g = ratio
            marker.color.b = 0.15
            markers.markers.append(marker)
        return markers

    def publish(self):
        self.profile_pub.publish(self.build_profile_msg())
        self.marker_pub.publish(self.build_marker_msg())


def main(args=None):
    rclpy.init(args=args)
    node = SpeedProfileNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
