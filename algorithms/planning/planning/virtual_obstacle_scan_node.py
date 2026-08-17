import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker, MarkerArray
import tf2_ros


def _yaw_from_quaternion(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def _parse_obstacles(raw):
    obstacles = []
    if not raw:
        return obstacles

    for item in str(raw).split(';'):
        fields = [field.strip() for field in item.split(',') if field.strip()]
        if len(fields) != 3:
            continue
        try:
            x, y, radius = (float(fields[0]), float(fields[1]),
                            float(fields[2]))
        except ValueError:
            continue
        if radius > 0.0:
            obstacles.append((x, y, radius))
    return obstacles


class VirtualObstacleScanNode(Node):
    """Inject map-frame circular obstacles into a LaserScan."""

    def __init__(self):
        super().__init__('virtual_obstacle_scan_node')

        self.declare_parameter('input_scan_topic', '/scan')
        self.declare_parameter('output_scan_topic', '/scan_with_obstacle')
        self.declare_parameter('marker_topic', '/planning/virtual_obstacles')
        self.declare_parameter('global_frame_id', 'map')
        self.declare_parameter('obstacles', '6.74,0.41,0.18')

        self.global_frame_id = self.get_parameter('global_frame_id').value
        self.obstacles = _parse_obstacles(
            self.get_parameter('obstacles').value)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.scan_pub = self.create_publisher(
            LaserScan, self.get_parameter('output_scan_topic').value, 10)
        self.marker_pub = self.create_publisher(
            MarkerArray, self.get_parameter('marker_topic').value, 1)
        self.scan_sub = self.create_subscription(
            LaserScan,
            self.get_parameter('input_scan_topic').value,
            self.scan_callback,
            10)

        self.get_logger().info(
            'Injecting %d virtual obstacle(s) into %s' % (
                len(self.obstacles),
                self.get_parameter('output_scan_topic').value))

    def scan_callback(self, msg):
        output = LaserScan()
        output.header = msg.header
        output.angle_min = msg.angle_min
        output.angle_max = msg.angle_max
        output.angle_increment = msg.angle_increment
        output.time_increment = msg.time_increment
        output.scan_time = msg.scan_time
        output.range_min = msg.range_min
        output.range_max = msg.range_max
        output.ranges = list(msg.ranges)
        output.intensities = list(msg.intensities)

        if self.obstacles:
            try:
                transform = self.tf_buffer.lookup_transform(
                    self.global_frame_id,
                    msg.header.frame_id,
                    rclpy.time.Time())
                self._inject_obstacles(output, transform)
            except Exception as exc:
                self.get_logger().warn(
                    'Cannot inject virtual obstacles: %s' % exc,
                    throttle_duration_sec=1.0)

        self.scan_pub.publish(output)
        self._publish_markers()

    def _inject_obstacles(self, scan, transform):
        t = transform.transform.translation
        yaw = _yaw_from_quaternion(transform.transform.rotation)
        cos_yaw = math.cos(yaw)
        sin_yaw = math.sin(yaw)

        angles = [
            scan.angle_min + index * scan.angle_increment
            for index in range(len(scan.ranges))
        ]
        beam_cos = [math.cos(angle) for angle in angles]
        beam_sin = [math.sin(angle) for angle in angles]

        for obs_x, obs_y, radius in self.obstacles:
            dx = obs_x - t.x
            dy = obs_y - t.y
            cx = cos_yaw * dx + sin_yaw * dy
            cy = -sin_yaw * dx + cos_yaw * dy
            center_sq = cx * cx + cy * cy

            if center_sq <= 1.0e-9 or cx < -radius:
                continue

            radius_sq = radius * radius
            for index, current_range in enumerate(scan.ranges):
                projection = cx * beam_cos[index] + cy * beam_sin[index]
                if projection <= 0.0:
                    continue
                perp_sq = center_sq - projection * projection
                if perp_sq > radius_sq:
                    continue

                hit_range = projection - math.sqrt(radius_sq - perp_sq)
                if hit_range < scan.range_min or hit_range > scan.range_max:
                    continue

                if (not math.isfinite(current_range)
                        or current_range < scan.range_min
                        or hit_range < current_range):
                    scan.ranges[index] = hit_range

    def _publish_markers(self):
        markers = MarkerArray()
        for index, (x, y, radius) in enumerate(self.obstacles):
            marker = Marker()
            marker.header.frame_id = self.global_frame_id
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = 'virtual_obstacles'
            marker.id = index
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position.x = x
            marker.pose.position.y = y
            marker.pose.position.z = radius
            marker.pose.orientation.w = 1.0
            marker.scale.x = 2.0 * radius
            marker.scale.y = 2.0 * radius
            marker.scale.z = 2.0 * radius
            marker.color.r = 0.95
            marker.color.g = 0.15
            marker.color.b = 0.10
            marker.color.a = 0.85
            markers.markers.append(marker)
        self.marker_pub.publish(markers)


def main(args=None):
    rclpy.init(args=args)
    node = VirtualObstacleScanNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
