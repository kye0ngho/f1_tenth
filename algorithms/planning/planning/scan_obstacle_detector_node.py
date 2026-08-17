import math

import numpy as np
import rclpy
from geometry_msgs.msg import Pose, PoseArray
from nav_msgs.msg import Path
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker, MarkerArray
import tf2_ros

from planning.path_utils import ClosedPath, quaternion_to_yaw


def _cluster_scan(ranges, angle_min, angle_increment, range_min, range_max,
                  jump_threshold, min_points):
    ranges = np.asarray(ranges, dtype=float)
    valid = np.isfinite(ranges) & (ranges >= range_min) & (ranges <= range_max)
    if not np.any(valid):
        return []

    angles = angle_min + angle_increment * np.arange(len(ranges))
    xs = ranges * np.cos(angles)
    ys = ranges * np.sin(angles)

    clusters = []
    index = 0
    while index < len(ranges):
        if not valid[index]:
            index += 1
            continue
        start = index
        index += 1
        while (index < len(ranges) and valid[index]
               and abs(ranges[index] - ranges[index - 1]) <= jump_threshold):
            index += 1
        if index - start >= min_points:
            width = float(math.hypot(xs[index - 1] - xs[start],
                                     ys[index - 1] - ys[start]))
            clusters.append((float(np.mean(xs[start:index])),
                             float(np.mean(ys[start:index])),
                             width, index - start))
    return clusters


class ScanObstacleDetectorNode(Node):
    def __init__(self):
        super().__init__('scan_obstacle_detector_node')

        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('path_topic', '/planning/global_path')
        self.declare_parameter('obstacle_topic', '/planning/detected_obstacles')
        self.declare_parameter('marker_topic', '/planning/detected_obstacle_markers')
        self.declare_parameter('global_frame_id', 'map')
        self.declare_parameter('obstacle_cluster_jump_m', 0.15)
        self.declare_parameter('obstacle_min_points', 3)
        self.declare_parameter('obstacle_max_width_m', 0.60)
        self.declare_parameter('obstacle_max_range_m', 4.00)
        self.declare_parameter('obstacle_corridor_m', 0.55)

        self.global_frame_id = self.get_parameter('global_frame_id').value
        self.cluster_jump = float(self.get_parameter('obstacle_cluster_jump_m').value)
        self.min_points = int(self.get_parameter('obstacle_min_points').value)
        self.max_width = float(self.get_parameter('obstacle_max_width_m').value)
        self.max_range = float(self.get_parameter('obstacle_max_range_m').value)
        self.corridor = float(self.get_parameter('obstacle_corridor_m').value)

        self.path = ClosedPath()
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.create_subscription(
            LaserScan, self.get_parameter('scan_topic').value, self.scan_callback, 10)
        self.create_subscription(
            Path, self.get_parameter('path_topic').value, self.path_callback, 10)
        self.obstacle_pub = self.create_publisher(
            PoseArray, self.get_parameter('obstacle_topic').value, 10)
        self.marker_pub = self.create_publisher(
            MarkerArray, self.get_parameter('marker_topic').value, 10)

        self.get_logger().info(
            'scan_obstacle_detector_node: %s -> %s' % (
                self.get_parameter('scan_topic').value,
                self.get_parameter('obstacle_topic').value))

    def path_callback(self, msg):
        self.path.update_from_path(msg)

    def scan_callback(self, msg):
        output = PoseArray()
        output.header.stamp = self.get_clock().now().to_msg()
        output.header.frame_id = self.global_frame_id

        if not self.path.ready:
            self.obstacle_pub.publish(output)
            self._publish_markers(output)
            return

        try:
            tf_msg = self.tf_buffer.lookup_transform(
                self.global_frame_id, msg.header.frame_id, rclpy.time.Time())
        except Exception as exc:
            self.get_logger().warn(
                'obstacle detector TF lookup failed: %s' % exc,
                throttle_duration_sec=1.0)
            self.obstacle_pub.publish(output)
            self._publish_markers(output)
            return

        t = tf_msg.transform.translation
        yaw = quaternion_to_yaw(tf_msg.transform.rotation)
        cos_yaw = math.cos(yaw)
        sin_yaw = math.sin(yaw)
        clusters = _cluster_scan(
            msg.ranges, msg.angle_min, msg.angle_increment,
            max(float(msg.range_min), 0.05),
            min(float(msg.range_max), self.max_range),
            self.cluster_jump, self.min_points)

        accepted = []
        for local_x, local_y, width, _count in clusters:
            if width > self.max_width:
                continue
            map_x = t.x + local_x * cos_yaw - local_y * sin_yaw
            map_y = t.y + local_x * sin_yaw + local_y * cos_yaw
            _s, lateral, distance, _path_yaw = self.path.nearest(map_x, map_y)
            if distance > self.corridor:
                continue
            accepted.append((map_x, map_y, abs(lateral)))

        accepted.sort(key=lambda item: item[2])
        for map_x, map_y, _lateral in accepted[:6]:
            pose = Pose()
            pose.position.x = float(map_x)
            pose.position.y = float(map_y)
            pose.orientation.w = 1.0
            output.poses.append(pose)

        self.obstacle_pub.publish(output)
        self._publish_markers(output)

    def _publish_markers(self, obstacles):
        markers = MarkerArray()
        clear = Marker()
        clear.header = obstacles.header
        clear.ns = 'detected_obstacles'
        clear.action = Marker.DELETEALL
        markers.markers.append(clear)

        for index, pose in enumerate(obstacles.poses):
            marker = Marker()
            marker.header = obstacles.header
            marker.ns = 'detected_obstacles'
            marker.id = index + 1
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose = pose
            marker.pose.position.z = 0.12
            marker.scale.x = 0.24
            marker.scale.y = 0.24
            marker.scale.z = 0.24
            marker.color.r = 1.0
            marker.color.g = 0.05
            marker.color.b = 0.05
            marker.color.a = 0.95
            markers.markers.append(marker)
        self.marker_pub.publish(markers)


def main(args=None):
    rclpy.init(args=args)
    node = ScanObstacleDetectorNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
