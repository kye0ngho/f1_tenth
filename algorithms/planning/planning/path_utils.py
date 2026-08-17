import math

import numpy as np
from geometry_msgs.msg import PoseStamped


def yaw_to_quaternion(yaw):
    qz = math.sin(0.5 * yaw)
    qw = math.cos(0.5 * yaw)
    return qz, qw


def quaternion_to_yaw(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def wrap_delta(a, b, length):
    return (a - b + 0.5 * length) % length - 0.5 * length


class ClosedPath:
    def __init__(self):
        self.points = None
        self.yaw = None
        self.segment_lengths = None
        self.cumulative = None
        self.length = None
        self.frame_id = 'map'

    @property
    def ready(self):
        return self.points is not None and self.length is not None

    def update_from_path(self, msg):
        points = np.asarray([
            [pose.pose.position.x, pose.pose.position.y]
            for pose in msg.poses
        ], dtype=float)
        if len(points) > 2 and np.linalg.norm(points[0] - points[-1]) < 1.0e-4:
            points = points[:-1]
        if len(points) < 4:
            return False

        next_points = np.roll(points, -1, axis=0)
        segments = next_points - points
        lengths = np.linalg.norm(segments, axis=1)
        if np.any(lengths < 1.0e-5):
            return False

        yaw = np.unwrap(np.arctan2(segments[:, 1], segments[:, 0]))
        self.points = points
        self.yaw = yaw
        self.segment_lengths = lengths
        self.cumulative = np.concatenate(([0.0], np.cumsum(lengths)))
        self.length = float(self.cumulative[-1])
        self.frame_id = msg.header.frame_id or self.frame_id
        return True

    def nearest(self, x, y):
        position = np.array([x, y], dtype=float)
        best = None
        count = len(self.points)
        for index in range(count):
            start = self.points[index]
            segment = self.points[(index + 1) % count] - start
            denom = float(np.dot(segment, segment))
            if denom <= 1.0e-9:
                continue
            fraction = float(np.dot(position - start, segment) / denom)
            fraction = max(0.0, min(1.0, fraction))
            projection = start + fraction * segment
            diff = position - projection
            distance = float(np.linalg.norm(diff))
            if best is None or distance < best[0]:
                best = (distance, index, fraction, projection)

        distance, index, fraction, projection = best
        s_value = self.cumulative[index] + fraction * self.segment_lengths[index]
        yaw = float(self.yaw[index])
        normal = np.array([-math.sin(yaw), math.cos(yaw)])
        signed_lateral = float(normal @ (position - projection))
        return float(s_value), signed_lateral, distance, yaw


def build_path_msg(node, points, frame_id):
    from nav_msgs.msg import Path
    path_msg = Path()
    path_msg.header.stamp = node.get_clock().now().to_msg()
    path_msg.header.frame_id = frame_id

    count = len(points)
    for index, point in enumerate(points):
        next_point = points[(index + 1) % count]
        yaw = math.atan2(next_point[1] - point[1], next_point[0] - point[0])
        qz, qw = yaw_to_quaternion(yaw)
        pose = PoseStamped()
        pose.header = path_msg.header
        pose.pose.position.x = float(point[0])
        pose.pose.position.y = float(point[1])
        pose.pose.position.z = 0.0
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw
        path_msg.poses.append(pose)
    return path_msg
