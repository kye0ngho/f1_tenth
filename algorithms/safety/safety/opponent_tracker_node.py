import math

import numpy as np

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Point, PoseArray, Pose
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32
from visualization_msgs.msg import Marker, MarkerArray


class OpponentTrackerNode(Node):
    """
    LiDAR 기반 동적 장애물(상대 차량) 추적기.

    알고리즘:
      1. scan에서 max_range 이하 포인트만 추출 → Cartesian 변환
      2. 인접 포인트 거리 기반 유클리드 클러스터링
      3. 클러스터 크기 필터 (너무 작은 노이즈 제거)
      4. 최근접 매칭으로 ID 유지 (프레임 간 추적)
      5. 속도 EMA 추정

    출력:
      /obstacles/markers       (MarkerArray — RViz 구체 + 텍스트)
      /obstacles/nearest_dist  (Float32 — 가장 가까운 장애물 거리 m)
      /obstacles/poses         (PoseArray — 장애물 중심 위치, laser frame)
    """

    def __init__(self):
        super().__init__('opponent_tracker_node')

        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('markers_topic', '/obstacles/markers')
        self.declare_parameter('nearest_topic', '/obstacles/nearest_dist')
        self.declare_parameter('poses_topic', '/obstacles/poses')
        self.declare_parameter('scan_frame_id', 'ego_racecar/laser')

        self.declare_parameter('max_range', 6.0)
        self.declare_parameter('cluster_dist', 0.3)       # 클러스터 분리 거리
        self.declare_parameter('min_cluster_points', 3)
        self.declare_parameter('max_cluster_points', 80)
        self.declare_parameter('max_cluster_size', 1.0)   # 클러스터 최대 직경 (m)
        self.declare_parameter('velocity_alpha', 0.3)     # EMA 계수

        scan_topic = self.get_parameter('scan_topic').value
        markers_topic = self.get_parameter('markers_topic').value
        nearest_topic = self.get_parameter('nearest_topic').value
        poses_topic = self.get_parameter('poses_topic').value
        self.frame_id = self.get_parameter('scan_frame_id').value

        self.max_range = float(self.get_parameter('max_range').value)
        self.cluster_dist = float(self.get_parameter('cluster_dist').value)
        self.min_pts = int(self.get_parameter('min_cluster_points').value)
        self.max_pts = int(self.get_parameter('max_cluster_points').value)
        self.max_size = float(self.get_parameter('max_cluster_size').value)
        self.alpha = float(self.get_parameter('velocity_alpha').value)

        self.tracks = {}       # id → {'cx':, 'cy':, 'vx':, 'vy':, 'age':}
        self.next_id = 0
        self.last_time = None

        self.create_subscription(LaserScan, scan_topic, self._cb_scan, 10)
        self.marker_pub = self.create_publisher(MarkerArray, markers_topic, 10)
        self.nearest_pub = self.create_publisher(Float32, nearest_topic, 10)
        self.poses_pub = self.create_publisher(PoseArray, poses_topic, 10)

        self.get_logger().info('opponent_tracker_node started')
        self.get_logger().info(f'  max_range    : {self.max_range} m')
        self.get_logger().info(f'  cluster_dist : {self.cluster_dist} m')

    def _cb_scan(self, msg):
        now = self.get_clock().now()
        dt = 0.0
        if self.last_time is not None:
            dt = (now - self.last_time).nanoseconds * 1e-9
        self.last_time = now

        # ── 스캔 → Cartesian 변환 ─────────────────────────────────
        angles = np.linspace(msg.angle_min, msg.angle_max, len(msg.ranges))
        ranges = np.array(msg.ranges, dtype=np.float32)

        valid = np.isfinite(ranges) & (ranges > 0.1) & (ranges < self.max_range)
        pts = np.column_stack([
            ranges[valid] * np.cos(angles[valid]),
            ranges[valid] * np.sin(angles[valid]),
        ])

        if len(pts) == 0:
            self._publish_empty(msg.header.stamp)
            return

        # ── 클러스터링 (연속 포인트 거리 기반) ─────────────────────
        clusters = self._cluster(pts)

        # ── 트래킹 업데이트 ────────────────────────────────────────
        centroids = [(np.mean(c[:, 0]), np.mean(c[:, 1])) for c in clusters]
        self._update_tracks(centroids, dt)

        # ── 퍼블리시 ──────────────────────────────────────────────
        self._publish(msg.header.stamp, centroids)

    def _cluster(self, pts):
        if len(pts) == 0:
            return []
        clusters = []
        current = [pts[0]]
        for i in range(1, len(pts)):
            if np.linalg.norm(pts[i] - pts[i - 1]) < self.cluster_dist:
                current.append(pts[i])
            else:
                arr = np.array(current)
                if self.min_pts <= len(arr) <= self.max_pts:
                    size = np.linalg.norm(arr.max(axis=0) - arr.min(axis=0))
                    if size < self.max_size:
                        clusters.append(arr)
                current = [pts[i]]
        if len(current) >= self.min_pts:
            arr = np.array(current)
            size = np.linalg.norm(arr.max(axis=0) - arr.min(axis=0))
            if size < self.max_size:
                clusters.append(arr)
        return clusters

    def _update_tracks(self, centroids, dt):
        if not centroids:
            for tid in list(self.tracks):
                self.tracks[tid]['age'] += 1
                if self.tracks[tid]['age'] > 5:
                    del self.tracks[tid]
            return

        # 최근접 매칭
        used_tracks = set()
        used_cents = set()

        for i, (cx, cy) in enumerate(centroids):
            best_id = None
            best_d = 1.0   # 최대 매칭 거리
            for tid, tr in self.tracks.items():
                if tid in used_tracks:
                    continue
                d = math.hypot(cx - tr['cx'], cy - tr['cy'])
                if d < best_d:
                    best_d = d
                    best_id = tid

            if best_id is not None:
                tr = self.tracks[best_id]
                if dt > 0:
                    vx_new = (cx - tr['cx']) / dt
                    vy_new = (cy - tr['cy']) / dt
                    tr['vx'] = self.alpha * vx_new + (1 - self.alpha) * tr['vx']
                    tr['vy'] = self.alpha * vy_new + (1 - self.alpha) * tr['vy']
                tr['cx'] = cx
                tr['cy'] = cy
                tr['age'] = 0
                used_tracks.add(best_id)
                used_cents.add(i)
            else:
                new_id = self.next_id
                self.next_id += 1
                self.tracks[new_id] = {'cx': cx, 'cy': cy, 'vx': 0.0, 'vy': 0.0, 'age': 0}
                used_cents.add(i)

        # 매칭 안 된 트랙은 age 증가 후 삭제
        for tid in list(self.tracks):
            if tid not in used_tracks:
                self.tracks[tid]['age'] += 1
                if self.tracks[tid]['age'] > 5:
                    del self.tracks[tid]

    def _publish(self, stamp, centroids):
        now_msg = self.get_clock().now().to_msg()
        ma = MarkerArray()
        pa = PoseArray()
        pa.header.stamp = stamp
        pa.header.frame_id = self.frame_id

        nearest = float('inf')
        mid = 0

        for tid, tr in self.tracks.items():
            cx, cy = tr['cx'], tr['cy']
            dist = math.hypot(cx, cy)
            nearest = min(nearest, dist)
            speed = math.hypot(tr['vx'], tr['vy'])

            # 구체 마커
            m = Marker()
            m.header.stamp = now_msg
            m.header.frame_id = self.frame_id
            m.ns = 'obstacles'
            m.id = mid; mid += 1
            m.type = Marker.SPHERE
            m.action = Marker.ADD
            m.pose.position.x = cx
            m.pose.position.y = cy
            m.pose.orientation.w = 1.0
            m.scale.x = m.scale.y = m.scale.z = 0.25
            m.color.a = 0.8
            m.color.r = 1.0
            m.color.g = 0.4
            m.color.b = 0.0
            m.lifetime.sec = 0
            m.lifetime.nanosec = int(0.2e9)
            ma.markers.append(m)

            # 텍스트 마커
            t = Marker()
            t.header = m.header
            t.ns = 'obstacle_text'
            t.id = mid; mid += 1
            t.type = Marker.TEXT_VIEW_FACING
            t.action = Marker.ADD
            t.pose.position.x = cx
            t.pose.position.y = cy
            t.pose.position.z = 0.35
            t.pose.orientation.w = 1.0
            t.scale.z = 0.15
            t.color.a = 1.0
            t.color.r = t.color.g = t.color.b = 1.0
            t.text = f'#{tid} {dist:.1f}m {speed:.1f}m/s'
            t.lifetime.sec = 0
            t.lifetime.nanosec = int(0.2e9)
            ma.markers.append(t)

            p = Pose()
            p.position.x = cx
            p.position.y = cy
            p.orientation.w = 1.0
            pa.poses.append(p)

        self.marker_pub.publish(ma)
        self.poses_pub.publish(pa)

        n_msg = Float32()
        n_msg.data = float(nearest) if nearest != float('inf') else -1.0
        self.nearest_pub.publish(n_msg)

    def _publish_empty(self, stamp):
        self.marker_pub.publish(MarkerArray())
        self.poses_pub.publish(PoseArray())
        n = Float32(); n.data = -1.0
        self.nearest_pub.publish(n)


def main(args=None):
    rclpy.init(args=args)
    node = OpponentTrackerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
