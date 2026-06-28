import math

import numpy as np
import rclpy
from rclpy.node import Node
from scipy.ndimage import distance_transform_edt

from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid, Odometry
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point


class ParticleFilterNode(Node):
    """
    Monte Carlo Localization (MCL) — LiDAR Likelihood Field 모델.

    파이프라인:
      1. /map 수신 → likelihood field 사전 계산
      2. /ego_racecar/odom 으로 모션 모델 업데이트 (odometry diff)
      3. /scan 으로 센서 모델 → 파티클 가중치 업데이트
      4. Low-variance resampling
      5. 가중 평균으로 추정 포즈 → /localization/odom, /localization/pose
    """

    def __init__(self):
        super().__init__('particle_filter_node')

        # ── Parameters ──────────────────────────────────────────────
        self.declare_parameter('map_topic', '/map')
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('odom_topic', '/ego_racecar/odom')
        self.declare_parameter('output_odom_topic', '/localization/odom')
        self.declare_parameter('output_pose_topic', '/localization/pose')
        self.declare_parameter('particle_cloud_topic', '/localization/particles')

        self.declare_parameter('num_particles', 500)
        self.declare_parameter('downsample_beams', 20)
        self.declare_parameter('max_range', 10.0)

        # Motion noise (std dev)
        self.declare_parameter('alpha1', 0.1)   # rot에 의한 rot 노이즈
        self.declare_parameter('alpha2', 0.1)   # trans에 의한 rot 노이즈
        self.declare_parameter('alpha3', 0.05)  # trans에 의한 trans 노이즈
        self.declare_parameter('alpha4', 0.05)  # rot에 의한 trans 노이즈

        # Sensor model
        self.declare_parameter('z_hit', 0.8)
        self.declare_parameter('z_rand', 0.2)
        self.declare_parameter('sigma_hit', 0.15)

        # Initial pose (NaN = 맵 전체에 균등 분포)
        self.declare_parameter('initial_x', float('nan'))
        self.declare_parameter('initial_y', float('nan'))
        self.declare_parameter('initial_theta', 0.0)
        self.declare_parameter('initial_spread_xy', 0.5)
        self.declare_parameter('initial_spread_theta', 0.3)

        self.declare_parameter('frame_id', 'map')

        # ── Get params ──────────────────────────────────────────────
        self.map_topic = self.get_parameter('map_topic').value
        self.scan_topic = self.get_parameter('scan_topic').value
        self.odom_topic = self.get_parameter('odom_topic').value
        self.output_odom = self.get_parameter('output_odom_topic').value
        self.output_pose = self.get_parameter('output_pose_topic').value
        self.particle_topic = self.get_parameter('particle_cloud_topic').value
        self.frame_id = self.get_parameter('frame_id').value

        self.N = int(self.get_parameter('num_particles').value)
        self.downsample = int(self.get_parameter('downsample_beams').value)
        self.max_range = float(self.get_parameter('max_range').value)

        self.alpha1 = float(self.get_parameter('alpha1').value)
        self.alpha2 = float(self.get_parameter('alpha2').value)
        self.alpha3 = float(self.get_parameter('alpha3').value)
        self.alpha4 = float(self.get_parameter('alpha4').value)

        self.z_hit = float(self.get_parameter('z_hit').value)
        self.z_rand = float(self.get_parameter('z_rand').value)
        self.sigma_hit = float(self.get_parameter('sigma_hit').value)

        self.init_x = self.get_parameter('initial_x').value
        self.init_y = self.get_parameter('initial_y').value
        self.init_theta = float(self.get_parameter('initial_theta').value)
        self.spread_xy = float(self.get_parameter('initial_spread_xy').value)
        self.spread_theta = float(self.get_parameter('initial_spread_theta').value)

        # ── State ───────────────────────────────────────────────────
        self.map_info = None
        self.likelihood_field = None      # 2D numpy array
        self.particles = None             # (N, 3): x, y, theta
        self.weights = None               # (N,)
        self.prev_odom = None             # (x, y, theta)
        self.initialized = False

        # ── Pub / Sub ────────────────────────────────────────────────
        self.odom_pub = self.create_publisher(Odometry, self.output_odom, 10)
        self.pose_pub = self.create_publisher(PoseStamped, self.output_pose, 10)
        self.particle_pub = self.create_publisher(MarkerArray, self.particle_topic, 10)

        self.create_subscription(OccupancyGrid, self.map_topic, self._cb_map, 1)
        self.create_subscription(LaserScan, self.scan_topic, self._cb_scan, 10)
        self.create_subscription(Odometry, self.odom_topic, self._cb_odom, 10)
        self.create_subscription(
            PoseWithCovarianceStamped, '/initialpose', self._cb_initialpose, 10
        )

        self.get_logger().info('particle_filter_node started')
        self.get_logger().info(f'  num_particles    : {self.N}')
        self.get_logger().info(f'  downsample_beams : {self.downsample}')
        self.get_logger().info(f'  z_hit / z_rand   : {self.z_hit} / {self.z_rand}')
        self.get_logger().info(f'  sigma_hit        : {self.sigma_hit}')
        self.get_logger().info('맵 수신 대기 중...')

    # ------------------------------------------------------------------ #
    # Map → Likelihood field
    # ------------------------------------------------------------------ #
    def _cb_map(self, msg):
        self.map_info = msg.info
        w, h = msg.info.width, msg.info.height

        # OccupancyGrid → 장애물 마스크 (0=자유, 1=장애물)
        grid = np.array(msg.data, dtype=np.int8).reshape(h, w)
        obstacle = (grid > 50).astype(np.float32)

        # distance_transform_edt: 각 셀에서 가장 가까운 장애물까지 픽셀 거리
        dist_px = distance_transform_edt(1 - obstacle)
        dist_m = dist_px * msg.info.resolution

        # Likelihood field: 가우시안 z_hit + uniform z_rand
        lf = self.z_hit * np.exp(
            -0.5 * (dist_m / self.sigma_hit) ** 2
        ) + self.z_rand / self.max_range
        self.likelihood_field = lf.astype(np.float32)

        self.get_logger().info(
            f'맵 로드 완료: {w}×{h} 셀, resolution={msg.info.resolution:.3f} m/cell'
        )
        self._init_particles()

    def _init_particles(self):
        if math.isnan(self.init_x) or math.isnan(self.init_y):
            # 맵 전체 자유 공간에 균등 분포
            h, w = self.likelihood_field.shape
            res = self.map_info.resolution
            ox = self.map_info.origin.position.x
            oy = self.map_info.origin.position.y

            free = np.argwhere(self.likelihood_field > 0.3)
            if len(free) == 0:
                free = np.argwhere(np.ones((h, w)))

            idx = np.random.choice(len(free), self.N)
            rows, cols = free[idx, 0], free[idx, 1]
            xs = cols * res + ox
            ys = rows * res + oy
            thetas = np.random.uniform(-math.pi, math.pi, self.N)
        else:
            xs = np.random.normal(self.init_x, self.spread_xy, self.N)
            ys = np.random.normal(self.init_y, self.spread_xy, self.N)
            thetas = np.random.normal(self.init_theta, self.spread_theta, self.N)
            thetas = self._wrap(thetas)

        self.particles = np.stack([xs, ys, thetas], axis=1)
        self.weights = np.ones(self.N) / self.N
        self.initialized = True
        self.get_logger().info(
            f'파티클 초기화 완료 ({self.N}개)'
        )

    # ------------------------------------------------------------------ #
    # RViz initial pose override
    # ------------------------------------------------------------------ #
    def _cb_initialpose(self, msg):
        if self.particles is None:
            return
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        theta = math.atan2(2 * q.w * q.z, 1 - 2 * q.z ** 2)

        self.particles[:, 0] = np.random.normal(x, self.spread_xy, self.N)
        self.particles[:, 1] = np.random.normal(y, self.spread_xy, self.N)
        self.particles[:, 2] = self._wrap(
            np.random.normal(theta, self.spread_theta, self.N)
        )
        self.weights = np.ones(self.N) / self.N
        self.get_logger().info(
            f'/initialpose 수신 → 파티클 리셋: ({x:.3f}, {y:.3f}, {math.degrees(theta):.1f}°)'
        )

    # ------------------------------------------------------------------ #
    # Odometry → motion model
    # ------------------------------------------------------------------ #
    def _cb_odom(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        theta = math.atan2(2 * q.w * q.z, 1 - 2 * q.z ** 2)
        curr = np.array([x, y, theta])

        if self.prev_odom is None:
            self.prev_odom = curr
            return

        if not self.initialized:
            self.prev_odom = curr
            return

        self._motion_update(curr)
        self.prev_odom = curr

    def _motion_update(self, curr):
        px, py, pth = self.prev_odom
        cx, cy, cth = curr

        dx = cx - px
        dy = cy - py
        trans = math.hypot(dx, dy)
        rot1 = math.atan2(dy, dx) - pth if trans > 1e-4 else 0.0
        rot2 = cth - pth - rot1

        if trans < 1e-4 and abs(rot2) < 1e-4:
            return

        N = self.N
        a1, a2, a3, a4 = self.alpha1, self.alpha2, self.alpha3, self.alpha4

        r1_hat = rot1 - np.random.normal(
            0, math.sqrt(a1 * rot1**2 + a2 * trans**2), N
        )
        t_hat = trans - np.random.normal(
            0, math.sqrt(a3 * trans**2 + a4 * (rot1**2 + rot2**2)), N
        )
        r2_hat = rot2 - np.random.normal(
            0, math.sqrt(a1 * rot2**2 + a2 * trans**2), N
        )

        self.particles[:, 0] += t_hat * np.cos(self.particles[:, 2] + r1_hat)
        self.particles[:, 1] += t_hat * np.sin(self.particles[:, 2] + r1_hat)
        self.particles[:, 2] = self._wrap(self.particles[:, 2] + r1_hat + r2_hat)

    # ------------------------------------------------------------------ #
    # Scan → sensor model → resample → publish
    # ------------------------------------------------------------------ #
    def _cb_scan(self, msg):
        if not self.initialized or self.likelihood_field is None:
            return

        # 빔 다운샘플링
        ranges = np.array(msg.ranges)
        angles = np.linspace(msg.angle_min, msg.angle_max, len(ranges))
        step = max(1, len(ranges) // self.downsample)
        ranges = ranges[::step]
        angles = angles[::step]

        valid = (ranges > msg.range_min) & (ranges < self.max_range)
        ranges = ranges[valid]
        angles = angles[valid]

        if len(ranges) == 0:
            return

        self._sensor_update(ranges, angles)
        self._resample()
        self._publish()

    def _sensor_update(self, ranges, angles):
        res = self.map_info.resolution
        ox = self.map_info.origin.position.x
        oy = self.map_info.origin.position.y
        h, w = self.likelihood_field.shape

        log_weights = np.zeros(self.N)

        px = self.particles[:, 0]
        py = self.particles[:, 1]
        pth = self.particles[:, 2]

        for r, a in zip(ranges, angles):
            # 각 파티클에서 빔 끝점 계산
            beam_x = px + r * np.cos(pth + a)
            beam_y = py + r * np.sin(pth + a)

            # 맵 좌표로 변환
            col = ((beam_x - ox) / res).astype(int)
            row = ((beam_y - oy) / res).astype(int)

            in_map = (col >= 0) & (col < w) & (row >= 0) & (row < h)

            prob = np.full(self.N, self.z_rand / self.max_range)
            prob[in_map] = self.likelihood_field[row[in_map], col[in_map]]

            log_weights += np.log(np.maximum(prob, 1e-9))

        # 수치 안정화 후 정규화
        log_weights -= log_weights.max()
        self.weights = np.exp(log_weights)
        self.weights /= self.weights.sum()

    def _resample(self):
        # Low-variance resampling
        positions = (np.arange(self.N) + np.random.uniform(0, 1)) / self.N
        cumsum = np.cumsum(self.weights)
        indices = np.searchsorted(cumsum, positions)
        indices = np.clip(indices, 0, self.N - 1)
        self.particles = self.particles[indices]
        self.weights = np.ones(self.N) / self.N

    def _publish(self):
        # 가중 평균 추정 포즈
        x_est = float(np.mean(self.particles[:, 0]))
        y_est = float(np.mean(self.particles[:, 1]))
        # 각도 평균: circular mean
        sin_mean = float(np.mean(np.sin(self.particles[:, 2])))
        cos_mean = float(np.mean(np.cos(self.particles[:, 2])))
        th_est = math.atan2(sin_mean, cos_mean)

        now = self.get_clock().now().to_msg()
        qz = math.sin(th_est * 0.5)
        qw = math.cos(th_est * 0.5)

        odom = Odometry()
        odom.header.stamp = now
        odom.header.frame_id = self.frame_id
        odom.child_frame_id = 'base_link'
        odom.pose.pose.position.x = x_est
        odom.pose.pose.position.y = y_est
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        self.odom_pub.publish(odom)

        pose = PoseStamped()
        pose.header = odom.header
        pose.pose = odom.pose.pose
        self.pose_pub.publish(pose)

        self._publish_particles(now)

    def _publish_particles(self, stamp):
        ma = MarkerArray()
        # 화살표 마커로 파티클 시각화 (최대 200개)
        step = max(1, self.N // 200)
        for i, (x, y, th) in enumerate(self.particles[::step]):
            m = Marker()
            m.header.stamp = stamp
            m.header.frame_id = self.frame_id
            m.ns = 'particles'
            m.id = i
            m.type = Marker.ARROW
            m.action = Marker.ADD
            m.pose.position.x = float(x)
            m.pose.position.y = float(y)
            m.pose.position.z = 0.0
            m.pose.orientation.z = math.sin(float(th) * 0.5)
            m.pose.orientation.w = math.cos(float(th) * 0.5)
            m.scale.x = 0.15
            m.scale.y = 0.03
            m.scale.z = 0.03
            m.color.a = 0.6
            m.color.r = 0.2
            m.color.g = 0.8
            m.color.b = 0.2
            ma.markers.append(m)
        self.particle_pub.publish(ma)

    # ------------------------------------------------------------------ #
    # Util
    # ------------------------------------------------------------------ #
    @staticmethod
    def _wrap(angles):
        return (angles + math.pi) % (2 * math.pi) - math.pi


def main(args=None):
    rclpy.init(args=args)
    node = ParticleFilterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
