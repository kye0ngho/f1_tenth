import csv
import math
import os

import numpy as np

import rclpy
from rclpy.node import Node

from std_srvs.srv import Trigger, SetBool
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point


class RaceLineOptimizerNode(Node):
    """
    최소 곡률 레이싱 라인 최적화.

    알고리즘: Laplacian Smoothing + 트랙 폭 제약
      각 웨이포인트를 이웃 웨이포인트의 중점 방향으로 이동시켜
      전체 경로 곡률을 최소화한다. 이동 허용 범위는
      경로 법선 방향으로 ±half_track_width.

    반복 수렴 후 새로운 yaw와 속도 프로파일을 계산하여
    output_csv에 저장.

    서비스:
      ~/optimize  → 최적화 실행 (Trigger)
      ~/toggle    → 실시간 반복 최적화 ON/OFF (SetBool)
    출력:
      /race_line/markers (MarkerArray — RViz)
    """

    def __init__(self):
        super().__init__('race_line_optimizer_node')

        self.declare_parameter('input_csv',
                               '/sim_ws/src/planning/waypoints/waypoints.csv')
        self.declare_parameter('output_csv',
                               '/sim_ws/src/planning/waypoints/race_line.csv')
        self.declare_parameter('marker_topic', '/race_line/markers')
        self.declare_parameter('frame_id', 'map')

        self.declare_parameter('half_track_width', 0.8)   # 트랙 반폭 (m)
        self.declare_parameter('max_iterations', 500)
        self.declare_parameter('learning_rate', 0.08)
        self.declare_parameter('convergence_thr', 1e-5)

        self.declare_parameter('min_speed', 0.4)
        self.declare_parameter('max_speed', 2.5)
        self.declare_parameter('speed_curvature_gain', 3.5)
        self.declare_parameter('auto_optimize_on_start', True)

        self.input_csv = self.get_parameter('input_csv').value
        self.output_csv = self.get_parameter('output_csv').value
        marker_topic = self.get_parameter('marker_topic').value
        self.frame_id = self.get_parameter('frame_id').value

        self.half_width = float(self.get_parameter('half_track_width').value)
        self.max_iter = int(self.get_parameter('max_iterations').value)
        self.lr = float(self.get_parameter('learning_rate').value)
        self.conv_thr = float(self.get_parameter('convergence_thr').value)

        self.v_min = float(self.get_parameter('min_speed').value)
        self.v_max = float(self.get_parameter('max_speed').value)
        self.v_curv_gain = float(self.get_parameter('speed_curvature_gain').value)
        auto_start = bool(self.get_parameter('auto_optimize_on_start').value)

        self.optimized_path = None
        self.running = False

        self.marker_pub = self.create_publisher(MarkerArray, marker_topic, 10)
        self.create_service(Trigger, '~/optimize', self._srv_optimize)
        self.create_service(SetBool, '~/toggle', self._srv_toggle)

        self.get_logger().info('race_line_optimizer_node started')
        self.get_logger().info(f'  input  : {self.input_csv}')
        self.get_logger().info(f'  output : {self.output_csv}')
        self.get_logger().info(f'  half_track_width : {self.half_width} m')

        if auto_start:
            self._run_optimization()

        self.create_timer(2.0, self._publish_markers)

    # ──────────────────────────────────────────────────────────────── #
    # Optimization
    # ──────────────────────────────────────────────────────────────── #
    def _load_csv(self):
        if not os.path.exists(self.input_csv):
            self.get_logger().error(f'파일 없음: {self.input_csv}')
            return None
        pts = []
        with open(self.input_csv, 'r') as f:
            reader = csv.DictReader(f)
            for row in reader:
                pts.append([float(row['x']), float(row['y'])])
        return np.array(pts)

    def _run_optimization(self):
        path = self._load_csv()
        if path is None or len(path) < 3:
            return
        self.get_logger().info(f'[Optimizer] 최적화 시작 ({len(path)} 웨이포인트) ...')
        optimized = self._optimize(path)
        yaws = self._compute_yaws(optimized)
        speeds = self._compute_speeds(optimized)
        self._save_csv(optimized, yaws, speeds)
        self.optimized_path = optimized
        self.get_logger().info('[Optimizer] 완료')

    def _optimize(self, path: np.ndarray) -> np.ndarray:
        n = len(path)
        pts = path.copy()

        for iteration in range(self.max_iter):
            new_pts = pts.copy()
            total_shift = 0.0

            for i in range(n):
                prev = pts[(i - 1) % n]
                curr = pts[i]
                nxt = pts[(i + 1) % n]

                # 이웃의 중점 (이 방향으로 이동하면 곡률 감소)
                target = (prev + nxt) / 2.0

                # 법선 방향 계산
                seg = nxt - prev
                seg_len = np.linalg.norm(seg)
                if seg_len < 1e-9:
                    continue
                normal = np.array([-seg[1], seg[0]]) / seg_len

                # 이동 벡터를 법선 방향으로 투영
                delta = target - curr
                proj = float(np.dot(delta, normal))

                # 트랙 폭 제약
                proj = float(np.clip(proj * self.lr, -self.half_width, self.half_width))
                shift = proj * normal

                new_pts[i] = curr + shift
                total_shift += abs(proj)

            pts = new_pts
            if total_shift / n < self.conv_thr:
                self.get_logger().info(f'[Optimizer] 수렴: iter={iteration}, shift={total_shift/n:.6f}')
                break

        return pts

    def _compute_yaws(self, pts: np.ndarray) -> np.ndarray:
        n = len(pts)
        yaws = np.zeros(n)
        for i in range(n):
            nxt = pts[(i + 1) % n]
            curr = pts[i]
            yaws[i] = math.atan2(nxt[1] - curr[1], nxt[0] - curr[0])
        return yaws

    def _compute_speeds(self, pts: np.ndarray) -> np.ndarray:
        n = len(pts)
        kappas = np.zeros(n)

        for i in range(n):
            a = pts[(i - 1) % n]
            b = pts[i]
            c = pts[(i + 1) % n]
            cross = abs((b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0]))
            la = np.linalg.norm(b - a)
            lb = np.linalg.norm(c - b)
            lc = np.linalg.norm(c - a)
            denom = la * lb * lc
            kappas[i] = cross / denom if denom > 1e-9 else 0.0

        kappa_max = max(kappas.max(), 1e-9)
        speeds = self.v_max - self.v_curv_gain * (kappas / kappa_max) * (self.v_max - self.v_min)
        return np.clip(speeds, self.v_min, self.v_max)

    def _save_csv(self, pts, yaws, speeds):
        with open(self.output_csv, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['x', 'y', 'yaw', 'speed'])
            for (x, y), yaw, spd in zip(pts, yaws, speeds):
                writer.writerow([round(x, 4), round(y, 4), round(yaw, 4), round(float(spd), 4)])
        self.get_logger().info(f'[Optimizer] 저장: {self.output_csv}')

    # ──────────────────────────────────────────────────────────────── #
    # Visualization
    # ──────────────────────────────────────────────────────────────── #
    def _publish_markers(self):
        if self.optimized_path is None:
            return
        ma = MarkerArray()
        now = self.get_clock().now().to_msg()

        line = Marker()
        line.header.stamp = now
        line.header.frame_id = self.frame_id
        line.ns = 'race_line'
        line.id = 0
        line.type = Marker.LINE_STRIP
        line.action = Marker.ADD
        line.scale.x = 0.06
        line.color.a = 1.0
        line.color.r = 1.0
        line.color.g = 0.0
        line.color.b = 1.0  # 보라색

        pts = list(self.optimized_path) + [self.optimized_path[0]]  # 닫힘
        for x, y in pts:
            p = Point(); p.x = float(x); p.y = float(y); p.z = 0.05
            line.points.append(p)
        ma.markers.append(line)
        self.marker_pub.publish(ma)

    # ──────────────────────────────────────────────────────────────── #
    # Services
    # ──────────────────────────────────────────────────────────────── #
    def _srv_optimize(self, _, response):
        self._run_optimization()
        response.success = True
        response.message = f'레이싱 라인 최적화 완료 → {self.output_csv}'
        return response

    def _srv_toggle(self, request, response):
        self.running = request.data
        response.success = True
        response.message = f'실시간 최적화 {"ON" if self.running else "OFF"}'
        return response


def main(args=None):
    rclpy.init(args=args)
    node = RaceLineOptimizerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
