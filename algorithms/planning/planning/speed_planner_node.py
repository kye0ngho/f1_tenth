import csv
import math
import os

import numpy as np

import rclpy
from rclpy.node import Node

from std_srvs.srv import Trigger


class SpeedPlannerNode(Node):
    """
    물리 기반 최적 속도 프로파일 계획.

    velocity_profile_node(곡률 휴리스틱)보다 정밀한
    마찰 원(Friction Circle) 모델 기반 속도 계획:

    1. 각 웨이포인트에서 곡률 κ 계산
    2. 횡방향 가속도 제한: v_lat[i] = sqrt(a_lat_max / κ[i])
    3. 전진 패스: v_fwd[i+1] = min(v_lat[i+1], sqrt(v[i]^2 + 2·a_lon_max·ds))
    4. 후진 패스: v[i]      = min(v_fwd[i], sqrt(v[i+1]^2 + 2·a_brake·ds))
    5. 결합하면 트랙 형태에 최적화된 사다리꼴 속도 프로파일 생성

    출력: input_csv를 속도 컬럼만 갱신해 output_csv에 저장.
    ~/plan 서비스로 재실행.
    """

    def __init__(self):
        super().__init__('speed_planner_node')

        self.declare_parameter('input_csv',
                               '/sim_ws/src/planning/waypoints/waypoints.csv')
        self.declare_parameter('output_csv',
                               '/sim_ws/src/planning/waypoints/waypoints.csv')

        self.declare_parameter('a_lat_max', 4.0)     # 횡방향 최대 가속도 (m/s²)
        self.declare_parameter('a_lon_max', 3.0)     # 종방향 최대 가속도 (m/s²)
        self.declare_parameter('a_brake', 5.0)       # 최대 제동 가속도 (m/s²)
        self.declare_parameter('v_min', 0.4)
        self.declare_parameter('v_max', 3.0)
        self.declare_parameter('smooth_window', 5)
        self.declare_parameter('auto_plan_on_start', True)

        self.input_csv = self.get_parameter('input_csv').value
        self.output_csv = self.get_parameter('output_csv').value
        self.a_lat = float(self.get_parameter('a_lat_max').value)
        self.a_lon = float(self.get_parameter('a_lon_max').value)
        self.a_brk = float(self.get_parameter('a_brake').value)
        self.v_min = float(self.get_parameter('v_min').value)
        self.v_max = float(self.get_parameter('v_max').value)
        self.smooth_w = int(self.get_parameter('smooth_window').value)
        auto = bool(self.get_parameter('auto_plan_on_start').value)

        self.create_service(Trigger, '~/plan', self._srv_plan)

        self.get_logger().info('speed_planner_node started')
        self.get_logger().info(f'  a_lat={self.a_lat}  a_lon={self.a_lon}  a_brake={self.a_brk} m/s²')
        self.get_logger().info(f'  v_range={self.v_min}~{self.v_max} m/s')

        if auto:
            self._run()

    def _run(self):
        pts, yaws, orig_speeds = self._load()
        if pts is None:
            return

        speeds = self._plan(pts)
        self._save(pts, yaws, speeds)

    def _load(self):
        if not os.path.exists(self.input_csv):
            self.get_logger().error(f'파일 없음: {self.input_csv}')
            return None, None, None
        pts, yaws, speeds = [], [], []
        with open(self.input_csv, 'r') as f:
            for row in csv.DictReader(f):
                pts.append([float(row['x']), float(row['y'])])
                yaws.append(float(row.get('yaw', 0)))
                speeds.append(float(row.get('speed', 1.0)))
        return np.array(pts), np.array(yaws), np.array(speeds)

    def _plan(self, pts: np.ndarray) -> np.ndarray:
        n = len(pts)

        # 구간 거리
        ds = np.linalg.norm(np.diff(pts, axis=0, append=pts[:1]), axis=1)
        ds = np.maximum(ds, 1e-6)

        # 곡률 계산 (3-point)
        kappas = np.zeros(n)
        for i in range(n):
            a = pts[(i - 1) % n]
            b = pts[i]
            c = pts[(i + 1) % n]
            cross = abs((b[0]-a[0])*(c[1]-a[1]) - (b[1]-a[1])*(c[0]-a[0]))
            denom = (np.linalg.norm(b-a) * np.linalg.norm(c-b) * np.linalg.norm(c-a))
            kappas[i] = cross / denom if denom > 1e-9 else 0.0

        # 이동 평균으로 곡률 스무딩
        if self.smooth_w > 1:
            pad = self.smooth_w // 2
            padded = np.concatenate([kappas[-pad:], kappas, kappas[:pad]])
            kappas = np.array([np.mean(padded[i:i+self.smooth_w]) for i in range(n)])

        # 횡방향 가속도 기반 최대 속도
        v_lat = np.where(kappas > 1e-6,
                         np.sqrt(self.a_lat / kappas),
                         self.v_max)
        v_lat = np.clip(v_lat, self.v_min, self.v_max)

        # 전진 패스: 가속 제한
        v_fwd = v_lat.copy()
        for _ in range(3):  # 루프 경로이므로 몇 번 반복
            for i in range(n):
                i_next = (i + 1) % n
                v_accel = math.sqrt(v_fwd[i] ** 2 + 2 * self.a_lon * ds[i])
                v_fwd[i_next] = min(v_fwd[i_next], v_accel)

        # 후진 패스: 제동 제한
        v_out = v_fwd.copy()
        for _ in range(3):
            for i in range(n - 1, -1, -1):
                i_next = (i + 1) % n
                v_brake = math.sqrt(v_out[i_next] ** 2 + 2 * self.a_brk * ds[i])
                v_out[i] = min(v_out[i], v_brake)

        v_out = np.clip(v_out, self.v_min, self.v_max)

        self.get_logger().info(
            f'[SpeedPlanner] 최적화 완료: '
            f'v_avg={v_out.mean():.2f}  v_max={v_out.max():.2f}  v_min={v_out.min():.2f} m/s'
        )
        return v_out

    def _save(self, pts, yaws, speeds):
        with open(self.output_csv, 'w', newline='') as f:
            w = csv.writer(f)
            w.writerow(['x', 'y', 'yaw', 'speed'])
            for (x, y), yaw, spd in zip(pts, yaws, speeds):
                w.writerow([round(x, 4), round(y, 4), round(float(yaw), 4), round(float(spd), 4)])
        self.get_logger().info(f'[SpeedPlanner] 저장: {self.output_csv}')

    def _srv_plan(self, _, response):
        pts, yaws, _ = self._load()
        if pts is None:
            response.success = False
            response.message = '파일 로드 실패'
            return response
        speeds = self._plan(pts)
        self._save(pts, yaws, speeds)
        response.success = True
        response.message = f'속도 프로파일 재계산 완료 → {self.output_csv}'
        return response


def main(args=None):
    rclpy.init(args=args)
    node = SpeedPlannerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
