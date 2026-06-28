import csv
import json
import math
import os
import time

import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry
from std_msgs.msg import Float32, Int32, String
from std_srvs.srv import Trigger


class SectorTimerNode(Node):
    """
    트랙 구간(Sector) 타이머.

    섹터 정의: sector_csv 파일에서 로드 (x, y, radius, name)
    차량이 각 섹터 게이트 반경 내에 진입하면 섹터 시간 기록.
    순서 대로 통과해야 함 (순환).

    출력:
      /sector_timer/current_sector  (Int32)
      /sector_timer/sector_time     (Float32 — 마지막 섹터 경과시간)
      /sector_timer/split           (String — JSON 형태 기록)
      /sector_timer/best_split      (String)

    서비스:
      ~/reset   — 기록 초기화
      ~/list    — 섹터 목록 반환
    """

    def __init__(self):
        super().__init__('sector_timer_node')

        self.declare_parameter('odom_topic', '/ego_racecar/odom')
        self.declare_parameter('sector_csv',
                               '/sim_ws/src/planning/waypoints/sectors.csv')
        self.declare_parameter('default_radius', 1.0)
        self.declare_parameter('min_sector_interval', 5.0)
        self.declare_parameter('log_dir', '/sim_ws/src/planning/logs')
        self.declare_parameter('publish_rate', 10.0)

        odom_topic = self.get_parameter('odom_topic').value
        self.sector_csv = self.get_parameter('sector_csv').value
        self.default_radius = float(self.get_parameter('default_radius').value)
        self.min_interval = float(self.get_parameter('min_sector_interval').value)
        self.log_dir = self.get_parameter('log_dir').value
        rate = float(self.get_parameter('publish_rate').value)

        self.sectors = []     # list of (x, y, radius, name)
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_sector = 0  # 다음에 통과해야 할 섹터 인덱스
        self.sector_enter_time = None
        self.last_sector_time = None
        self.sector_times = []     # 이번 랩 섹터 시간
        self.best_times = {}       # 섹터 이름 → 최고 기록
        self.in_sector = False

        self._load_sectors()

        os.makedirs(self.log_dir, exist_ok=True)
        ts_str = time.strftime('%Y%m%d_%H%M%S')
        self.log_path = os.path.join(self.log_dir, f'sectors_{ts_str}.csv')

        self.create_subscription(Odometry, odom_topic, self._cb_odom, 10)
        self.sector_pub = self.create_publisher(Int32, '/sector_timer/current_sector', 10)
        self.time_pub = self.create_publisher(Float32, '/sector_timer/sector_time', 10)
        self.split_pub = self.create_publisher(String, '/sector_timer/split', 10)
        self.best_pub = self.create_publisher(String, '/sector_timer/best_split', 10)

        self.create_service(Trigger, '~/reset', self._srv_reset)
        self.create_service(Trigger, '~/list', self._srv_list)

        self.create_timer(1.0 / rate, self._publish)

        self.get_logger().info('sector_timer_node started')
        self.get_logger().info(f'  섹터 수 : {len(self.sectors)}')
        for i, (x, y, r, name) in enumerate(self.sectors):
            self.get_logger().info(f'  [{i}] {name} ({x:.1f},{y:.1f}) r={r:.1f}m')

    def _load_sectors(self):
        if not os.path.exists(self.sector_csv):
            self.get_logger().warn(f'섹터 파일 없음: {self.sector_csv}')
            self.get_logger().warn('기본 섹터(S1/S2/S3)를 자동 생성합니다. 직접 수정하세요.')
            self._create_default_sectors()
            return

        with open(self.sector_csv, 'r') as f:
            reader = csv.DictReader(f)
            for row in reader:
                self.sectors.append((
                    float(row.get('x', 0)),
                    float(row.get('y', 0)),
                    float(row.get('radius', self.default_radius)),
                    row.get('name', f'S{len(self.sectors) + 1}'),
                ))

    def _create_default_sectors(self):
        os.makedirs(os.path.dirname(self.sector_csv), exist_ok=True)
        defaults = [
            (0.0, 0.0, self.default_radius, 'S1-Start'),
            (5.0, 0.0, self.default_radius, 'S2-Middle'),
            (0.0, 5.0, self.default_radius, 'S3-End'),
        ]
        with open(self.sector_csv, 'w', newline='') as f:
            w = csv.writer(f)
            w.writerow(['x', 'y', 'radius', 'name'])
            for row in defaults:
                w.writerow(row)
        self.sectors = list(defaults)

    def _cb_odom(self, msg):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        self._check_sector()

    def _check_sector(self):
        if not self.sectors:
            return

        sx, sy, sr, sname = self.sectors[self.current_sector]
        dist = math.hypot(self.current_x - sx, self.current_y - sy)

        if dist < sr:
            if not self.in_sector:
                now = time.time()
                self.in_sector = True
                # 이전 섹터 통과 시간 계산
                if self.sector_enter_time is not None:
                    elapsed = now - self.sector_enter_time
                    if elapsed >= self.min_interval:
                        self._record_sector(sname, elapsed, now)
                self.sector_enter_time = now
        else:
            self.in_sector = False

    def _record_sector(self, name, elapsed, now):
        self.last_sector_time = elapsed
        self.sector_times.append((name, elapsed))

        if name not in self.best_times or elapsed < self.best_times[name]:
            self.best_times[name] = elapsed
            self.get_logger().info(f'[Sector] {name} 신기록! {elapsed:.3f}s')
        else:
            self.get_logger().info(f'[Sector] {name} {elapsed:.3f}s '
                                   f'(best: {self.best_times[name]:.3f}s)')

        # 로그 파일 기록
        with open(self.log_path, 'a', newline='') as f:
            w = csv.writer(f)
            w.writerow([time.strftime('%H:%M:%S'), name, round(elapsed, 3)])

        self.current_sector = (self.current_sector + 1) % len(self.sectors)

    def _publish(self):
        idx = Int32(); idx.data = self.current_sector
        self.sector_pub.publish(idx)

        if self.last_sector_time is not None:
            t = Float32(); t.data = float(self.last_sector_time)
            self.time_pub.publish(t)

        if self.sector_times:
            split_data = {name: round(t, 3) for name, t in self.sector_times[-len(self.sectors):]}
            s = String(); s.data = json.dumps(split_data)
            self.split_pub.publish(s)

        if self.best_times:
            b = String(); b.data = json.dumps({k: round(v, 3) for k, v in self.best_times.items()})
            self.best_pub.publish(b)

    def _srv_reset(self, _, response):
        self.current_sector = 0
        self.sector_enter_time = None
        self.last_sector_time = None
        self.sector_times.clear()
        self.best_times.clear()
        self.in_sector = False
        response.success = True
        response.message = '섹터 타이머 초기화 완료'
        return response

    def _srv_list(self, _, response):
        lines = [f'[{i}] {n} ({x:.1f},{y:.1f}) r={r:.1f}'
                 for i, (x, y, r, n) in enumerate(self.sectors)]
        response.success = True
        response.message = '\n'.join(lines) if lines else '섹터 없음'
        return response


def main(args=None):
    rclpy.init(args=args)
    node = SectorTimerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
