import collections
import csv
import math
import os
import time

import numpy as np

import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry, Path
from std_msgs.msg import Float32, String
from std_srvs.srv import Trigger


class TrajectoryEvaluatorNode(Node):
    """
    컨트롤러 성능 평가기.

    실제 주행 궤적(odom)과 계획 경로(path)를 비교해
    횡방향 오차(Cross-Track Error) 통계를 실시간으로 계산:
      - 현재 CTE, 평균 CTE, 최대 CTE, RMS CTE

    또한 속도 추종 오차(speed error)도 추적.

    서비스:
      ~/reset  → 통계 초기화
      ~/save   → 세션 결과 CSV 저장

    출력:
      /evaluator/cte_current   (Float32)
      /evaluator/cte_rms       (Float32)
      /evaluator/speed_error   (Float32)
      /evaluator/report        (String — JSON 요약)
    """

    def __init__(self):
        super().__init__('trajectory_evaluator_node')

        self.declare_parameter('odom_topic', '/ego_racecar/odom')
        self.declare_parameter('path_topic', '/planning/path')
        self.declare_parameter('log_dir', '/sim_ws/src/planning/logs')
        self.declare_parameter('publish_rate', 10.0)
        self.declare_parameter('target_speed', 1.5)
        self.declare_parameter('window_size', 500)

        odom_topic = self.get_parameter('odom_topic').value
        path_topic = self.get_parameter('path_topic').value
        self.log_dir = self.get_parameter('log_dir').value
        rate = float(self.get_parameter('publish_rate').value)
        self.target_speed = float(self.get_parameter('target_speed').value)
        window = int(self.get_parameter('window_size').value)

        self.x = self.y = self.speed = 0.0
        self.ref_path = None

        self.cte_window = collections.deque(maxlen=window)
        self.spd_err_window = collections.deque(maxlen=window)
        self.max_cte = 0.0
        self.session_start = time.time()

        self.create_subscription(Odometry, odom_topic, self._cb_odom, 10)
        self.create_subscription(Path, path_topic, self._cb_path, 10)

        self.cte_cur_pub = self.create_publisher(Float32, '/evaluator/cte_current', 10)
        self.cte_rms_pub = self.create_publisher(Float32, '/evaluator/cte_rms', 10)
        self.spd_err_pub = self.create_publisher(Float32, '/evaluator/speed_error', 10)
        self.report_pub = self.create_publisher(String, '/evaluator/report', 10)

        self.create_service(Trigger, '~/reset', self._srv_reset)
        self.create_service(Trigger, '~/save', self._srv_save)

        self.create_timer(1.0 / rate, self._publish)

        self.get_logger().info('trajectory_evaluator_node started')

    def _cb_odom(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        self.speed = msg.twist.twist.linear.x

        if self.ref_path is None or len(self.ref_path) < 2:
            return

        cte = self._compute_cte()
        self.cte_window.append(abs(cte))
        self.max_cte = max(self.max_cte, abs(cte))

        spd_err = self.target_speed - abs(self.speed)
        self.spd_err_window.append(abs(spd_err))

    def _cb_path(self, msg):
        self.ref_path = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]

    def _compute_cte(self) -> float:
        """가장 가까운 경로 세그먼트에 대한 횡방향 오차 (부호 있음)."""
        path = self.ref_path
        n = len(path)

        dists = [math.hypot(px - self.x, py - self.y) for px, py in path]
        nearest = int(np.argmin(dists))

        p1 = np.array(path[nearest])
        p2 = np.array(path[(nearest + 1) % n])
        seg = p2 - p1
        seg_len = np.linalg.norm(seg)
        if seg_len < 1e-9:
            return 0.0
        seg_dir = seg / seg_len
        normal = np.array([-seg_dir[1], seg_dir[0]])
        to_car = np.array([self.x - p1[0], self.y - p1[1]])
        return float(np.dot(to_car, normal))

    def _publish(self):
        if not self.cte_window:
            return

        cte_arr = np.array(self.cte_window)
        cte_cur = cte_arr[-1] if len(cte_arr) else 0.0
        cte_rms = float(np.sqrt(np.mean(cte_arr ** 2)))
        cte_mean = float(np.mean(cte_arr))
        spd_err_mean = float(np.mean(self.spd_err_window)) if self.spd_err_window else 0.0

        self.cte_cur_pub.publish(Float32(data=cte_cur))
        self.cte_rms_pub.publish(Float32(data=cte_rms))
        self.spd_err_pub.publish(Float32(data=spd_err_mean))

        import json
        report = {
            'elapsed_s': round(time.time() - self.session_start, 1),
            'cte': {
                'current': round(cte_cur, 4),
                'mean': round(cte_mean, 4),
                'rms': round(cte_rms, 4),
                'max': round(self.max_cte, 4),
            },
            'speed_error_mean': round(spd_err_mean, 4),
            'samples': len(self.cte_window),
        }
        s = String(); s.data = json.dumps(report)
        self.report_pub.publish(s)

    def _srv_reset(self, _, response):
        self.cte_window.clear()
        self.spd_err_window.clear()
        self.max_cte = 0.0
        self.session_start = time.time()
        response.success = True
        response.message = '통계 초기화 완료'
        return response

    def _srv_save(self, _, response):
        os.makedirs(self.log_dir, exist_ok=True)
        ts = time.strftime('%Y%m%d_%H%M%S')
        path = os.path.join(self.log_dir, f'eval_{ts}.csv')
        cte_arr = list(self.cte_window)
        spd_arr = list(self.spd_err_window)
        with open(path, 'w', newline='') as f:
            w = csv.writer(f)
            w.writerow(['cte_abs', 'speed_error'])
            for c, s in zip(cte_arr, spd_arr):
                w.writerow([round(c, 5), round(s, 5)])
        response.success = True
        response.message = f'저장: {path}'
        self.get_logger().info(f'[Evaluator] {path}')
        return response


def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryEvaluatorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
