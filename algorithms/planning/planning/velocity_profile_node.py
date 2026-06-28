import csv
import math
import os

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger


class VelocityProfileNode(Node):
    """
    waypoints.csv를 읽어 곡률 기반 속도 프로파일을 계산하고
    최적화된 CSV를 덮어쓴다. 노드 시작 시 자동 실행되며
    ~/apply 서비스로 재실행 가능.
    """

    def __init__(self):
        super().__init__('velocity_profile_node')

        self.declare_parameter('input_csv', '/sim_ws/src/planning/waypoints/waypoints.csv')
        self.declare_parameter('output_csv', '/sim_ws/src/planning/waypoints/waypoints.csv')
        self.declare_parameter('min_speed', 0.4)
        self.declare_parameter('max_speed', 2.0)
        self.declare_parameter('curvature_gain', 3.0)
        self.declare_parameter('smooth_window', 5)

        self.input_csv = self.get_parameter('input_csv').value
        self.output_csv = self.get_parameter('output_csv').value
        self.min_speed = float(self.get_parameter('min_speed').value)
        self.max_speed = float(self.get_parameter('max_speed').value)
        self.curvature_gain = float(self.get_parameter('curvature_gain').value)
        self.smooth_window = int(self.get_parameter('smooth_window').value)

        self.create_service(Trigger, '~/apply', self._srv_apply)

        self.get_logger().info('velocity_profile_node started')
        self.get_logger().info(f'  input csv      : {self.input_csv}')
        self.get_logger().info(f'  output csv     : {self.output_csv}')
        self.get_logger().info(f'  speed range    : {self.min_speed} ~ {self.max_speed} m/s')
        self.get_logger().info(f'  curvature_gain : {self.curvature_gain}')
        self.get_logger().info(f'  smooth_window  : {self.smooth_window}')

        self._run()

    # ------------------------------------------------------------------ #
    # Service
    # ------------------------------------------------------------------ #
    def _srv_apply(self, request, response):
        try:
            n = self._run()
            response.success = True
            response.message = f'{n}개 웨이포인트 속도 프로파일 적용 완료'
        except Exception as e:
            response.success = False
            response.message = str(e)
        self.get_logger().info(f'[service/apply] {response.message}')
        return response

    # ------------------------------------------------------------------ #
    # Core
    # ------------------------------------------------------------------ #
    def _run(self):
        waypoints = self._load(self.input_csv)
        curvatures = self._compute_curvatures(waypoints)
        curvatures = self._smooth(curvatures, self.smooth_window)
        speeds = self._curvature_to_speed(curvatures)
        self._save(waypoints, speeds, self.output_csv)

        self.get_logger().info(
            f'속도 프로파일 완료: {len(waypoints)}개 웨이포인트  '
            f'min={min(speeds):.2f} avg={sum(speeds)/len(speeds):.2f} max={max(speeds):.2f} m/s'
        )
        return len(waypoints)

    def _load(self, path):
        waypoints = []
        with open(path, 'r') as f:
            for row in csv.DictReader(f):
                x = float(row['x'])
                y = float(row['y'])
                yaw = float(row.get('yaw', 0.0))
                speed = float(row.get('speed', self.max_speed))
                waypoints.append((x, y, yaw, speed))
        if len(waypoints) < 3:
            raise RuntimeError('웨이포인트가 3개 미만입니다.')
        return waypoints

    def _compute_curvatures(self, wps):
        n = len(wps)
        curvatures = []
        for i in range(n):
            prev = wps[(i - 1) % n]
            curr = wps[i]
            nxt = wps[(i + 1) % n]

            ax = curr[0] - prev[0]
            ay = curr[1] - prev[1]
            bx = nxt[0] - curr[0]
            by = nxt[1] - curr[1]

            cross = ax * by - ay * bx
            norm_a = math.hypot(ax, ay)
            norm_b = math.hypot(bx, by)

            denom = norm_a * norm_b
            if denom < 1e-9:
                curvatures.append(0.0)
            else:
                # κ = 2·|cross| / (|a|·|b|·|a+b|)
                norm_ab = math.hypot(ax + bx, ay + by)
                kappa = 2.0 * abs(cross) / max(norm_a * norm_b * norm_ab, 1e-9)
                curvatures.append(kappa)

        return curvatures

    def _smooth(self, values, window):
        if window <= 1:
            return values
        n = len(values)
        half = window // 2
        smoothed = []
        for i in range(n):
            indices = [(i + k - half) % n for k in range(window)]
            smoothed.append(sum(values[j] for j in indices) / window)
        return smoothed

    def _curvature_to_speed(self, curvatures):
        max_k = max(curvatures) if max(curvatures) > 1e-9 else 1.0
        speeds = []
        for k in curvatures:
            # 곡률이 클수록 속도 감소: v = v_max - gain * (κ/κ_max) * (v_max - v_min)
            ratio = min(k / max_k, 1.0)
            v = self.max_speed - self.curvature_gain * ratio * (self.max_speed - self.min_speed)
            speeds.append(max(self.min_speed, min(self.max_speed, v)))
        return speeds

    def _save(self, waypoints, speeds, path):
        os.makedirs(os.path.dirname(path), exist_ok=True)
        with open(path, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['x', 'y', 'yaw', 'speed'])
            for (x, y, yaw, _), speed in zip(waypoints, speeds):
                writer.writerow([round(x, 4), round(y, 4), round(yaw, 6), round(speed, 4)])


def main(args=None):
    rclpy.init(args=args)
    node = VelocityProfileNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
