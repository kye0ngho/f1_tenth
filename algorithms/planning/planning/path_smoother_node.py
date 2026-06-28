import math

import numpy as np
from scipy.interpolate import CubicSpline

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from std_srvs.srv import Trigger


class PathSmootherNode(Node):
    """
    웨이포인트 경로를 3차 스플라인(Cubic Spline)으로 보간.

    raw waypoint → 호 길이 파라미터화 → 주기 경계 조건 3차 스플라인 →
    고해상도 재샘플링 → /planning/smooth_path 퍼블리시.

    컨트롤러(pure_pursuit/stanley/mpc)의 path_topic을
    /planning/smooth_path로 바꾸면 조향 명령이 크게 부드러워진다.
    """

    def __init__(self):
        super().__init__('path_smoother_node')

        self.declare_parameter('input_topic', '/planning/path')
        self.declare_parameter('output_topic', '/planning/smooth_path')
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('output_points', 300)     # 재샘플링 포인트 수
        self.declare_parameter('min_segment_len', 0.01)  # 중복 제거 최소 거리

        in_topic = self.get_parameter('input_topic').value
        out_topic = self.get_parameter('output_topic').value
        self.frame_id = self.get_parameter('frame_id').value
        self.n_out = int(self.get_parameter('output_points').value)
        self.min_seg = float(self.get_parameter('min_segment_len').value)

        self.raw_path = None

        self.create_subscription(Path, in_topic, self._cb_path, 10)
        self.pub = self.create_publisher(Path, out_topic, 10)
        self.create_service(Trigger, '~/resmooth', self._srv_resmooth)

        self.get_logger().info('path_smoother_node started')
        self.get_logger().info(f'  {in_topic} → {out_topic}')
        self.get_logger().info(f'  output_points : {self.n_out}')

    def _cb_path(self, msg):
        self.raw_path = msg
        smooth = self._smooth(msg)
        if smooth is not None:
            self.pub.publish(smooth)

    def _smooth(self, msg: Path):
        poses = msg.poses
        if len(poses) < 4:
            return msg  # 포인트 부족 시 그대로 릴레이

        xs = np.array([p.pose.position.x for p in poses])
        ys = np.array([p.pose.position.y for p in poses])

        # 중복 포인트 제거
        keep = [0]
        for i in range(1, len(xs)):
            if math.hypot(xs[i] - xs[keep[-1]], ys[i] - ys[keep[-1]]) >= self.min_seg:
                keep.append(i)
        xs, ys = xs[keep], ys[keep]

        if len(xs) < 4:
            return msg

        # 호 길이 파라미터 t
        ds = np.hypot(np.diff(xs), np.diff(ys))
        s = np.concatenate([[0.0], np.cumsum(ds)])
        total = s[-1]

        # 루프 클로저: 마지막 포인트와 첫 포인트 연결
        closure_dist = math.hypot(xs[-1] - xs[0], ys[-1] - ys[0])
        xs_c = np.append(xs, xs[0])
        ys_c = np.append(ys, ys[0])
        s_c = np.append(s, total + closure_dist)

        # 주기 경계 조건 3차 스플라인
        try:
            cs_x = CubicSpline(s_c, xs_c, bc_type='periodic' if closure_dist < 0.5 else 'not-a-knot')
            cs_y = CubicSpline(s_c, ys_c, bc_type='periodic' if closure_dist < 0.5 else 'not-a-knot')
        except Exception as e:
            self.get_logger().warn(f'스플라인 실패: {e}')
            return msg

        # 재샘플링
        s_new = np.linspace(0.0, total, self.n_out)
        xs_s = cs_x(s_new)
        ys_s = cs_y(s_new)
        dxds = cs_x(s_new, 1)
        dyds = cs_y(s_new, 1)
        yaws = np.arctan2(dyds, dxds)

        # Path 메시지 생성
        out = Path()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = self.frame_id

        for x, y, yaw in zip(xs_s, ys_s, yaws):
            ps = PoseStamped()
            ps.header = out.header
            ps.pose.position.x = float(x)
            ps.pose.position.y = float(y)
            ps.pose.position.z = 0.0
            ps.pose.orientation.z = math.sin(yaw / 2)
            ps.pose.orientation.w = math.cos(yaw / 2)
            out.poses.append(ps)

        return out

    def _srv_resmooth(self, _, response):
        if self.raw_path is None:
            response.success = False
            response.message = '경로 없음'
            return response
        smooth = self._smooth(self.raw_path)
        if smooth:
            self.pub.publish(smooth)
        response.success = True
        response.message = f'{self.n_out}포인트로 재보간 완료'
        return response


def main(args=None):
    rclpy.init(args=args)
    node = PathSmootherNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
