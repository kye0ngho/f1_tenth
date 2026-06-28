import glob
import os

import csv

import rclpy
from rclpy.node import Node

from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from std_srvs.srv import Trigger

from rcl_interfaces.msg import SetParametersResult


class WaypointManagerNode(Node):
    """
    여러 웨이포인트 CSV 파일을 관리하고 런타임에 전환.
    서비스:
      ~/list      → 현재 디렉터리의 CSV 목록 반환
      ~/next      → 다음 파일로 전환
      ~/prev      → 이전 파일로 전환
      ~/reload    → 현재 파일 재로드
    파라미터 업데이트: waypoint_csv → 즉시 로드
    퍼블리시: /planning/path (Path), /waypoint_manager/current_file (String)
    """

    def __init__(self):
        super().__init__('waypoint_manager_node')

        self.declare_parameter('waypoint_dir', '/sim_ws/src/planning/waypoints')
        self.declare_parameter('waypoint_csv', '')
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('publish_rate', 2.0)

        self.waypoint_dir = self.get_parameter('waypoint_dir').value
        initial_csv = self.get_parameter('waypoint_csv').value
        self.frame_id = self.get_parameter('frame_id').value
        rate = float(self.get_parameter('publish_rate').value)

        self.path_pub = self.create_publisher(Path, '/planning/path', 10)
        self.file_pub = self.create_publisher(String, '/waypoint_manager/current_file', 10)

        self.create_service(Trigger, '~/list', self._srv_list)
        self.create_service(Trigger, '~/next', self._srv_next)
        self.create_service(Trigger, '~/prev', self._srv_prev)
        self.create_service(Trigger, '~/reload', self._srv_reload)

        self.add_on_set_parameters_callback(self._on_param_change)

        self.files = []
        self.current_idx = 0
        self.current_path = None

        self._refresh_file_list()

        if initial_csv and os.path.exists(initial_csv):
            self._load_csv(initial_csv)
        elif self.files:
            self._load_csv(self.files[0])

        self.create_timer(1.0 / rate, self._publish)

        self.get_logger().info('waypoint_manager_node started')
        self.get_logger().info(f'  waypoint_dir : {self.waypoint_dir}')
        self.get_logger().info(f'  files found  : {len(self.files)}')

    def _refresh_file_list(self):
        pattern = os.path.join(self.waypoint_dir, '*.csv')
        self.files = sorted(glob.glob(pattern))

    def _load_csv(self, path):
        if not os.path.exists(path):
            self.get_logger().error(f'파일 없음: {path}')
            return False

        waypoints = []
        with open(path, 'r') as f:
            reader = csv.DictReader(f)
            for row in reader:
                waypoints.append((
                    float(row.get('x', 0)),
                    float(row.get('y', 0)),
                    float(row.get('yaw', 0)),
                ))

        msg = Path()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id

        import math
        for x, y, yaw in waypoints:
            ps = PoseStamped()
            ps.header = msg.header
            ps.pose.position.x = x
            ps.pose.position.y = y
            ps.pose.position.z = 0.0
            ps.pose.orientation.z = math.sin(yaw / 2)
            ps.pose.orientation.w = math.cos(yaw / 2)
            msg.poses.append(ps)

        self.current_path = msg
        if path in self.files:
            self.current_idx = self.files.index(path)

        self.get_logger().info(f'로드: {os.path.basename(path)} ({len(waypoints)} waypoints)')

        s = String()
        s.data = path
        self.file_pub.publish(s)
        return True

    def _publish(self):
        if self.current_path is None:
            return
        self.current_path.header.stamp = self.get_clock().now().to_msg()
        self.path_pub.publish(self.current_path)

        s = String()
        s.data = self.files[self.current_idx] if self.files else ''
        self.file_pub.publish(s)

    def _srv_list(self, _, response):
        self._refresh_file_list()
        names = [os.path.basename(f) for f in self.files]
        response.success = True
        response.message = ', '.join(names) if names else '파일 없음'
        return response

    def _srv_next(self, _, response):
        if not self.files:
            response.success = False
            response.message = '파일 없음'
            return response
        self.current_idx = (self.current_idx + 1) % len(self.files)
        ok = self._load_csv(self.files[self.current_idx])
        response.success = ok
        response.message = os.path.basename(self.files[self.current_idx])
        return response

    def _srv_prev(self, _, response):
        if not self.files:
            response.success = False
            response.message = '파일 없음'
            return response
        self.current_idx = (self.current_idx - 1) % len(self.files)
        ok = self._load_csv(self.files[self.current_idx])
        response.success = ok
        response.message = os.path.basename(self.files[self.current_idx])
        return response

    def _srv_reload(self, _, response):
        if not self.files:
            response.success = False
            response.message = '파일 없음'
            return response
        ok = self._load_csv(self.files[self.current_idx])
        response.success = ok
        response.message = '재로드 완료'
        return response

    def _on_param_change(self, params):
        for p in params:
            if p.name == 'waypoint_csv' and p.value:
                self._load_csv(p.value)
        return SetParametersResult(successful=True)


def main(args=None):
    rclpy.init(args=args)
    node = WaypointManagerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
