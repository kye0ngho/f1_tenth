import math

import numpy as np

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseArray, PoseStamped
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import Bool, Float32, String
from std_srvs.srv import SetBool


class OvertakePlannerNode(Node):
    """
    상대 차량 오버테이크 경로 생성기.

    /obstacles/nearest_dist 로 전방 장애물 감지 →
    횡방향 오프셋 경로를 /planning/routed_path로 퍼블리시.
    장애물이 없을 때는 원래 /planning/path를 릴레이.

    pure_pursuit / stanley / adaptive_pp 의 path_topic을
    /planning/routed_path 로 변경하면 오버테이크 기능 활성화.

    상태:
      NORMAL   : /planning/path 릴레이
      APPROACH : 장애물 감지, 오버테이크 준비
      OVERTAKE : 오프셋 경로 퍼블리시
      MERGE    : 원래 경로로 복귀
    """

    def __init__(self):
        super().__init__('overtake_planner_node')

        self.declare_parameter('path_topic', '/planning/path')
        self.declare_parameter('output_topic', '/planning/routed_path')
        self.declare_parameter('odom_topic', '/ego_racecar/odom')
        self.declare_parameter('obstacles_dist_topic', '/obstacles/nearest_dist')
        self.declare_parameter('active_topic', '/overtake/active')
        self.declare_parameter('state_topic', '/overtake/state')

        self.declare_parameter('overtake_trigger_dist', 2.5)  # 오버테이크 시작 거리 (m)
        self.declare_parameter('overtake_clear_dist', 4.0)    # 오버테이크 해제 거리 (m)
        self.declare_parameter('lateral_offset', 0.6)         # 횡방향 이동량 (m, 왼쪽=양수)
        self.declare_parameter('approach_speed', 1.0)         # 접근 시 속도 제한
        self.declare_parameter('merge_waypoints', 15)         # 복귀 전환 웨이포인트 수
        self.declare_parameter('frame_id', 'map')

        path_topic = self.get_parameter('path_topic').value
        output_topic = self.get_parameter('output_topic').value
        odom_topic = self.get_parameter('odom_topic').value
        obs_topic = self.get_parameter('obstacles_dist_topic').value
        active_topic = self.get_parameter('active_topic').value
        state_topic = self.get_parameter('state_topic').value
        self.frame_id = self.get_parameter('frame_id').value

        self.trigger_dist = float(self.get_parameter('overtake_trigger_dist').value)
        self.clear_dist = float(self.get_parameter('overtake_clear_dist').value)
        self.lat_offset = float(self.get_parameter('lateral_offset').value)
        self.merge_n = int(self.get_parameter('merge_waypoints').value)

        self.state = 'NORMAL'
        self.car_x = self.car_y = self.car_yaw = 0.0
        self.nearest_dist = float('inf')
        self.ref_path = None
        self._merge_ticks = 0

        self.create_subscription(Path, path_topic, self._cb_path, 10)
        self.create_subscription(Odometry, odom_topic, self._cb_odom, 10)
        self.create_subscription(Float32, obs_topic, self._cb_obs, 10)

        self.path_pub = self.create_publisher(Path, output_topic, 10)
        self.active_pub = self.create_publisher(Bool, active_topic, 10)
        self.state_pub = self.create_publisher(String, state_topic, 10)

        self.create_service(SetBool, '~/force_overtake', self._srv_force)

        self.create_timer(0.2, self._update)  # 5 Hz

        self.get_logger().info('overtake_planner_node started')
        self.get_logger().info(f'  trigger_dist : {self.trigger_dist} m')
        self.get_logger().info(f'  lateral_offset : {self.lat_offset} m')
        self.get_logger().info(f'  output → {output_topic}')

    def _cb_odom(self, msg):
        self.car_x = msg.pose.pose.position.x
        self.car_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        self.car_yaw = math.atan2(2 * q.w * q.z, 1 - 2 * q.z ** 2)

    def _cb_path(self, msg):
        self.ref_path = msg

    def _cb_obs(self, msg):
        dist = msg.data
        self.nearest_dist = dist if dist > 0 else float('inf')

    def _update(self):
        # 상태 전이
        if self.state == 'NORMAL':
            if self.nearest_dist < self.trigger_dist:
                self.state = 'APPROACH'
                self.get_logger().info(f'[Overtake] APPROACH: 장애물 {self.nearest_dist:.2f}m')

        elif self.state == 'APPROACH':
            if self.nearest_dist < self.trigger_dist * 0.7:
                self.state = 'OVERTAKE'
                self.get_logger().info('[Overtake] OVERTAKE: 오프셋 경로 시작')
            elif self.nearest_dist > self.clear_dist:
                self.state = 'NORMAL'

        elif self.state == 'OVERTAKE':
            if self.nearest_dist > self.clear_dist:
                self.state = 'MERGE'
                self._merge_ticks = self.merge_n
                self.get_logger().info('[Overtake] MERGE: 원래 경로 복귀')

        elif self.state == 'MERGE':
            self._merge_ticks -= 1
            if self._merge_ticks <= 0:
                self.state = 'NORMAL'
                self.get_logger().info('[Overtake] NORMAL: 복귀 완료')

        if self.ref_path is None:
            return

        # 경로 퍼블리시
        if self.state in ('NORMAL', 'MERGE'):
            self.path_pub.publish(self.ref_path)
        elif self.state in ('APPROACH', 'OVERTAKE'):
            offset_path = self._generate_offset_path(self.ref_path)
            self.path_pub.publish(offset_path)

        active = Bool()
        active.data = self.state in ('APPROACH', 'OVERTAKE')
        self.active_pub.publish(active)

        s = String(); s.data = self.state
        self.state_pub.publish(s)

    def _generate_offset_path(self, original: Path) -> Path:
        """원래 경로를 법선 방향으로 lat_offset만큼 이동."""
        poses = original.poses
        n = len(poses)
        if n < 2:
            return original

        out = Path()
        out.header = original.header
        out.header.stamp = self.get_clock().now().to_msg()

        for i, ps in enumerate(poses):
            x = ps.pose.position.x
            y = ps.pose.position.y

            # 법선 벡터 계산
            i_next = min(i + 1, n - 1)
            i_prev = max(i - 1, 0)
            dx = poses[i_next].pose.position.x - poses[i_prev].pose.position.x
            dy = poses[i_next].pose.position.y - poses[i_prev].pose.position.y
            seg_len = math.hypot(dx, dy)
            if seg_len < 1e-6:
                nx, ny = 0.0, 1.0
            else:
                nx = -dy / seg_len   # 법선 (왼쪽)
                ny = dx / seg_len

            new_ps = PoseStamped()
            new_ps.header = ps.header
            new_ps.pose.position.x = x + self.lat_offset * nx
            new_ps.pose.position.y = y + self.lat_offset * ny
            new_ps.pose.position.z = ps.pose.position.z
            new_ps.pose.orientation = ps.pose.orientation
            out.poses.append(new_ps)

        return out

    def _srv_force(self, request, response):
        if request.data:
            self.state = 'OVERTAKE'
            response.message = '강제 오버테이크 모드'
        else:
            self.state = 'NORMAL'
            response.message = '일반 모드 복귀'
        response.success = True
        return response


def main(args=None):
    rclpy.init(args=args)
    node = OvertakePlannerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
