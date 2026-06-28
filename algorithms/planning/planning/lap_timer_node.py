import math
import os
import time

import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry
from std_msgs.msg import Float32, String
from std_srvs.srv import Trigger


class LapTimerNode(Node):
    """
    출발점 반경 진입 → 이탈 → 재진입을 감지해 랩 타임을 측정한다.
    첫 번째 odom 수신 위치를 출발점으로 자동 설정하거나
    start_x / start_y 파라미터로 직접 지정 가능.
    """

    def __init__(self):
        super().__init__('lap_timer_node')

        self.declare_parameter('odom_topic', '/ego_racecar/odom')
        self.declare_parameter('lap_time_topic', '/lap_timer/lap_time')
        self.declare_parameter('status_topic', '/lap_timer/status')
        self.declare_parameter('trigger_radius', 0.5)
        self.declare_parameter('min_lap_distance', 2.0)
        self.declare_parameter('log_file', '/sim_ws/src/planning/waypoints/lap_times.txt')
        self.declare_parameter('start_x', float('nan'))
        self.declare_parameter('start_y', float('nan'))

        self.odom_topic = self.get_parameter('odom_topic').value
        self.lap_time_topic = self.get_parameter('lap_time_topic').value
        self.status_topic = self.get_parameter('status_topic').value
        self.trigger_radius = float(self.get_parameter('trigger_radius').value)
        self.min_lap_distance = float(self.get_parameter('min_lap_distance').value)
        self.log_file = self.get_parameter('log_file').value
        start_x = self.get_parameter('start_x').value
        start_y = self.get_parameter('start_y').value

        # 출발점: 파라미터로 지정되지 않으면 첫 odom에서 자동 설정
        if not (math.isnan(start_x) or math.isnan(start_y)):
            self.start_x = start_x
            self.start_y = start_y
            self._start_locked = True
        else:
            self.start_x = None
            self.start_y = None
            self._start_locked = False

        self._in_zone = False          # 현재 출발 반경 안에 있는지
        self._traveled = 0.0           # 이번 랩 누적 이동 거리
        self._lap_start = None         # 랩 시작 시각 (monotonic)
        self._last_x = None
        self._last_y = None
        self._lap_count = 0
        self._best = None

        self.create_subscription(Odometry, self.odom_topic, self.odom_callback, 10)

        self.lap_pub = self.create_publisher(Float32, self.lap_time_topic, 10)
        self.status_pub = self.create_publisher(String, self.status_topic, 10)

        self.create_service(Trigger, '~/reset', self._srv_reset)
        self.create_service(Trigger, '~/set_start', self._srv_set_start)

        self.get_logger().info('lap_timer_node started')
        self.get_logger().info(f'  odom_topic      : {self.odom_topic}')
        self.get_logger().info(f'  trigger_radius  : {self.trigger_radius} m')
        self.get_logger().info(f'  min_lap_distance: {self.min_lap_distance} m')
        self.get_logger().info(f'  log_file        : {self.log_file}')
        if self._start_locked:
            self.get_logger().info(
                f'  출발점 (파라미터): ({self.start_x:.3f}, {self.start_y:.3f})'
            )
        else:
            self.get_logger().info('  출발점: 첫 odom 위치에서 자동 설정')

    # ------------------------------------------------------------------ #
    # Odom
    # ------------------------------------------------------------------ #
    def odom_callback(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        now = time.monotonic()

        # 이동 거리 누적
        if self._last_x is not None:
            self._traveled += math.hypot(x - self._last_x, y - self._last_y)
        self._last_x = x
        self._last_y = y

        # 출발점 자동 설정 (첫 odom)
        if not self._start_locked:
            self.start_x = x
            self.start_y = y
            self._start_locked = True
            self._in_zone = True
            self._lap_start = now
            self._traveled = 0.0
            self.get_logger().info(
                f'출발점 자동 설정: ({x:.3f}, {y:.3f})'
            )
            self._publish_status(f'출발점 설정 ({x:.3f}, {y:.3f}) — 주행 시작 시 랩 측정 시작')
            return

        dist_to_start = math.hypot(x - self.start_x, y - self.start_y)
        in_zone_now = dist_to_start <= self.trigger_radius

        # 출발 반경 이탈
        if self._in_zone and not in_zone_now:
            self._in_zone = False
            if self._lap_start is None:
                self._lap_start = now
                self._traveled = 0.0

        # 출발 반경 재진입 → 랩 완료
        if not self._in_zone and in_zone_now and self._traveled >= self.min_lap_distance:
            self._in_zone = True
            lap_time = now - self._lap_start

            self._lap_count += 1
            if self._best is None or lap_time < self._best:
                self._best = lap_time
                best_str = f'★ 베스트!'
            else:
                best_str = f'베스트: {self._best:.3f} s'

            msg_str = (
                f'Lap {self._lap_count}  {lap_time:.3f} s  '
                f'({self._traveled:.1f} m)  {best_str}'
            )
            self.get_logger().info(f'[LAP] {msg_str}')

            lap_msg = Float32()
            lap_msg.data = float(lap_time)
            self.lap_pub.publish(lap_msg)
            self._publish_status(msg_str)
            self._log(lap_time)

            self._lap_start = now
            self._traveled = 0.0

    # ------------------------------------------------------------------ #
    # Services
    # ------------------------------------------------------------------ #
    def _srv_reset(self, request, response):
        self._lap_count = 0
        self._best = None
        self._traveled = 0.0
        self._lap_start = None
        self._in_zone = False
        response.success = True
        response.message = '랩 타이머 초기화 완료'
        self.get_logger().info('[service/reset] 랩 타이머 초기화')
        return response

    def _srv_set_start(self, request, response):
        if self._last_x is None:
            response.success = False
            response.message = 'odom 수신 전'
            return response
        self.start_x = self._last_x
        self.start_y = self._last_y
        self._in_zone = True
        self._traveled = 0.0
        self._lap_start = None
        response.success = True
        response.message = f'출발점 갱신: ({self.start_x:.3f}, {self.start_y:.3f})'
        self.get_logger().info(f'[service/set_start] {response.message}')
        return response

    # ------------------------------------------------------------------ #
    # Helpers
    # ------------------------------------------------------------------ #
    def _publish_status(self, text):
        msg = String()
        msg.data = text
        self.status_pub.publish(msg)

    def _log(self, lap_time):
        try:
            os.makedirs(os.path.dirname(self.log_file), exist_ok=True)
            with open(self.log_file, 'a') as f:
                ts = time.strftime('%Y-%m-%d %H:%M:%S')
                f.write(f'[{ts}] Lap {self._lap_count}  {lap_time:.3f} s\n')
        except Exception as e:
            self.get_logger().warn(f'로그 저장 실패: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = LapTimerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
