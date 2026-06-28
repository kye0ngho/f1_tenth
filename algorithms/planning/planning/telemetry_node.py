import json
import os
import time

import rclpy
from rclpy.node import Node

from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, Float32, String


class TelemetryNode(Node):
    """
    주행 텔레메트리 집계 및 퍼블리셔.

    수집:
      - 속도 (평균 / 최대 / 현재)
      - 제동 횟수 및 비율
      - 현재 위치 (x, y)
      - 현재 행동 모드
      - 랩 타임 (lap_timer에서)
      - 토픽 수신 주파수 (odom, drive)
      - 세션 경과 시간

    출력:
      /telemetry/stats (String — JSON)
      /telemetry/avg_speed (Float32)
      로그 파일: telemetry_<timestamp>.jsonl
    """

    def __init__(self):
        super().__init__('telemetry_node')

        self.declare_parameter('odom_topic', '/ego_racecar/odom')
        self.declare_parameter('drive_topic', '/drive')
        self.declare_parameter('braking_topic', '/safety/braking')
        self.declare_parameter('mode_topic', '/behavior/mode')
        self.declare_parameter('lap_time_topic', '/lap_timer/lap_time')
        self.declare_parameter('stats_topic', '/telemetry/stats')
        self.declare_parameter('avg_speed_topic', '/telemetry/avg_speed')
        self.declare_parameter('log_dir', '/sim_ws/src/planning/logs')
        self.declare_parameter('publish_rate', 1.0)
        self.declare_parameter('enable_file_log', True)

        odom_topic = self.get_parameter('odom_topic').value
        drive_topic = self.get_parameter('drive_topic').value
        braking_topic = self.get_parameter('braking_topic').value
        mode_topic = self.get_parameter('mode_topic').value
        lap_topic = self.get_parameter('lap_time_topic').value
        stats_topic = self.get_parameter('stats_topic').value
        avg_topic = self.get_parameter('avg_speed_topic').value
        self.log_dir = self.get_parameter('log_dir').value
        rate = float(self.get_parameter('publish_rate').value)
        self.file_log = bool(self.get_parameter('enable_file_log').value)

        # 누적 통계
        self.session_start = time.time()
        self.current_speed = 0.0
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_mode = 'unknown'
        self.braking = False

        self.speed_samples = []
        self.brake_count = 0
        self.total_ticks = 0
        self.brake_ticks = 0
        self.lap_times = []
        self.best_lap = None

        # 주파수 카운터
        self.odom_count = 0
        self.drive_count = 0
        self.freq_window = 5.0   # 5초 윈도우
        self.freq_reset_time = time.time()

        self.create_subscription(Odometry, odom_topic, self._cb_odom, 10)
        self.create_subscription(AckermannDriveStamped, drive_topic, self._cb_drive, 10)
        self.create_subscription(Bool, braking_topic, self._cb_braking, 10)
        self.create_subscription(String, mode_topic, self._cb_mode, 10)
        self.create_subscription(Float32, lap_topic, self._cb_lap, 10)

        self.stats_pub = self.create_publisher(String, stats_topic, 10)
        self.avg_pub = self.create_publisher(Float32, avg_topic, 10)

        if self.file_log:
            os.makedirs(self.log_dir, exist_ok=True)
            ts = time.strftime('%Y%m%d_%H%M%S')
            self.log_path = os.path.join(self.log_dir, f'telemetry_{ts}.jsonl')
            self.log_file = open(self.log_path, 'w')
            self.get_logger().info(f'[Telemetry] 로그: {self.log_path}')
        else:
            self.log_file = None

        self.create_timer(1.0 / rate, self._publish)

        self.get_logger().info('telemetry_node started')

    def _cb_odom(self, msg):
        self.current_speed = msg.twist.twist.linear.x
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        self.speed_samples.append(abs(self.current_speed))
        self.odom_count += 1

    def _cb_drive(self, msg):
        self.drive_count += 1

    def _cb_braking(self, msg):
        prev = self.braking
        self.braking = msg.data
        if self.braking and not prev:
            self.brake_count += 1
        if self.braking:
            self.brake_ticks += 1
        self.total_ticks += 1

    def _cb_mode(self, msg):
        self.current_mode = msg.data

    def _cb_lap(self, msg):
        lt = float(msg.data)
        self.lap_times.append(lt)
        if self.best_lap is None or lt < self.best_lap:
            self.best_lap = lt
            self.get_logger().info(f'[Telemetry] 신기록! {lt:.3f}s')

    def _publish(self):
        now = time.time()
        elapsed = now - self.session_start

        # 주파수 계산
        freq_dt = now - self.freq_reset_time
        odom_hz = self.odom_count / freq_dt if freq_dt > 0 else 0.0
        drive_hz = self.drive_count / freq_dt if freq_dt > 0 else 0.0
        if freq_dt >= self.freq_window:
            self.odom_count = 0
            self.drive_count = 0
            self.freq_reset_time = now

        avg_speed = float(sum(self.speed_samples) / len(self.speed_samples)) if self.speed_samples else 0.0
        max_speed = float(max(self.speed_samples)) if self.speed_samples else 0.0
        brake_ratio = (self.brake_ticks / self.total_ticks * 100) if self.total_ticks > 0 else 0.0

        stats = {
            'timestamp': round(now, 2),
            'elapsed_s': round(elapsed, 1),
            'position': {'x': round(self.current_x, 3), 'y': round(self.current_y, 3)},
            'speed': {
                'current': round(self.current_speed, 3),
                'avg': round(avg_speed, 3),
                'max': round(max_speed, 3),
            },
            'braking': {
                'count': self.brake_count,
                'ratio_pct': round(brake_ratio, 1),
            },
            'mode': self.current_mode,
            'laps': {
                'count': len(self.lap_times),
                'best_s': round(self.best_lap, 3) if self.best_lap else None,
                'last_s': round(self.lap_times[-1], 3) if self.lap_times else None,
            },
            'topic_hz': {
                'odom': round(odom_hz, 1),
                'drive': round(drive_hz, 1),
            },
        }

        s = String(); s.data = json.dumps(stats, ensure_ascii=False)
        self.stats_pub.publish(s)

        a = Float32(); a.data = avg_speed
        self.avg_pub.publish(a)

        if self.log_file:
            self.log_file.write(json.dumps(stats) + '\n')
            self.log_file.flush()

    def destroy_node(self):
        if self.log_file:
            self.log_file.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = TelemetryNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
