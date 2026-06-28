import csv
import os
import time

import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger, SetBool


class DataLoggerNode(Node):
    """
    주행 상태를 CSV로 저장하는 로거.
    컬럼: timestamp, x, y, yaw, speed, steering, braking, behavior_mode
    서비스: ~/start (Trigger), ~/stop (Trigger), ~/enable (SetBool)
    """

    def __init__(self):
        super().__init__('data_logger_node')

        self.declare_parameter('odom_topic', '/ego_racecar/odom')
        self.declare_parameter('drive_topic', '/drive')
        self.declare_parameter('braking_topic', '/safety/braking')
        self.declare_parameter('mode_topic', '/behavior/mode')
        self.declare_parameter('log_dir', '/sim_ws/src/planning/logs')
        self.declare_parameter('log_rate', 20.0)
        self.declare_parameter('auto_start', True)

        self.odom_topic = self.get_parameter('odom_topic').value
        self.drive_topic = self.get_parameter('drive_topic').value
        self.braking_topic = self.get_parameter('braking_topic').value
        self.mode_topic = self.get_parameter('mode_topic').value
        self.log_dir = self.get_parameter('log_dir').value
        log_rate = float(self.get_parameter('log_rate').value)
        auto_start = bool(self.get_parameter('auto_start').value)

        os.makedirs(self.log_dir, exist_ok=True)

        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.speed = 0.0
        self.steering = 0.0
        self.braking = False
        self.mode = 'unknown'

        self.logging = False
        self.csv_file = None
        self.writer = None

        self.create_subscription(Odometry, self.odom_topic, self._cb_odom, 10)
        self.create_subscription(AckermannDriveStamped, self.drive_topic, self._cb_drive, 10)
        self.create_subscription(Bool, self.braking_topic, self._cb_braking, 10)
        self.create_subscription(String, self.mode_topic, self._cb_mode, 10)

        self.create_service(Trigger, '~/start', self._srv_start)
        self.create_service(Trigger, '~/stop', self._srv_stop)
        self.create_service(SetBool, '~/enable', self._srv_enable)

        self.create_timer(1.0 / log_rate, self._log_tick)

        if auto_start:
            self._open_log()

        self.get_logger().info('data_logger_node started')
        self.get_logger().info(f'  log_dir : {self.log_dir}')
        self.get_logger().info(f'  auto_start : {auto_start}')

    def _cb_odom(self, msg):
        import math
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        self.yaw = math.atan2(2 * q.w * q.z, 1 - 2 * q.z ** 2)
        self.speed = msg.twist.twist.linear.x

    def _cb_drive(self, msg):
        self.steering = msg.drive.steering_angle

    def _cb_braking(self, msg):
        self.braking = msg.data

    def _cb_mode(self, msg):
        self.mode = msg.data

    def _log_tick(self):
        if not self.logging or self.writer is None:
            return
        row = {
            'timestamp': time.time(),
            'x': round(self.x, 4),
            'y': round(self.y, 4),
            'yaw': round(self.yaw, 4),
            'speed': round(self.speed, 4),
            'steering': round(self.steering, 4),
            'braking': int(self.braking),
            'behavior_mode': self.mode,
        }
        self.writer.writerow(row)

    def _open_log(self):
        ts = time.strftime('%Y%m%d_%H%M%S')
        path = os.path.join(self.log_dir, f'run_{ts}.csv')
        self.csv_file = open(path, 'w', newline='')
        fieldnames = ['timestamp', 'x', 'y', 'yaw', 'speed', 'steering', 'braking', 'behavior_mode']
        self.writer = csv.DictWriter(self.csv_file, fieldnames=fieldnames)
        self.writer.writeheader()
        self.logging = True
        self.get_logger().info(f'[DataLogger] 로깅 시작: {path}')
        return path

    def _close_log(self):
        self.logging = False
        if self.csv_file:
            self.csv_file.close()
            self.csv_file = None
            self.writer = None
        self.get_logger().info('[DataLogger] 로깅 중지')

    def _srv_start(self, _, response):
        if self.logging:
            response.success = False
            response.message = '이미 로깅 중'
        else:
            path = self._open_log()
            response.success = True
            response.message = f'로깅 시작: {path}'
        return response

    def _srv_stop(self, _, response):
        self._close_log()
        response.success = True
        response.message = '로깅 중지'
        return response

    def _srv_enable(self, request, response):
        if request.data:
            if not self.logging:
                self._open_log()
            response.success = True
            response.message = 'logging enabled'
        else:
            self._close_log()
            response.success = True
            response.message = 'logging disabled'
        return response

    def destroy_node(self):
        self._close_log()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = DataLoggerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
