import math

import numpy as np

import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from geometry_msgs.msg import PoseWithCovarianceStamped


class ImuOdometryNode(Node):
    """
    IMU + 휠 오도메트리 상보 필터(Complementary Filter) 융합.

    문제: 바퀴 슬립이나 미끄러짐이 있으면 odom heading이 드리프트.
    해결: IMU 자이로 적분(yaw rate)과 odom heading을 상보 필터로 융합.

        yaw_fused = α · yaw_odom  +  (1-α) · yaw_imu_integrated

    α=1 → odom 완전 신뢰 (시뮬 기본)
    α=0 → IMU 완전 신뢰 (실차 고속 시)
    권장: α=0.3~0.6 (실차)

    출력: /localization/odom_fused
    pure_pursuit / stanley / mpc의 odom_topic을 여기로 바꾸면
    IMU 보정이 적용된 더 정밀한 상태 추정 사용.
    """

    def __init__(self):
        super().__init__('imu_odometry_node')

        self.declare_parameter('odom_topic', '/ego_racecar/odom')
        self.declare_parameter('imu_topic', '/imu/data')
        self.declare_parameter('output_topic', '/localization/odom_fused')
        self.declare_parameter('alpha', 0.5)          # odom 가중치 (0~1)
        self.declare_parameter('gyro_bias_samples', 100)

        odom_topic = self.get_parameter('odom_topic').value
        imu_topic = self.get_parameter('imu_topic').value
        out_topic = self.get_parameter('output_topic').value
        self.alpha = float(self.get_parameter('alpha').value)
        self.bias_samples = int(self.get_parameter('gyro_bias_samples').value)

        # 상태
        self.odom_msg = None
        self.yaw_imu = 0.0
        self.imu_prev_time = None
        self.gyro_bias = 0.0
        self.bias_buf = []
        self.bias_calibrated = False

        self.create_subscription(Odometry, odom_topic, self._cb_odom, 10)
        self.create_subscription(Imu, imu_topic, self._cb_imu, 10)
        self.pub = self.create_publisher(Odometry, out_topic, 10)

        self.get_logger().info('imu_odometry_node started')
        self.get_logger().info(f'  alpha={self.alpha}  (0=IMU only, 1=odom only)')
        self.get_logger().info(f'  gyro bias 보정: {self.bias_samples} 샘플 대기 중...')

    def _cb_imu(self, msg: Imu):
        omega_z = msg.angular_velocity.z

        now = self.get_clock().now()

        # 자이로 바이어스 수집 (초기 정지 상태)
        if not self.bias_calibrated:
            self.bias_buf.append(omega_z)
            if len(self.bias_buf) >= self.bias_samples:
                self.gyro_bias = float(np.mean(self.bias_buf))
                self.bias_calibrated = True
                self.get_logger().info(f'  자이로 바이어스 보정 완료: {self.gyro_bias:.5f} rad/s')
            self.imu_prev_time = now
            return

        if self.imu_prev_time is None:
            self.imu_prev_time = now
            return

        dt = (now - self.imu_prev_time).nanoseconds * 1e-9
        self.imu_prev_time = now

        if dt <= 0 or dt > 0.5:
            return

        # 자이로 적분 (바이어스 보정)
        self.yaw_imu += (omega_z - self.gyro_bias) * dt
        self.yaw_imu = self._wrap(self.yaw_imu)

        self._fuse_and_publish()

    def _cb_odom(self, msg: Odometry):
        self.odom_msg = msg
        # IMU 없이도 동작하도록: bias 미보정 시 odom 그대로 퍼블리시
        if not self.bias_calibrated:
            self.pub.publish(msg)

    def _fuse_and_publish(self):
        if self.odom_msg is None:
            return

        msg = self.odom_msg
        q = msg.pose.pose.orientation
        yaw_odom = math.atan2(2 * q.w * q.z, 1 - 2 * q.z ** 2)

        # 상보 필터 융합
        # 두 yaw의 차이가 π를 넘으면 보정
        d = self._wrap(self.yaw_imu - yaw_odom)
        yaw_fused = self._wrap(yaw_odom + (1 - self.alpha) * d)

        # Odometry 메시지 복사 후 heading만 교체
        out = Odometry()
        out.header = msg.header
        out.child_frame_id = msg.child_frame_id
        out.pose = msg.pose
        out.twist = msg.twist

        out.pose.pose.orientation.x = 0.0
        out.pose.pose.orientation.y = 0.0
        out.pose.pose.orientation.z = math.sin(yaw_fused / 2)
        out.pose.pose.orientation.w = math.cos(yaw_fused / 2)

        self.pub.publish(out)

    @staticmethod
    def _wrap(a):
        return (a + math.pi) % (2 * math.pi) - math.pi


def main(args=None):
    rclpy.init(args=args)
    node = ImuOdometryNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
