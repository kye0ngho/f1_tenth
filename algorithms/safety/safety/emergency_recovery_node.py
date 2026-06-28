import enum

import rclpy
from rclpy.node import Node

from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, String


class State(enum.Enum):
    NORMAL = 'NORMAL'
    DETECTING = 'DETECTING'
    REVERSING = 'REVERSING'
    TURNING = 'TURNING'
    RESUMING = 'RESUMING'


class EmergencyRecoveryNode(Node):
    """
    차량 고착(Stuck) 감지 및 자동 복구.

    감지 조건:
      - 명령 속도 > cmd_speed_threshold
      - 실제 속도 < stuck_speed_threshold
      - stuck_detect_time 초 이상 지속

    복구 시퀀스:
      REVERSING → reverse_speed로 reverse_time 초 후진
      TURNING   → turn_steer_angle로 turn_time 초 회전 (좌회전)
      RESUMING  → 0.5 초 서행 후 NORMAL 복귀

    출력: /drive 토픽에 직접 발행 (safety_brake보다 상위 우선순위)
    """

    def __init__(self):
        super().__init__('emergency_recovery_node')

        self.declare_parameter('odom_topic', '/ego_racecar/odom')
        self.declare_parameter('drive_input_topic', '/control/drive')
        self.declare_parameter('drive_output_topic', '/drive')
        self.declare_parameter('state_topic', '/emergency_recovery/state')
        self.declare_parameter('active_topic', '/emergency_recovery/active')

        self.declare_parameter('stuck_detect_time', 2.0)
        self.declare_parameter('stuck_speed_threshold', 0.05)
        self.declare_parameter('cmd_speed_threshold', 0.15)

        self.declare_parameter('reverse_speed', -0.5)
        self.declare_parameter('reverse_time', 1.5)
        self.declare_parameter('turn_steer_angle', 0.35)
        self.declare_parameter('turn_time', 1.0)
        self.declare_parameter('resume_time', 0.5)

        self.declare_parameter('check_rate', 20.0)

        odom_topic = self.get_parameter('odom_topic').value
        drive_in = self.get_parameter('drive_input_topic').value
        drive_out = self.get_parameter('drive_output_topic').value
        state_topic = self.get_parameter('state_topic').value
        active_topic = self.get_parameter('active_topic').value

        self.stuck_detect_time = float(self.get_parameter('stuck_detect_time').value)
        self.stuck_spd_thresh = float(self.get_parameter('stuck_speed_threshold').value)
        self.cmd_spd_thresh = float(self.get_parameter('cmd_speed_threshold').value)
        self.rev_speed = float(self.get_parameter('reverse_speed').value)
        self.rev_time = float(self.get_parameter('reverse_time').value)
        self.turn_steer = float(self.get_parameter('turn_steer_angle').value)
        self.turn_time = float(self.get_parameter('turn_time').value)
        self.resume_time = float(self.get_parameter('resume_time').value)
        rate = float(self.get_parameter('check_rate').value)

        # state
        self.state = State.NORMAL
        self.actual_speed = 0.0
        self.cmd_speed = 0.0
        self.cmd_steer = 0.0
        self.stuck_timer = 0.0
        self.phase_timer = 0.0
        self.passthrough_drive = None  # 마지막 /control/drive 메시지

        self.create_subscription(Odometry, odom_topic, self._cb_odom, 10)
        self.create_subscription(AckermannDriveStamped, drive_in, self._cb_drive, 10)
        self.drive_pub = self.create_publisher(AckermannDriveStamped, drive_out, 10)
        self.state_pub = self.create_publisher(String, state_topic, 10)
        self.active_pub = self.create_publisher(Bool, active_topic, 10)

        dt = 1.0 / rate
        self.dt = dt
        self.create_timer(dt, self._tick)

        self.get_logger().info('emergency_recovery_node started')
        self.get_logger().info(f'  stuck_detect_time : {self.stuck_detect_time}s')
        self.get_logger().info(f'  reverse_speed     : {self.rev_speed} m/s  for {self.rev_time}s')

    def _cb_odom(self, msg):
        self.actual_speed = msg.twist.twist.linear.x

    def _cb_drive(self, msg):
        self.cmd_speed = msg.drive.speed
        self.cmd_steer = msg.drive.steering_angle
        self.passthrough_drive = msg

    def _tick(self):
        if self.state == State.NORMAL:
            # 고착 감지
            if (abs(self.cmd_speed) > self.cmd_spd_thresh and
                    abs(self.actual_speed) < self.stuck_spd_thresh):
                self.stuck_timer += self.dt
                if self.stuck_timer >= self.stuck_detect_time:
                    self.get_logger().warn('[Recovery] 고착 감지! 복구 시작')
                    self.state = State.REVERSING
                    self.phase_timer = 0.0
                    self.stuck_timer = 0.0
            else:
                self.stuck_timer = 0.0
                # 정상: 입력 그대로 패스스루
                if self.passthrough_drive is not None:
                    self.drive_pub.publish(self.passthrough_drive)

        elif self.state == State.REVERSING:
            self.phase_timer += self.dt
            self._send_cmd(self.rev_speed, 0.0)
            if self.phase_timer >= self.rev_time:
                self.get_logger().info('[Recovery] 후진 완료 → 회전')
                self.state = State.TURNING
                self.phase_timer = 0.0

        elif self.state == State.TURNING:
            self.phase_timer += self.dt
            self._send_cmd(self.rev_speed * 0.5, self.turn_steer)
            if self.phase_timer >= self.turn_time:
                self.get_logger().info('[Recovery] 회전 완료 → 재시도')
                self.state = State.RESUMING
                self.phase_timer = 0.0

        elif self.state == State.RESUMING:
            self.phase_timer += self.dt
            self._send_cmd(0.3, 0.0)
            if self.phase_timer >= self.resume_time:
                self.get_logger().info('[Recovery] 복구 완료 → 정상 모드')
                self.state = State.NORMAL
                self.phase_timer = 0.0

        # 상태 퍼블리시
        s = String(); s.data = self.state.value
        self.state_pub.publish(s)
        b = Bool(); b.data = (self.state != State.NORMAL)
        self.active_pub.publish(b)

    def _send_cmd(self, speed, steer):
        msg = AckermannDriveStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.drive.speed = float(speed)
        msg.drive.steering_angle = float(steer)
        self.drive_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = EmergencyRecoveryNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
