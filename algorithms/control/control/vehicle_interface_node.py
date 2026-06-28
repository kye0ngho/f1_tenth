import math

import rclpy
from rclpy.node import Node

from ackermann_msgs.msg import AckermannDriveStamped
from std_msgs.msg import Float64, Bool
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Joy


class VehicleInterfaceNode(Node):
    """
    실차 VESC 인터페이스.
    /drive (AckermannDriveStamped) → /commands/motor/speed + /commands/servo/position

    모드:
      autonomous : /drive 명령을 그대로 변환
      manual     : /joy 조이스틱 입력을 직접 변환
      e-stop     : 모든 출력 0

    안전:
      - 조이스틱 데드맨 버튼(버튼 4) 누르는 동안만 자율주행
      - 버튼 5 = 비상정지
      - max_speed_real 파라미터로 최대 속도 제한
    """

    def __init__(self):
        super().__init__('vehicle_interface_node')

        self.declare_parameter('drive_topic', '/drive')
        self.declare_parameter('joy_topic', '/joy')
        self.declare_parameter('motor_speed_topic', '/commands/motor/speed')
        self.declare_parameter('servo_topic', '/commands/servo/position')
        self.declare_parameter('estop_topic', '/vehicle/estop')

        self.declare_parameter('speed_to_erpm_gain', 3000.0)
        self.declare_parameter('speed_to_erpm_offset', 0.0)
        self.declare_parameter('servo_center', 0.5)
        self.declare_parameter('servo_gain', -0.5)
        self.declare_parameter('servo_min', 0.15)
        self.declare_parameter('servo_max', 0.85)
        self.declare_parameter('max_speed_real', 2.0)
        self.declare_parameter('max_steering', 0.4189)

        self.declare_parameter('deadman_button', 4)
        self.declare_parameter('estop_button', 5)
        self.declare_parameter('joy_speed_axis', 1)
        self.declare_parameter('joy_steer_axis', 3)
        self.declare_parameter('joy_max_speed', 1.5)

        drive_topic = self.get_parameter('drive_topic').value
        joy_topic = self.get_parameter('joy_topic').value
        motor_topic = self.get_parameter('motor_speed_topic').value
        servo_topic = self.get_parameter('servo_topic').value
        estop_topic = self.get_parameter('estop_topic').value

        self.erpm_gain = float(self.get_parameter('speed_to_erpm_gain').value)
        self.erpm_offset = float(self.get_parameter('speed_to_erpm_offset').value)
        self.servo_center = float(self.get_parameter('servo_center').value)
        self.servo_gain = float(self.get_parameter('servo_gain').value)
        self.servo_min = float(self.get_parameter('servo_min').value)
        self.servo_max = float(self.get_parameter('servo_max').value)
        self.max_speed = float(self.get_parameter('max_speed_real').value)
        self.max_steer = float(self.get_parameter('max_steering').value)

        self.deadman_btn = int(self.get_parameter('deadman_button').value)
        self.estop_btn = int(self.get_parameter('estop_button').value)
        self.joy_spd_ax = int(self.get_parameter('joy_speed_axis').value)
        self.joy_str_ax = int(self.get_parameter('joy_steer_axis').value)
        self.joy_max_spd = float(self.get_parameter('joy_max_speed').value)

        self.autonomous = False
        self.estop = False

        self.create_subscription(AckermannDriveStamped, drive_topic, self._cb_drive, 10)
        self.create_subscription(Joy, joy_topic, self._cb_joy, 10)

        self.motor_pub = self.create_publisher(Float64, motor_topic, 10)
        self.servo_pub = self.create_publisher(Float64, servo_topic, 10)
        self.estop_pub = self.create_publisher(Bool, estop_topic, 10)

        self.get_logger().info('vehicle_interface_node started')
        self.get_logger().info('  실차 모드 — VESC 인터페이스 활성화')
        self.get_logger().info(f'  max_speed : {self.max_speed} m/s')
        self.get_logger().info(f'  deadman   : 버튼 {self.deadman_btn}')

    def _cb_joy(self, msg):
        buttons = list(msg.buttons)

        # 비상정지 버튼
        if len(buttons) > self.estop_btn and buttons[self.estop_btn]:
            if not self.estop:
                self.get_logger().warn('[Interface] 비상정지!')
            self.estop = True
            self._publish_stop()
            b = Bool(); b.data = True
            self.estop_pub.publish(b)
            return

        # 데드맨 버튼
        if len(buttons) > self.deadman_btn:
            self.autonomous = bool(buttons[self.deadman_btn])

        if not self.autonomous:
            # 수동 조이스틱 제어
            axes = list(msg.axes)
            speed = 0.0
            steer = 0.0
            if len(axes) > self.joy_spd_ax:
                speed = float(axes[self.joy_spd_ax]) * self.joy_max_spd
            if len(axes) > self.joy_str_ax:
                steer = float(axes[self.joy_str_ax]) * self.max_steer
            self._send(speed, steer)

    def _cb_drive(self, msg):
        if self.estop or not self.autonomous:
            return
        speed = float(msg.drive.speed)
        steer = float(msg.drive.steering_angle)
        speed = max(-self.max_speed, min(self.max_speed, speed))
        steer = max(-self.max_steer, min(self.max_steer, steer))
        self._send(speed, steer)

    def _send(self, speed, steer):
        erpm = speed * self.erpm_gain + self.erpm_offset
        servo = self.servo_center + self.servo_gain * steer
        servo = max(self.servo_min, min(self.servo_max, servo))

        m = Float64(); m.data = float(erpm)
        self.motor_pub.publish(m)

        s = Float64(); s.data = float(servo)
        self.servo_pub.publish(s)

    def _publish_stop(self):
        m = Float64(); m.data = 0.0
        self.motor_pub.publish(m)
        s = Float64(); s.data = self.servo_center
        self.servo_pub.publish(s)


def main(args=None):
    rclpy.init(args=args)
    node = VehicleInterfaceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
