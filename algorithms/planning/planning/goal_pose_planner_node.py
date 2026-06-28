import math

import numpy as np

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import String
from std_srvs.srv import Trigger


class GoalPosePlannerNode(Node):
    """
    목표 위치(Goal Pose)까지 직선 또는 웨이포인트 보간 경로를 생성.
    /goal_pose 토픽 또는 ~/set_goal 서비스로 목표 설정.
    도달 반경 내 진입 시 ~/goal_reached 이벤트 → /goal_planner/status 퍼블리시.

    현재 위치 → 목표 위치를 직선 분할한 Path를 /planning/goal_path로 퍼블리시.
    behavior_selector 또는 pure_pursuit에서 이 path를 사용하도록 전환 가능.
    """

    def __init__(self):
        super().__init__('goal_pose_planner_node')

        self.declare_parameter('odom_topic', '/ego_racecar/odom')
        self.declare_parameter('goal_topic', '/goal_pose')
        self.declare_parameter('path_topic', '/planning/goal_path')
        self.declare_parameter('status_topic', '/goal_planner/status')
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('num_points', 50)
        self.declare_parameter('goal_radius', 0.3)
        self.declare_parameter('publish_rate', 5.0)

        odom_topic = self.get_parameter('odom_topic').value
        goal_topic = self.get_parameter('goal_topic').value
        path_topic = self.get_parameter('path_topic').value
        status_topic = self.get_parameter('status_topic').value
        self.frame_id = self.get_parameter('frame_id').value
        self.num_points = int(self.get_parameter('num_points').value)
        self.goal_radius = float(self.get_parameter('goal_radius').value)
        rate = float(self.get_parameter('publish_rate').value)

        self.current_x = 0.0
        self.current_y = 0.0
        self.goal = None       # (x, y, yaw)
        self.reached = False

        self.create_subscription(Odometry, odom_topic, self._cb_odom, 10)
        self.create_subscription(PoseStamped, goal_topic, self._cb_goal, 10)

        self.path_pub = self.create_publisher(Path, path_topic, 10)
        self.status_pub = self.create_publisher(String, status_topic, 10)

        self.create_service(Trigger, '~/cancel', self._srv_cancel)

        self.create_timer(1.0 / rate, self._publish)

        self.get_logger().info('goal_pose_planner_node started')
        self.get_logger().info(f'  goal_topic  : {goal_topic}')
        self.get_logger().info(f'  goal_radius : {self.goal_radius} m')

    def _cb_odom(self, msg):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        if self.goal and not self.reached:
            dist = math.hypot(self.current_x - self.goal[0], self.current_y - self.goal[1])
            if dist < self.goal_radius:
                self.reached = True
                self.get_logger().info(f'[GoalPlanner] 목표 도달! dist={dist:.2f}m')

    def _cb_goal(self, msg):
        gx = msg.pose.position.x
        gy = msg.pose.position.y
        q = msg.pose.orientation
        gyaw = math.atan2(2 * q.w * q.z, 1 - 2 * q.z ** 2)
        self.goal = (gx, gy, gyaw)
        self.reached = False
        self.get_logger().info(f'[GoalPlanner] 새 목표: ({gx:.2f}, {gy:.2f})')

    def _publish(self):
        status = String()
        if self.goal is None:
            status.data = 'IDLE'
            self.status_pub.publish(status)
            return

        if self.reached:
            status.data = 'REACHED'
            self.status_pub.publish(status)
            return

        # 현재 위치 → 목표까지 선형 보간
        gx, gy, gyaw = self.goal
        xs = np.linspace(self.current_x, gx, self.num_points)
        ys = np.linspace(self.current_y, gy, self.num_points)

        path = Path()
        path.header.stamp = self.get_clock().now().to_msg()
        path.header.frame_id = self.frame_id

        for i in range(len(xs)):
            ps = PoseStamped()
            ps.header = path.header
            ps.pose.position.x = float(xs[i])
            ps.pose.position.y = float(ys[i])
            ps.pose.position.z = 0.0
            # 마지막 포인트는 목표 yaw, 나머지는 진행 방향
            if i < len(xs) - 1:
                seg_yaw = math.atan2(ys[i + 1] - ys[i], xs[i + 1] - xs[i])
            else:
                seg_yaw = gyaw
            ps.pose.orientation.z = math.sin(seg_yaw / 2)
            ps.pose.orientation.w = math.cos(seg_yaw / 2)
            path.poses.append(ps)

        self.path_pub.publish(path)

        dist = math.hypot(self.current_x - gx, self.current_y - gy)
        status.data = f'NAVIGATING dist={dist:.2f}m'
        self.status_pub.publish(status)

    def _srv_cancel(self, _, response):
        self.goal = None
        self.reached = False
        response.success = True
        response.message = '목표 취소됨'
        self.get_logger().info('[GoalPlanner] 목표 취소')
        return response


def main(args=None):
    rclpy.init(args=args)
    node = GoalPosePlannerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
