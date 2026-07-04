import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped


class AmclBridgeNode(Node):
    """Bridges /amcl_pose → /localization/odom so the rest of the stack
    can consume a single Odometry topic regardless of localization backend."""

    def __init__(self):
        super().__init__('amcl_bridge_node')

        self.declare_parameter('amcl_pose_topic', '/amcl_pose')
        self.declare_parameter('raw_odom_topic', '/ego_racecar/odom')
        self.declare_parameter('output_odom_topic', '/localization/odom')
        self.declare_parameter('output_pose_topic', '/localization/pose')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('base_frame', 'base_link')

        amcl_topic = self.get_parameter('amcl_pose_topic').value
        raw_odom_topic = self.get_parameter('raw_odom_topic').value
        out_odom = self.get_parameter('output_odom_topic').value
        out_pose = self.get_parameter('output_pose_topic').value
        self._map_frame = self.get_parameter('map_frame').value
        self._base_frame = self.get_parameter('base_frame').value

        self._latest_amcl: PoseWithCovarianceStamped | None = None
        self._latest_raw: Odometry | None = None

        self.create_subscription(PoseWithCovarianceStamped, amcl_topic,
                                  self._amcl_cb, 10)
        self.create_subscription(Odometry, raw_odom_topic,
                                  self._raw_odom_cb, 10)

        self._pub_odom = self.create_publisher(Odometry, out_odom, 10)
        self._pub_pose = self.create_publisher(PoseStamped, out_pose, 10)

        self.get_logger().info(
            f'amcl_bridge: {amcl_topic} + {raw_odom_topic} → {out_odom}'
        )

    def _amcl_cb(self, msg: PoseWithCovarianceStamped):
        self._latest_amcl = msg
        self._publish()

    def _raw_odom_cb(self, msg: Odometry):
        # Cache velocity from raw odom; AMCL doesn't provide twist
        self._latest_raw = msg

    def _publish(self):
        if self._latest_amcl is None:
            return

        stamp = self._latest_amcl.header.stamp

        # Odometry with AMCL pose + raw odom twist
        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = self._map_frame
        odom.child_frame_id = self._base_frame
        odom.pose.pose = self._latest_amcl.pose.pose
        odom.pose.covariance = self._latest_amcl.pose.covariance
        if self._latest_raw is not None:
            odom.twist = self._latest_raw.twist
        self._pub_odom.publish(odom)

        # Convenience PoseStamped
        pose = PoseStamped()
        pose.header.stamp = stamp
        pose.header.frame_id = self._map_frame
        pose.pose = self._latest_amcl.pose.pose
        self._pub_pose.publish(pose)


def main(args=None):
    rclpy.init(args=args)
    node = AmclBridgeNode()
    rclpy.spin(node)
    rclpy.shutdown()
