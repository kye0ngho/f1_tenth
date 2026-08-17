import math

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener


class LocalizedOdomNode(Node):
    """Publish controller-friendly odometry from map-frame localization.

    Nav2 AMCL publishes /amcl_pose and the map->odom TF, while this repo's
    controllers consume nav_msgs/Odometry. This node combines the latest
    map-frame TF pose with the latest wheel/sim odometry twist and republishes
    the result as /car_state/odom.
    """

    def __init__(self):
        super().__init__('localized_odom_node')

        self.declare_parameter('source_odom_topic', '/ego_racecar/odom')
        self.declare_parameter('amcl_pose_topic', '/amcl_pose')
        self.declare_parameter('output_odom_topic', '/car_state/odom')
        self.declare_parameter('global_frame_id', 'map')
        self.declare_parameter('base_frame_id', 'ego_racecar/base_link')
        self.declare_parameter('publish_rate', 50.0)
        self.declare_parameter('odom_timeout', 0.50)
        self.declare_parameter('amcl_timeout', 2.00)
        self.declare_parameter('require_amcl_pose', False)

        self.source_odom_topic = self.get_parameter(
            'source_odom_topic').value
        self.amcl_pose_topic = self.get_parameter('amcl_pose_topic').value
        self.output_odom_topic = self.get_parameter(
            'output_odom_topic').value
        self.global_frame_id = self.get_parameter('global_frame_id').value
        self.base_frame_id = self.get_parameter('base_frame_id').value
        self.publish_rate = float(self.get_parameter('publish_rate').value)
        self.odom_timeout = float(self.get_parameter('odom_timeout').value)
        self.amcl_timeout = float(self.get_parameter('amcl_timeout').value)
        self.require_amcl_pose = bool(
            self.get_parameter('require_amcl_pose').value)

        if self.publish_rate <= 0.0:
            raise RuntimeError('publish_rate must be positive')

        self.current_odom = None
        self.current_amcl_pose = None
        self.last_odom_time = None
        self.last_amcl_time = None

        self.tf_buffer = Buffer(cache_time=Duration(seconds=5.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.create_subscription(
            Odometry, self.source_odom_topic, self.odom_callback, 10)
        self.create_subscription(
            PoseWithCovarianceStamped,
            self.amcl_pose_topic,
            self.amcl_pose_callback,
            10)
        self.odom_pub = self.create_publisher(
            Odometry, self.output_odom_topic, 10)

        self.timer = self.create_timer(
            1.0 / self.publish_rate, self.publish_localized_odom)

        self.get_logger().info(
            'localized_odom_node: %s + %s -> %s (%s frame)'
            % (self.source_odom_topic, self.amcl_pose_topic,
               self.output_odom_topic, self.global_frame_id))

    def odom_callback(self, msg):
        self.current_odom = msg
        self.last_odom_time = self.get_clock().now()

    def amcl_pose_callback(self, msg):
        self.current_amcl_pose = msg
        self.last_amcl_time = self.get_clock().now()

    def _age(self, stamp):
        if stamp is None:
            return math.inf
        return (self.get_clock().now() - stamp).nanoseconds * 1.0e-9

    def publish_localized_odom(self):
        if self.current_odom is None:
            return
        if self._age(self.last_odom_time) > self.odom_timeout:
            self.get_logger().warn(
                'source odometry is stale; not publishing localized odom',
                throttle_duration_sec=1.0)
            return
        if self.require_amcl_pose \
                and self._age(self.last_amcl_time) > self.amcl_timeout:
            self.get_logger().warn(
                'AMCL pose is stale; not publishing localized odom',
                throttle_duration_sec=1.0)
            return

        try:
            transform = self.tf_buffer.lookup_transform(
                self.global_frame_id,
                self.base_frame_id,
                Time())
        except TransformException as error:
            self.get_logger().warn(
                'localized odom TF lookup failed: ' + str(error),
                throttle_duration_sec=1.0)
            return

        message = Odometry()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = self.global_frame_id
        message.child_frame_id = self.base_frame_id

        translation = transform.transform.translation
        rotation = transform.transform.rotation
        message.pose.pose.position.x = float(translation.x)
        message.pose.pose.position.y = float(translation.y)
        message.pose.pose.position.z = float(translation.z)
        message.pose.pose.orientation = rotation

        if self.current_amcl_pose is not None:
            message.pose.covariance = self.current_amcl_pose.pose.covariance
        else:
            message.pose.covariance = self.current_odom.pose.covariance

        message.twist = self.current_odom.twist
        self.odom_pub.publish(message)


def main(args=None):
    rclpy.init(args=args)
    node = LocalizedOdomNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
