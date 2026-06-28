import csv
import math
import os
from collections import deque

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
from std_srvs.srv import SetBool, Trigger
from visualization_msgs.msg import Marker, MarkerArray


def _quat_to_yaw(q):
    return math.atan2(2.0 * q.w * q.z, 1.0 - 2.0 * q.z * q.z)


class WaypointRecorderNode(Node):
    def __init__(self):
        super().__init__('waypoint_recorder_node')

        self.declare_parameter('odom_topic', '/ego_racecar/odom')
        self.declare_parameter('output_csv', '/sim_ws/src/planning/waypoints/waypoints.csv')
        self.declare_parameter('min_distance', 0.2)
        self.declare_parameter('default_speed', 1.0)
        self.declare_parameter('use_actual_speed', True)
        self.declare_parameter('marker_topic', '/planning/record_markers')
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('autosave_interval', 30.0)
        self.declare_parameter('speed_smooth_window', 5)
        self.declare_parameter('loop_closure_distance', 0.5)
        self.declare_parameter('loop_closure_min_waypoints', 20)
        self.declare_parameter('autostart', True)

        self.odom_topic = self.get_parameter('odom_topic').value
        self.output_csv = self.get_parameter('output_csv').value
        self.min_distance = self.get_parameter('min_distance').value
        self.default_speed = self.get_parameter('default_speed').value
        self.use_actual_speed = self.get_parameter('use_actual_speed').value
        self.marker_topic = self.get_parameter('marker_topic').value
        self.frame_id = self.get_parameter('frame_id').value
        self.autosave_interval = self.get_parameter('autosave_interval').value
        self.speed_smooth_window = self.get_parameter('speed_smooth_window').value
        self.loop_closure_distance = self.get_parameter('loop_closure_distance').value
        self.loop_closure_min_waypoints = self.get_parameter('loop_closure_min_waypoints').value

        self.recording = self.get_parameter('autostart').value
        self.loop_closed = False
        self.recorded = []
        self.last_x = None
        self.last_y = None
        self._start_time = None
        self._total_dist = 0.0
        self._speed_buf = deque(maxlen=self.speed_smooth_window)

        self.sub = self.create_subscription(
            Odometry, self.odom_topic, self.odom_callback, 10
        )

        self.marker_pub = self.create_publisher(MarkerArray, self.marker_topic, 10)

        self.create_service(SetBool, '~/record', self._srv_record)
        self.create_service(Trigger, '~/save', self._srv_save)
        self.create_service(Trigger, '~/clear', self._srv_clear)

        if self.autosave_interval > 0.0:
            self.create_timer(self.autosave_interval, self._autosave)

        self.get_logger().info('waypoint_recorder_node started')
        self.get_logger().info(f'  odom topic            : {self.odom_topic}')
        self.get_logger().info(f'  output csv            : {self.output_csv}')
        self.get_logger().info(f'  min distance          : {self.min_distance} m')
        self.get_logger().info(f'  use actual speed      : {self.use_actual_speed}')
        self.get_logger().info(f'  speed smooth window   : {self.speed_smooth_window}')
        self.get_logger().info(f'  loop closure distance : {self.loop_closure_distance} m')
        self.get_logger().info(f'  autosave interval     : {self.autosave_interval} s')
        self.get_logger().info(f'  recording             : {self.recording}')
        self.get_logger().info('Services:')
        self.get_logger().info('  ~/record (SetBool) — true=record, false=pause')
        self.get_logger().info('  ~/save   (Trigger) — write CSV')
        self.get_logger().info('  ~/clear  (Trigger) — discard all waypoints')

    # ------------------------------------------------------------------ #
    # Odom
    # ------------------------------------------------------------------ #
    def odom_callback(self, msg):
        if not self.recording or self.loop_closed:
            return

        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        yaw = _quat_to_yaw(msg.pose.pose.orientation)
        vx = msg.twist.twist.linear.x

        if self._start_time is None:
            self._start_time = self.get_clock().now()

        if self.last_x is None:
            self._record(x, y, yaw, vx)
            return

        dist = math.hypot(x - self.last_x, y - self.last_y)
        if dist < self.min_distance:
            return

        self._total_dist += dist

        # Loop closure: car returned near start after enough waypoints
        if len(self.recorded) >= self.loop_closure_min_waypoints:
            sx, sy, _, _ = self.recorded[0]
            if math.hypot(x - sx, y - sy) <= self.loop_closure_distance:
                self.loop_closed = True
                self.recorded.append(self.recorded[0])  # close the loop exactly
                self.get_logger().info(
                    f'Loop closed! {len(self.recorded)} waypoints  '
                    f'{self._total_dist:.1f} m — call ~/save to write CSV.'
                )
                self._publish_markers()
                return

        self._record(x, y, yaw, vx)

    def _record(self, x, y, yaw, vx):
        self._speed_buf.append(abs(vx))
        smoothed = sum(self._speed_buf) / len(self._speed_buf)
        if smoothed < 0.05:
            smoothed = self.default_speed
        speed = smoothed if self.use_actual_speed else self.default_speed

        self.recorded.append((x, y, yaw, speed))
        self.last_x = x
        self.last_y = y

        self.get_logger().info(
            f'[{len(self.recorded):>4}] x={x:.3f}  y={y:.3f}  '
            f'yaw={math.degrees(yaw):+.1f}°  spd={speed:.2f}  dist={self._total_dist:.1f} m'
        )
        self._publish_markers()

    # ------------------------------------------------------------------ #
    # Services
    # ------------------------------------------------------------------ #
    def _srv_record(self, request, response):
        self.recording = request.data
        state = 'RECORDING' if self.recording else 'PAUSED'
        response.success = True
        response.message = state
        self.get_logger().info(f'[service/record] {state}')
        return response

    def _srv_save(self, request, response):
        if not self.recorded:
            response.success = False
            response.message = 'No waypoints to save.'
        else:
            self._write_csv(self.output_csv)
            response.success = True
            response.message = f'Saved {len(self.recorded)} waypoints -> {self.output_csv}'
        self.get_logger().info(f'[service/save] {response.message}')
        return response

    def _srv_clear(self, request, response):
        n = len(self.recorded)
        self.recorded.clear()
        self.last_x = None
        self.last_y = None
        self._total_dist = 0.0
        self._start_time = None
        self.loop_closed = False
        self._speed_buf.clear()
        response.success = True
        response.message = f'Cleared {n} waypoints.'
        self.get_logger().info(f'[service/clear] {response.message}')
        self._publish_markers()
        return response

    # ------------------------------------------------------------------ #
    # Markers
    # ------------------------------------------------------------------ #
    def _publish_markers(self):
        now = self.get_clock().now().to_msg()
        ma = MarkerArray()

        # Path line: orange while open, blue when loop closed
        line = Marker()
        line.header.stamp = now
        line.header.frame_id = self.frame_id
        line.ns = 'recorded_path'
        line.id = 0
        line.type = Marker.LINE_STRIP
        line.action = Marker.ADD
        line.scale.x = 0.05
        line.color.a = 1.0
        line.color.r = 0.0 if self.loop_closed else 1.0
        line.color.g = 0.0 if self.loop_closed else 0.5
        line.color.b = 1.0 if self.loop_closed else 0.0
        for x, y, _, _ in self.recorded:
            p = Point()
            p.x = x
            p.y = y
            p.z = 0.05
            line.points.append(p)
        ma.markers.append(line)

        # Start sphere (green)
        if self.recorded:
            sx, sy, _, _ = self.recorded[0]
            m = self._sphere(now, 1, sx, sy, r=0.0, g=1.0, b=0.0, scale=0.3)
            ma.markers.append(m)

        # Latest point sphere (red = recording, blue = loop closed)
        if len(self.recorded) > 1:
            lx, ly, _, _ = self.recorded[-1]
            m = self._sphere(
                now, 2, lx, ly,
                r=0.0 if self.loop_closed else 1.0,
                g=0.0,
                b=1.0 if self.loop_closed else 0.0,
                scale=0.2
            )
            ma.markers.append(m)

        self.marker_pub.publish(ma)

    def _sphere(self, stamp, mid, x, y, r, g, b, scale):
        m = Marker()
        m.header.stamp = stamp
        m.header.frame_id = self.frame_id
        m.ns = 'recorded_path'
        m.id = mid
        m.type = Marker.SPHERE
        m.action = Marker.ADD
        m.pose.position.x = x
        m.pose.position.y = y
        m.pose.position.z = 0.1
        m.pose.orientation.w = 1.0
        m.scale.x = m.scale.y = m.scale.z = scale
        m.color.a = 1.0
        m.color.r = r
        m.color.g = g
        m.color.b = b
        return m

    # ------------------------------------------------------------------ #
    # Save helpers
    # ------------------------------------------------------------------ #
    def _autosave(self):
        if not self.recorded:
            return
        backup = self.output_csv.replace('.csv', '_backup.csv')
        self._write_csv(backup)
        self.get_logger().info(
            f'[autosave] {len(self.recorded)} waypoints -> {backup}'
        )

    def save(self):
        if not self.recorded:
            self.get_logger().warn('No waypoints recorded. CSV not saved.')
            return
        self._write_csv(self.output_csv)
        elapsed = 0.0
        if self._start_time is not None:
            elapsed = (self.get_clock().now() - self._start_time).nanoseconds / 1e9
        self.get_logger().info(
            f'Saved {len(self.recorded)} waypoints -> {self.output_csv} '
            f'| dist={self._total_dist:.1f} m  elapsed={elapsed:.0f} s'
        )

    def _write_csv(self, path):
        os.makedirs(os.path.dirname(path), exist_ok=True)
        with open(path, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['x', 'y', 'yaw', 'speed'])
            for x, y, yaw, speed in self.recorded:
                writer.writerow([
                    round(x, 4), round(y, 4),
                    round(yaw, 6), round(speed, 4)
                ])


def main(args=None):
    rclpy.init(args=args)
    node = WaypointRecorderNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.save()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
