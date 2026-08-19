#!/usr/bin/env python3
"""Ad-hoc diagnostic: log every pose in /planning/detected_obstacles
(scan_obstacle_detector_node's raw clustered output, before
local_avoidance_planner_node's EMA/reset/confirm-gate ever touches it),
projected onto track02_raceline_safe.csv for (s, lateral). Goal: see
directly how much the raw cluster position jitters frame-to-frame for
what should be the same stationary obstacle, to find out whether it's a
small (~obstacle radius) viewing-angle bias or a much larger jump
consistent with the cluster boundary merging with nearby wall points.
Read-only, no changes to any running node.

Run: python3 scripts/watch_detected_obstacles.py [duration_s]
"""
import csv
import math
import sys

import numpy as np
import rclpy
from geometry_msgs.msg import PoseArray
from rclpy.node import Node


def load_raceline(path):
    rows = list(csv.DictReader(open(path)))
    s = np.array([float(r['s_m']) for r in rows])
    xy = np.array([[float(r['x_m']), float(r['y_m'])] for r in rows])
    yaw = np.array([float(r['psi_rad']) for r in rows])
    return s, xy, yaw


def nearest_s_lateral(x, y, s_arr, xy_arr, yaw_arr):
    d2 = (xy_arr[:, 0] - x) ** 2 + (xy_arr[:, 1] - y) ** 2
    idx = int(np.argmin(d2))
    yaw = yaw_arr[idx]
    dx = x - xy_arr[idx, 0]
    dy = y - xy_arr[idx, 1]
    lateral = -dx * math.sin(yaw) + dy * math.cos(yaw)
    return s_arr[idx], lateral


class Watcher(Node):
    def __init__(self, s_arr, xy_arr, yaw_arr, duration_s):
        super().__init__('watch_detected_obstacles')
        self.s_arr = s_arr
        self.xy_arr = xy_arr
        self.yaw_arr = yaw_arr
        self.duration_s = duration_s
        self.start_time = None
        self.create_subscription(
            PoseArray, '/planning/detected_obstacles', self.cb, 10)

    def cb(self, msg):
        now = self.get_clock().now().nanoseconds * 1.0e-9
        if self.start_time is None:
            self.start_time = now
        elapsed = now - self.start_time
        if elapsed > self.duration_s:
            print(f'done at {elapsed:.1f}s')
            rclpy.shutdown()
            return
        if not msg.poses:
            return
        parts = []
        for pose in msg.poses:
            x = pose.position.x
            y = pose.position.y
            s_val, lateral = nearest_s_lateral(
                x, y, self.s_arr, self.xy_arr, self.yaw_arr)
            parts.append(f'(x={x:.2f},y={y:.2f} s={s_val:.2f} lat={lateral:.2f})')
        print(f't={elapsed:6.2f}s n={len(msg.poses)} ' + ' '.join(parts))


def main():
    duration = float(sys.argv[1]) if len(sys.argv) > 1 else 30.0
    s_arr, xy_arr, yaw_arr = load_raceline(
        '/sim_ws/src/planning/waypoints/track02_raceline_safe.csv')
    rclpy.init()
    node = Watcher(s_arr, xy_arr, yaw_arr, duration)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass


if __name__ == '__main__':
    main()
