#!/usr/bin/env python3
"""Ad-hoc diagnostic: subscribe to /car_state/odom and log (s, actual speed)
by projecting the vehicle's position onto track02_raceline_safe.csv, to
check whether the car actually achieves the planned speed_profile_node
target through the sharp s~12 corner (kappa up to 1.15 rad/m) at speed, or
whether it's overshooting it. Also subscribes to /ego_racecar/collision
and /planning/replan_state so that WHEN a collision happens, the exact
(s, kappa, speed, replan_state) at that instant is captured -- rather than
guessing the crash location from nearby unrelated log lines, which is what
had to be done earlier this session. Read-only, no changes to any running
node.

Run: python3 scripts/watch_speed_vs_s.py [duration_s]
Exits early (before duration_s) the moment a collision is reported.
"""
import csv
import math
import sys

import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Bool, String


def load_raceline(path):
    rows = list(csv.DictReader(open(path)))
    s = np.array([float(r['s_m']) for r in rows])
    xy = np.array([[float(r['x_m']), float(r['y_m'])] for r in rows])
    kappa = np.array([float(r['kappa_radpm']) for r in rows])
    return s, xy, kappa


class Watcher(Node):
    def __init__(self, s_arr, xy_arr, kappa_arr, duration_s):
        super().__init__('watch_speed_vs_s')
        self.s_arr = s_arr
        self.xy_arr = xy_arr
        self.kappa_arr = kappa_arr
        self.duration_s = duration_s
        self.start_time = None
        self.last_print = 0.0
        self.last_state = None
        self.replan_state = 'GLOBAL'
        self.create_subscription(Odometry, '/car_state/odom', self.odom_cb, 10)
        self.create_subscription(
            Bool, '/ego_racecar/collision', self.collision_cb, 10)
        self.create_subscription(
            String, '/planning/replan_state', self.replan_cb, 10)

    def replan_cb(self, msg):
        self.replan_state = msg.data

    def collision_cb(self, msg):
        if not msg.data:
            return
        now = self.get_clock().now().nanoseconds * 1.0e-9
        elapsed = now - self.start_time if self.start_time else 0.0
        if self.last_state is None:
            print(f'COLLISION at t={elapsed:.2f}s -- no odom sample yet')
        else:
            t_s, kappa, speed = self.last_state
            print(f'*** COLLISION at t={elapsed:.2f}s -- last known state: '
                  f's={t_s:.2f} kappa={kappa:.3f} speed={speed:.3f} m/s '
                  f'replan_state={self.replan_state} ***')
        rclpy.shutdown()

    def odom_cb(self, msg):
        now = self.get_clock().now().nanoseconds * 1.0e-9
        if self.start_time is None:
            self.start_time = now
        elapsed = now - self.start_time
        if elapsed > self.duration_s:
            print(f'no collision in {self.duration_s:.0f}s window')
            rclpy.shutdown()
            return

        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        vx = msg.twist.twist.linear.x
        vy = msg.twist.twist.linear.y
        speed = math.hypot(vx, vy)

        d2 = (self.xy_arr[:, 0] - x) ** 2 + (self.xy_arr[:, 1] - y) ** 2
        idx = int(np.argmin(d2))
        s_val = self.s_arr[idx]
        kappa = self.kappa_arr[idx]
        self.last_state = (s_val, kappa, speed)

        if now - self.last_print < 0.15:
            return
        self.last_print = now
        print(f't={elapsed:6.2f}s s={s_val:6.2f} kappa={kappa:7.3f} '
              f'speed={speed:.3f} m/s state={self.replan_state}')


def main():
    duration = float(sys.argv[1]) if len(sys.argv) > 1 else 40.0
    s_arr, xy_arr, kappa_arr = load_raceline(
        '/sim_ws/src/planning/waypoints/track02_raceline_safe.csv')
    rclpy.init()
    node = Watcher(s_arr, xy_arr, kappa_arr, duration)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass


if __name__ == '__main__':
    main()
