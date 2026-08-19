#!/usr/bin/env python3
"""Focused, non-ROS-graph unit test for the 2026-08-19 live-scan clearance
refactor in local_avoidance_planner_node.py (_point_scan_clearance,
_score_candidate).

Replaces distance-to-estimated-obstacle-centroid scoring with a direct
check against the current raw LaserScan, since the centroid produced by
scan_obstacle_detector_node's clustering has a real geometric bias that
drifts as the viewing angle changes (confirmed live: filtered_s drifted
6.00->5.03 m and target_d drifted 0.32->0.47 m in under a second, right
before a 3.0 m/s collision this same session).

_point_scan_clearance went through two designs this session:
  v1 (ray occlusion): is there a closer LaserScan return along the exact
      bearing from the sensor to the candidate point. Live-tested at
      2.0 m/s and found broken: a candidate point can be genuinely clear
      in its own neighborhood but share a bearing with a nearer wall
      corner from the sensor's single vantage point, reading as falsely
      blocked -- confirmed at track02's obs_s~6.7 corner, obstacle_margin
      pinned near the clamp floor on ~160/160 avoidance ticks over a 3
      minute run despite the pre-refactor centroid-distance approach
      reading sensible positive margins at the identical encounter.
  v2 (nearest-return-point distance, current): distance from the
      candidate point to the nearest live scan return point in the map
      frame, direction-independent -- mirrors _point_wall_clearance's
      nearest-occupied-cell search against the static map, but against
      live scan returns. Test 5 below reproduces the v1 failure
      geometrically and asserts v2 gets it right.

Feeds synthetic scan state directly into the node's cached fields (no
real LaserScan message or TF lookup needed).

Run inside the ROS2 environment (host rclpy jazzy or container humble,
either works, no simulator required):
    python3 scripts/focused_test_scan_clearance.py
"""
import math
import sys
from pathlib import Path

import numpy as np
import rclpy
from rclpy.time import Time
from rclpy.duration import Duration

sys.path.insert(
    0, str(Path(__file__).resolve().parent.parent
           / 'algorithms' / 'planning'))

from planning.local_avoidance_planner_node import LocalAvoidancePlannerNode  # noqa: E402
from planning.path_utils import ClosedPath  # noqa: E402


def make_straight_path(node, length_m=20.0, spacing_m=0.1):
    n = int(length_m / spacing_m)
    xs = np.arange(n) * spacing_m
    ys = np.zeros(n)
    points = np.stack([xs, ys], axis=1)
    path = ClosedPath()
    path.points = points
    path.yaw = np.zeros(n)
    path.curvature = np.zeros(n)
    path.segment_lengths = np.full(n, spacing_m)
    path.cumulative = np.arange(n + 1) * spacing_m
    path.length = float(path.cumulative[-1])
    node.path = path


class FakeClock:
    """Holds a fixed time between explicit tick() calls -- see
    focused_test_avoidance_slew.py for why (multiple get_clock().now()
    calls per invocation would otherwise make effective dt depend on
    incidental call counts)."""

    def __init__(self, dt_s):
        self.dt = dt_s
        self._now = Time(seconds=1000)

    def tick(self):
        self._now = self._now + Duration(seconds=self.dt)

    def now(self):
        return self._now


def make_obstacle(obs_s=5.0, obs_x=5.0, obs_y=0.0, lateral=0.0, path_yaw=0.0):
    return (0.0, obs_x, obs_y, obs_s, lateral, path_yaw)


def set_scan_points(node, xs, ys, fresh_time=None):
    node.scan_points_x = np.asarray(xs, dtype=float)
    node.scan_points_y = np.asarray(ys, dtype=float)
    node.last_scan_time = fresh_time


def run():
    rclpy.init()
    node = LocalAvoidancePlannerNode()
    make_straight_path(node, length_m=20.0)
    node._point_wall_clearance = lambda x, y: 5.0

    dt = 0.05
    fake_clock = FakeClock(dt)
    node.get_clock = lambda: fake_clock

    failures = []

    # --- Test 1: a return point very close to the candidate -> blocked. ---
    fake_clock.tick()
    set_scan_points(node, [1.0], [0.0], fresh_time=fake_clock.now())
    clearance = node._point_scan_clearance(1.05, 0.0)
    margin = clearance - node.scan_obstacle_clearance
    print(f'Test 1 (return point 0.05m from candidate): clearance={clearance:.3f}, '
          f'margin={margin:.3f} (expect margin < 0)')
    if not margin < 0.0:
        failures.append(
            'Test 1 FAILED: a candidate 0.05m from an actual return point '
            'did not clear as blocked')

    # --- Test 2: nearest return point is far away -> clear. ---
    set_scan_points(node, [5.0], [0.0], fresh_time=fake_clock.now())
    clearance = node._point_scan_clearance(1.0, 0.0)
    margin = clearance - node.scan_obstacle_clearance
    print(f'Test 2 (nearest return 4.0m away): clearance={clearance:.3f}, '
          f'margin={margin:.3f} (expect margin > 0)')
    if not margin > 0.0:
        failures.append(
            'Test 2 FAILED: a candidate far from every return point did '
            'not clear the required scan_obstacle_clearance_m buffer')

    # --- Test 3: stale scan degrades to the neutral fallback, regardless
    # of how close/far the cached return points would otherwise say. ---
    set_scan_points(node, [1.05], [0.0], fresh_time=fake_clock.now())
    for _ in range(10):  # 10 * dt=0.05s = 0.5s, past scan_stale_timeout_s=0.25
        fake_clock.tick()
    stale = node._point_scan_clearance(1.0, 0.0)
    print(f'Test 3 (stale scan, 0.5s old, would-be-blocked point): '
          f'clearance={stale:.3f} '
          f'(expect == scan_obstacle_clearance={node.scan_obstacle_clearance:.3f})')
    if not math.isclose(stale, node.scan_obstacle_clearance):
        failures.append(
            'Test 3 FAILED: stale scan did not degrade to the neutral '
            'fallback value')

    # --- Test 4: fresh scan with zero valid returns (nothing detected
    # anywhere) -> same neutral fallback, not an overclaimed "fully clear". ---
    fake_clock.tick()
    set_scan_points(node, [], [], fresh_time=fake_clock.now())
    empty = node._point_scan_clearance(1.0, 0.0)
    print(f'Test 4 (fresh scan, zero returns): clearance={empty:.3f} '
          f'(expect == scan_obstacle_clearance={node.scan_obstacle_clearance:.3f})')
    if not math.isclose(empty, node.scan_obstacle_clearance):
        failures.append(
            'Test 4 FAILED: an empty-but-fresh scan did not return the '
            'neutral fallback value')

    # --- Test 5: the exact v1 (ray-occlusion) failure mode, reproduced.
    # Sensor at the map origin. A wall-corner return sits at (1.0, 0.5),
    # collinear with and closer than a candidate point at (2.0, 1.0) --
    # same bearing from the sensor, so a ray-occlusion check would have
    # read the candidate as deeply blocked (beam terminates well short of
    # it). The candidate's true nearest neighbor is that same corner
    # return, at true 2D distance ~1.12m -- clearly clear. ---
    fake_clock.tick()
    set_scan_points(node, [1.0], [0.5], fresh_time=fake_clock.now())
    clearance = node._point_scan_clearance(2.0, 1.0)
    margin = clearance - node.scan_obstacle_clearance
    print(f'Test 5 (corner-grazing regression): clearance={clearance:.3f}, '
          f'margin={margin:.3f} (expect margin > 0 -- a v1 ray-occlusion '
          f'check would have scored this deeply negative)')
    if not margin > 0.0:
        failures.append(
            'Test 5 FAILED: a candidate collinear with but well past a '
            'nearby corner return scored as blocked -- the ray-occlusion '
            'blind spot found live on 2026-08-19 has resurfaced')

    # --- Test 6: end-to-end, real (non-mocked) _score_candidate/
    # _point_scan_clearance, all-clear scan, across ~20 ticks -- assert
    # zero spurious side flips. ---
    node.smoothed_target_lateral = None
    node.smoothed_target_time = None
    node.committed_side = None
    node.filtered_obstacle = None
    node.pending_obstacle = None
    obstacle = make_obstacle(obs_s=5.0, obs_x=5.0, obs_y=0.0)
    sides_seen = []
    for _ in range(20):
        fake_clock.tick()
        set_scan_points(node, [20.0], [20.0], fresh_time=fake_clock.now())
        _points, side, blocked = node._build_replanned_points(obstacle, 0.0)
        sides_seen.append(None if blocked else side)
    flips = sum(
        1 for a, b in zip(sides_seen, sides_seen[1:])
        if a is not None and b is not None and a != b)
    print(f'Test 6 (end-to-end, all-clear scan, 20 ticks): sides={sides_seen}, '
          f'flips={flips} (expect 0)')
    if flips > 0:
        failures.append(
            'Test 6 FAILED: real scoring path flipped sides %d times over '
            '20 ticks with an all-clear scan and a stationary obstacle '
            'anchor' % flips)

    rclpy.shutdown()

    if failures:
        print('\n'.join(['', 'FAILURES:'] + failures))
        return 1
    print('\nAll focused scan-clearance checks passed.')
    return 0


if __name__ == '__main__':
    sys.exit(run())
