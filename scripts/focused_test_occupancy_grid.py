#!/usr/bin/env python3
"""Focused, non-ROS-graph unit test for the 2026-08-19 refactor that
replaces the fragmenting tracked-obstacle pipeline (_select_obstacle /
_stabilize_obstacle, driven by scan_obstacle_detector_node's clustering)
with a stateless live occupancy grid + line-of-sight check, adapted from
stanley_avoidance.py (refer/f1tenth_ws/src/stanley_avoidance/
stanley_avoidance/stanley_avoidance.py).

Root cause being fixed: a single real obstacle's LiDAR cluster was
fragmenting into 2-4 simultaneous separate detections at close range
(confirmed live: s hopping 5.6<->7.4 within under a second in
/planning/detected_obstacles), because scan_obstacle_detector_node's
range-jump clustering splits a close curved object at its silhouette
edges. _select_obstacle/_stabilize_obstacle then surfed between whichever
fragment won each tick's sort, injecting a discontinuous target_lateral
that caused a real collision during active avoidance (128s into a 3.0 m/s
trial). _first_blocked_point recomputes the obstacle's arc-length fresh
every tick straight from the live scan (scanning forward along the actual
path and returning the s of the first point whose clearance is too
tight) -- nothing about it depends on how a detector would have
clustered the same points, so it cannot be affected by fragmentation.
Test 5 below is the direct regression test for this.

This went through two live-tested design iterations before landing here:
v1 anchored the ramp at a FIXED lookahead point (ego_s + interest_horizon)
and used one straight-line collision check to it -- that straight chord
cut across corners and clipped walls the real curving raceline safely
avoids, latching the car into permanent BLOCKED at the first corner
(confirmed live: car frozen at its start pose for 15s+). Fixing the
straight-line issue alone still left the ramp anchored at the wrong s (a
fixed lookahead point, not the obstacle's real location), so the ramp's
full-strength zone didn't line up with the actual obstacle and candidate
points barely deviated regardless of target_lateral (confirmed live:
obs~0.02m on every candidate, unchanging). _first_blocked_point fixes
both by finding the real blocking location from live data every tick.

Run inside the ROS2 environment (host rclpy jazzy or container humble,
either works, no simulator required):
    python3 scripts/focused_test_occupancy_grid.py
"""
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
from planning.occupancy_grid_utils import LocalOccupancyGrid, traverse_grid  # noqa: E402
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
    def __init__(self, dt_s):
        self.dt = dt_s
        self._now = Time(seconds=1000)

    def tick(self):
        self._now = self._now + Duration(seconds=self.dt)

    def now(self):
        return self._now


def set_scan(node, xs, ys, fresh_time):
    node.scan_points_x = np.asarray(xs, dtype=float)
    node.scan_points_y = np.asarray(ys, dtype=float)
    node.last_scan_time = fresh_time


def run():
    failures = []

    # --- Tests 1-2: occupancy_grid_utils primitives in isolation. ---
    print(traverse_grid((0, 0), (3, 0)))
    grid = LocalOccupancyGrid(0.0, 0.0, resolution_m=0.05, radius_m=5.0)
    grid.populate_from_world_points([1.5], [0.0], inflate_radius_m=0.2)
    blocked_on_path = grid.check_collision((0.0, 0.0), (3.0, 0.0), margin_m=0.0)
    print(f'Test 1 (grid: obstacle directly on segment): blocked={blocked_on_path} '
          f'(expect True)')
    if not blocked_on_path:
        failures.append('Test 1 FAILED: obstacle directly on the checked segment '
                         'was not detected as blocking')

    grid2 = LocalOccupancyGrid(0.0, 0.0, resolution_m=0.05, radius_m=5.0)
    grid2.populate_from_world_points([1.5], [0.25], inflate_radius_m=0.05)
    clear_no_margin = not grid2.check_collision((0.0, 0.0), (3.0, 0.0), margin_m=0.0)
    blocked_with_margin = grid2.check_collision((0.0, 0.0), (3.0, 0.0), margin_m=0.3)
    print(f'Test 2 (grid: perpendicular margin): clear@margin=0 -> {clear_no_margin}, '
          f'blocked@margin=0.3 -> {blocked_with_margin} (expect both True)')
    if not (clear_no_margin and blocked_with_margin):
        failures.append('Test 2 FAILED: perpendicular-margin sweep did not behave '
                         'as expected for an off-axis obstacle point')

    # --- Node-level tests. ---
    rclpy.init()
    node = LocalAvoidancePlannerNode()
    make_straight_path(node, length_m=20.0)
    node._point_wall_clearance = lambda x, y: 5.0

    dt = 0.05
    fake_clock = FakeClock(dt)
    node.get_clock = lambda: fake_clock
    fake_clock.tick()

    ego_x, ego_y, ego_s = 0.0, 0.0, 0.0
    obstacle_s = 3.0  # world x == s on this straight path

    # --- Test 3: gate -- obstacle ahead on the raceline is found and its
    # arc-length is returned; an obstacle far off to the side is not. ---
    set_scan(node, [obstacle_s], [0.0], fake_clock.now())
    node._active_grid = node._build_local_grid(ego_x, ego_y)
    found_s = node._first_blocked_point(ego_s)
    # _first_blocked_point finds where the corridor STARTS being too
    # tight, scanning forward -- that's up to scan_obstacle_clearance
    # (~0.235m) before the obstacle's own true center, by design (it's a
    # "where does this stop being safe" query, not "where is the
    # obstacle's centroid"), plus up to one path-point spacing (0.1m)
    # of grid-snap. 0.4m tolerance covers both with margin.
    print(f'Test 3a (obstacle on raceline ahead): found_s={found_s} '
          f'(expect <= {obstacle_s}, within ~0.4m of it)')
    if found_s is None or not (obstacle_s - 0.4 <= found_s <= obstacle_s + 0.05):
        failures.append('Test 3a FAILED: obstacle directly ahead on the raceline '
                         'was not found at roughly its true s')

    set_scan(node, [obstacle_s], [3.0], fake_clock.now())
    node._active_grid = node._build_local_grid(ego_x, ego_y)
    clear = node._first_blocked_point(ego_s) is None
    print(f'Test 3b (obstacle far off to the side): corridor_clear={clear} '
          f'(expect True)')
    if not clear:
        failures.append('Test 3b FAILED: an obstacle far off to the side was '
                         'reported as blocking the corridor')

    # --- Test 4: _lateral_bias_at sign correctness. ---
    set_scan(node, [obstacle_s], [0.30], fake_clock.now())  # obstacle to the left
    node._active_grid = node._build_local_grid(ego_x, ego_y)
    bias_obstacle_left = node._lateral_bias_at(obstacle_s)
    print(f'Test 4 (obstacle to the left of centerline): lateral_bias='
          f'{bias_obstacle_left:.3f} (expect < 0, biasing right)')
    if not bias_obstacle_left < 0.0:
        failures.append('Test 4 FAILED: an obstacle left of centerline did not '
                         'produce a rightward (negative) lateral bias')

    # --- Test 5: THE regression test. Same physical obstacle footprint,
    # fed as 1, 2, and 4 separate point groups (as scan_obstacle_detector_
    # node's clustering was observed to fragment a single close obstacle
    # live) -- the found obstacle_s must be identical (within one grid
    # cell) across all three, since nothing here is derived from cluster
    # identity, only from the live point cloud as a whole. ---
    groups = {
        'n=1': ([obstacle_s], [0.0]),
        'n=2': ([obstacle_s - 0.05, obstacle_s + 0.05], [0.02, -0.02]),
        'n=4': ([obstacle_s - 0.08, obstacle_s - 0.02,
                  obstacle_s + 0.03, obstacle_s + 0.09],
                 [0.03, -0.01, 0.02, -0.03]),
    }
    results = {}
    for label, (xs, ys) in groups.items():
        set_scan(node, xs, ys, fake_clock.now())
        node._active_grid = node._build_local_grid(ego_x, ego_y)
        results[label] = node._first_blocked_point(ego_s)
    print(f'Test 5 (fragmentation invariance): {results} '
          f'(expect near-identical found_s across n=1/2/4)')
    values = [v for v in results.values() if v is not None]
    if len(values) != 3 or (max(values) - min(values)) > 0.15:
        failures.append(
            'Test 5 FAILED: the same physical obstacle fragmented into a '
            'different number of detections changed the found location '
            '(%s) -- the exact fragmentation bug this refactor was meant '
            'to eliminate' % (results,))

    # --- Test 6: end-to-end _build_replanned_points with the synthetic
    # obstacle tuple -- confirm the downstream machinery (unchanged)
    # still returns the same shape/keys and picks a feasible side. ---
    set_scan(node, [obstacle_s], [0.0], fake_clock.now())
    node._active_grid = node._build_local_grid(ego_x, ego_y)
    node.committed_side = None
    node.smoothed_target_lateral = None
    node.smoothed_target_time = None
    found_s = node._first_blocked_point(ego_s)
    obstacle = node._synthetic_obstacle(found_s)
    points, side, blocked = node._build_replanned_points(obstacle, 0.0)
    print(f'Test 6 (end-to-end _build_replanned_points): side={side}, '
          f'blocked={blocked}, points.shape={getattr(points, "shape", None)} '
          f'(expect blocked=False, a nonzero side, matching path point count)')
    if blocked or side == 0.0 or points.shape != node.path.points.shape:
        failures.append(
            'Test 6 FAILED: end-to-end avoidance through the synthetic '
            'obstacle tuple did not produce a feasible shaped path')

    rclpy.shutdown()

    if failures:
        print('\n'.join(['', 'FAILURES:'] + failures))
        return 1
    print('\nAll focused occupancy-grid checks passed.')
    return 0


if __name__ == '__main__':
    sys.exit(run())
