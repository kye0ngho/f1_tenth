#!/usr/bin/env python3
"""Focused, non-ROS-graph unit test for the 2026-08-19 wall-clearance
continuity fix in local_avoidance_planner_node.py (_point_wall_clearance).

The old implementation returned `(ring_radius - 1) * map_resolution`, which
is constant across an entire ring of grid cells -- two candidate lateral
offsets a few cm apart could land in the same ring and get an identical
clearance value, so _score_candidate's argmax picked between them on
floating-point noise instead of a real preference. Confirmed live (RViz +
2026-08-19 sim run) as target_lateral swinging ~0.32-0.52m tick to tick for
a single stationary obstacle, twice flipping side outright.

This test builds a synthetic occupancy grid with a straight wall and sweeps
the query point across it, checking that the returned clearance is
(a) monotonic as the point approaches the wall, and (b) not flat across
multiple map-resolution-sized steps -- i.e. no plateau wider than one grid
cell, which is what the old ring-index formula produced.

Run inside the ROS2 environment (host rclpy jazzy or container humble):
    python3 scripts/focused_test_wall_clearance_continuity.py
"""
import sys
from pathlib import Path

import numpy as np
import rclpy

sys.path.insert(
    0, str(Path(__file__).resolve().parent.parent
           / 'algorithms' / 'planning'))

from planning.local_avoidance_planner_node import LocalAvoidancePlannerNode  # noqa: E402


def make_wall_grid(node, resolution=0.05, width_m=4.0, height_m=4.0,
                    wall_x=2.0):
    width = int(width_m / resolution)
    height = int(height_m / resolution)
    grid = np.zeros((height, width), dtype=np.int16)
    wall_col = int(wall_x / resolution)
    grid[:, wall_col:] = 100
    node.map_resolution = resolution
    node.map_origin_x = 0.0
    node.map_origin_y = -height_m / 2.0
    node.map_width = width
    node.map_height = height
    node.map_grid = grid
    return wall_x


def main():
    rclpy.init()
    node = LocalAvoidancePlannerNode()
    try:
        wall_x = make_wall_grid(node)
        y = 0.0
        xs = np.arange(1.0, wall_x, 0.01)
        clearances = [node._point_wall_clearance(float(x), y) for x in xs]

        # (a) monotonic non-increasing as we approach the wall
        diffs = np.diff(clearances)
        non_monotonic = np.where(diffs > 1.0e-6)[0]
        assert len(non_monotonic) == 0, (
            f'clearance increased while approaching the wall at indices '
            f'{non_monotonic[:5]} (xs={xs[non_monotonic[:5]]})')
        print('PASS: clearance is monotonic non-increasing toward the wall')

        # (b) no plateau wider than one grid cell (the old bug: whole rings
        # of ~map_resolution width returned an identical value). Excludes
        # samples pinned at clearance_check_radius -- that's an intentional
        # saturation clamp once a point is farther than the search radius
        # from any wall, not the ring-quantization staircase under test.
        resolution = node.map_resolution
        in_range = [
            (x, c) for x, c in zip(xs, clearances)
            if c < node.clearance_check_radius - 1.0e-6]
        run_start = 0
        max_plateau_m = 0.0
        for i in range(1, len(in_range) + 1):
            if i == len(in_range) or abs(
                    in_range[i][1] - in_range[run_start][1]) > 1.0e-9:
                plateau_m = (
                    (in_range[i - 1][0] - in_range[run_start][0])
                    if i > run_start else 0.0)
                max_plateau_m = max(max_plateau_m, plateau_m)
                run_start = i
        print(f'max flat run: {max_plateau_m:.3f} m '
              f'(map_resolution={resolution:.3f} m)')
        assert max_plateau_m < resolution, (
            f'found a flat plateau ({max_plateau_m:.3f} m) as wide as or '
            f'wider than one grid cell ({resolution:.3f} m) -- continuity '
            f'fix did not remove the staircase')
        print('PASS: no plateau as wide as one grid cell '
              '(old ring-index bug would have produced ~0.05m plateaus)')

        # (c) two candidates 5cm apart (typical lateral_candidate_step_m)
        # now get distinguishable scores instead of tying
        near_wall_a = node._point_wall_clearance(wall_x - 0.30, y)
        near_wall_b = node._point_wall_clearance(wall_x - 0.25, y)
        assert abs(near_wall_a - near_wall_b) > 1.0e-6, (
            'two candidates 5cm apart still returned an identical '
            'clearance -- this is exactly the tie that caused argmax to '
            'pick between them on noise')
        print(f'PASS: candidates 5cm apart get distinct clearances '
              f'({near_wall_a:.4f} vs {near_wall_b:.4f})')

        print('\nAll focused wall-clearance continuity checks passed.')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
