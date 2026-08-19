#!/usr/bin/env python3
"""Focused, non-ROS-graph unit test for the 2026-08-19 hard slew-rate-limit
fix in local_avoidance_planner_node.py (_build_replanned_points).

Instantiates the real node (rclpy.init() only, no simulator/topics needed)
and drives _build_replanned_points directly with a synthetic straight path
and a fake clock, so ticks advance by a controlled dt instead of real time.
_point_wall_clearance and _score_candidate are monkeypatched to isolate the
selection-stability mechanism from map-grid/geometry noise -- this is not an
integration test, it targets the specific regression that caused the real
2026-08-18 collision (raw argmax winner feeding straight into the committed
side/geometry).

Run inside the ROS2 environment (host has rclpy jazzy installed; container
has humble -- either works for this test, no simulator required):
    python3 scripts/focused_test_avoidance_slew.py
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
    """Holds a fixed time between explicit tick() calls instead of
    advancing on every .now() -- _build_replanned_points calls
    get_clock().now() more than once per invocation (slew limiter, throttled
    logging), so a clock that advanced on every call would make the
    effective per-tick dt depend on incidental call counts rather than the
    single dt the test intends per simulated tick."""

    def __init__(self, dt_s):
        self.dt = dt_s
        self._now = Time(seconds=1000)

    def tick(self):
        self._now = self._now + Duration(seconds=self.dt)

    def now(self):
        return self._now


def make_obstacle(obs_s=5.0, obs_x=5.0, obs_y=0.0, lateral=0.0, path_yaw=0.0):
    return (0.0, obs_x, obs_y, obs_s, lateral, path_yaw)


def run():
    rclpy.init()
    node = LocalAvoidancePlannerNode()
    make_straight_path(node, length_m=20.0)

    dt = 0.05  # 20 Hz, matches the node's default publish_rate
    fake_clock = FakeClock(dt)
    node.get_clock = lambda: fake_clock

    node._point_wall_clearance = lambda x, y: 5.0

    rate_limit = node.target_lateral_rate_limit
    lane_offset = node.lane_offset
    min_flip_ticks = (2.0 * lane_offset) / (rate_limit * dt)

    print(f'target_lateral_rate_limit_mps={rate_limit}, '
          f'lane_offset_m={lane_offset}, dt={dt}s -> '
          f'min ticks to complete a side flip ~= {min_flip_ticks:.1f}')

    failures = []

    # --- Test 1: adversarial per-tick argmax flip should NOT translate
    # into a committed side flip within a handful of ticks. ---
    node.smoothed_target_lateral = None
    node.smoothed_target_time = None
    node.committed_side = None
    tick = {'n': 0}

    def flipping_score(points, active_indices, obstacle,
                        target_lateral, ego_lateral, side):
        tick_favored = 1.0 if (tick['n'] % 2 == 0) else -1.0
        feasible = abs(target_lateral) >= 0.1
        score = (1.0 if side == tick_favored else 0.5) if feasible else -1.0
        return {
            'feasible': feasible,
            'score': score,
            'min_wall_clearance': 5.0,
            'min_obstacle_distance': 5.0,
            'obstacle_margin': 5.0,
            'smoothness_cost': 0.0,
            'continuity_cost': 0.0,
        }

    node._score_candidate = flipping_score

    sides_seen = []
    obstacle = make_obstacle(lateral=0.0)
    for i in range(6):
        tick['n'] = i
        fake_clock.tick()
        _points, side, blocked = node._build_replanned_points(obstacle, 0.0)
        sides_seen.append(None if blocked else side)

    flips = sum(
        1 for a, b in zip(sides_seen, sides_seen[1:])
        if a is not None and b is not None and a != b)
    print(f'Test 1 (adversarial per-tick flip-flop): sides over 6 ticks = '
          f'{sides_seen}, flips={flips}')
    if flips > 0:
        failures.append(
            'Test 1 FAILED: committed side flipped within 6 ticks despite '
            'the raw argmax favoring a different side every single tick '
            '(expected 0 flips given min_flip_ticks ~= %.1f)' % min_flip_ticks)

    # --- Test 2: sustained one-sided win over enough ticks DOES eventually
    # flip (mechanism isn't a permanent lock, just rate-limited). ---
    node.smoothed_target_lateral = None
    node.smoothed_target_time = None
    node.committed_side = None

    def sustained_favor_score(points, active_indices, obstacle,
                               target_lateral, ego_lateral, side):
        favored = tick['n2_favored']
        feasible = abs(target_lateral) >= 0.1
        score = (1.0 if side == favored else 0.5) if feasible else -1.0
        return {
            'feasible': feasible,
            'score': score,
            'min_wall_clearance': 5.0,
            'min_obstacle_distance': 5.0,
            'obstacle_margin': 5.0,
            'smoothness_cost': 0.0,
            'continuity_cost': 0.0,
        }

    node._score_candidate = sustained_favor_score
    n_ticks = int(min_flip_ticks) + 20
    tick['n2_favored'] = 1.0
    final_side = None
    for i in range(n_ticks):
        fake_clock.tick()
        _points, side, blocked = node._build_replanned_points(obstacle, 0.0)
        final_side = None if blocked else side
    print(f'Test 2 (sustained {n_ticks}-tick one-sided win): final_side='
          f'{final_side} (expect 1.0, i.e. eventually commits to the '
          f'consistently-favored side)')
    if final_side != 1.0:
        failures.append(
            'Test 2 FAILED: side never converged to the consistently-'
            'favored side after %d ticks -- slew limit may be too '
            'aggressive or wired wrong' % n_ticks)

    # --- Test 3: infeasible mid-flip point causes a hold/revert instead of
    # advancing into it. ---
    node.smoothed_target_lateral = None
    node.smoothed_target_time = None
    node.committed_side = None

    def reject_near_zero_score(points, active_indices, obstacle,
                                target_lateral, ego_lateral, side):
        feasible = abs(target_lateral) >= 0.1
        score = 1.0 if feasible else -1.0
        return {
            'feasible': feasible,
            'score': score,
            'min_wall_clearance': 5.0,
            'min_obstacle_distance': 5.0,
            'obstacle_margin': 5.0,
            'smoothness_cost': 0.0,
            'continuity_cost': 0.0,
        }

    node._score_candidate = reject_near_zero_score
    tick['n2_favored'] = 1.0
    prev_targets = []
    for i in range(n_ticks):
        fake_clock.tick()
        _points, side, blocked = node._build_replanned_points(obstacle, 0.0)
        prev_targets.append(node.smoothed_target_lateral)
    near_zero_hits = sum(1 for t in prev_targets if abs(t) < 0.1)
    print(f'Test 3 (reject-near-zero feasibility): '
          f'{near_zero_hits} of {n_ticks} smoothed_target_lateral values '
          f'landed inside the rejected zone (expect 0 -- revert should '
          f'prevent ever publishing/tracking an infeasible mid-flip point)')
    if near_zero_hits > 0:
        failures.append(
            'Test 3 FAILED: %d/%d ticks tracked a target_lateral inside '
            'the zone _score_candidate marked infeasible -- the revert-on-'
            'infeasible-recheck is not holding' % (near_zero_hits, n_ticks))

    rclpy.shutdown()

    if failures:
        print('\n'.join(['', 'FAILURES:'] + failures))
        return 1
    print('\nAll focused avoidance-slew checks passed.')
    return 0


if __name__ == '__main__':
    sys.exit(run())
