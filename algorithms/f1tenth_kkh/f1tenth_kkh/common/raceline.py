"""Closed-loop raceline handling: nearest-point search + interpolation.

Ported from LinearMpcNode.set_closed_path / candidate_indices /
nearest_path_state / interpolate_path in
algorithms/control/control/linear_mpc_node.py, factored into a standalone,
node-independent object shared by all f1tenth_kkh controllers.
"""

import math

import numpy as np


def clamp(value, minimum, maximum):
    return max(minimum, min(value, maximum))


def angle_difference(target, source):
    return math.atan2(
        math.sin(target - source), math.cos(target - source))


class ClosedRaceline:
    def __init__(self, search_back_points=5, search_forward_points=30):
        self.points = None
        self.yaw = None
        self.curvature = None
        self.segment_lengths = None
        self.cumulative = None
        self.length = None
        self.yaw_lap_change = None
        self._nearest_index = None
        self.search_back_points = search_back_points
        self.search_forward_points = search_forward_points

    @property
    def ready(self) -> bool:
        return self.points is not None

    def update(self, points_xy):
        points = np.asarray(points_xy, dtype=float)
        next_points = np.roll(points, -1, axis=0)
        segments = next_points - points
        segment_lengths = np.linalg.norm(segments, axis=1)
        if np.any(segment_lengths < 1e-4):
            raise RuntimeError(
                'Global path contains duplicate adjacent points')

        yaw = np.unwrap(np.arctan2(segments[:, 1], segments[:, 0]))
        direction = 1.0 if np.sum(np.diff(yaw)) >= 0.0 else -1.0
        yaw_lap_change = direction * 2.0 * math.pi
        closed_yaw = yaw[0] + yaw_lap_change
        while closed_yaw - yaw[-1] > math.pi:
            closed_yaw -= 2.0 * math.pi
        while closed_yaw - yaw[-1] < -math.pi:
            closed_yaw += 2.0 * math.pi
        yaw_lap_change = closed_yaw - yaw[0]

        previous_yaw = np.roll(yaw, 1)
        previous_yaw[0] = yaw[-1] - yaw_lap_change
        next_yaw = np.roll(yaw, -1)
        next_yaw[-1] = yaw[0] + yaw_lap_change
        arc_span = np.roll(segment_lengths, 1) + segment_lengths
        curvature = (next_yaw - previous_yaw) / np.maximum(arc_span, 1e-6)

        # A same-length update (e.g. the local avoidance planner shifting a
        # few points laterally) keeps point indices aligned with the same
        # arc-length neighborhood, so the previous search-hint is still a
        # good starting point -- only a genuinely new path (different point
        # count) needs the windowed search to restart from scratch.
        keep_hint = self.points is not None and len(points) == len(self.points)

        self.points = points
        self.yaw = yaw
        self.curvature = curvature
        self.segment_lengths = segment_lengths
        self.cumulative = np.concatenate(([0.0], np.cumsum(segment_lengths)))
        self.length = float(self.cumulative[-1])
        self.yaw_lap_change = float(yaw_lap_change)
        if not keep_hint:
            self._nearest_index = None

    def candidate_indices(self):
        count = len(self.points)
        if self._nearest_index is None:
            return range(count)
        return [
            (self._nearest_index + offset) % count
            for offset in range(-self.search_back_points,
                                 self.search_forward_points + 1)
        ]

    def nearest_state(self, x, y):
        """Return (index, distance, path_heading, path_s) of the closest
        point on the raceline, and remember it as the search hint for the
        next call."""
        position = np.array([x, y])
        best = None
        count = len(self.points)
        for index in self.candidate_indices():
            segment = self.points[(index + 1) % count] - self.points[index]
            relative = position - self.points[index]
            fraction = clamp(
                float(np.dot(relative, segment) / np.dot(segment, segment)),
                0.0, 1.0,
            )
            projection = self.points[index] + fraction * segment
            distance = float(np.linalg.norm(projection - position))
            candidate = (distance, index, fraction)
            if best is None or candidate[0] < best[0]:
                best = candidate

        distance, nearest, fraction = best
        path_s = (
            self.cumulative[nearest]
            + fraction * self.segment_lengths[nearest])
        self._nearest_index = nearest
        return nearest, distance, float(self.yaw[nearest]), path_s

    def nearest_arc_length_stateless(self, x, y):
        """Return (path_s, lateral_offset) of the closest point on the
        raceline to (x, y), scanning every segment.

        Unlike nearest_state, this does not read or update the
        search-hint cache (_nearest_index), so it is safe to call for
        points other than the vehicle's own pose -- e.g. projecting a
        detected obstacle onto the raceline in mpcc_node.py -- without
        disturbing ego-pose tracking's search hint.
        """
        position = np.array([x, y])
        count = len(self.points)
        best = None
        for index in range(count):
            segment = self.points[(index + 1) % count] - self.points[index]
            relative = position - self.points[index]
            fraction = clamp(
                float(np.dot(relative, segment) / np.dot(segment, segment)),
                0.0, 1.0,
            )
            projection = self.points[index] + fraction * segment
            distance = float(np.linalg.norm(projection - position))
            candidate = (distance, index, fraction)
            if best is None or candidate[0] < best[0]:
                best = candidate

        distance, nearest, fraction = best
        path_s = (
            self.cumulative[nearest]
            + fraction * self.segment_lengths[nearest])
        return path_s, distance

    def interpolate(self, values, sample_s, lap_change=0.0):
        laps = np.floor(sample_s / self.length).astype(int)
        wrapped = np.mod(sample_s, self.length)
        closed_values = np.concatenate(
            ([values[0]], values[1:], [values[0] + lap_change]))
        return np.interp(wrapped, self.cumulative, closed_values) + (
            laps * lap_change)


def path_msg_to_closed_points(path_msg, min_points=4):
    """Extract an (N, 2) closed-loop point array from a nav_msgs/Path,
    dropping a duplicated closing point if present. Returns None if the
    path has fewer than min_points points."""
    points = np.asarray([
        [pose.pose.position.x, pose.pose.position.y]
        for pose in path_msg.poses
    ], dtype=float)
    if len(points) < min_points:
        return None
    if len(points) > 2 and np.linalg.norm(points[0] - points[-1]) < 1e-4:
        points = points[:-1]
    if len(points) < min_points:
        return None
    return points
