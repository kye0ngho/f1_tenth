"""LaserScan clustering for static-obstacle detection.

Pure numpy, no ROS dependency, matching the style of raceline.py. Consumed
by mpcc_node.py's scan_callback/update_obstacle_constraints -- see that
module's docstring for the corridor-filtering step that turns a raw
cluster into a confirmed on-track obstacle.
"""

from collections import namedtuple

import numpy as np

ScanCluster = namedtuple('ScanCluster', ['x', 'y', 'width', 'count'])


def cluster_scan(ranges, angle_min, angle_increment, range_min, range_max,
                 jump_threshold, min_points):
    """Segment a LaserScan into clusters of contiguous close-range beams.

    A new cluster starts whenever a beam is invalid (out of
    [range_min, range_max] or non-finite) or the range jumps by more than
    jump_threshold from the previous beam -- the standard range-image
    segmentation used to separate a small foreground object from the
    track wall behind it. Clusters shorter than min_points beams (sensor
    noise) are dropped.

    Returns a list of ScanCluster with (x, y) in the scan's own frame
    (local_x forward, local_y left, per sensor_msgs/LaserScan's angle
    convention) and width = chord length between the cluster's first and
    last point (a long, wall-like return has a large chord; a small
    object does not).
    """
    ranges = np.asarray(ranges, dtype=float)
    count = len(ranges)
    valid = np.isfinite(ranges) & (ranges >= range_min) & (ranges <= range_max)
    if not np.any(valid):
        return []

    angles = angle_min + angle_increment * np.arange(count)
    xs = ranges * np.cos(angles)
    ys = ranges * np.sin(angles)

    clusters = []
    index = 0
    while index < count:
        if not valid[index]:
            index += 1
            continue
        start = index
        index += 1
        while (index < count and valid[index]
               and abs(ranges[index] - ranges[index - 1]) <= jump_threshold):
            index += 1
        beams = index - start
        if beams >= min_points:
            segment_x = xs[start:index]
            segment_y = ys[start:index]
            width = float(np.hypot(
                segment_x[-1] - segment_x[0], segment_y[-1] - segment_y[0]))
            clusters.append(ScanCluster(
                x=float(np.mean(segment_x)), y=float(np.mean(segment_y)),
                width=width, count=beams))
    return clusters
