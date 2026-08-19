"""Map-frame live occupancy grid + line-of-sight collision check.

Grid construction and the check_collision/check_collision_loose idea are
adapted from stanley_avoidance.py's ego-local-frame version
(refer/f1tenth_ws/src/stanley_avoidance/stanley_avoidance/
stanley_avoidance.py, cloned from https://github.com/CL2-UWaterloo/
f1tenth_ws), reimplemented here in the map frame since
local_avoidance_planner_node.py's scan_callback already produces
TF-correct map-frame scan points. The reference's check_collision shifts
its grid's *column* axis by +/-margin because column is exactly lateral
in its ego-local grid; here the grid is plain map x/y, so margin is
instead a world-space perpendicular offset applied to both segment
endpoints before rasterizing.

traverse_grid is Bresenham's line algorithm, essentially as in the
reference file (itself credited there to roguebasin.com).

2026-08-19: built to replace a fragmenting per-tick tracked obstacle
position (a single small obstacle's LiDAR cluster was splitting into 2-4
simultaneous separate detections at close range) with a stateless
recompute-every-tick occupancy check -- nothing here is ever tracked
across ticks, so there is nothing to fragment or drift.
"""
import math

import numpy as np


def traverse_grid(cell_a, cell_b):
    """Bresenham's line algorithm between two integer grid cells
    (gx, gy), inclusive of both endpoints."""
    x1, y1 = cell_a
    x2, y2 = cell_b
    is_steep = abs(y2 - y1) > abs(x2 - x1)
    if is_steep:
        x1, y1 = y1, x1
        x2, y2 = y2, x2

    swapped = False
    if x1 > x2:
        x1, x2 = x2, x1
        y1, y2 = y2, y1
        swapped = True

    dx = x2 - x1
    dy = y2 - y1
    error = dx // 2
    y_step = 1 if y1 < y2 else -1

    cells = []
    y = y1
    for x in range(x1, x2 + 1):
        cells.append((y, x) if is_steep else (x, y))
        error -= abs(dy)
        if error < 0:
            y += y_step
            error += dx

    if swapped:
        cells.reverse()
    return cells


class LocalOccupancyGrid:
    """Square map-frame occupancy grid, centered on the ego position and
    rebuilt fresh from live scan points every call to
    populate_from_world_points -- no state persists between calls."""

    IS_FREE = 0
    IS_OCCUPIED = 100

    def __init__(self, center_x, center_y, resolution_m, radius_m):
        self.resolution = max(resolution_m, 1.0e-3)
        self.origin_x = center_x - radius_m
        self.origin_y = center_y - radius_m
        self.size = max(4, int(math.ceil(2.0 * radius_m / self.resolution)))
        self.grid = np.full((self.size, self.size), self.IS_FREE, dtype=np.int8)

    def world_to_grid(self, x, y):
        gx = int(math.floor((x - self.origin_x) / self.resolution))
        gy = int(math.floor((y - self.origin_y) / self.resolution))
        return gx, gy

    def grid_to_world(self, gx, gy):
        x = self.origin_x + (gx + 0.5) * self.resolution
        y = self.origin_y + (gy + 0.5) * self.resolution
        return x, y

    def populate_from_world_points(self, xs, ys, inflate_radius_m):
        self.grid.fill(self.IS_FREE)
        xs = np.asarray(xs, dtype=float)
        ys = np.asarray(ys, dtype=float)
        if xs.size == 0:
            return

        gx = np.floor((xs - self.origin_x) / self.resolution).astype(int)
        gy = np.floor((ys - self.origin_y) / self.resolution).astype(int)
        valid = (gx >= 0) & (gx < self.size) & (gy >= 0) & (gy < self.size)
        gx = gx[valid]
        gy = gy[valid]
        if gx.size == 0:
            return

        inflate_cells = max(0, int(math.ceil(inflate_radius_m / self.resolution)))
        offsets = np.arange(-inflate_cells, inflate_cells + 1)
        dx_grid, dy_grid = np.meshgrid(offsets, offsets)
        disk_mask = (dx_grid * dx_grid + dy_grid * dy_grid) <= inflate_cells * inflate_cells
        dx_grid = dx_grid[disk_mask]
        dy_grid = dy_grid[disk_mask]

        all_x = (gx[:, None] + dx_grid[None, :]).ravel()
        all_y = (gy[:, None] + dy_grid[None, :]).ravel()
        valid2 = (all_x >= 0) & (all_x < self.size) & (all_y >= 0) & (all_y < self.size)
        self.grid[all_y[valid2], all_x[valid2]] = self.IS_OCCUPIED

    def _segment_blocked(self, world_a, world_b):
        cell_a = self.world_to_grid(*world_a)
        cell_b = self.world_to_grid(*world_b)
        for gx, gy in traverse_grid(cell_a, cell_b):
            if gx < 0 or gx >= self.size or gy < 0 or gy >= self.size:
                continue
            if self.grid[gy, gx] == self.IS_OCCUPIED:
                return True
        return False

    def check_collision(self, world_a, world_b, margin_m=0.0):
        """Is any point along the segment world_a->world_b, swept by
        +/-margin_m perpendicular to the segment, occupied?"""
        ax, ay = world_a
        bx, by = world_b
        seg_dx, seg_dy = bx - ax, by - ay
        length = math.hypot(seg_dx, seg_dy)
        if length < 1.0e-6:
            return self._segment_blocked(world_a, world_b)

        nx, ny = -seg_dy / length, seg_dx / length
        if margin_m <= 0.0:
            offsets = [0.0]
        else:
            steps = max(1, int(math.ceil(margin_m / self.resolution)))
            offsets = [margin_m * i / steps for i in range(-steps, steps + 1)]

        for offset in offsets:
            a = (ax + nx * offset, ay + ny * offset)
            b = (bx + nx * offset, by + ny * offset)
            if self._segment_blocked(a, b):
                return True
        return False

    def check_collision_loose(self, world_a, world_b, margin_m=0.0):
        """Same as check_collision but only sweeps the second half of the
        segment (midpoint -> world_b) -- a fallback for when world_a is
        already very close to occupied space and a full-segment check
        would trivially fail."""
        mid = (0.5 * (world_a[0] + world_b[0]), 0.5 * (world_a[1] + world_b[1]))
        return self.check_collision(mid, world_b, margin_m=margin_m)
