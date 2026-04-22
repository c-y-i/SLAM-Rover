"""Log-odds 2-D occupancy grid with Bresenham ray-casting."""

from __future__ import annotations

import math

import numpy as np


class OccupancyGrid:
    RESOLUTION_M: float = 0.05
    LOG_OCC: float = 0.85
    LOG_FREE: float = -0.40
    CLAMP_MIN: float = -5.0
    CLAMP_MAX: float = 5.0

    # Initial size in cells; doubles when the robot drives near the edge.
    _INITIAL_CELLS: int = 800   # 800 × 0.05 m = 40 m square

    def __init__(self) -> None:
        n = self._INITIAL_CELLS
        self._grid = np.zeros((n, n), dtype=np.float32)
        # World-space origin corresponding to cell (0, 0)
        half = n * self.RESOLUTION_M / 2.0
        self._origin_x = -half
        self._origin_y = -half

    # ── public ──────────────────────────────────────────────────────────────

    def insert(
        self,
        scan_pts_robot: np.ndarray,
        pose: tuple[float, float, float],
    ) -> None:
        """
        Ray-cast each beam from robot pose to its endpoint.
        scan_pts_robot : (N, 2) points in robot frame.
        pose           : (x, y, theta) in metres / radians.
        """
        if scan_pts_robot.shape[0] == 0:
            return

        rx, ry, theta = pose
        c, s = math.cos(theta), math.sin(theta)

        # Transform scan to world frame
        wx = rx + c * scan_pts_robot[:, 0] - s * scan_pts_robot[:, 1]
        wy = ry + s * scan_pts_robot[:, 0] + c * scan_pts_robot[:, 1]

        # Robot cell
        rc = self._world_to_cell(rx, ry)
        if rc is None:
            return

        for ex, ey in zip(wx, wy):
            ec = self._world_to_cell(ex, ey)
            if ec is None:
                continue
            # Mark free cells along the ray
            for fc in _bresenham(rc[0], rc[1], ec[0], ec[1]):
                self._add(fc[0], fc[1], self.LOG_FREE)
            # Mark endpoint occupied
            self._add(ec[0], ec[1], self.LOG_OCC)

    def to_image(self) -> np.ndarray:
        """
        Return uint8 image: 0 = free, 128 = unknown, 255 = occupied.
        Shape is (H, W, 3) for easy viser display.
        """
        prob = 1.0 / (1.0 + np.exp(-self._grid.astype(np.float64)))
        img_gray = np.where(
            self._grid == 0.0,
            128,
            np.clip(prob * 255, 0, 255),
        ).astype(np.uint8)
        return np.stack([img_gray, img_gray, img_gray], axis=2)

    @property
    def grid_shape(self) -> tuple[int, int]:
        return self._grid.shape  # (rows, cols) = (H, W)

    @property
    def world_bounds(self) -> tuple[float, float, float, float]:
        """(x_min, y_min, x_max, y_max) of the grid in world metres."""
        h, w = self._grid.shape
        return (
            self._origin_x,
            self._origin_y,
            self._origin_x + w * self.RESOLUTION_M,
            self._origin_y + h * self.RESOLUTION_M,
        )

    # ── internal ────────────────────────────────────────────────────────────

    def _world_to_cell(self, wx: float, wy: float) -> tuple[int, int] | None:
        col = int((wx - self._origin_x) / self.RESOLUTION_M)
        row = int((wy - self._origin_y) / self.RESOLUTION_M)
        h, w = self._grid.shape
        if 0 <= col < w and 0 <= row < h:
            return (col, row)
        return None

    def _add(self, col: int, row: int, value: float) -> None:
        v = self._grid[row, col] + value
        self._grid[row, col] = max(self.CLAMP_MIN, min(self.CLAMP_MAX, v))


def _bresenham(x0: int, y0: int, x1: int, y1: int) -> list[tuple[int, int]]:
    """Yield integer cells along the line from (x0,y0) to (x1,y1), exclusive of endpoint."""
    cells: list[tuple[int, int]] = []
    dx = abs(x1 - x0)
    dy = abs(y1 - y0)
    sx = 1 if x0 < x1 else -1
    sy = 1 if y0 < y1 else -1
    err = dx - dy
    while not (x0 == x1 and y0 == y1):
        cells.append((x0, y0))
        e2 = 2 * err
        if e2 > -dy:
            err -= dy
            x0 += sx
        if e2 < dx:
            err += dx
            y0 += sy
    return cells
