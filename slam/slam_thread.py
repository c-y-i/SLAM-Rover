"""SLAM orchestration thread — ties ICP, occupancy grid, and viser display together."""

from __future__ import annotations

import logging
import math
import threading
import time
from typing import TYPE_CHECKING

import numpy as np
import viser

from .icp import icp_2d, scan_to_points, voxel_downsample
from .occupancy_grid import OccupancyGrid

if TYPE_CHECKING:
    # Duck-typed: any object with these fields works (real bridge or sim mock).
    from typing import Protocol

    class SnapshotLike(Protocol):
        x_m: np.ndarray
        y_m: np.ndarray
        distance_m: np.ndarray
        yaw_deg: float
        scan_count: int

    class BridgeLike(Protocol):
        def get_snapshot(self) -> SnapshotLike: ...

logger = logging.getLogger("slam")

# ICP correction beyond this (on top of the IMU prediction) is treated as
# divergence and the frame is skipped.
MAX_ICP_CORRECTION = math.radians(8.0)
MAX_ICP_TRANSLATION_M = 0.35

MIN_POINTS = 20
GRID_PUSH_INTERVAL_S = 0.5


def _theta_to_wxyz(theta: float) -> tuple[float, float, float, float]:
    h = theta * 0.5
    return (math.cos(h), 0.0, 0.0, math.sin(h))


class SlamThread:
    """
    Background thread that runs IMU-assisted ICP SLAM.

    Works with any bridge that exposes get_snapshot() returning an object with
    x_m, y_m, distance_m (numpy arrays), yaw_deg (float), scan_count (int).
    """

    def __init__(
        self,
        bridge: BridgeLike,
        robot_frame: viser.FrameHandle,
        server: viser.ViserServer,
    ) -> None:
        self._bridge = bridge
        self._robot_frame = robot_frame
        self._server = server
        self._grid = OccupancyGrid()

        self._running = False
        self._thread: threading.Thread | None = None

        self._x = 0.0
        self._y = 0.0
        self._theta = 0.0

        self._prev_scan: np.ndarray | None = None
        self._prev_yaw_deg: float | None = None
        self._prev_scan_count: int = -1
        self._last_grid_push = 0.0

    # ── public ──────────────────────────────────────────────────────────────

    def set_initial_pose(self, x: float, y: float) -> None:
        """
        Set the world-space starting position (call before start()).
        Heading is taken from the first IMU reading automatically.
        """
        self._x = x
        self._y = y

    def start(self) -> None:
        if self._thread is not None:
            return
        self._running = True
        self._thread = threading.Thread(target=self._loop, daemon=True, name="slam")
        self._thread.start()
        logger.info("SLAM thread started")

    def stop(self) -> None:
        self._running = False
        if self._thread is not None:
            self._thread.join(timeout=2.0)
            self._thread = None
        logger.info("SLAM thread stopped")

    @property
    def pose(self) -> tuple[float, float, float]:
        return (self._x, self._y, self._theta)

    # ── internal ────────────────────────────────────────────────────────────

    def _loop(self) -> None:
        while self._running:
            snap = self._bridge.get_snapshot()
            if snap.scan_count == self._prev_scan_count:
                time.sleep(0.01)
                continue
            self._prev_scan_count = snap.scan_count
            try:
                self._update(snap)
            except Exception:
                logger.exception("SLAM update error")

    def _update(self, snap: SnapshotLike) -> None:
        pts = voxel_downsample(scan_to_points(snap.x_m, snap.y_m, snap.distance_m))
        if pts.shape[0] < MIN_POINTS:
            return

        if self._prev_scan is None or self._prev_yaw_deg is None:
            self._prev_scan = pts
            self._prev_yaw_deg = snap.yaw_deg
            self._theta = math.radians(snap.yaw_deg)
            self._push_pose()
            return

        # IMU rotation delta (wrapped to [-π, π])
        delta_theta = math.radians(snap.yaw_deg - self._prev_yaw_deg)
        delta_theta = (delta_theta + math.pi) % (2.0 * math.pi) - math.pi

        prev_theta = self._theta

        R, t, ok = icp_2d(pts, self._prev_scan, init_theta=delta_theta)
        icp_theta = math.atan2(R[1, 0], R[0, 0])

        # How much ICP deviated from the IMU seed — large deviation = bad match.
        icp_correction = icp_theta - delta_theta
        translation_norm = float(np.linalg.norm(t))

        # Trust IMU for heading to prevent long-horizon rotational drift.
        self._theta = prev_theta + delta_theta

        if (
            not ok
            or abs(icp_correction) > MAX_ICP_CORRECTION
            or translation_norm > MAX_ICP_TRANSLATION_M
        ):
            # ICP diverged: keep IMU heading, skip translation update.
            logger.debug("ICP skipped: ok=%s correction=%.1f°", ok, math.degrees(icp_correction))
        else:
            # t is in the PREVIOUS robot frame — rotate to world frame first.
            c, s = math.cos(prev_theta), math.sin(prev_theta)
            self._x += c * float(t[0]) - s * float(t[1])
            self._y += s * float(t[0]) + c * float(t[1])

        self._prev_scan = pts
        self._prev_yaw_deg = snap.yaw_deg

        self._push_pose()
        self._grid.insert(pts, (self._x, self._y, self._theta))
        self._maybe_push_grid()

    def _push_pose(self) -> None:
        self._robot_frame.position = (self._x, self._y, 0.0)
        self._robot_frame.wxyz = _theta_to_wxyz(self._theta)

    def _maybe_push_grid(self) -> None:
        now = time.time()
        if now - self._last_grid_push < GRID_PUSH_INTERVAL_S:
            return
        self._last_grid_push = now

        img = self._grid.to_image()
        x_min, y_min, x_max, y_max = self._grid.world_bounds
        cx = (x_min + x_max) / 2.0
        cy = (y_min + y_max) / 2.0

        self._server.scene.add_image(
            "/slam/grid",
            image=img,
            render_width=x_max - x_min,
            render_height=y_max - y_min,
            wxyz=(1.0, 0.0, 0.0, 0.0),
            position=(cx, cy, -0.01),
        )
