"""LD06 lidar 2D top-down viewer with map accumulation and SLAM hooks."""

from __future__ import annotations

import argparse
import logging
import math
import time
from collections import deque

import numpy as np
import viser

from . import config
from .scene import create_scene
from .serial_reader import SerialReader, ScanSnapshot

logger = logging.getLogger("ld06_viewer")


# ── colour helpers ────────────────────────────────────────────────────────────

def _colors_radar(intensity: np.ndarray) -> np.ndarray:
    """Cyan-green palette scaled by intensity — classic radar look."""
    t = intensity.astype(np.float32) / 255.0
    r = np.zeros(len(t), dtype=np.uint8)
    g = (t * 190 + 30).astype(np.uint8)
    b = (t * 195 + 60).astype(np.uint8)
    return np.stack([r, g, b], axis=1)


def _colors_distance(distance_m: np.ndarray) -> np.ndarray:
    """Blue (near) → red (far) gradient."""
    t = np.clip(
        (distance_m - config.MIN_DIST_M) / (config.MAX_DIST_M - config.MIN_DIST_M),
        0.0, 1.0,
    ).astype(np.float32)
    r = (t * 255).astype(np.uint8)
    g = np.zeros(len(t), dtype=np.uint8)
    b = ((1.0 - t) * 255).astype(np.uint8)
    return np.stack([r, g, b], axis=1)


def _colors_intensity(intensity: np.ndarray) -> np.ndarray:
    v = intensity[:, np.newaxis].repeat(3, axis=1)
    return v


def _get_colors(
    distance_m: np.ndarray,
    intensity: np.ndarray,
    mode: str,
) -> np.ndarray:
    if mode == "Intensity":
        return _colors_intensity(intensity)
    if mode == "Distance":
        return _colors_distance(distance_m)
    return _colors_radar(intensity)  # "Radar" default


def _theta_to_wxyz(theta_rad: float) -> tuple[float, float, float, float]:
    """Z-axis rotation quaternion (wxyz)."""
    h = theta_rad * 0.5
    return (math.cos(h), 0.0, 0.0, math.sin(h))


# ── viewer ───────────────────────────────────────────────────────────────────

class LD06Viewer:
    """
    Viser-based 2D top-down viewer for the LD06 lidar.

    SLAM integration hooks
    ----------------------
    Call ``update_pose(x, y, theta_rad)`` from your SLAM thread to move the
    robot frame in world space.  The current scan is rendered in the robot
    frame so it auto-transforms with pose updates.  ``get_latest_scan()``
    returns the most recent ``ScanSnapshot`` for your algorithm to consume.
    """

    def __init__(self, port: str, baud: int = config.BAUD_RATE) -> None:
        self.serial_reader = SerialReader(port, baud)
        self.scene = None

        # Robot pose (set by SLAM or left at identity)
        self._robot_x = 0.0
        self._robot_y = 0.0
        self._robot_theta = 0.0

        # Map accumulation — deque of (wx, wy, intensity) arrays
        self._map: deque[tuple[np.ndarray, np.ndarray, np.ndarray]] = deque()
        self._last_scan_count = -1

    # ── SLAM hooks ────────────────────────────────────────────────────────────

    def update_pose(self, x: float, y: float, theta_rad: float) -> None:
        """Update the robot's estimated world pose (call from SLAM thread)."""
        self._robot_x = x
        self._robot_y = y
        self._robot_theta = theta_rad

    def get_latest_scan(self) -> ScanSnapshot:
        """Return the latest scan snapshot (call from SLAM thread)."""
        return self.serial_reader.get_snapshot()

    # ── internal ──────────────────────────────────────────────────────────────

    def _setup_gui(self, server: viser.ViserServer) -> None:
        with server.gui.add_folder("Sensor"):
            self.status_text   = server.gui.add_text("Status",    initial_value="Waiting…")
            self.scan_fps_text = server.gui.add_text("Scan FPS",  initial_value="0.0")
            self.imu_text      = server.gui.add_text("IMU",       initial_value="Not detected")
            self.imu_fps_text  = server.gui.add_text("IMU FPS",   initial_value="0.0")
            self.points_text   = server.gui.add_text("Points",    initial_value="0")
            self.range_text    = server.gui.add_text("Range",     initial_value="--")

        with server.gui.add_folder("View"):
            self.color_mode = server.gui.add_dropdown(
                "Color Mode",
                options=["Radar", "Distance", "Intensity"],
                initial_value="Radar",
            )
            self.point_size_slider = server.gui.add_slider(
                "Point Size", min=0.005, max=0.15, step=0.005, initial_value=0.04,
            )
            self.max_dist_slider = server.gui.add_slider(
                "Max Distance (m)", min=1.0, max=config.MAX_DIST_M, step=0.5,
                initial_value=config.MAX_DIST_M,
            )
            self.apply_imu_checkbox = server.gui.add_checkbox(
                "Apply IMU Rotation", initial_value=True,
            )

        with server.gui.add_folder("Map"):
            self.accumulate_checkbox = server.gui.add_checkbox(
                "Accumulate Scans", initial_value=False,
            )
            self.max_scans_slider = server.gui.add_slider(
                "Max Scans", min=10, max=config.MAX_MAP_SCANS, step=10,
                initial_value=100,
            )
            self.map_point_size_slider = server.gui.add_slider(
                "Map Point Size", min=0.005, max=0.1, step=0.005, initial_value=0.025,
            )
            self.clear_map_button = server.gui.add_button("Clear Map")

            @self.clear_map_button.on_click
            def _clear(_) -> None:
                self._map.clear()
                server.scene.remove_by_name("/map/points")

    def _maybe_accumulate(self, snapshot: ScanSnapshot) -> None:
        if not self.accumulate_checkbox.value:
            return
        if snapshot.scan_count == self._last_scan_count:
            return
        if snapshot.x_m.size == 0:
            return

        self._last_scan_count = snapshot.scan_count

        # Transform current scan to world frame using robot pose
        c = math.cos(self._robot_theta)
        s = math.sin(self._robot_theta)
        wx = self._robot_x + c * snapshot.x_m - s * snapshot.y_m
        wy = self._robot_y + s * snapshot.x_m + c * snapshot.y_m
        self._map.append((wx, wy, snapshot.intensity.copy()))

        max_scans = int(self.max_scans_slider.value)
        while len(self._map) > max_scans:
            self._map.popleft()

    def _draw_map(self, server: viser.ViserServer) -> None:
        if not self._map:
            return
        all_wx = np.concatenate([e[0] for e in self._map])
        all_wy = np.concatenate([e[1] for e in self._map])
        all_i  = np.concatenate([e[2] for e in self._map])
        pts = np.stack([all_wx, all_wy, np.zeros_like(all_wx)], axis=1).astype(np.float32)
        # Warm amber — visually distinct from the live cyan scan
        t = all_i.astype(np.float32) / 255.0
        r = (t * 140 + 80).astype(np.uint8)
        g = (t * 100 + 60).astype(np.uint8)
        b = np.full(len(t), 20, dtype=np.uint8)
        colors = np.stack([r, g, b], axis=1)
        server.scene.add_point_cloud(
            "/map/points",
            points=pts,
            colors=colors,
            point_size=float(self.map_point_size_slider.value),
            point_shape="circle",
        )

    def _process_frame(self, server: viser.ViserServer) -> None:
        snapshot: ScanSnapshot = self.serial_reader.get_snapshot()

        # ── status labels ──────────────────────────────────────────────────
        self.status_text.value   = "Connected" if snapshot.connected else "Disconnected"
        self.scan_fps_text.value = f"{snapshot.scan_fps:.1f}"
        self.imu_text.value      = "Connected" if snapshot.imu_connected else "Not detected"
        self.imu_fps_text.value  = f"{snapshot.imu_fps:.1f}"
        self.points_text.value   = str(snapshot.x_m.size)

        # ── robot frame pose ───────────────────────────────────────────────
        self.scene.robot.position = (self._robot_x, self._robot_y, 0.0)

        if self.apply_imu_checkbox.value and snapshot.imu_connected:
            self.scene.robot.wxyz = tuple(snapshot.quaternion)
        else:
            self.scene.robot.wxyz = _theta_to_wxyz(self._robot_theta)

        # ── map accumulation ───────────────────────────────────────────────
        self._maybe_accumulate(snapshot)
        if self.accumulate_checkbox.value and self._map:
            self._draw_map(server)
        elif not self.accumulate_checkbox.value:
            server.scene.remove_by_name("/map/points")

        # ── current scan ───────────────────────────────────────────────────
        if not snapshot.connected or snapshot.x_m.size == 0:
            self.range_text.value = "--"
            server.scene.remove_by_name("/robot/scan/points")
            return

        max_dist = float(self.max_dist_slider.value)
        mask = snapshot.distance_m <= max_dist
        if not np.any(mask):
            self.range_text.value = "No points in range"
            server.scene.remove_by_name("/robot/scan/points")
            return

        x    = snapshot.x_m[mask]
        y    = snapshot.y_m[mask]
        dist = snapshot.distance_m[mask]
        inten = snapshot.intensity[mask]

        pts3d  = np.stack([x, y, np.zeros_like(x)], axis=1).astype(np.float32)
        colors = _get_colors(dist, inten, str(self.color_mode.value))

        server.scene.add_point_cloud(
            "/robot/scan/points",
            points=pts3d,
            colors=colors,
            point_size=float(self.point_size_slider.value),
            point_shape="circle",
        )
        self.range_text.value = f"{float(dist.min()):.2f}–{float(dist.max()):.2f} m"

    def run(self, host: str = "0.0.0.0", viser_port: int = 8080) -> None:
        self.serial_reader.connect()
        self.serial_reader.start()

        server = viser.ViserServer(host=host, port=viser_port)
        self.scene = create_scene(server)
        self._setup_gui(server)

        @server.on_client_connect
        def _on_client_connect(client: viser.ClientHandle) -> None:
            client.camera.position = (0.0, 0.0, 7.0)
            client.camera.look_at  = (0.0, 0.0, 0.0)
            client.camera.up       = (0.0, 1.0, 0.0)
            client.camera.fov      = 0.9

        logger.info("Viewer running at http://localhost:%d", viser_port)

        try:
            while True:
                t0 = time.time()
                self._process_frame(server)
                elapsed = time.time() - t0
                if elapsed < config.FRAME_TIME:
                    time.sleep(config.FRAME_TIME - elapsed)
        except KeyboardInterrupt:
            logger.info("Stopping viewer")
        finally:
            self.serial_reader.stop()


def main() -> None:
    parser = argparse.ArgumentParser(description="LD06 Lidar 2D Viewer")
    parser.add_argument("--port",       "-p", required=True,              help="Serial port")
    parser.add_argument("--baud",       "-b", type=int, default=config.BAUD_RATE)
    parser.add_argument("--host",             default="0.0.0.0")
    parser.add_argument("--viser-port",       type=int, default=8080)
    parser.add_argument("--debug",            action="store_true")
    args = parser.parse_args()

    logging.basicConfig(
        level=logging.DEBUG if args.debug else logging.INFO,
        format="%(asctime)s %(levelname)s %(name)s: %(message)s",
    )

    LD06Viewer(port=args.port, baud=args.baud).run(host=args.host, viser_port=args.viser_port)
