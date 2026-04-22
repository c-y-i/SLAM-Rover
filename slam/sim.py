#!/usr/bin/env python3
"""
SLAM simulator — no hardware required.

Generates a 2-D world, drives a robot along a waypoint loop, synthesises
LD06-style LiDAR scans + BNO085 IMU yaw, and runs the real SLAM stack.

Usage (from repo root):
    python -m slam.sim [--web-port 8080] [--map random|office|warehouse|maze|nav2_*] [--seed 42]

Open http://localhost:<web-port>.

    Blue   trail / axes → ground truth
    Magenta trail / axes → SLAM estimate
"""

from __future__ import annotations

import argparse
import math
import threading
import time
from dataclasses import dataclass
from pathlib import Path
import sys

import numpy as np
import viser

if __package__ in (None, ""):
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
    from slam.slam_thread import SlamThread
    from slam.sim_world import BASE_SPEED_M_S, MAX_RANGE, MAP_BUILDERS, SCAN_HZ
    from slam.plot_sim import MAP_CHOICES, _build_environment
else:
    from .slam_thread import SlamThread
    from .sim_world import BASE_SPEED_M_S, MAX_RANGE, MAP_BUILDERS, SCAN_HZ
    from .plot_sim import MAP_CHOICES, _build_environment


# ── mock bridge ───────────────────────────────────────────────────────────────

@dataclass(frozen=True)
class SimSnapshot:
    x_m: np.ndarray
    y_m: np.ndarray
    distance_m: np.ndarray
    intensity: np.ndarray
    connected: bool
    imu_connected: bool
    imu_ok: bool
    yaw_deg: float
    yaw_rate_dps: float
    status_text: str
    scan_count: int


_EMPTY = SimSnapshot(
    x_m=np.zeros(0, np.float32), y_m=np.zeros(0, np.float32),
    distance_m=np.zeros(0, np.float32), intensity=np.zeros(0, np.uint8),
    connected=False, imu_connected=False, imu_ok=False,
    yaw_deg=0.0, yaw_rate_dps=0.0, status_text="starting", scan_count=0,
)


class SimBridge:
    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._snap = _EMPTY

    def push(self, snap: SimSnapshot) -> None:
        with self._lock:
            self._snap = snap

    def get_snapshot(self) -> SimSnapshot:
        with self._lock:
            return self._snap


# ── helpers ───────────────────────────────────────────────────────────────────

def _wxyz(theta: float) -> tuple[float, float, float, float]:
    h = theta * 0.5
    return (math.cos(h), 0.0, 0.0, math.sin(h))


def _wrap(a: float) -> float:
    return (a + math.pi) % (2 * math.pi) - math.pi


# ── simulator ─────────────────────────────────────────────────────────────────

class Simulator:
    def __init__(self, map_name: str, seed: int | None, server: viser.ViserServer) -> None:
        self._map_name = map_name
        self._seed     = seed
        self._server   = server

        # Sim settings (overridden by GUI)
        self._speed       = 1.5
        self._range_noise = 0.02
        self._yaw_noise   = 0.5
        self._paused      = False
        self._reset_flag  = False

        # Ground-truth state
        self._scan_count = 0
        self._wp_idx     = 1
        self._loop_count = 0
        self._rtheta     = 0.0

        # Trails
        self._truth_trail: list[tuple[float, float]] = []
        self._slam_trail:  list[tuple[float, float]] = []
        self._wall_paths:  list[str] = []

        self._bridge: SimBridge      = SimBridge()
        self._slam:   SlamThread | None = None
        self._truth_frame: viser.FrameHandle | None = None

        self._env = self._make_environment()
        self._rx, self._ry = self._env.waypoints[0]
        self._rebuild_scene()
        self._setup_gui()

    # ── world / scene ────────────────────────────────────────────────────────

    def _make_environment(self):
        return _build_environment(self._map_name, self._seed)

    def _clear_world_nodes(self) -> None:
        for p in self._wall_paths:
            self._server.scene.remove_by_name(p)
        self._wall_paths.clear()
        for p in ("/trails/truth", "/trails/slam", "/truth/scan", "/slam/grid"):
            self._server.scene.remove_by_name(p)

    def _draw_walls(self) -> None:
        for i, ((ax, ay), (bx, by)) in enumerate(self._env.walls):
            pts  = np.array([[ax, ay, 0.0], [bx, by, 0.0]], dtype=np.float32)
            path = f"/world/wall_{i}"
            self._server.scene.add_spline_catmull_rom(
                path, positions=pts, line_width=3.5, color=(215, 215, 230),
            )
            self._wall_paths.append(path)

    def _rebuild_scene(self) -> None:
        self._clear_world_nodes()
        self._truth_trail.clear()
        self._slam_trail.clear()

        self._draw_walls()
        self._server.scene.add_frame("/origin", axes_length=0.15, axes_radius=0.004)

        self._truth_frame = self._server.scene.add_frame(
            "/truth", axes_length=0.35, axes_radius=0.010, show_axes=True,
        )
        slam_frame = self._server.scene.add_frame(
            "/robot", axes_length=0.35, axes_radius=0.010, show_axes=True,
        )
        if self._slam is not None:
            self._slam.stop()
        self._bridge = SimBridge()
        self._slam   = SlamThread(self._bridge, slam_frame, self._server)
        self._slam.set_initial_pose(self._rx, self._ry)
        self._slam.start()

    # ── GUI ──────────────────────────────────────────────────────────────────

    def _setup_gui(self) -> None:
        with self._server.gui.add_folder("Simulation"):
            self._g_map     = self._server.gui.add_text("Map",     self._map_name)
            self._g_scans   = self._server.gui.add_text("Scans",   "0")
            self._g_elapsed = self._server.gui.add_text("Elapsed", "0s")
            self._g_loops   = self._server.gui.add_text("Loops",   "0")

        with self._server.gui.add_folder("Ground Truth"):
            self._g_tx = self._server.gui.add_text("X (m)", "--")
            self._g_ty = self._server.gui.add_text("Y (m)", "--")
            self._g_th = self._server.gui.add_text("θ (°)",  "--")

        with self._server.gui.add_folder("SLAM Estimate"):
            self._g_sx   = self._server.gui.add_text("X (m)",         "--")
            self._g_sy   = self._server.gui.add_text("Y (m)",         "--")
            self._g_sh   = self._server.gui.add_text("θ (°)",          "--")
            self._g_perr = self._server.gui.add_text("Pos error (m)", "--")
            self._g_herr = self._server.gui.add_text("Hdg error (°)", "--")

        with self._server.gui.add_folder("Controls"):
            self._sl_speed  = self._server.gui.add_slider("Speed ×",        min=0.2, max=5.0, step=0.1,  initial_value=self._speed)
            self._sl_rnoise = self._server.gui.add_slider("LiDAR noise (m)", min=0.0, max=0.15, step=0.005, initial_value=self._range_noise)
            self._sl_ynoise = self._server.gui.add_slider("IMU noise (°)",   min=0.0, max=3.0, step=0.1,  initial_value=self._yaw_noise)
            self._btn_pause  = self._server.gui.add_button("⏸  Pause / Resume")
            self._btn_newmap = self._server.gui.add_button("🗺  New Random Map")

        with self._server.gui.add_folder("Legend"):
            self._server.gui.add_text("Blue trail",    "ground truth path  (/trails/truth)")
            self._server.gui.add_text("Magenta trail", "SLAM estimate path (/trails/slam)")
            self._server.gui.add_text("Scan colours",  "green=near → blue=far  (/truth/scan)")
            self._server.gui.add_text("Amber cloud",   "occupancy grid     (/slam/grid)")

        @self._btn_pause.on_click
        def _on_pause(_) -> None:
            self._paused = not self._paused

        @self._btn_newmap.on_click
        def _on_newmap(_) -> None:
            self._reset_flag = True

    # ── point cloud helpers ───────────────────────────────────────────────────

    def _push_truth_trail(self) -> None:
        self._truth_trail.append((self._rx, self._ry))
        pts  = np.array([[p[0], p[1], 0.01] for p in self._truth_trail[-1200:]], np.float32)
        cols = np.tile([30, 120, 255], (len(pts), 1)).astype(np.uint8)
        # World-space path — must NOT be under /truth (which is a moving frame).
        self._server.scene.add_point_cloud("/trails/truth", points=pts, colors=cols, point_size=0.06)

    def _push_slam_trail(self) -> None:
        sx, sy, _ = self._slam.pose
        self._slam_trail.append((sx, sy))
        pts  = np.array([[p[0], p[1], 0.01] for p in self._slam_trail[-1200:]], np.float32)
        cols = np.tile([220, 50, 200], (len(pts), 1)).astype(np.uint8)
        self._server.scene.add_point_cloud("/trails/slam", points=pts, colors=cols, point_size=0.06)

    def _push_scan_cloud(self, x_m: np.ndarray, y_m: np.ndarray, dist_m: np.ndarray) -> None:
        """Current scan rendered in the /truth robot frame (viser auto-transforms it)."""
        pts3 = np.stack([x_m, y_m, np.zeros_like(x_m)], axis=1)
        t    = dist_m / MAX_RANGE
        cols = np.stack([
            np.zeros(len(t), np.uint8),
            ((1 - t) * 210 + 20).astype(np.uint8),  # green channel: bright near
            (t * 180 + 40).astype(np.uint8),         # blue channel:  bright far
        ], axis=1)
        self._server.scene.add_point_cloud("/truth/scan", points=pts3, colors=cols, point_size=0.04)

    # ── GUI refresh ───────────────────────────────────────────────────────────

    def _refresh_gui(self, elapsed: float) -> None:
        sx, sy, st = self._slam.pose
        pos_err = math.hypot(self._rx - sx, self._ry - sy)
        hdg_err = abs(math.degrees(_wrap(self._rtheta - st)))

        self._g_scans.value   = str(self._scan_count)
        self._g_elapsed.value = f"{elapsed:.0f}s"
        self._g_loops.value   = str(self._loop_count)

        self._g_tx.value = f"{self._rx:+.2f}"
        self._g_ty.value = f"{self._ry:+.2f}"
        self._g_th.value = f"{math.degrees(self._rtheta):+.1f}"

        self._g_sx.value   = f"{sx:+.2f}"
        self._g_sy.value   = f"{sy:+.2f}"
        self._g_sh.value   = f"{math.degrees(st):+.1f}"
        self._g_perr.value = f"{pos_err:.3f}"
        self._g_herr.value = f"{hdg_err:.1f}"

        # Pull live slider values
        self._speed       = float(self._sl_speed.value)
        self._range_noise = float(self._sl_rnoise.value)
        self._yaw_noise   = float(self._sl_ynoise.value)

    # ── motion ───────────────────────────────────────────────────────────────

    def _step_toward(self, tx: float, ty: float, dt: float) -> bool:
        dx, dy = tx - self._rx, ty - self._ry
        dist   = math.hypot(dx, dy)
        step   = self._speed * BASE_SPEED_M_S * dt
        if dist <= step:
            self._rx, self._ry = tx, ty
            return True
        self._rtheta = math.atan2(dy, dx)
        self._rx    += (dx / dist) * step
        self._ry    += (dy / dist) * step
        return False

    # ── reset ────────────────────────────────────────────────────────────────

    def _do_reset(self) -> None:
        self._reset_flag = False
        if self._map_name == "random":
            self._seed = None  # new seed → new layout
        self._env = self._make_environment()
        self._rx, self._ry = self._env.waypoints[0]
        self._rtheta  = 0.0
        self._scan_count = 0
        self._wp_idx  = 1
        self._loop_count = 0
        self._rebuild_scene()
        self._g_map.value = self._map_name

    # ── main run loop ────────────────────────────────────────────────────────

    def run(self) -> None:
        dt          = 1.0 / SCAN_HZ
        prev_theta  = self._rtheta
        t_start     = time.time()

        try:
            while True:
                if self._reset_flag:
                    self._do_reset()
                    t_start    = time.time()
                    prev_theta = self._rtheta

                if self._paused:
                    time.sleep(0.05)
                    continue

                wps      = self._env.waypoints
                tx, ty   = wps[self._wp_idx % len(wps)]
                if self._step_toward(tx, ty, dt):
                    self._wp_idx += 1
                    if self._wp_idx % len(wps) == 0:
                        self._loop_count += 1

                noisy_yaw    = math.degrees(self._rtheta) + np.random.normal(0.0, self._yaw_noise)
                yaw_rate_dps = math.degrees(self._rtheta - prev_theta) / dt
                prev_theta   = self._rtheta

                x_m, y_m, dist_m = self._env.scan_fn(
                    self._rx, self._ry, self._rtheta, self._range_noise,
                )
                inten = np.clip((1 - dist_m / MAX_RANGE) * 200 + 55, 0, 255).astype(np.uint8)
                self._scan_count += 1

                self._bridge.push(SimSnapshot(
                    x_m=x_m, y_m=y_m, distance_m=dist_m, intensity=inten,
                    connected=True, imu_connected=True, imu_ok=True,
                    yaw_deg=float(noisy_yaw), yaw_rate_dps=float(yaw_rate_dps),
                    status_text="sim", scan_count=self._scan_count,
                ))

                assert self._truth_frame is not None
                self._truth_frame.position = (self._rx, self._ry, 0.0)
                self._truth_frame.wxyz     = _wxyz(self._rtheta)

                self._push_truth_trail()
                self._push_slam_trail()
                self._push_scan_cloud(x_m, y_m, dist_m)
                self._refresh_gui(time.time() - t_start)

                time.sleep(dt)
        finally:
            if self._slam is not None:
                self._slam.stop()


# ── entry point ───────────────────────────────────────────────────────────────

def main() -> None:
    parser = argparse.ArgumentParser(description="SLAM simulator — no hardware required")
    parser.add_argument("--web-port", type=int, default=8080)
    parser.add_argument("--map",  choices=MAP_CHOICES, default="random",
                        help="World layout  (default: random)")
    parser.add_argument("--seed", type=int, default=None,
                        help="RNG seed for --map random; omit for a new map each run")
    args = parser.parse_args()

    server = viser.ViserServer(host="0.0.0.0", port=args.web_port)

    @server.on_client_connect
    def _on_connect(client: viser.ClientHandle) -> None:
        client.camera.position = (0.0, 0.0, 30.0)
        client.camera.look_at  = (0.0, 0.0, 0.0)
        client.camera.up       = (0.0, 1.0, 0.0)
        client.camera.fov      = 0.9

    print(f"Open http://localhost:{args.web_port}")
    print("Blue trail = ground truth  |  Magenta trail = SLAM estimate")
    print("Ctrl-C to stop")
    sim = Simulator(map_name=args.map, seed=args.seed, server=server)
    try:
        sim.run()
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
