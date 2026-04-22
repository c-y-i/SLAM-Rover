#!/usr/bin/env python3
"""
Matplotlib SLAM simulator.

Runs the same IMU-assisted ICP + occupancy-grid pipeline as `slam.slam_thread`,
but renders results as plots (static or live-updating).

Usage (from repo root):
    python -m slam.plot_sim
    python -m slam.plot_sim --map nav2_tb3_sandbox

Defaults:
    - map = random
    - live GUI updates enabled
    - auto-save final plot to artifacts/plots/ (keeps latest 5 final plots)
"""

from __future__ import annotations

import argparse
import math
from dataclasses import dataclass
from pathlib import Path
import sys
import time
from typing import Callable

import numpy as np

if __package__ in (None, ""):
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
    from slam.icp import icp_2d, scan_to_points, voxel_downsample
    from slam.occupancy_grid import OccupancyGrid
    from slam.sim_world import BASE_SPEED_M_S, MAP_BUILDERS, MAX_RANGE, N_RAYS, SCAN_HZ, cast_scan
else:
    from .icp import icp_2d, scan_to_points, voxel_downsample
    from .occupancy_grid import OccupancyGrid
    from .sim_world import BASE_SPEED_M_S, MAP_BUILDERS, MAX_RANGE, N_RAYS, SCAN_HZ, cast_scan

MAX_ICP_CORRECTION = math.radians(8.0)
MAX_ICP_TRANSLATION_M = 0.35
MIN_POINTS = 20
MAX_REPO_PLOTS = 5
FINAL_PLOT_GLOB = "slam_plot_final_*.png"

STATIC_MAP_YAMLS = {
    "nav2_tb3_sandbox": Path(__file__).resolve().parent / "static_maps" / "nav2" / "tb3_sandbox.yaml",
    "nav2_warehouse": Path(__file__).resolve().parent / "static_maps" / "nav2" / "warehouse.yaml",
    "nav2_depot": Path(__file__).resolve().parent / "static_maps" / "nav2" / "depot.yaml",
}

MAP_CHOICES = list(MAP_BUILDERS.keys()) + list(STATIC_MAP_YAMLS.keys())


@dataclass
class SimResult:
    truth_xy: np.ndarray
    est_xy: np.ndarray
    pos_err_m: np.ndarray
    heading_err_deg: np.ndarray
    grid: OccupancyGrid
    walls: list[tuple[tuple[float, float], tuple[float, float]]]
    world_bounds: tuple[float, float, float, float]  # x_min, x_max, y_min, y_max
    reference_img: np.ndarray | None = None
    reference_extent: tuple[float, float, float, float] | None = None


@dataclass
class Metrics:
    rmse_m: float
    final_pos_err_m: float
    mean_heading_err_deg: float


@dataclass
class StaticMap:
    blocked: np.ndarray  # (H, W), row 0 = bottom row in world frame
    free: np.ndarray
    resolution: float
    origin_x: float
    origin_y: float

    @property
    def height(self) -> int:
        return int(self.blocked.shape[0])

    @property
    def width(self) -> int:
        return int(self.blocked.shape[1])

    @property
    def bounds(self) -> tuple[float, float, float, float]:
        return (
            self.origin_x,
            self.origin_x + self.width * self.resolution,
            self.origin_y,
            self.origin_y + self.height * self.resolution,
        )


@dataclass
class SimEnvironment:
    waypoints: list[tuple[float, float]]
    walls: list[tuple[tuple[float, float], tuple[float, float]]]
    world_bounds: tuple[float, float, float, float]  # x_min, x_max, y_min, y_max
    scan_fn: Callable[[float, float, float, float], tuple[np.ndarray, np.ndarray, np.ndarray]]
    reference_img: np.ndarray | None = None
    reference_extent: tuple[float, float, float, float] | None = None


def _wrap(theta: float) -> float:
    return (theta + math.pi) % (2.0 * math.pi) - math.pi


def _step_toward(
    rx: float,
    ry: float,
    rtheta: float,
    tx: float,
    ty: float,
    dt: float,
    speed_scale: float,
) -> tuple[float, float, float, bool]:
    dx = tx - rx
    dy = ty - ry
    dist = math.hypot(dx, dy)
    step = speed_scale * BASE_SPEED_M_S * dt
    if dist <= step:
        return tx, ty, rtheta, True
    rtheta = math.atan2(dy, dx)
    rx += (dx / dist) * step
    ry += (dy / dist) * step
    return rx, ry, rtheta, False


def _compute_metrics(result: SimResult) -> Metrics:
    rmse = float(np.sqrt(np.mean(result.pos_err_m ** 2))) if result.pos_err_m.size else 0.0
    final_err = float(result.pos_err_m[-1]) if result.pos_err_m.size else 0.0
    mean_hdg = float(np.mean(result.heading_err_deg)) if result.heading_err_deg.size else 0.0
    return Metrics(rmse_m=rmse, final_pos_err_m=final_err, mean_heading_err_deg=mean_hdg)


def _format_hms(seconds: float) -> str:
    total = max(0, int(seconds))
    h = total // 3600
    m = (total % 3600) // 60
    s = total % 60
    return f"{h:02d}:{m:02d}:{s:02d}"


def _wall_bounds(
    walls: list[tuple[tuple[float, float], tuple[float, float]]],
) -> tuple[float, float, float, float]:
    xs: list[float] = []
    ys: list[float] = []
    for (x0, y0), (x1, y1) in walls:
        xs.extend((x0, x1))
        ys.extend((y0, y1))
    if not xs:
        return (-10.0, 10.0, -10.0, 10.0)
    margin = 1.0
    return (min(xs) - margin, max(xs) + margin, min(ys) - margin, max(ys) + margin)


def _repo_plot_path(map_name: str) -> Path:
    root = Path(__file__).resolve().parents[1]
    out_dir = root / "artifacts" / "plots"
    out_dir.mkdir(parents=True, exist_ok=True)
    t = time.time()
    stamp = time.strftime("%Y%m%d_%H%M%S", time.localtime(t))
    millis = int((t % 1.0) * 1000.0)
    return out_dir / f"slam_plot_final_{map_name}_{stamp}_{millis:03d}.png"


def _repo_plots_dir() -> Path:
    return Path(__file__).resolve().parents[1] / "artifacts" / "plots"


def _prune_repo_plots(max_keep: int = MAX_REPO_PLOTS) -> None:
    plots_dir = _repo_plots_dir()
    if not plots_dir.exists():
        return

    files = sorted(
        plots_dir.glob(FINAL_PLOT_GLOB),
        key=lambda p: p.stat().st_mtime,
        reverse=True,
    )
    for old_file in files[max_keep:]:
        try:
            old_file.unlink()
            print(f"Pruned old plot: {old_file}")
        except OSError:
            pass


def _read_token(f) -> bytes | None:
    token = bytearray()
    while True:
        ch = f.read(1)
        if not ch:
            return bytes(token) if token else None
        if ch == b"#":
            f.readline()
            if token:
                return bytes(token)
            continue
        if ch.isspace():
            if token:
                return bytes(token)
            continue
        token.extend(ch)


def _load_pgm(path: Path) -> np.ndarray:
    with path.open("rb") as f:
        magic = _read_token(f)
        if magic not in (b"P5", b"P2"):
            raise ValueError(f"Unsupported PGM format in {path}: {magic}")

        w_tok = _read_token(f)
        h_tok = _read_token(f)
        max_tok = _read_token(f)
        if w_tok is None or h_tok is None or max_tok is None:
            raise ValueError(f"Invalid PGM header in {path}")

        width = int(w_tok)
        height = int(h_tok)
        maxval = int(max_tok)

        if magic == b"P5":
            if maxval < 256:
                data = np.frombuffer(f.read(width * height), dtype=np.uint8)
            else:
                data = np.frombuffer(f.read(width * height * 2), dtype=">u2")
                data = (data.astype(np.float64) * (255.0 / maxval)).astype(np.uint8)
        else:
            vals: list[int] = []
            while True:
                tok = _read_token(f)
                if tok is None:
                    break
                vals.append(int(tok))
            data = np.asarray(vals, dtype=np.float64)
            data = (data * (255.0 / maxval)).astype(np.uint8)

    if data.size != width * height:
        raise ValueError(f"PGM size mismatch in {path}: got {data.size}, expected {width * height}")
    return data.reshape((height, width))


def _parse_yaml_scalar(value: str):
    v = value.strip().strip('"').strip("'")
    if not v:
        return v
    if v.startswith("[") and v.endswith("]"):
        body = v[1:-1].strip()
        if not body:
            return []
        return [float(x.strip()) for x in body.split(",")]
    try:
        if any(c in v for c in (".", "e", "E")):
            return float(v)
        return int(v)
    except ValueError:
        return v


def _load_map_yaml(path: Path) -> dict[str, object]:
    data: dict[str, object] = {}
    for line in path.read_text().splitlines():
        line = line.split("#", 1)[0].strip()
        if not line or ":" not in line:
            continue
        k, v = line.split(":", 1)
        data[k.strip()] = _parse_yaml_scalar(v)
    return data


def _load_static_map(yaml_path: Path) -> StaticMap:
    meta = _load_map_yaml(yaml_path)
    image_field = str(meta.get("image", "")).strip()
    if not image_field:
        raise ValueError(f"Missing image field in {yaml_path}")

    pgm_path = (yaml_path.parent / image_field).resolve()
    img = _load_pgm(pgm_path).astype(np.float64)

    resolution = float(meta.get("resolution", 0.05))
    origin = meta.get("origin", [0.0, 0.0, 0.0])
    if not isinstance(origin, list) or len(origin) < 2:
        origin = [0.0, 0.0, 0.0]
    origin_x = float(origin[0])
    origin_y = float(origin[1])

    negate = int(meta.get("negate", 0))
    occ_thresh = float(meta.get("occupied_thresh", 0.65))
    free_thresh = float(meta.get("free_thresh", 0.196))

    if negate == 0:
        occ_prob = (255.0 - img) / 255.0
    else:
        occ_prob = img / 255.0

    occupied = occ_prob > occ_thresh
    free = occ_prob < free_thresh
    unknown = ~(occupied | free)

    # World frame rows grow upward; flip image rows so row 0 is world-bottom.
    occupied = np.flipud(occupied)
    free = np.flipud(free)
    unknown = np.flipud(unknown)

    blocked = occupied | unknown
    return StaticMap(
        blocked=blocked,
        free=free,
        resolution=resolution,
        origin_x=origin_x,
        origin_y=origin_y,
    )


def _world_to_grid(static_map: StaticMap, x: float, y: float) -> tuple[float, float]:
    gx = (x - static_map.origin_x) / static_map.resolution
    gy = (y - static_map.origin_y) / static_map.resolution
    return gx, gy


def _grid_to_world(static_map: StaticMap, c: float, r: float) -> tuple[float, float]:
    x = static_map.origin_x + (c + 0.5) * static_map.resolution
    y = static_map.origin_y + (r + 0.5) * static_map.resolution
    return x, y


def _cell_safe(static_map: StaticMap, row: int, col: int, radius_cells: int = 2) -> bool:
    if not (0 <= row < static_map.height and 0 <= col < static_map.width):
        return False
    if static_map.blocked[row, col]:
        return False
    r0 = max(0, row - radius_cells)
    r1 = min(static_map.height, row + radius_cells + 1)
    c0 = max(0, col - radius_cells)
    c1 = min(static_map.width, col + radius_cells + 1)
    return not static_map.blocked[r0:r1, c0:c1].any()


def _line_is_free(static_map: StaticMap, r0: int, c0: int, r1: int, c1: int) -> bool:
    steps = int(max(abs(r1 - r0), abs(c1 - c0)) * 2) + 1
    for k in range(steps + 1):
        t = k / max(1, steps)
        rr = int(round(r0 + (r1 - r0) * t))
        cc = int(round(c0 + (c1 - c0) * t))
        if not (0 <= rr < static_map.height and 0 <= cc < static_map.width):
            return False
        if static_map.blocked[rr, cc]:
            return False
    return True


def _sample_static_waypoints(static_map: StaticMap, seed: int | None, count: int = 10) -> list[tuple[float, float]]:
    rng = np.random.default_rng(seed)

    def sample_cell() -> tuple[int, int]:
        for _ in range(5000):
            rr = int(rng.integers(0, static_map.height))
            cc = int(rng.integers(0, static_map.width))
            if _cell_safe(static_map, rr, cc, radius_cells=2):
                return rr, cc
        raise RuntimeError("Could not find a safe free cell in static map")

    points_cells: list[tuple[int, int]] = []
    points_cells.append(sample_cell())

    min_dist_m = 1.5
    max_dist_m = 8.0
    min_dist_cells = min_dist_m / static_map.resolution
    max_dist_cells = max_dist_m / static_map.resolution

    while len(points_cells) < count:
        prev_r, prev_c = points_cells[-1]
        chosen: tuple[int, int] | None = None
        for _ in range(1000):
            rr, cc = sample_cell()
            d_cells = math.hypot(rr - prev_r, cc - prev_c)
            if d_cells < min_dist_cells or d_cells > max_dist_cells:
                continue
            if _line_is_free(static_map, prev_r, prev_c, rr, cc):
                chosen = (rr, cc)
                break
        if chosen is None:
            chosen = sample_cell()
        points_cells.append(chosen)

    return [_grid_to_world(static_map, cc, rr) for rr, cc in points_cells]


def _raycast_grid(static_map: StaticMap, rx: float, ry: float, theta: float, max_range_m: float) -> float:
    gx, gy = _world_to_grid(static_map, rx, ry)
    ix = int(math.floor(gx))
    iy = int(math.floor(gy))

    if not (0 <= ix < static_map.width and 0 <= iy < static_map.height):
        return max_range_m

    if static_map.blocked[iy, ix]:
        return 0.02

    dir_x = math.cos(theta) / static_map.resolution
    dir_y = math.sin(theta) / static_map.resolution

    if abs(dir_x) < 1e-12:
        step_x = 0
        t_max_x = float("inf")
        t_delta_x = float("inf")
    elif dir_x > 0:
        step_x = 1
        t_max_x = (ix + 1 - gx) / dir_x
        t_delta_x = 1.0 / dir_x
    else:
        step_x = -1
        t_max_x = (gx - ix) / (-dir_x)
        t_delta_x = 1.0 / (-dir_x)

    if abs(dir_y) < 1e-12:
        step_y = 0
        t_max_y = float("inf")
        t_delta_y = float("inf")
    elif dir_y > 0:
        step_y = 1
        t_max_y = (iy + 1 - gy) / dir_y
        t_delta_y = 1.0 / dir_y
    else:
        step_y = -1
        t_max_y = (gy - iy) / (-dir_y)
        t_delta_y = 1.0 / (-dir_y)

    t = 0.0
    for _ in range(10000):
        if t >= max_range_m:
            return max_range_m
        if not (0 <= ix < static_map.width and 0 <= iy < static_map.height):
            return min(max_range_m, max(0.02, t))
        if static_map.blocked[iy, ix]:
            return min(max_range_m, max(0.02, t))

        if t_max_x < t_max_y:
            t = t_max_x
            t_max_x += t_delta_x
            ix += step_x
        else:
            t = t_max_y
            t_max_y += t_delta_y
            iy += step_y

    return max_range_m


def _cast_scan_static(
    static_map: StaticMap,
    rx: float,
    ry: float,
    theta: float,
    noise_m: float,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    world_angles = np.linspace(0.0, 2.0 * math.pi, N_RAYS, endpoint=False) + theta
    ranges = np.empty(N_RAYS, dtype=np.float32)

    for i, ang in enumerate(world_angles):
        ranges[i] = _raycast_grid(static_map, rx, ry, float(ang), MAX_RANGE)

    if noise_m > 0:
        ranges += np.random.normal(0.0, noise_m, N_RAYS).astype(np.float32)

    ranges = np.clip(ranges, 0.02, MAX_RANGE).astype(np.float32)
    robot_angles = np.linspace(0.0, 2.0 * math.pi, N_RAYS, endpoint=False)
    x_m = (ranges * np.cos(robot_angles)).astype(np.float32)
    y_m = (ranges * np.sin(robot_angles)).astype(np.float32)
    return x_m, y_m, ranges


def _build_environment(map_name: str, seed: int | None) -> SimEnvironment:
    if map_name in MAP_BUILDERS:
        builder = MAP_BUILDERS[map_name]
        world = builder(seed) if map_name == "random" else builder()
        bounds = _wall_bounds(world.walls)
        return SimEnvironment(
            waypoints=world.waypoints,
            walls=world.walls,
            world_bounds=bounds,
            scan_fn=lambda rx, ry, th, noise: cast_scan(rx, ry, th, world.walls, noise),
            reference_img=None,
            reference_extent=None,
        )

    if map_name in STATIC_MAP_YAMLS:
        yaml_path = STATIC_MAP_YAMLS[map_name]
        if not yaml_path.exists():
            raise SystemExit(f"Static map YAML not found: {yaml_path}")

        static_map = _load_static_map(yaml_path)
        waypoints = _sample_static_waypoints(static_map, seed=seed, count=12)

        x0, x1, y0, y1 = static_map.bounds
        boundary = [
            ((x0, y0), (x1, y0)),
            ((x1, y0), (x1, y1)),
            ((x1, y1), (x0, y1)),
            ((x0, y1), (x0, y0)),
        ]

        # Lighter free space, darker blocked space.
        reference_img = np.where(static_map.blocked, 40, 220).astype(np.uint8)
        reference_extent = (x0, x1, y0, y1)

        return SimEnvironment(
            waypoints=waypoints,
            walls=boundary,
            world_bounds=(x0, x1, y0, y1),
            scan_fn=lambda rx, ry, th, noise: _cast_scan_static(static_map, rx, ry, th, noise),
            reference_img=reference_img,
            reference_extent=reference_extent,
        )

    raise SystemExit(f"Unknown map: {map_name}")


def run_plot_sim(
    map_name: str,
    seed: int | None,
    steps: int,
    speed: float,
    range_noise_m: float,
    yaw_noise_deg: float,
    sim_seed: int | None,
    scan_hz: float,
    step_callback: Callable[[int, SimResult], None] | None = None,
    update_every: int = 25,
    environment: SimEnvironment | None = None,
) -> SimResult:
    if sim_seed is not None:
        np.random.seed(sim_seed)

    env = environment or _build_environment(map_name, seed)

    dt = 1.0 / scan_hz
    rx, ry = env.waypoints[0]
    rtheta = 0.0
    wp_idx = 1

    grid = OccupancyGrid()

    est_x = rx
    est_y = ry
    est_theta = 0.0
    prev_scan: np.ndarray | None = None
    prev_yaw_deg: float | None = None

    truth_pts: list[tuple[float, float]] = []
    est_pts: list[tuple[float, float]] = []
    pos_err: list[float] = []
    heading_err: list[float] = []

    stride = max(1, update_every)

    for i in range(steps):
        tx, ty = env.waypoints[wp_idx % len(env.waypoints)]
        rx, ry, rtheta, reached = _step_toward(rx, ry, rtheta, tx, ty, dt, speed)
        if reached:
            wp_idx += 1

        noisy_yaw_deg = math.degrees(rtheta) + np.random.normal(0.0, yaw_noise_deg)

        x_m, y_m, dist_m = env.scan_fn(rx, ry, rtheta, range_noise_m)
        pts = voxel_downsample(scan_to_points(x_m, y_m, dist_m))

        if pts.shape[0] >= MIN_POINTS:
            if prev_scan is None or prev_yaw_deg is None:
                prev_scan = pts
                prev_yaw_deg = noisy_yaw_deg
                est_theta = math.radians(noisy_yaw_deg)
            else:
                delta_theta = math.radians(noisy_yaw_deg - prev_yaw_deg)
                delta_theta = _wrap(delta_theta)
                prev_theta = est_theta

                R, t, ok = icp_2d(pts, prev_scan, init_theta=delta_theta)
                icp_theta = math.atan2(R[1, 0], R[0, 0])
                icp_correction = _wrap(icp_theta - delta_theta)
                translation_norm = float(np.linalg.norm(t))

                # Trust IMU heading for robustness; use ICP mainly for translation.
                est_theta = prev_theta + delta_theta

                if (
                    ok
                    and abs(icp_correction) <= MAX_ICP_CORRECTION
                    and translation_norm <= MAX_ICP_TRANSLATION_M
                ):
                    c = math.cos(prev_theta)
                    s = math.sin(prev_theta)
                    est_x += c * float(t[0]) - s * float(t[1])
                    est_y += s * float(t[0]) + c * float(t[1])

                prev_scan = pts
                prev_yaw_deg = noisy_yaw_deg

            grid.insert(pts, (est_x, est_y, est_theta))

        truth_pts.append((rx, ry))
        est_pts.append((est_x, est_y))
        pos_err.append(math.hypot(rx - est_x, ry - est_y))
        heading_err.append(abs(math.degrees(_wrap(rtheta - est_theta))))

        if step_callback and (((i + 1) % stride == 0) or (i == steps - 1)):
            partial = SimResult(
                truth_xy=np.asarray(truth_pts, dtype=np.float64),
                est_xy=np.asarray(est_pts, dtype=np.float64),
                pos_err_m=np.asarray(pos_err, dtype=np.float64),
                heading_err_deg=np.asarray(heading_err, dtype=np.float64),
                grid=grid,
                walls=env.walls,
                world_bounds=env.world_bounds,
                reference_img=env.reference_img,
                reference_extent=env.reference_extent,
            )
            step_callback(i + 1, partial)

    return SimResult(
        truth_xy=np.asarray(truth_pts, dtype=np.float64),
        est_xy=np.asarray(est_pts, dtype=np.float64),
        pos_err_m=np.asarray(pos_err, dtype=np.float64),
        heading_err_deg=np.asarray(heading_err, dtype=np.float64),
        grid=grid,
        walls=env.walls,
        world_bounds=env.world_bounds,
        reference_img=env.reference_img,
        reference_extent=env.reference_extent,
    )


def _save_figure(fig: object, save_path: Path | None) -> None:
    if save_path is None:
        return
    save_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(str(save_path), bbox_inches="tight")
    print(f"Saved plot: {save_path}")
    try:
        if save_path.resolve().parent == _repo_plots_dir().resolve():
            _prune_repo_plots()
    except OSError:
        pass


def plot_result(
    result: SimResult,
    map_name: str,
    range_noise_m: float,
    yaw_noise_deg: float,
    speed: float,
    scan_hz: float,
    save_path: Path | None,
    show: bool,
) -> None:
    try:
        import matplotlib.pyplot as plt
    except ModuleNotFoundError as exc:
        raise SystemExit(
            "matplotlib is required for slam.plot_sim. "
            "Install with: pip install -r py_scripts/requirements.txt"
        ) from exc

    metrics = _compute_metrics(result)
    print(
        "SLAM plot metrics: "
        f"RMSE={metrics.rmse_m:.3f} m, final_pos_err={metrics.final_pos_err_m:.3f} m, "
        f"mean_heading_err={metrics.mean_heading_err_deg:.2f} deg"
    )

    img = result.grid.to_image()[:, :, 0]
    x_min, y_min, x_max, y_max = result.grid.world_bounds

    fig, (ax_map, ax_err) = plt.subplots(1, 2, figsize=(14, 6), dpi=130)

    if result.reference_img is not None and result.reference_extent is not None:
        rx0, rx1, ry0, ry1 = result.reference_extent
        ax_map.imshow(
            result.reference_img,
            cmap="gray",
            origin="lower",
            extent=[rx0, rx1, ry0, ry1],
            vmin=0,
            vmax=255,
            alpha=0.35,
        )

    ax_map.imshow(
        img,
        cmap="gray",
        origin="lower",
        extent=[x_min, x_max, y_min, y_max],
        vmin=0,
        vmax=255,
        alpha=0.95,
    )

    for (ax0, ay0), (ax1, ay1) in result.walls:
        ax_map.plot([ax0, ax1], [ay0, ay1], color="white", linewidth=1.5, alpha=0.85)

    ax_map.plot(result.truth_xy[:, 0], result.truth_xy[:, 1], color="#2f66ff", linewidth=2.0, label="ground truth")
    ax_map.plot(result.est_xy[:, 0], result.est_xy[:, 1], color="#ff2fcf", linewidth=2.0, label="estimated")
    ax_map.scatter(result.truth_xy[0, 0], result.truth_xy[0, 1], color="#00d27d", s=40, label="start")
    ax_map.scatter(result.truth_xy[-1, 0], result.truth_xy[-1, 1], color="#ffb000", s=40, label="end")

    bx0, bx1, by0, by1 = result.world_bounds
    ax_map.set_xlim(bx0, bx1)
    ax_map.set_ylim(by0, by1)

    ax_map.set_aspect("equal", adjustable="box")
    ax_map.set_title(
        f"SLAM Map/Traj ({map_name})\\nRMSE={metrics.rmse_m:.3f}m  final={metrics.final_pos_err_m:.3f}m  mean heading err={metrics.mean_heading_err_deg:.2f}deg"
    )
    ax_map.set_xlabel("X (m)")
    ax_map.set_ylabel("Y (m)")
    ax_map.legend(loc="upper right")
    ax_map.grid(True, alpha=0.2)

    scans = np.arange(result.pos_err_m.shape[0])
    ax_err.plot(scans, result.pos_err_m, color="#ff5a5a", linewidth=1.8, label="position error (m)")
    ax_err.plot(scans, result.heading_err_deg, color="#21b4b4", linewidth=1.4, label="heading error (deg)")
    ax_err.set_title(
        f"Error Over Time\\nspeed={speed:.2f}x  scan_hz={scan_hz:.1f}  range_noise={range_noise_m:.3f}m  yaw_noise={yaw_noise_deg:.2f}deg"
    )
    ax_err.set_xlabel("Scan index")
    ax_err.set_ylabel("Error")
    ax_err.grid(True, alpha=0.3)
    ax_err.legend(loc="upper left")

    fig.tight_layout()
    _save_figure(fig, save_path)

    if show:
        plt.show()
    else:
        plt.close(fig)


def plot_result_live(
    map_name: str,
    steps: int,
    speed: float,
    scan_hz: float,
    range_noise_m: float,
    yaw_noise_deg: float,
    seed: int | None,
    sim_seed: int | None,
    update_every: int,
    progress_every: int,
    run_start_time: float,
    save_path: Path | None,
) -> None:
    try:
        import matplotlib.pyplot as plt
    except ModuleNotFoundError as exc:
        raise SystemExit(
            "matplotlib is required for slam.plot_sim. "
            "Install with: pip install -r py_scripts/requirements.txt"
        ) from exc

    env = _build_environment(map_name, seed)

    plt.ion()
    fig, (ax_map, ax_err) = plt.subplots(1, 2, figsize=(14, 6), dpi=130)

    x_min, x_max, y_min, y_max = env.world_bounds
    ax_map.set_xlim(x_min, x_max)
    ax_map.set_ylim(y_min, y_max)
    ax_map.set_aspect("equal", adjustable="box")
    ax_map.set_xlabel("X (m)")
    ax_map.set_ylabel("Y (m)")
    ax_map.grid(True, alpha=0.2)

    if env.reference_img is not None and env.reference_extent is not None:
        rx0, rx1, ry0, ry1 = env.reference_extent
        ax_map.imshow(
            env.reference_img,
            cmap="gray",
            origin="lower",
            extent=[rx0, rx1, ry0, ry1],
            vmin=0,
            vmax=255,
            alpha=0.35,
        )

    for (ax0, ay0), (ax1, ay1) in env.walls:
        ax_map.plot([ax0, ax1], [ay0, ay1], color="white", linewidth=1.5, alpha=0.85)

    img_artist = ax_map.imshow(
        np.full((10, 10), 128, dtype=np.uint8),
        cmap="gray",
        origin="lower",
        extent=[x_min, x_max, y_min, y_max],
        vmin=0,
        vmax=255,
        alpha=0.90,
    )

    truth_line, = ax_map.plot([], [], color="#2f66ff", linewidth=2.0, label="ground truth")
    est_line, = ax_map.plot([], [], color="#ff2fcf", linewidth=2.0, label="estimated")
    ax_map.legend(loc="upper right")

    pos_line, = ax_err.plot([], [], color="#ff5a5a", linewidth=1.8, label="position error (m)")
    hdg_line, = ax_err.plot([], [], color="#21b4b4", linewidth=1.4, label="heading error (deg)")
    ax_err.set_xlabel("Scan index")
    ax_err.set_ylabel("Error")
    ax_err.grid(True, alpha=0.3)
    ax_err.legend(loc="upper left")

    def _on_step(step_idx: int, partial: SimResult) -> None:
        slam_img = partial.grid.to_image()[:, :, 0]
        gx0, gy0, gx1, gy1 = partial.grid.world_bounds
        img_artist.set_data(slam_img)
        img_artist.set_extent([gx0, gx1, gy0, gy1])

        truth_line.set_data(partial.truth_xy[:, 0], partial.truth_xy[:, 1])
        est_line.set_data(partial.est_xy[:, 0], partial.est_xy[:, 1])

        scans = np.arange(partial.pos_err_m.shape[0])
        pos_line.set_data(scans, partial.pos_err_m)
        hdg_line.set_data(scans, partial.heading_err_deg)
        ax_err.relim()
        ax_err.autoscale_view()

        metrics = _compute_metrics(partial)
        ax_map.set_title(
            f"Live SLAM ({map_name}) step {step_idx}/{steps}\\nRMSE={metrics.rmse_m:.3f}m  final={metrics.final_pos_err_m:.3f}m"
        )
        ax_err.set_title(
            f"Error Over Time\\nspeed={speed:.2f}x  scan_hz={scan_hz:.1f}  range_noise={range_noise_m:.3f}m  yaw_noise={yaw_noise_deg:.2f}deg"
        )

        if step_idx % max(1, progress_every) == 0 or step_idx == steps:
            elapsed = time.time() - run_start_time
            rate = step_idx / elapsed if elapsed > 1e-6 else 0.0
            eta = (steps - step_idx) / rate if rate > 1e-6 else float("inf")
            print(
                f"[progress] {step_idx}/{steps} ({100.0 * step_idx / max(1, steps):5.1f}%) "
                f"elapsed={_format_hms(elapsed)} eta={_format_hms(eta)} rmse={metrics.rmse_m:.3f}m"
            )

        fig.canvas.draw_idle()
        plt.pause(0.001)

    result = run_plot_sim(
        map_name=map_name,
        seed=seed,
        steps=steps,
        speed=speed,
        range_noise_m=range_noise_m,
        yaw_noise_deg=yaw_noise_deg,
        sim_seed=sim_seed,
        scan_hz=scan_hz,
        step_callback=_on_step,
        update_every=update_every,
        environment=env,
    )

    metrics = _compute_metrics(result)
    print(
        "SLAM plot metrics: "
        f"RMSE={metrics.rmse_m:.3f} m, final_pos_err={metrics.final_pos_err_m:.3f} m, "
        f"mean_heading_err={metrics.mean_heading_err_deg:.2f} deg"
    )

    fig.tight_layout()
    _save_figure(fig, save_path)
    plt.ioff()
    plt.show()


def main() -> None:
    parser = argparse.ArgumentParser(description="Matplotlib SLAM simulator")
    parser.add_argument("--map", choices=MAP_CHOICES, default="random")
    parser.add_argument("--seed", type=int, default=None, help="Seed for random/generated map setup")
    parser.add_argument("--sim-seed", type=int, default=42, help="Noise seed for reproducible runs")
    parser.add_argument("--steps", type=int, default=1200, help="Number of simulated scan steps")
    parser.add_argument("--scan-hz", type=float, default=SCAN_HZ)
    parser.add_argument("--speed", type=float, default=1.5, help="Robot speed scale")
    parser.add_argument("--range-noise", type=float, default=0.02, help="LiDAR range noise (m)")
    parser.add_argument("--yaw-noise", type=float, default=0.5, help="IMU yaw noise (deg)")
    parser.add_argument(
        "--live",
        dest="live",
        action="store_true",
        default=True,
        help="Update plot live during simulation (default)",
    )
    parser.add_argument(
        "--no-live",
        dest="live",
        action="store_false",
        help="Disable live redraw and render once at end",
    )
    parser.add_argument("--update-every", type=int, default=25, help="Live redraw interval in steps")
    parser.add_argument("--progress-every", type=int, default=50, help="Terminal progress interval in steps")
    parser.add_argument("--save", type=str, default="", help="Save figure to a specific image file")
    parser.add_argument(
        "--save-repo",
        dest="save_repo",
        action="store_true",
        default=True,
        help="Auto-save final figure to artifacts/plots in the repo (default)",
    )
    parser.add_argument(
        "--no-save-repo",
        dest="save_repo",
        action="store_false",
        help="Disable auto-save to artifacts/plots",
    )
    parser.add_argument("--no-show", action="store_true", help="Do not open interactive window")
    args = parser.parse_args()

    save_path: Path | None = None
    if args.save:
        save_path = Path(args.save).expanduser().resolve()
    elif args.save_repo:
        save_path = _repo_plot_path(args.map)

    print(
        "Running slam.plot_sim: "
        f"map={args.map}, steps={args.steps}, speed={args.speed}, "
        f"range_noise={args.range_noise}, yaw_noise={args.yaw_noise}, "
        f"live={args.live}, update_every={max(1, args.update_every)}, "
        f"progress_every={max(1, args.progress_every)}"
    )
    if args.map in STATIC_MAP_YAMLS:
        print(f"Using static map YAML: {STATIC_MAP_YAMLS[args.map]}")
    if save_path is not None:
        print(f"Plot output path: {save_path}")

    t0 = time.time()

    if args.live and not args.no_show:
        plot_result_live(
            map_name=args.map,
            steps=args.steps,
            speed=args.speed,
            scan_hz=args.scan_hz,
            range_noise_m=args.range_noise,
            yaw_noise_deg=args.yaw_noise,
            seed=args.seed,
            sim_seed=args.sim_seed,
            update_every=max(1, args.update_every),
            progress_every=max(1, args.progress_every),
            run_start_time=t0,
            save_path=save_path,
        )
    else:
        if args.live and args.no_show:
            print("Note: --live ignored with --no-show (running static mode).")

        def _progress_step(step_idx: int, partial: SimResult) -> None:
            metrics = _compute_metrics(partial)
            elapsed = time.time() - t0
            rate = step_idx / elapsed if elapsed > 1e-6 else 0.0
            eta = (args.steps - step_idx) / rate if rate > 1e-6 else float("inf")
            print(
                f"[progress] {step_idx}/{args.steps} ({100.0 * step_idx / max(1, args.steps):5.1f}%) "
                f"elapsed={_format_hms(elapsed)} eta={_format_hms(eta)} rmse={metrics.rmse_m:.3f}m"
            )

        result = run_plot_sim(
            map_name=args.map,
            seed=args.seed,
            steps=args.steps,
            speed=args.speed,
            range_noise_m=args.range_noise,
            yaw_noise_deg=args.yaw_noise,
            sim_seed=args.sim_seed,
            scan_hz=args.scan_hz,
            step_callback=_progress_step,
            update_every=max(1, args.progress_every),
        )

        plot_result(
            result=result,
            map_name=args.map,
            range_noise_m=args.range_noise,
            yaw_noise_deg=args.yaw_noise,
            speed=args.speed,
            scan_hz=args.scan_hz,
            save_path=save_path,
            show=not args.no_show,
        )

    print(f"Simulation finished in {time.time() - t0:.1f}s")


if __name__ == "__main__":
    main()
