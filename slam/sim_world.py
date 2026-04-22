"""Shared simulator world generation and LiDAR ray-casting utilities."""

from __future__ import annotations

import math
from typing import NamedTuple

import numpy as np

Wall = tuple[tuple[float, float], tuple[float, float]]


class World(NamedTuple):
    walls: list[Wall]
    waypoints: list[tuple[float, float]]
    room_w: float
    room_h: float


def _outer(w: float, h: float) -> list[Wall]:
    hw, hh = w / 2, h / 2
    return [
        ((-hw, -hh), (hw, -hh)),
        ((hw, -hh), (hw, hh)),
        ((hw, hh), (-hw, hh)),
        ((-hw, hh), (-hw, -hh)),
    ]


def _box(cx: float, cy: float, bw: float, bh: float) -> list[Wall]:
    x0, y0, x1, y1 = cx - bw / 2, cy - bh / 2, cx + bw / 2, cy + bh / 2
    return [
        ((x0, y0), (x1, y0)),
        ((x1, y0), (x1, y1)),
        ((x1, y1), (x0, y1)),
        ((x0, y1), (x0, y0)),
    ]


def _random_waypoints(
    room_w: float,
    room_h: float,
    obstacles: list[tuple[float, float, float, float]],
    n: int,
    rng: np.random.Generator,
) -> list[tuple[float, float]]:
    margin = 1.5
    pts: list[tuple[float, float]] = []
    for _ in range(n * 60):
        if len(pts) >= n:
            break
        x = float(rng.uniform(-room_w / 2 + margin, room_w / 2 - margin))
        y = float(rng.uniform(-room_h / 2 + margin, room_h / 2 - margin))
        if not any(
            abs(x - cx) < bw / 2 + 0.8 and abs(y - cy) < bh / 2 + 0.8
            for cx, cy, bw, bh in obstacles
        ):
            pts.append((x, y))
    return pts or [(0.0, 0.0)]


def build_random(seed: int | None = None) -> World:
    rng = np.random.default_rng(seed)
    w_room, h_room = 20.0, 14.0
    walls = _outer(w_room, h_room)
    boxes: list[tuple[float, float, float, float]] = []

    for _ in range(int(rng.integers(4, 8)) * 6):
        if len(boxes) >= 7:
            break
        bw = float(rng.uniform(1.0, 4.0))
        bh = float(rng.uniform(1.0, 4.0))
        cx = float(rng.uniform(-w_room / 2 + 2.5, w_room / 2 - 2.5))
        cy = float(rng.uniform(-h_room / 2 + 2.5, h_room / 2 - 2.5))
        if any(
            abs(cx - ox) < (bw + ow) / 2 + 1.0 and abs(cy - oy) < (bh + oh) / 2 + 1.0
            for ox, oy, ow, oh in boxes
        ):
            continue
        boxes.append((cx, cy, bw, bh))
        walls += _box(cx, cy, bw, bh)

    for _ in range(int(rng.integers(3, 6))):
        cx = float(rng.uniform(-w_room / 2 + 2, w_room / 2 - 2))
        cy = float(rng.uniform(-h_room / 2 + 2, h_room / 2 - 2))
        length = float(rng.uniform(2.0, 5.0))
        angle = float(rng.choice([0.0, math.pi / 2, math.pi / 4, -math.pi / 4]))
        dx, dy = length / 2 * math.cos(angle), length / 2 * math.sin(angle)
        walls.append(((cx - dx, cy - dy), (cx + dx, cy + dy)))

    wps = _random_waypoints(w_room, h_room, boxes, 10, rng)
    return World(walls=walls, waypoints=wps, room_w=w_room, room_h=h_room)


def build_office() -> World:
    walls = _outer(18.0, 12.0)
    walls += [
        ((0.0, -6.0), (0.0, -1.8)),
        ((0.0, 1.8), (0.0, 6.0)),
    ]
    walls += _box(-6.0, -3.0, 2.0, 1.2)
    walls += _box(-6.0, 3.0, 2.0, 1.2)
    walls += _box(5.0, 0.0, 2.5, 4.0)
    walls += _box(5.0, -4.0, 1.5, 1.5)
    wps = [(-7, -5), (-7, 5), (-1, 0), (1, -4), (7, -5), (7, 5), (1, 4), (-1, 0)]
    return World(walls=walls, waypoints=wps, room_w=18.0, room_h=12.0)


def build_warehouse() -> World:
    walls = _outer(22.0, 16.0)
    for row_y in (-5.0, 0.0, 5.0):
        for col_x in (-7.0, -3.0, 1.0, 5.0):
            walls += _box(col_x, row_y, 1.2, 3.0)
    wps = [(-9, -7), (9, -7), (9, 7), (-9, 7), (0, -2.5), (0, 2.5)]
    return World(walls=walls, waypoints=wps, room_w=22.0, room_h=16.0)


def build_maze() -> World:
    walls = _outer(18.0, 16.0)
    for i, y in enumerate((-6.0, -2.0, 2.0, 6.0)):
        if i % 2 == 0:
            walls.append(((-8.0, y), (2.5, y)))
        else:
            walls.append(((-2.5, y), (8.0, y)))
    for px, py in ((-6, -4), (6, 4), (0, 0)):
        walls += _box(px, py, 0.8, 0.8)
    wps = [(-6, -7), (6, -7), (6, -3), (-6, 1), (6, 1), (-6, 5), (6, 7), (-6, 7)]
    return World(walls=walls, waypoints=wps, room_w=18.0, room_h=16.0)


MAP_BUILDERS = {
    "random": build_random,
    "office": build_office,
    "warehouse": build_warehouse,
    "maze": build_maze,
}

N_RAYS = 360
MAX_RANGE = 8.0
SCAN_HZ = 10.0
BASE_SPEED_M_S = 1.0


def _ray_hit(
    ox: float,
    oy: float,
    dx: float,
    dy: float,
    ax: float,
    ay: float,
    bx: float,
    by: float,
) -> float | None:
    sx, sy = bx - ax, by - ay
    denom = dx * sy - dy * sx
    if abs(denom) < 1e-10:
        return None
    qpx, qpy = ax - ox, ay - oy
    t = (qpx * sy - qpy * sx) / denom
    u = (qpx * dy - qpy * dx) / denom
    return t if t >= 0.0 and 0.0 <= u <= 1.0 else None


def cast_scan(
    rx: float,
    ry: float,
    theta: float,
    walls: list[Wall],
    noise_m: float,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    world_a = np.linspace(0.0, 2 * math.pi, N_RAYS, endpoint=False) + theta
    ranges = np.full(N_RAYS, MAX_RANGE)
    for i, ang in enumerate(world_a):
        dx, dy = math.cos(ang), math.sin(ang)
        for (ax, ay), (bx, by) in walls:
            d = _ray_hit(rx, ry, dx, dy, ax, ay, bx, by)
            if d is not None and d < ranges[i]:
                ranges[i] = d
    if noise_m > 0:
        ranges += np.random.normal(0.0, noise_m, N_RAYS)
    ranges = np.clip(ranges, 0.02, MAX_RANGE).astype(np.float32)
    robot_a = np.linspace(0.0, 2 * math.pi, N_RAYS, endpoint=False)
    x_m = (ranges * np.cos(robot_a)).astype(np.float32)
    y_m = (ranges * np.sin(robot_a)).astype(np.float32)
    return x_m, y_m, ranges
