"""Viser scene construction helpers."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np
import viser

from . import config
from .geometry import ZoneGeometry, distances_to_points


def _yaw_to_wxyz(yaw_deg: float) -> tuple[float, float, float, float]:
    yaw_rad = np.deg2rad(yaw_deg)
    return (float(np.cos(yaw_rad / 2.0)), 0.0, 0.0, float(np.sin(yaw_rad / 2.0)))


@dataclass
class SceneHandles:
    rig: viser.FrameHandle
    imu_board: viser.FrameHandle
    tof_board: viser.FrameHandle
    tof_sensor: viser.FrameHandle
    zone_rays: list[object]


def create_scene(server: viser.ViserServer, zone_geometry: ZoneGeometry) -> SceneHandles:
    """Create a simple rig scene with IMU and ToF board frames."""

    server.scene.add_frame("/origin", axes_length=0.02, axes_radius=0.001)
    server.scene.add_grid(
        "/grid",
        width=1.0,
        height=1.0,
        width_segments=10,
        height_segments=10,
        plane="xy",
    )

    rig = server.scene.add_frame("/rig", show_axes=False)

    imu_board_position = tuple(
        np.array(config.IMU_BOARD.world_position) - np.array(config.IMU_BOARD.sensor_offset)
    )
    imu_board = server.scene.add_frame("/rig/imu_board", show_axes=False, position=imu_board_position)
    server.scene.add_box(
        "/rig/imu_board/body",
        dimensions=config.IMU_BOARD.dimensions,
        color=config.IMU_BOARD.color,
    )
    server.scene.add_frame(
        "/rig/imu_board/sensor",
        position=config.IMU_BOARD.sensor_offset,
        axes_length=0.015,
        axes_radius=0.001,
    )

    tof_board_position = tuple(
        np.array(config.TOF_BOARD.world_position) - np.array(config.TOF_BOARD.sensor_offset)
    )
    tof_board = server.scene.add_frame("/rig/tof_board", show_axes=False, position=tof_board_position)
    server.scene.add_box(
        "/rig/tof_board/body",
        dimensions=config.TOF_BOARD.dimensions,
        color=config.TOF_BOARD.color,
    )
    tof_sensor = server.scene.add_frame(
        "/rig/tof_board/sensor",
        position=config.TOF_BOARD.sensor_offset,
        axes_length=0.015,
        axes_radius=0.001,
        wxyz=_yaw_to_wxyz(config.TOF_BOARD.sensor_yaw_deg),
    )

    zone_rays = create_zone_rays(server, zone_geometry)
    return SceneHandles(
        rig=rig,
        imu_board=imu_board,
        tof_board=tof_board,
        tof_sensor=tof_sensor,
        zone_rays=zone_rays,
    )


def create_zone_rays(server: viser.ViserServer, zone_geometry: ZoneGeometry) -> list[object]:
    """Create one ray per VL53L5CX zone."""

    min_points = distances_to_points(
        np.full(config.NUM_ZONES, config.MIN_RANGE_MM, dtype=np.float32),
        zone_geometry,
    )
    max_points = distances_to_points(
        np.full(config.NUM_ZONES, config.MAX_RANGE_MM, dtype=np.float32),
        zone_geometry,
    )

    rays: list[object] = []
    for index in range(config.NUM_ZONES):
        ray = server.scene.add_spline_catmull_rom(
            f"/rig/tof_board/sensor/rays/ray_{index}",
            positions=np.array([min_points[index], max_points[index]], dtype=np.float32),
            color=(110, 165, 255),
            line_width=1.2,
        )
        rays.append(ray)

    return rays


def set_rays_visible(rays: list[object], visible: bool) -> None:
    for ray in rays:
        ray.visible = visible
