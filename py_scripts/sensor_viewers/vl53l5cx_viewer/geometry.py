"""Geometry helpers for converting VL53L5CX data into 3D points."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np
from scipy.spatial.transform import Rotation

from . import config


@dataclass(frozen=True)
class ZoneGeometry:
    """Precomputed trigonometric values for the VL53L5CX zones."""

    sin_pitch: np.ndarray
    cos_pitch: np.ndarray
    sin_yaw: np.ndarray
    cos_yaw: np.ndarray
    ray_directions: np.ndarray


def compute_zone_geometry() -> ZoneGeometry:
    pitch_rad = np.deg2rad(np.asarray(config.ST_PITCH_ANGLES_DEG, dtype=np.float64))
    yaw_rad = np.deg2rad(np.asarray(config.ST_YAW_ANGLES_DEG, dtype=np.float64))

    sin_pitch = np.sin(pitch_rad)
    cos_pitch = np.cos(pitch_rad)
    sin_yaw = np.sin(yaw_rad)
    cos_yaw = np.cos(yaw_rad)

    ray_directions = np.column_stack(
        [
            -cos_yaw * cos_pitch,
            sin_yaw * cos_pitch,
            sin_pitch,
        ]
    ).astype(np.float32)
    ray_norm = np.linalg.norm(ray_directions, axis=1, keepdims=True)
    ray_directions = ray_directions / np.clip(ray_norm, 1.0e-6, None)

    return ZoneGeometry(
        sin_pitch=sin_pitch.astype(np.float32),
        cos_pitch=cos_pitch.astype(np.float32),
        sin_yaw=sin_yaw.astype(np.float32),
        cos_yaw=cos_yaw.astype(np.float32),
        ray_directions=ray_directions,
    )


def distances_to_points(distances_mm: np.ndarray, zone_geometry: ZoneGeometry) -> np.ndarray:
    """Convert perpendicular VL53L5CX distances into XYZ points in meters."""

    distances_mm = np.asarray(distances_mm, dtype=np.float32)
    perpendicular_m = np.clip(distances_mm, 0.0, None) / 1000.0

    hyp_mm = np.divide(
        distances_mm,
        zone_geometry.sin_pitch,
        out=np.zeros_like(distances_mm, dtype=np.float32),
        where=np.abs(zone_geometry.sin_pitch) > 1.0e-6,
    )
    hyp_m = np.clip(hyp_mm, 0.0, None) / 1000.0

    x = -zone_geometry.cos_yaw * zone_geometry.cos_pitch * hyp_m
    y = zone_geometry.sin_yaw * zone_geometry.cos_pitch * hyp_m
    z = perpendicular_m

    return np.column_stack([x, y, z]).astype(np.float32)


def valid_mask(distances_mm: np.ndarray, status: np.ndarray) -> np.ndarray:
    status = np.asarray(status, dtype=np.uint8)
    distances_mm = np.asarray(distances_mm, dtype=np.float32)
    return np.isin(status, config.VALID_STATUS_CODES) & (distances_mm >= config.MIN_RANGE_MM)


def get_colors(distances_mm: np.ndarray, status: np.ndarray) -> np.ndarray:
    """Color valid points from blue to red and invalid points gray."""

    distances_mm = np.asarray(distances_mm, dtype=np.float32)
    d_norm = np.clip(
        (distances_mm - config.MIN_RANGE_MM) / (config.MAX_RANGE_MM - config.MIN_RANGE_MM),
        0.0,
        1.0,
    )

    r = (d_norm * 255.0).astype(np.uint8)
    g = ((1.0 - np.abs(2.0 * d_norm - 1.0)) * 200.0).astype(np.uint8)
    b = ((1.0 - d_norm) * 255.0).astype(np.uint8)
    colors = np.column_stack([r, g, b]).astype(np.uint8)
    colors[~valid_mask(distances_mm, status)] = np.array([120, 120, 120], dtype=np.uint8)
    return colors


def apply_imu_correction(quaternion_wxyz: np.ndarray) -> np.ndarray:
    """Apply a configurable correction from raw IMU frame to viewer frame."""

    quat = np.asarray(quaternion_wxyz, dtype=np.float64)
    quat_xyzw = np.array([quat[1], quat[2], quat[3], quat[0]], dtype=np.float64)
    imu_rot = Rotation.from_quat(quat_xyzw)
    correction = Rotation.from_euler(
        "xyz",
        config.IMU_QUAT_CORRECTION_EULER_DEG,
        degrees=True,
    )
    corrected_xyzw = (imu_rot * correction).as_quat()
    return np.array(
        [
            corrected_xyzw[3],
            corrected_xyzw[0],
            corrected_xyzw[1],
            corrected_xyzw[2],
        ],
        dtype=np.float32,
    )
