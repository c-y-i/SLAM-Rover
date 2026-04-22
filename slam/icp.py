"""Scan preprocessing and 2-D point-to-point ICP."""

from __future__ import annotations

import math

import numpy as np
from scipy.spatial import KDTree


def scan_to_points(x_m: np.ndarray, y_m: np.ndarray, distance_m: np.ndarray,
                   max_dist_m: float = 8.0) -> np.ndarray:
    """Return (N, 2) array of valid scan points in robot frame."""
    mask = (distance_m > 0.05) & (distance_m < max_dist_m)
    return np.stack([x_m[mask], y_m[mask]], axis=1).astype(np.float32)


def voxel_downsample(pts: np.ndarray, cell_m: float = 0.05) -> np.ndarray:
    """Keep one point per voxel cell — reduces ICP cost significantly."""
    if pts.shape[0] == 0:
        return pts
    keys = (pts / cell_m).astype(np.int32)
    _, idx = np.unique(keys, axis=0, return_index=True)
    return pts[idx]


def rotation_2d(theta: float) -> np.ndarray:
    """2×2 rotation matrix for angle theta (radians)."""
    c, s = math.cos(theta), math.sin(theta)
    return np.array([[c, -s], [s, c]], dtype=np.float64)


def icp_2d(
    source: np.ndarray,
    target: np.ndarray,
    init_theta: float = 0.0,
    max_iter: int = 20,
    tol: float = 1e-4,
    outlier_factor: float = 3.0,
    min_correspondences: int = 10,
    max_correspondence_dist: float = 0.6,
) -> tuple[np.ndarray, np.ndarray, bool]:
    """
    Point-to-point ICP aligning `source` onto `target`.

    Parameters
    ----------
    source, target : (N, 2) and (M, 2) float arrays in the same frame.
    init_theta     : initial rotation guess in radians (from IMU delta).

    Returns
    -------
    R   : (2, 2) rotation matrix
    t   : (2,) translation vector
    ok  : True if ICP converged with enough correspondences
    """
    source = source.astype(np.float64)
    target = target.astype(np.float64)

    R = rotation_2d(init_theta)
    t = np.zeros(2, dtype=np.float64)

    tree = KDTree(target)
    ok = False

    for _ in range(max_iter):
        src_transformed = (R @ source.T).T + t
        dist, idx = tree.query(src_transformed, workers=-1)

        # reject outliers beyond outlier_factor × median distance
        med = np.median(dist)
        if med == 0.0:
            ok = True
            break
        mask = (dist < outlier_factor * med) & (dist < max_correspondence_dist)
        if mask.sum() < min_correspondences:
            break

        src_m = source[mask]
        tgt_m = target[idx[mask]]

        # SVD closed-form solve
        src_mean = src_m.mean(axis=0)
        tgt_mean = tgt_m.mean(axis=0)
        H = (src_m - src_mean).T @ (tgt_m - tgt_mean)
        U, _, Vt = np.linalg.svd(H)
        R_new = Vt.T @ U.T
        if np.linalg.det(R_new) < 0:
            Vt[-1] *= -1
            R_new = Vt.T @ U.T
        t_new = tgt_mean - R_new @ src_mean

        delta_R_angle = abs(math.atan2(R_new[1, 0], R_new[0, 0]) - math.atan2(R[1, 0], R[0, 0]))
        delta_t = np.linalg.norm(t_new - t)
        R, t = R_new, t_new

        if delta_t + delta_R_angle < tol:
            ok = True
            break

    return R, t, ok
