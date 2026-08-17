from __future__ import annotations

import numpy as np


def hat(v: np.ndarray) -> np.ndarray:
    x, y, z = np.asarray(v, dtype=float)
    return np.array(
        [
            [0.0, -z, y],
            [z, 0.0, -x],
            [-y, x, 0.0],
        ]
    )


def robust_rmse(residuals: np.ndarray) -> float:
    residuals = np.asarray(residuals, dtype=float)
    if residuals.size == 0:
        return float("nan")
    return float(np.sqrt(np.mean(np.sum(residuals * residuals, axis=1))))


def validation_stats(residuals: np.ndarray):
    from .types import ValidationStats

    residuals = np.asarray(residuals, dtype=float)
    return ValidationStats(
        mean=np.mean(residuals, axis=0),
        std=np.std(residuals, axis=0),
        rmse=robust_rmse(residuals),
        max_abs=np.max(np.abs(residuals), axis=0),
    )


def downsample_indices(length: int, max_samples: int) -> np.ndarray:
    if length <= max_samples:
        return np.arange(length)
    return np.unique(np.linspace(0, length - 1, max_samples).astype(int))


def ensure_rotation_matrix(R: np.ndarray) -> np.ndarray:
    U, _, Vt = np.linalg.svd(R)
    R_fixed = U @ Vt
    if np.linalg.det(R_fixed) < 0.0:
        U[:, -1] *= -1.0
        R_fixed = U @ Vt
    return R_fixed
