from __future__ import annotations

import numpy as np
from typing import List, Tuple

from .types import CalibrationConfig, CalibrationResult, ImuSamples


def rotation_observability_eigenvalues(mid: ImuSamples) -> np.ndarray:
    centered = mid.gyro - np.mean(mid.gyro, axis=0)
    cov = centered.T @ centered / max(len(centered) - 1, 1)
    return np.linalg.eigvalsh(cov)


def acceptance_notes(result: CalibrationResult, config: CalibrationConfig) -> Tuple[bool, List[str]]:
    notes: List[str] = []
    if not result.rotation_success:
        notes.append("rotation optimizer did not report success")
    if result.gyro_stats.rmse > config.accepted_gyro_rmse_rad_s:
        notes.append("gyro RMSE is above threshold")
    if result.accel_stats.rmse > config.accepted_accel_rmse_m_s2:
        notes.append("accel RMSE is above threshold")
    if np.min(result.rotation_observability_eigenvalues) < config.min_rotation_eigenvalue:
        notes.append("rotation excitation is weak or close to single-axis")
    if np.min(result.translation_information_eigenvalues) < config.min_translation_eigenvalue:
        notes.append("translation lever-arm excitation is weak")
    if result.translation_condition_number > config.max_translation_condition_number:
        notes.append("translation information matrix is ill-conditioned")
    return len(notes) == 0, notes
