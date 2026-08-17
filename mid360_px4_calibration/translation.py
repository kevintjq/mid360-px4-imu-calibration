from __future__ import annotations

import numpy as np
from typing import Tuple

from .interpolation import ImuInterpolator, paired_times
from .math_utils import hat, robust_rmse
from .types import ImuSamples, TranslationResult


def solve_translation(
    mid: ImuSamples,
    body: ImuSamples,
    alpha_m: np.ndarray,
    R_M_B: np.ndarray,
    dt_B_wrt_M: float,
    max_samples: int,
    huber_delta: float = 1.0,
    irls_iterations: int = 12,
) -> TranslationResult:
    times = paired_times(mid, body, dt_B_wrt_M, max_samples)
    H, y = build_translation_system(mid, body, alpha_m, R_M_B, dt_B_wrt_M, times)
    x, *_ = np.linalg.lstsq(H, y.reshape(-1), rcond=None)

    for _ in range(irls_iterations):
        residual = (H @ x - y.reshape(-1)).reshape(-1, 3)
        norm = np.linalg.norm(residual, axis=1)
        weights = np.ones_like(norm)
        mask = norm > huber_delta
        weights[mask] = huber_delta / np.maximum(norm[mask], 1.0e-12)
        W = np.repeat(np.sqrt(weights), 3)
        x_new, *_ = np.linalg.lstsq(H * W[:, None], y.reshape(-1) * W, rcond=None)
        if np.linalg.norm(x_new - x) < 1.0e-10:
            x = x_new
            break
        x = x_new

    residual = (H @ x - y.reshape(-1)).reshape(-1, 3)
    eig, cond = translation_observability_from_system(mid, alpha_m, times)
    return TranslationResult(
        translation_M_B_m=x[:3].copy(),
        accel_bias_rotated_B_minus_M=x[3:6].copy(),
        rmse_m_s2=robust_rmse(residual),
        information_eigenvalues=eig,
        condition_number=cond,
    )


def build_translation_system(
    mid: ImuSamples,
    body: ImuSamples,
    alpha_m: np.ndarray,
    R_M_B: np.ndarray,
    dt_B_wrt_M: float,
    times: np.ndarray,
) -> Tuple[np.ndarray, np.ndarray]:
    mid_interp = ImuInterpolator(mid)
    body_interp = ImuInterpolator(body)
    omega = mid_interp.gyro(times)
    accel_m = mid_interp.accel(times)
    accel_b = body_interp.accel(times + dt_B_wrt_M)
    alpha = _interp_array(mid.timestamps, alpha_m, times)

    H = np.zeros((3 * len(times), 6), dtype=float)
    y = np.zeros((len(times), 3), dtype=float)
    for i, (om, al) in enumerate(zip(omega, alpha)):
        J = hat(al) + hat(om) @ hat(om)
        H[3 * i : 3 * i + 3, 0:3] = J
        H[3 * i : 3 * i + 3, 3:6] = np.eye(3)
        y[i] = R_M_B @ accel_b[i] - accel_m[i]
    return H, y


def accel_residuals(
    mid: ImuSamples,
    body: ImuSamples,
    alpha_m: np.ndarray,
    R_M_B: np.ndarray,
    translation_M_B_m: np.ndarray,
    dt_B_wrt_M: float,
    accel_bias: np.ndarray,
    max_samples: int,
) -> np.ndarray:
    times = paired_times(mid, body, dt_B_wrt_M, max_samples)
    H, y = build_translation_system(mid, body, alpha_m, R_M_B, dt_B_wrt_M, times)
    x = np.r_[translation_M_B_m, accel_bias]
    return (H @ x - y.reshape(-1)).reshape(-1, 3)


def translation_observability_from_system(
    mid: ImuSamples,
    alpha_m: np.ndarray,
    times: np.ndarray,
) -> Tuple[np.ndarray, float]:
    omega = ImuInterpolator(mid).gyro(times)
    alpha = _interp_array(mid.timestamps, alpha_m, times)
    Js = np.array([hat(al) + hat(om) @ hat(om) for om, al in zip(omega, alpha)])
    J_centered = Js - np.mean(Js, axis=0, keepdims=True)
    info = np.zeros((3, 3), dtype=float)
    for J in J_centered:
        info += J.T @ J
    eig = np.linalg.eigvalsh(info)
    if eig[0] <= 0.0:
        cond = float("inf")
    else:
        cond = float(eig[-1] / eig[0])
    return eig, cond


def _interp_array(t_src: np.ndarray, values: np.ndarray, t_query: np.ndarray) -> np.ndarray:
    out = np.empty((len(t_query), values.shape[1]), dtype=float)
    for axis in range(values.shape[1]):
        out[:, axis] = np.interp(t_query, t_src, values[:, axis])
    return out
