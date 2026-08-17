from __future__ import annotations

import numpy as np
from typing import Tuple
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation

from .interpolation import ImuInterpolator, paired_times
from .math_utils import ensure_rotation_matrix
from .types import ImuSamples, RotationResult


def solve_rotation_and_time(
    mid: ImuSamples,
    body: ImuSamples,
    initial_dt: float,
    time_bounds: Tuple[float, float],
    max_samples: int,
    gyro_sigma: float,
    robust_loss: str,
) -> Tuple[RotationResult, float]:
    times = paired_times(mid, body, initial_dt, max_samples)
    body_interp = ImuInterpolator(body)
    omega_m = mid.gyro[np.searchsorted(mid.timestamps, times)]
    omega_b = body_interp.gyro(times + initial_dt)
    R_B_M0 = wahba_rotation(omega_m, omega_b)
    bg0 = np.mean(omega_b - (R_B_M0 @ omega_m.T).T, axis=0)
    initial_residual = _gyro_residual_vector(mid, body, R_B_M0, initial_dt, bg0, times, gyro_sigma)
    initial_cost = float(0.5 * np.dot(initial_residual, initial_residual))

    x0 = np.r_[Rotation.from_matrix(R_B_M0).as_rotvec(), initial_dt, bg0]
    lower = np.array([-np.inf, -np.inf, -np.inf, time_bounds[0], -np.inf, -np.inf, -np.inf])
    upper = np.array([np.inf, np.inf, np.inf, time_bounds[1], np.inf, np.inf, np.inf])

    def residual_fn(x: np.ndarray) -> np.ndarray:
        R_B_M = Rotation.from_rotvec(x[:3]).as_matrix()
        dt = float(x[3])
        bg = x[4:7]
        local_times = paired_times(mid, body, dt, max_samples)
        return _gyro_residual_vector(mid, body, R_B_M, dt, bg, local_times, gyro_sigma)

    opt = least_squares(
        residual_fn,
        x0,
        bounds=(lower, upper),
        loss=robust_loss,
        f_scale=1.0,
        max_nfev=200,
        x_scale="jac",
    )
    R_B_M = ensure_rotation_matrix(Rotation.from_rotvec(opt.x[:3]).as_matrix())
    result = RotationResult(
        R_B_M=R_B_M,
        gyro_bias_B_minus_rotated_M=opt.x[4:7].copy(),
        initial_cost=initial_cost,
        refined_cost=float(opt.cost),
        success=bool(opt.success),
        message=str(opt.message),
    )
    return result, float(opt.x[3])


def wahba_rotation(omega_m: np.ndarray, omega_b: np.ndarray) -> np.ndarray:
    x = np.asarray(omega_m, dtype=float)
    y = np.asarray(omega_b, dtype=float)
    x = x - np.mean(x, axis=0)
    y = y - np.mean(y, axis=0)
    H = y.T @ x
    U, _, Vt = np.linalg.svd(H)
    S = np.eye(3)
    if np.linalg.det(U @ Vt) < 0.0:
        S[-1, -1] = -1.0
    return U @ S @ Vt


def gyro_residuals(
    mid: ImuSamples,
    body: ImuSamples,
    R_B_M: np.ndarray,
    dt_B_wrt_M: float,
    gyro_bias: np.ndarray,
    max_samples: int,
) -> np.ndarray:
    times = paired_times(mid, body, dt_B_wrt_M, max_samples)
    body_interp = ImuInterpolator(body)
    omega_m = ImuInterpolator(mid).gyro(times)
    omega_b = body_interp.gyro(times + dt_B_wrt_M)
    return omega_b - (R_B_M @ omega_m.T).T - gyro_bias


def _gyro_residual_vector(
    mid: ImuSamples,
    body: ImuSamples,
    R_B_M: np.ndarray,
    dt_B_wrt_M: float,
    gyro_bias: np.ndarray,
    times: np.ndarray,
    gyro_sigma: float,
) -> np.ndarray:
    omega_m = ImuInterpolator(mid).gyro(times)
    omega_b = ImuInterpolator(body).gyro(times + dt_B_wrt_M)
    residual = omega_b - (R_B_M @ omega_m.T).T - gyro_bias
    return (residual / gyro_sigma).reshape(-1)
