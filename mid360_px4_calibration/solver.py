from __future__ import annotations

import numpy as np
from typing import Optional, Tuple
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation

from .filtering import angular_acceleration, savgol_filter_samples, trim_samples
from .interpolation import ImuInterpolator, paired_times
from .math_utils import ensure_rotation_matrix, validation_stats
from .observability import acceptance_notes, rotation_observability_eigenvalues
from .rotation import gyro_residuals, solve_rotation_and_time
from .time_offset import estimate_time_offset
from .translation import accel_residuals, build_translation_system, solve_translation
from .types import CalibrationConfig, CalibrationResult, ImuSamples


def calibrate(mid_raw: ImuSamples, body_raw: ImuSamples, config: Optional[CalibrationConfig] = None) -> CalibrationResult:
    config = config or CalibrationConfig()
    mid = trim_samples(savgol_filter_samples(mid_raw, config.filter_window_s, config.filter_polyorder), config.trim_s)
    body = trim_samples(savgol_filter_samples(body_raw, config.filter_window_s, config.filter_polyorder), config.trim_s)
    time = estimate_time_offset(
        mid,
        body,
        config.time_search_min_s,
        config.time_search_max_s,
        config.time_search_step_s,
    )
    rotation, dt = solve_rotation_and_time(
        mid,
        body,
        initial_dt=time.dt_B_wrt_M,
        time_bounds=(config.time_search_min_s, config.time_search_max_s),
        max_samples=config.max_samples,
        gyro_sigma=config.gyro_sigma_rad_s,
        robust_loss=config.robust_loss,
    )
    alpha_m = angular_acceleration(mid, config.filter_window_s, config.filter_polyorder)
    R_M_B = rotation.R_B_M.T
    translation = solve_translation(
        mid,
        body,
        alpha_m,
        R_M_B,
        dt,
        config.max_samples,
        huber_delta=max(config.accel_sigma_m_s2 * 2.0, 1.0e-6),
    )
    R_M_B, t_M_B, dt, gyro_bias, accel_bias = _joint_refine(
        mid,
        body,
        alpha_m,
        R_M_B,
        translation.translation_M_B_m,
        dt,
        rotation.gyro_bias_B_minus_rotated_M,
        translation.accel_bias_rotated_B_minus_M,
        config,
    )

    R_B_M = R_M_B.T
    gyro_stats = validation_stats(gyro_residuals(mid, body, R_B_M, dt, gyro_bias, config.max_samples))
    accel_stats = validation_stats(accel_residuals(mid, body, alpha_m, R_M_B, t_M_B, dt, accel_bias, config.max_samples))
    rot_eig = rotation_observability_eigenvalues(mid)
    times = paired_times(mid, body, dt, config.max_samples)
    H, _ = build_translation_system(mid, body, alpha_m, R_M_B, dt, times)
    J_only = H.reshape(-1, 3, 6)[:, :, :3]
    J_centered = J_only - np.mean(J_only, axis=0, keepdims=True)
    info = np.einsum("nij,nik->jk", J_centered, J_centered)
    trans_eig = np.linalg.eigvalsh(info)
    trans_cond = float("inf") if trans_eig[0] <= 0.0 else float(trans_eig[-1] / trans_eig[0])

    result = CalibrationResult(
        R_M_B=R_M_B,
        translation_M_B_m=t_M_B,
        dt_B_wrt_M=dt,
        gyro_bias_B_minus_rotated_M=gyro_bias,
        accel_bias_rotated_B_minus_M=accel_bias,
        gyro_stats=gyro_stats,
        accel_stats=accel_stats,
        time_offset=time,
        rotation_success=rotation.success,
        translation_information_eigenvalues=trans_eig,
        translation_condition_number=trans_cond,
        rotation_observability_eigenvalues=rot_eig,
        accepted=False,
    )
    accepted, notes = acceptance_notes(result, config)
    result.accepted = accepted
    result.notes = notes
    return result


def _joint_refine(
    mid: ImuSamples,
    body: ImuSamples,
    alpha_m: np.ndarray,
    R_M_B0: np.ndarray,
    t_M_B0: np.ndarray,
    dt0: float,
    gyro_bias0: np.ndarray,
    accel_bias0: np.ndarray,
    config: CalibrationConfig,
) -> Tuple[np.ndarray, np.ndarray, float, np.ndarray, np.ndarray]:
    x0 = np.r_[Rotation.from_matrix(R_M_B0).as_rotvec(), t_M_B0, dt0, gyro_bias0, accel_bias0]
    lower = np.r_[[-np.inf] * 6, config.time_search_min_s, [-np.inf] * 6]
    upper = np.r_[[np.inf] * 6, config.time_search_max_s, [np.inf] * 6]

    def residual_fn(x: np.ndarray) -> np.ndarray:
        R_M_B = Rotation.from_rotvec(x[:3]).as_matrix()
        R_B_M = R_M_B.T
        t_M_B = x[3:6]
        dt = float(x[6])
        gyro_bias = x[7:10]
        accel_bias = x[10:13]
        times = paired_times(mid, body, dt, config.max_samples)
        mid_interp = ImuInterpolator(mid)
        body_interp = ImuInterpolator(body)
        omega_m = mid_interp.gyro(times)
        omega_b = body_interp.gyro(times + dt)
        gyro_r = omega_b - (R_B_M @ omega_m.T).T - gyro_bias

        H, y = build_translation_system(mid, body, alpha_m, R_M_B, dt, times)
        accel_r = (H @ np.r_[t_M_B, accel_bias] - y.reshape(-1)).reshape(-1, 3)
        return np.r_[
            (gyro_r / config.gyro_sigma_rad_s).reshape(-1),
            (accel_r / config.accel_sigma_m_s2).reshape(-1),
        ]

    opt = least_squares(
        residual_fn,
        x0,
        bounds=(lower, upper),
        loss=config.robust_loss,
        f_scale=1.0,
        max_nfev=120,
        x_scale="jac",
    )
    R_M_B = ensure_rotation_matrix(Rotation.from_rotvec(opt.x[:3]).as_matrix())
    return R_M_B, opt.x[3:6].copy(), float(opt.x[6]), opt.x[7:10].copy(), opt.x[10:13].copy()
