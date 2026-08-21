from __future__ import annotations

from typing import Tuple

import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation

from .interpolation import ImuInterpolator, paired_times
from .math_utils import ensure_rotation_matrix, hat, validation_stats
from .observability import acceptance_notes
from .rotation import solve_rotation_and_time
from .time_offset import estimate_time_offset
from .types import CalibrationConfig, CalibrationResult, ImuSamples, TimeOffsetResult, ValidationStats
from .lidar_motion import LidarMotionSamples


def calibrate_lidar_body(lidar: LidarMotionSamples, body: ImuSamples, config: CalibrationConfig) -> CalibrationResult:
    body = body.with_values(
        gyro=body.gyro * float(config.body_gyro_scale),
        accel=body.accel * float(config.body_accel_scale),
    )
    lidar_imu = lidar.as_imu_samples()
    time = estimate_time_offset(
        lidar_imu,
        body,
        config.time_search_min_s,
        config.time_search_max_s,
        config.time_search_step_s,
    )
    rotation, dt = solve_rotation_and_time(
        lidar_imu,
        body,
        initial_dt=time.dt_B_wrt_M,
        time_bounds=(config.time_search_min_s, config.time_search_max_s),
        max_samples=config.max_samples,
        gyro_sigma=config.gyro_sigma_rad_s,
        robust_loss=config.robust_loss,
    )
    R_L_B0 = rotation.R_B_M.T
    lidar_interp = _LidarInterpolator(lidar)
    body_interp = ImuInterpolator(body)
    alpha_l = _angular_acceleration(lidar)
    R_L_B, t_L_B, dt, bg, ba, gravity = _joint_refine(
        lidar,
        body,
        alpha_l,
        R_L_B0,
        dt,
        rotation.gyro_bias_B_minus_rotated_M,
        config,
    )
    times = paired_times(lidar_imu, body, dt, config.max_samples)
    omega_l = lidar_interp.gyro(times)
    body_gyro = body_interp.gyro(times + dt)
    gyro_residual = body_gyro - (R_L_B.T @ omega_l.T).T - bg
    accel_residual = _accel_residuals(lidar, body, alpha_l, R_L_B, t_L_B, dt, ba, gravity, config.max_samples)
    rot_eig = np.linalg.eigvalsh(np.cov(lidar.gyro.T))
    J = np.asarray([hat(a) + hat(o) @ hat(o) for a, o in zip(_interp(lidar.timestamps, alpha_l, times), omega_l)])
    info = J.reshape(-1, 3, 3).transpose(0, 2, 1) @ J.reshape(-1, 3, 3)
    trans_eig = np.linalg.eigvalsh(np.sum(info, axis=0))
    cond = float("inf") if trans_eig[0] <= 0.0 else float(trans_eig[-1] / trans_eig[0])
    result = CalibrationResult(
        R_M_B=R_L_B,
        translation_M_B_m=t_L_B,
        dt_B_wrt_M=dt,
        gyro_bias_B_minus_rotated_M=bg,
        accel_bias_rotated_B_minus_M=ba,
        gyro_stats=validation_stats(gyro_residual),
        accel_stats=validation_stats(accel_residual),
        time_offset=time,
        rotation_success=rotation.success,
        translation_information_eigenvalues=trans_eig,
        translation_condition_number=cond,
        rotation_observability_eigenvalues=rot_eig,
        accepted=False,
        notes=["calibration frame is the point-cloud/LiDAR frame"],
    )
    result.accepted, notes = acceptance_notes(result, config)
    result.notes.extend(notes)
    if np.linalg.norm(gravity) < 8.0 or np.linalg.norm(gravity) > 11.5:
        result.accepted = False
        result.notes.append("estimated gravity norm is outside the valid range")
    return result


def combine_lidar_sensor_results(
    lidar_to_mid: CalibrationResult,
    lidar_to_body: CalibrationResult,
    config: CalibrationConfig,
) -> CalibrationResult:
    """Chain L<-Mid360 and L<-body calibrations into the requested Mid<-body.

    Each input result stores ``R_L_S`` and ``t_L_S`` even though the generic
    result field is named ``R_M_B``/``translation_M_B_m``.  Therefore:

        R_M_B = R_L_M.T @ R_L_B
        t_M_B = R_L_M.T @ (t_L_B - t_L_M)
    """
    R_L_M = lidar_to_mid.R_M_B
    R_L_B = lidar_to_body.R_M_B
    t_L_M = lidar_to_mid.translation_M_B_m
    t_L_B = lidar_to_body.translation_M_B_m
    R_M_B = ensure_rotation_matrix(R_L_M.T @ R_L_B)
    t_M_B = R_L_M.T @ (t_L_B - t_L_M)
    dt = float(lidar_to_body.dt_B_wrt_M - lidar_to_mid.dt_B_wrt_M)
    time = TimeOffsetResult(
        dt_B_wrt_M=dt,
        peak=min(lidar_to_mid.time_offset.peak, lidar_to_body.time_offset.peak),
        second_peak=max(lidar_to_mid.time_offset.second_peak, lidar_to_body.time_offset.second_peak),
        peak_ratio=min(lidar_to_mid.time_offset.peak_ratio, lidar_to_body.time_offset.peak_ratio),
    )
    notes = [
        "calibration uses point-cloud motion as a common reference",
        "final Mid360-to-body transform is chained from two LiDAR-referenced solves",
    ]
    notes.extend("Mid360 solve: " + note for note in lidar_to_mid.notes)
    notes.extend("body solve: " + note for note in lidar_to_body.notes)
    return CalibrationResult(
        R_M_B=R_M_B,
        translation_M_B_m=t_M_B,
        dt_B_wrt_M=dt,
        gyro_bias_B_minus_rotated_M=lidar_to_body.gyro_bias_B_minus_rotated_M,
        accel_bias_rotated_B_minus_M=lidar_to_body.accel_bias_rotated_B_minus_M,
        gyro_stats=lidar_to_body.gyro_stats,
        accel_stats=ValidationStats(
            mean=lidar_to_body.accel_stats.mean,
            std=lidar_to_body.accel_stats.std,
            rmse=float(np.sqrt(0.5 * (lidar_to_mid.accel_stats.rmse ** 2 + lidar_to_body.accel_stats.rmse ** 2))),
            max_abs=np.maximum(lidar_to_mid.accel_stats.max_abs, lidar_to_body.accel_stats.max_abs),
        ),
        time_offset=time,
        rotation_success=lidar_to_mid.rotation_success and lidar_to_body.rotation_success,
        translation_information_eigenvalues=np.minimum(
            lidar_to_mid.translation_information_eigenvalues,
            lidar_to_body.translation_information_eigenvalues,
        ),
        translation_condition_number=max(
            lidar_to_mid.translation_condition_number,
            lidar_to_body.translation_condition_number,
        ),
        rotation_observability_eigenvalues=np.minimum(
            lidar_to_mid.rotation_observability_eigenvalues,
            lidar_to_body.rotation_observability_eigenvalues,
        ),
        accepted=lidar_to_mid.accepted and lidar_to_body.accepted,
        notes=notes,
    )


def _joint_refine(
    lidar: LidarMotionSamples,
    body: ImuSamples,
    alpha_l: np.ndarray,
    R_L_B0: np.ndarray,
    dt0: float,
    gyro_bias0: np.ndarray,
    config: CalibrationConfig,
) -> Tuple[np.ndarray, np.ndarray, float, np.ndarray, np.ndarray, np.ndarray]:
    x0 = np.r_[Rotation.from_matrix(R_L_B0).as_rotvec(), np.zeros(3), dt0, gyro_bias0, np.zeros(3), [0.0, 0.0, -9.80665]]
    lower = np.r_[[-np.inf] * 6, config.time_search_min_s, [-np.inf] * 9]
    upper = np.r_[[np.inf] * 6, config.time_search_max_s, [np.inf] * 9]

    def residual_fn(x: np.ndarray) -> np.ndarray:
        R_L_B = Rotation.from_rotvec(x[:3]).as_matrix()
        t_L_B = x[3:6]
        dt = float(x[6])
        bg = x[7:10]
        ba = x[10:13]
        gravity = x[13:16]
        times = paired_times(lidar.as_imu_samples(), body, dt, config.max_samples)
        omega_l = _interp(lidar.timestamps, lidar.gyro, times)
        body_interp = ImuInterpolator(body)
        body_gyro = body_interp.gyro(times + dt)
        gyro_r = body_gyro - (R_L_B.T @ omega_l.T).T - bg
        accel_r = _accel_residuals(lidar, body, alpha_l, R_L_B, t_L_B, dt, ba, gravity, config.max_samples, times)
        gravity_r = np.array([(np.linalg.norm(gravity) - 9.80665) / 0.05])
        return np.r_[(gyro_r / config.gyro_sigma_rad_s).reshape(-1), (accel_r / config.accel_sigma_m_s2).reshape(-1), gravity_r]

    opt = least_squares(
        residual_fn,
        x0,
        bounds=(lower, upper),
        loss=config.robust_loss,
        f_scale=1.0,
        max_nfev=80,
        x_scale="jac",
    )
    return (
        ensure_rotation_matrix(Rotation.from_rotvec(opt.x[:3]).as_matrix()),
        opt.x[3:6].copy(),
        float(opt.x[6]),
        opt.x[7:10].copy(),
        opt.x[10:13].copy(),
        opt.x[13:16].copy(),
    )


def _accel_residuals(
    lidar: LidarMotionSamples,
    body: ImuSamples,
    alpha_l: np.ndarray,
    R_L_B: np.ndarray,
    t_L_B: np.ndarray,
    dt: float,
    ba: np.ndarray,
    gravity: np.ndarray,
    max_samples: int,
    times: np.ndarray = None,
) -> np.ndarray:
    if times is None:
        times = paired_times(lidar.as_imu_samples(), body, dt, max_samples)
    omega_l = _interp(lidar.timestamps, lidar.gyro, times)
    alpha = _interp(lidar.timestamps, alpha_l, times)
    accel_world = _interp(lidar.timestamps, lidar.accel_world, times)
    rotations = _interp_rotations(lidar.timestamps, lidar.rotations_world_lidar, times)
    body_accel = ImuInterpolator(body).accel(times + dt)
    specific_lidar = np.asarray([(R.T @ (a - gravity)) for R, a in zip(rotations, accel_world)])
    J = np.asarray([hat(a) + hat(o) @ hat(o) for a, o in zip(alpha, omega_l)])
    return (R_L_B @ body_accel.T).T - specific_lidar - np.einsum("nij,j->ni", J, t_L_B) - ba


class _LidarInterpolator:
    def __init__(self, samples: LidarMotionSamples) -> None:
        self.samples = samples

    def gyro(self, times: np.ndarray) -> np.ndarray:
        return _interp(self.samples.timestamps, self.samples.gyro, times)


def _interp(t: np.ndarray, values: np.ndarray, query: np.ndarray) -> np.ndarray:
    query = np.asarray(query, dtype=float)
    out = np.empty((len(query), values.shape[1]), dtype=float)
    for i in range(values.shape[1]):
        out[:, i] = np.interp(query, t, values[:, i])
    return out


def _interp_rotations(t: np.ndarray, rotations: np.ndarray, query: np.ndarray) -> np.ndarray:
    result = []
    for value in query:
        index = int(np.clip(np.searchsorted(t, value), 1, len(t) - 1))
        u = (value - t[index - 1]) / max(t[index] - t[index - 1], 1.0e-9)
        r0 = Rotation.from_matrix(rotations[index - 1])
        r1 = Rotation.from_matrix(rotations[index])
        result.append((r0 * Rotation.from_rotvec((r0.inv() * r1).as_rotvec() * u)).as_matrix())
    return np.asarray(result)


def _angular_acceleration(lidar: LidarMotionSamples) -> np.ndarray:
    return np.gradient(lidar.gyro, lidar.timestamps, axis=0, edge_order=1)
