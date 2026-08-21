from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional

import numpy as np


@dataclass(frozen=True)
class ImuSamples:
    timestamps: np.ndarray
    gyro: np.ndarray
    accel: np.ndarray
    frame_id: str = ""

    def __post_init__(self) -> None:
        object.__setattr__(self, "timestamps", np.asarray(self.timestamps, dtype=float))
        object.__setattr__(self, "gyro", np.asarray(self.gyro, dtype=float))
        object.__setattr__(self, "accel", np.asarray(self.accel, dtype=float))
        if self.timestamps.ndim != 1:
            raise ValueError("timestamps must be a 1-D array")
        if self.gyro.shape != (len(self.timestamps), 3):
            raise ValueError("gyro must have shape (N, 3)")
        if self.accel.shape != (len(self.timestamps), 3):
            raise ValueError("accel must have shape (N, 3)")
        if len(self.timestamps) < 3:
            raise ValueError("at least 3 IMU samples are required")

    @property
    def start(self) -> float:
        return float(self.timestamps[0])

    @property
    def end(self) -> float:
        return float(self.timestamps[-1])

    @property
    def median_dt(self) -> float:
        return float(np.median(np.diff(self.timestamps)))

    @property
    def frequency_hz(self) -> float:
        return 1.0 / self.median_dt

    def with_values(self, gyro: Optional[np.ndarray] = None, accel: Optional[np.ndarray] = None) -> "ImuSamples":
        return ImuSamples(
            timestamps=self.timestamps.copy(),
            gyro=self.gyro.copy() if gyro is None else gyro,
            accel=self.accel.copy() if accel is None else accel,
            frame_id=self.frame_id,
        )


@dataclass
class CalibrationConfig:
    mid_frame: str = "mid360_imu"
    body_frame: str = "base_link"
    # Convert raw topic units to SI units before calibration. For example,
    # set mid_accel_scale=9.80665 when a Livox IMU publishes acceleration in g.
    mid_accel_scale: float = 1.0
    body_accel_scale: float = 1.0
    mid_gyro_scale: float = 1.0
    body_gyro_scale: float = 1.0
    time_search_min_s: float = -0.5
    time_search_max_s: float = 0.5
    time_search_step_s: Optional[float] = None
    max_samples: int = 6000
    filter_window_s: float = 0.08
    filter_polyorder: int = 3
    trim_s: float = 0.5
    gyro_sigma_rad_s: float = 0.03
    accel_sigma_m_s2: float = 0.2
    robust_loss: str = "huber"
    accepted_gyro_rmse_rad_s: float = 0.12
    accepted_accel_rmse_m_s2: float = 0.8
    min_rotation_eigenvalue: float = 1.0e-3
    min_translation_eigenvalue: float = 1.0e-3
    max_translation_condition_number: float = 1.0e6


@dataclass
class TimeOffsetResult:
    dt_B_wrt_M: float
    peak: float
    second_peak: float
    peak_ratio: float
    lag_values: List[float] = field(default_factory=list)
    correlation_values: List[float] = field(default_factory=list)


@dataclass
class RotationResult:
    R_B_M: np.ndarray
    gyro_bias_B_minus_rotated_M: np.ndarray
    initial_cost: float
    refined_cost: float
    success: bool
    message: str


@dataclass
class TranslationResult:
    translation_M_B_m: np.ndarray
    accel_bias_rotated_B_minus_M: np.ndarray
    rmse_m_s2: float
    information_eigenvalues: np.ndarray
    condition_number: float


@dataclass
class ValidationStats:
    mean: np.ndarray
    std: np.ndarray
    rmse: float
    max_abs: np.ndarray


@dataclass
class CalibrationResult:
    R_M_B: np.ndarray
    translation_M_B_m: np.ndarray
    dt_B_wrt_M: float
    gyro_bias_B_minus_rotated_M: np.ndarray
    accel_bias_rotated_B_minus_M: np.ndarray
    gyro_stats: ValidationStats
    accel_stats: ValidationStats
    time_offset: TimeOffsetResult
    rotation_success: bool
    translation_information_eigenvalues: np.ndarray
    translation_condition_number: float
    rotation_observability_eigenvalues: np.ndarray
    accepted: bool
    notes: List[str] = field(default_factory=list)
    split_half_rotation_deg: Optional[float] = None
    split_half_translation_cm: Optional[float] = None
    pointcloud_motion_samples: Optional[int] = None
    pointcloud_mean_icp_error_m: Optional[float] = None

    def as_yaml_dict(self, config: CalibrationConfig) -> Dict[str, Any]:
        from scipy.spatial.transform import Rotation

        quat_xyzw = Rotation.from_matrix(self.R_M_B).as_quat()
        quat_wxyz = [float(quat_xyzw[3]), float(quat_xyzw[0]), float(quat_xyzw[1]), float(quat_xyzw[2])]
        R_B_M = self.R_M_B.T
        t_B_M = -R_B_M @ self.translation_M_B_m
        quat_bm_xyzw = Rotation.from_matrix(R_B_M).as_quat()
        quat_bm_wxyz = [
            float(quat_bm_xyzw[3]),
            float(quat_bm_xyzw[0]),
            float(quat_bm_xyzw[1]),
            float(quat_bm_xyzw[2]),
        ]
        rpy_bm_rad = Rotation.from_matrix(R_B_M).as_euler("xyz", degrees=False)
        rpy_bm_deg = Rotation.from_matrix(R_B_M).as_euler("xyz", degrees=True)
        return {
            "convention": "T_M_B maps PX4 body B to Mid360 frame M",
            "mid_frame": config.mid_frame,
            "body_frame": config.body_frame,
            "time_offset": {
                "definition": "t_B = t_M + dt_B_wrt_M",
                "dt_B_wrt_M": float(self.dt_B_wrt_M),
                "correlation_peak": float(self.time_offset.peak),
                "correlation_second_peak": float(self.time_offset.second_peak),
                "correlation_peak_ratio": float(self.time_offset.peak_ratio),
            },
            "extrinsic": {
                "quaternion_M_B_wxyz": quat_wxyz,
                "rotation_matrix_M_B": [float(v) for v in self.R_M_B.reshape(-1)],
                "translation_M_B_m": [float(v) for v in self.translation_M_B_m],
            },
            "mid360_relative_to_body_enu": {
                "convention": "T_B_M maps Mid360 frame M to PX4 body frame B; ROS/MAVROS ENU-side convention, no NED/FRD conversion",
                "translation_B_M_m": [float(v) for v in t_B_M],
                "rpy_B_M_rad": [float(v) for v in rpy_bm_rad],
                "rpy_B_M_deg": [float(v) for v in rpy_bm_deg],
                "quaternion_B_M_wxyz": quat_bm_wxyz,
                "rotation_matrix_B_M": [float(v) for v in R_B_M.reshape(-1)],
            },
            "combined_bias": {
                "gyro_B_minus_rotated_M_rad_s": [float(v) for v in self.gyro_bias_B_minus_rotated_M],
                "accel_rotated_B_minus_M_m_s2": [float(v) for v in self.accel_bias_rotated_B_minus_M],
            },
            "quality": {
                "gyro_mean_rad_s": [float(v) for v in self.gyro_stats.mean],
                "gyro_std_rad_s": [float(v) for v in self.gyro_stats.std],
                "gyro_rmse_rad_s": float(self.gyro_stats.rmse),
                "gyro_max_abs_rad_s": [float(v) for v in self.gyro_stats.max_abs],
                "accel_mean_m_s2": [float(v) for v in self.accel_stats.mean],
                "accel_std_m_s2": [float(v) for v in self.accel_stats.std],
                "accel_rmse_m_s2": float(self.accel_stats.rmse),
                "accel_max_abs_m_s2": [float(v) for v in self.accel_stats.max_abs],
                "rotation_information_eigenvalues": [float(v) for v in self.rotation_observability_eigenvalues],
                "translation_information_eigenvalues": [float(v) for v in self.translation_information_eigenvalues],
                "translation_condition_number": float(self.translation_condition_number),
                "split_half_rotation_deg": None
                if self.split_half_rotation_deg is None
                else float(self.split_half_rotation_deg),
                "split_half_translation_cm": None
                if self.split_half_translation_cm is None
                else float(self.split_half_translation_cm),
                "pointcloud_motion_samples": None
                if self.pointcloud_motion_samples is None
                else int(self.pointcloud_motion_samples),
                "pointcloud_mean_icp_error_m": None
                if self.pointcloud_mean_icp_error_m is None
                else float(self.pointcloud_mean_icp_error_m),
                "accepted": bool(self.accepted),
                "notes": list(self.notes),
            },
        }
