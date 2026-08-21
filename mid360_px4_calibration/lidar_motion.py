from __future__ import annotations

"""Small, dependency-light point-cloud motion frontend.

This is intentionally a calibration frontend rather than a replacement for
FAST-LIO.  It estimates scan-to-scan motion with robust point-to-point ICP so
the calibration backend has an independent LiDAR angular/linear motion signal.
For difficult or dynamic scenes, a real LIO odometry source remains preferable.
"""

from dataclasses import dataclass
from typing import Iterable, List, Optional, Sequence, Tuple

import numpy as np
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation

from .filtering import savgol_filter_samples
from .math_utils import ensure_rotation_matrix, hat
from .types import ImuSamples


@dataclass(frozen=True)
class PointCloudScan:
    timestamp: float
    points: np.ndarray
    frame_id: str = ""


@dataclass(frozen=True)
class LidarMotionSamples:
    timestamps: np.ndarray
    gyro: np.ndarray
    accel_world: np.ndarray
    rotations_world_lidar: np.ndarray
    frame_id: str = ""
    fitness: np.ndarray = None

    def __post_init__(self) -> None:
        object.__setattr__(self, "timestamps", np.asarray(self.timestamps, dtype=float))
        object.__setattr__(self, "gyro", np.asarray(self.gyro, dtype=float))
        object.__setattr__(self, "accel_world", np.asarray(self.accel_world, dtype=float))
        object.__setattr__(self, "rotations_world_lidar", np.asarray(self.rotations_world_lidar, dtype=float))
        if self.fitness is None:
            object.__setattr__(self, "fitness", np.ones(len(self.timestamps), dtype=float))
        else:
            object.__setattr__(self, "fitness", np.asarray(self.fitness, dtype=float))
        if len(self.timestamps) < 5:
            raise ValueError("at least 5 LiDAR motion samples are required")
        if self.gyro.shape != (len(self.timestamps), 3):
            raise ValueError("LiDAR gyro must have shape (N, 3)")
        if self.accel_world.shape != (len(self.timestamps), 3):
            raise ValueError("LiDAR acceleration must have shape (N, 3)")
        if self.rotations_world_lidar.shape != (len(self.timestamps), 3, 3):
            raise ValueError("LiDAR rotations must have shape (N, 3, 3)")

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

    def as_imu_samples(self) -> ImuSamples:
        return ImuSamples(self.timestamps, self.gyro, np.zeros_like(self.gyro), self.frame_id)


def estimate_lidar_motion(
    scans: Sequence[PointCloudScan],
    voxel_size: float = 0.15,
    max_correspondence_m: float = 2.0,
    max_iterations: int = 20,
    min_correspondences: int = 80,
    max_points: int = 2500,
) -> LidarMotionSamples:
    ordered = sorted(scans, key=lambda s: s.timestamp)
    if len(ordered) < 5:
        raise ValueError("at least 5 point-cloud scans are required")
    first = _prepare_points(ordered[0].points, voxel_size, max_points)
    if len(first) < min_correspondences:
        raise ValueError("first point cloud has too few usable points")

    timestamps: List[float] = [ordered[0].timestamp]
    rotations: List[np.ndarray] = [np.eye(3)]
    positions: List[np.ndarray] = [np.zeros(3)]
    fitness: List[float] = [0.0]
    previous = first
    previous_timestamp = ordered[0].timestamp
    previous_delta = (np.eye(3), np.zeros(3))

    for scan in ordered[1:]:
        current = _prepare_points(scan.points, voxel_size, max_points)
        if len(current) < min_correspondences:
            continue
        dt = float(scan.timestamp - previous_timestamp)
        if dt <= 1.0e-4 or dt > 2.0:
            continue
        try:
            R_pc, t_pc, score = _icp_point_to_point(
                previous,
                current,
                initial=previous_delta,
                max_correspondence_m=max_correspondence_m,
                max_iterations=max_iterations,
                min_correspondences=min_correspondences,
            )
        except ValueError:
            continue
        R_world_prev = rotations[-1]
        p_world_prev = positions[-1]
        R_world_current = ensure_rotation_matrix(R_world_prev @ R_pc)
        p_world_current = p_world_prev + R_world_prev @ t_pc
        timestamps.append(scan.timestamp)
        rotations.append(R_world_current)
        positions.append(p_world_current)
        fitness.append(score)
        previous = current
        previous_timestamp = scan.timestamp
        previous_delta = (R_pc, t_pc)

    if len(timestamps) < 5:
        raise ValueError("point-cloud ICP produced fewer than 5 valid motion samples")

    timestamps_np = np.asarray(timestamps, dtype=float)
    rotations_np = np.asarray(rotations, dtype=float)
    positions_np = np.asarray(positions, dtype=float)
    # Differentiate the accumulated trajectory only after smoothing. ICP
    # translation noise is otherwise amplified twice by acceleration.
    positions_s = _smooth_vector(timestamps_np, positions_np)
    velocity = np.gradient(positions_s, timestamps_np, axis=0, edge_order=1)
    acceleration = np.gradient(velocity, timestamps_np, axis=0, edge_order=1)
    gyro = np.zeros((len(timestamps_np), 3), dtype=float)
    for i in range(1, len(timestamps_np)):
        dt = timestamps_np[i] - timestamps_np[i - 1]
        R_prev_current = rotations_np[i - 1].T @ rotations_np[i]
        # ICP gives a current->previous rotation. Convert its log vector to
        # the current LiDAR frame, matching a sensor_msgs/Imu gyro vector.
        rotvec_previous = Rotation.from_matrix(R_prev_current).as_rotvec()
        gyro[i] = R_prev_current.T @ rotvec_previous / max(dt, 1.0e-6)
    gyro[0] = gyro[1]
    gyro = _smooth_vector(timestamps_np, gyro)
    return LidarMotionSamples(
        timestamps_np,
        gyro,
        acceleration,
        rotations_np,
        frame_id=next((s.frame_id for s in ordered if s.frame_id), "mid360_lidar"),
        fitness=np.asarray(fitness, dtype=float),
    )


def _prepare_points(points: np.ndarray, voxel_size: float, max_points: int) -> np.ndarray:
    points = np.asarray(points, dtype=float)
    if points.ndim != 2 or points.shape[1] != 3:
        raise ValueError("point cloud must have shape (N, 3)")
    points = points[np.all(np.isfinite(points), axis=1)]
    if len(points) == 0:
        return points
    if voxel_size > 0.0:
        keys = np.floor(points / voxel_size).astype(np.int64)
        _, unique = np.unique(keys, axis=0, return_index=True)
        points = points[np.sort(unique)]
    if len(points) > max_points:
        indices = np.linspace(0, len(points) - 1, max_points).astype(int)
        points = points[indices]
    return points


def _icp_point_to_point(
    reference: np.ndarray,
    current: np.ndarray,
    initial: Tuple[np.ndarray, np.ndarray],
    max_correspondence_m: float,
    max_iterations: int,
    min_correspondences: int,
) -> Tuple[np.ndarray, np.ndarray, float]:
    R, t = initial[0].copy(), initial[1].copy()
    tree = cKDTree(reference)
    last_error = float("inf")
    for _ in range(max_iterations):
        transformed = (R @ current.T).T + t
        distances, indices = tree.query(transformed, k=1)
        mask = distances < max_correspondence_m
        if int(np.sum(mask)) < min_correspondences:
            raise ValueError("not enough ICP correspondences")
        source = transformed[mask]
        target = reference[indices[mask]]
        med = float(np.median(distances[mask]))
        robust = distances[mask] < max(3.0 * med, 0.05)
        if int(np.sum(robust)) >= min_correspondences:
            source = source[robust]
            target = target[robust]
        dR, dt = _rigid_fit(source, target)
        R_new = ensure_rotation_matrix(dR @ R)
        t_new = dR @ t + dt
        error = float(np.sqrt(np.mean(np.sum((source - target) ** 2, axis=1))))
        R, t = R_new, t_new
        if abs(last_error - error) < 1.0e-5 and np.linalg.norm(Rotation.from_matrix(dR).as_rotvec()) < 1.0e-4:
            break
        last_error = error
    if not np.isfinite(last_error):
        raise ValueError("ICP did not converge")
    return R, t, last_error


def _rigid_fit(source: np.ndarray, target: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
    source_mean = np.mean(source, axis=0)
    target_mean = np.mean(target, axis=0)
    H = (source - source_mean).T @ (target - target_mean)
    U, _, Vt = np.linalg.svd(H)
    R = Vt.T @ U.T
    if np.linalg.det(R) < 0.0:
        Vt[-1, :] *= -1.0
        R = Vt.T @ U.T
    t = target_mean - R @ source_mean
    return R, t


def _smooth_vector(timestamps: np.ndarray, values: np.ndarray) -> np.ndarray:
    if len(timestamps) < 7:
        return values.copy()
    dt = float(np.median(np.diff(timestamps)))
    window = int(round(0.5 / max(dt, 1.0e-3)))
    window = max(5, min(window, len(timestamps) - 1))
    if window % 2 == 0:
        window -= 1
    if window < 5:
        return values.copy()
    samples = ImuSamples(timestamps, values, values)
    return savgol_filter_samples(samples, min(0.5, window * dt), 2).gyro
