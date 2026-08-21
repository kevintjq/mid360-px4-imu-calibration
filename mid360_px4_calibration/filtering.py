from __future__ import annotations

import numpy as np
from scipy.signal import savgol_filter

from .types import ImuSamples


def resample_uniform(samples: ImuSamples) -> ImuSamples:
    """Interpolate an IMU stream onto its median-period uniform time grid."""
    dt = samples.median_dt
    if not np.isfinite(dt) or dt <= 0.0:
        return samples
    count = int(np.floor((samples.end - samples.start) / dt)) + 1
    if count < 3:
        return samples
    timestamps = samples.start + np.arange(count, dtype=float) * dt
    if timestamps[-1] < samples.end - 0.25 * dt:
        timestamps = np.r_[timestamps, samples.end]
    gyro = np.column_stack([np.interp(timestamps, samples.timestamps, samples.gyro[:, i]) for i in range(3)])
    accel = np.column_stack([np.interp(timestamps, samples.timestamps, samples.accel[:, i]) for i in range(3)])
    return ImuSamples(timestamps, gyro, accel, frame_id=samples.frame_id)


def savgol_filter_samples(samples: ImuSamples, window_s: float, polyorder: int) -> ImuSamples:
    window = _window_length(samples.median_dt, window_s, len(samples.timestamps), polyorder)
    if window <= polyorder + 1:
        return samples
    gyro = savgol_filter(samples.gyro, window_length=window, polyorder=polyorder, axis=0, mode="interp")
    accel = savgol_filter(samples.accel, window_length=window, polyorder=polyorder, axis=0, mode="interp")
    return samples.with_values(gyro=gyro, accel=accel)


def angular_acceleration(samples: ImuSamples, window_s: float, polyorder: int) -> np.ndarray:
    window = _window_length(samples.median_dt, window_s, len(samples.timestamps), polyorder)
    if window > polyorder + 1:
        return savgol_filter(
            samples.gyro,
            window_length=window,
            polyorder=polyorder,
            deriv=1,
            delta=samples.median_dt,
            axis=0,
            mode="interp",
        )
    return np.gradient(samples.gyro, samples.timestamps, axis=0)


def trim_samples(samples: ImuSamples, trim_s: float) -> ImuSamples:
    if trim_s <= 0.0:
        return samples
    mask = (samples.timestamps >= samples.start + trim_s) & (samples.timestamps <= samples.end - trim_s)
    if int(np.sum(mask)) < 3:
        return samples
    return ImuSamples(samples.timestamps[mask], samples.gyro[mask], samples.accel[mask], frame_id=samples.frame_id)


def _window_length(dt: float, window_s: float, n: int, polyorder: int) -> int:
    if window_s <= 0.0 or dt <= 0.0:
        return 0
    window = int(round(window_s / dt))
    window = max(window, polyorder + 2)
    if window % 2 == 0:
        window += 1
    if window >= n:
        window = n - 1 if n % 2 == 0 else n
    if window % 2 == 0:
        window -= 1
    return max(window, 0)
