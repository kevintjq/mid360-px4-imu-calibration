from __future__ import annotations

import numpy as np

from .types import ImuSamples


class ImuInterpolator:
    def __init__(self, samples: ImuSamples) -> None:
        self.samples = samples

    def in_range(self, t: np.ndarray) -> np.ndarray:
        t = np.asarray(t, dtype=float)
        return (t >= self.samples.start) & (t <= self.samples.end)

    def gyro(self, t: np.ndarray) -> np.ndarray:
        return self._interp(t, self.samples.gyro)

    def accel(self, t: np.ndarray) -> np.ndarray:
        return self._interp(t, self.samples.accel)

    def _interp(self, t: np.ndarray, values: np.ndarray) -> np.ndarray:
        t = np.asarray(t, dtype=float)
        if np.any(~self.in_range(t)):
            raise ValueError("interpolation requested outside sample time range")
        out = np.empty((len(t), 3), dtype=float)
        for axis in range(3):
            out[:, axis] = np.interp(t, self.samples.timestamps, values[:, axis])
        return out


def paired_times(mid: ImuSamples, body: ImuSamples, dt_B_wrt_M: float, max_samples: int) -> np.ndarray:
    start = max(mid.start, body.start - dt_B_wrt_M)
    end = min(mid.end, body.end - dt_B_wrt_M)
    mask = (mid.timestamps >= start) & (mid.timestamps <= end)
    times = mid.timestamps[mask]
    if len(times) < 10:
        raise ValueError("Mid360 and PX4 IMU time ranges do not overlap enough for this time offset")
    if len(times) > max_samples:
        idx = np.unique(np.linspace(0, len(times) - 1, max_samples).astype(int))
        times = times[idx]
    return times
