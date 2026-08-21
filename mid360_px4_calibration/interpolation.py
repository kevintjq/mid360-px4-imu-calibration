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
        if np.any(t < self.samples.start - 1.0e-8) or np.any(t > self.samples.end + 1.0e-8):
            raise ValueError("interpolation requested outside sample time range")
        t = np.clip(t, self.samples.start, self.samples.end)
        out = np.empty((len(t), 3), dtype=float)
        for axis in range(3):
            out[:, axis] = np.interp(t, self.samples.timestamps, values[:, axis])
        return out


def paired_times(mid: ImuSamples, body: ImuSamples, dt_B_wrt_M: float, max_samples: int) -> np.ndarray:
    start = max(mid.start, body.start - dt_B_wrt_M)
    end = min(mid.end, body.end - dt_B_wrt_M)
    if end <= start:
        raise ValueError("Mid360 and PX4 IMU time ranges do not overlap")
    # Keep the residual vector length independent of dt. Using a timestamp
    # mask here makes least_squares fail whenever a trial dt crosses a sample
    # boundary and silently changes the number of residuals.
    count = min(int(max_samples), len(mid.timestamps))
    if count < 10:
        raise ValueError("Mid360 and PX4 IMU time ranges do not overlap enough for this time offset")
    times = np.linspace(start, end, count, dtype=float)
    # Keep interpolation queries strictly inside both source ranges despite
    # floating-point addition of dt at the final sample.
    lower = max(mid.start, body.start - dt_B_wrt_M) + 1.0e-9
    upper = min(mid.end, body.end - dt_B_wrt_M) - 1.0e-9
    if upper <= lower:
        raise ValueError("overlap is too short after numerical guard")
    times = np.clip(times, lower, upper)
    return times
