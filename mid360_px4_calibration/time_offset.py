from __future__ import annotations

import numpy as np
from typing import Optional

from .interpolation import ImuInterpolator
from .types import ImuSamples, TimeOffsetResult


def estimate_time_offset(
    mid: ImuSamples,
    body: ImuSamples,
    search_min_s: float,
    search_max_s: float,
    step_s: Optional[float],
) -> TimeOffsetResult:
    if step_s is None:
        step_s = max(mid.median_dt, body.median_dt)
    if step_s <= 0.0:
        raise ValueError("time search step must be positive")

    mid_interp = ImuInterpolator(mid)
    body_interp = ImuInterpolator(body)
    lags = np.arange(search_min_s, search_max_s + 0.5 * step_s, step_s)
    corrs = np.array([_correlation_at_lag(mid, body, mid_interp, body_interp, float(dt), step_s) for dt in lags])
    finite = np.isfinite(corrs)
    if not np.any(finite):
        raise ValueError("no valid overlap while estimating time offset")

    valid_indices = np.flatnonzero(finite)
    best_i = valid_indices[int(np.argmax(corrs[finite]))]
    best_dt = float(lags[best_i])
    if 0 < best_i < len(lags) - 1 and np.all(np.isfinite(corrs[best_i - 1 : best_i + 2])):
        best_dt = _parabolic_peak(lags[best_i - 1 : best_i + 2], corrs[best_i - 1 : best_i + 2])
    sorted_corrs = np.sort(corrs[finite])
    peak = float(sorted_corrs[-1])
    second = float(sorted_corrs[-2]) if len(sorted_corrs) > 1 else 0.0
    ratio = float(peak / max(abs(second), 1.0e-12))
    return TimeOffsetResult(
        dt_B_wrt_M=best_dt,
        peak=peak,
        second_peak=second,
        peak_ratio=ratio,
        lag_values=[float(v) for v in lags],
        correlation_values=[float(v) if np.isfinite(v) else float("nan") for v in corrs],
    )


def _correlation_at_lag(
    mid: ImuSamples,
    body: ImuSamples,
    mid_interp: ImuInterpolator,
    body_interp: ImuInterpolator,
    dt: float,
    step_s: float,
) -> float:
    start = max(mid.start, body.start - dt)
    end = min(mid.end, body.end - dt)
    if end <= start + 10.0 * step_s:
        return float("nan")
    t_mid = np.arange(start, end, step_s)
    if len(t_mid) < 10:
        return float("nan")
    s_mid = np.linalg.norm(mid_interp.gyro(t_mid), axis=1)
    s_body = np.linalg.norm(body_interp.gyro(t_mid + dt), axis=1)
    s_mid = s_mid - np.mean(s_mid)
    s_body = s_body - np.mean(s_body)
    denom = np.linalg.norm(s_mid) * np.linalg.norm(s_body)
    if denom < 1.0e-12:
        return float("nan")
    return float(np.dot(s_mid, s_body) / denom)


def _parabolic_peak(x: np.ndarray, y: np.ndarray) -> float:
    step = float(x[1] - x[0])
    denom = float(y[0] - 2.0 * y[1] + y[2])
    if abs(denom) < 1.0e-12:
        return float(x[1])
    offset_steps = 0.5 * float(y[0] - y[2]) / denom
    offset_steps = float(np.clip(offset_steps, -1.0, 1.0))
    return float(x[1] + offset_steps * step)
