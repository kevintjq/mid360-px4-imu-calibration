from __future__ import annotations

from pathlib import Path
from typing import Any, Dict, Optional, Union

import yaml
import numpy as np

from .types import CalibrationConfig, ImuSamples


def sanitize_samples(samples: ImuSamples) -> ImuSamples:
    order = np.argsort(samples.timestamps)
    t = samples.timestamps[order]
    gyro = samples.gyro[order]
    accel = samples.accel[order]
    keep = np.ones(len(t), dtype=bool)
    keep[1:] = np.diff(t) > 0.0
    finite = np.isfinite(t) & np.all(np.isfinite(gyro), axis=1) & np.all(np.isfinite(accel), axis=1)
    keep &= finite
    t = t[keep]
    gyro = gyro[keep]
    accel = accel[keep]
    if len(t) < 3:
        raise ValueError("not enough valid IMU samples after sanitizing")
    return ImuSamples(t, gyro, accel, frame_id=samples.frame_id)


def load_config(path: Optional[Union[str, Path]]) -> CalibrationConfig:
    if path is None:
        return CalibrationConfig()
    with open(path, "r", encoding="utf-8") as f:
        raw = yaml.safe_load(f) or {}
    known = {field.name for field in CalibrationConfig.__dataclass_fields__.values()}
    kwargs: Dict[str, Any] = {k: v for k, v in raw.items() if k in known}
    return CalibrationConfig(**kwargs)


def write_result_yaml(path: Union[str, Path], result: Any, config: CalibrationConfig) -> None:
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    with open(path, "w", encoding="utf-8") as f:
        yaml.safe_dump(result.as_yaml_dict(config), f, sort_keys=False, allow_unicode=False)


def write_result_txt(path: Union[str, Path], result: Any, config: CalibrationConfig) -> None:
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    data = result.as_yaml_dict(config)
    extrinsic = data["extrinsic"]
    mid360_in_body = data["mid360_relative_to_body_enu"]
    quality = data["quality"]
    lines = [
        "Mid360 / PX4 IMU Extrinsic Calibration Result",
        "",
        "Convention:",
        "  T_M_B maps PX4 body B to Mid360 frame M",
        "  T_W_B = T_W_M * T_M_B",
        "  t_B = t_M + dt_B_wrt_M",
        "",
        "Frames:",
        "  mid_frame: {}".format(data["mid_frame"]),
        "  body_frame: {}".format(data["body_frame"]),
        "",
        "Time Offset:",
        "  dt_B_wrt_M_s: {:.9f}".format(data["time_offset"]["dt_B_wrt_M"]),
        "  correlation_peak: {:.6f}".format(data["time_offset"]["correlation_peak"]),
        "  correlation_second_peak: {:.6f}".format(data["time_offset"]["correlation_second_peak"]),
        "  correlation_peak_ratio: {:.6f}".format(data["time_offset"]["correlation_peak_ratio"]),
        "",
        "Extrinsic T_M_B:",
        "  quaternion_M_B_wxyz: {}".format(_fmt_list(extrinsic["quaternion_M_B_wxyz"], 9)),
        "  translation_M_B_m: {}".format(_fmt_list(extrinsic["translation_M_B_m"], 9)),
        "  rotation_matrix_M_B:",
    ]
    R = extrinsic["rotation_matrix_M_B"]
    for row in range(3):
        lines.append("    {}".format(_fmt_list(R[3 * row : 3 * row + 3], 9)))
    lines.extend(
        [
            "",
            "Requested Output: Mid360 Relative To PX4 Body (ENU-side ROS/MAVROS convention)",
            "  convention: T_B_M maps Mid360 frame M to PX4 body frame B; no NED/FRD conversion",
            "  translation_B_M_m: {}".format(_fmt_list(mid360_in_body["translation_B_M_m"], 9)),
            "  rpy_B_M_rad: {}".format(_fmt_list(mid360_in_body["rpy_B_M_rad"], 9)),
            "  rpy_B_M_deg: {}".format(_fmt_list(mid360_in_body["rpy_B_M_deg"], 9)),
            "  quaternion_B_M_wxyz: {}".format(_fmt_list(mid360_in_body["quaternion_B_M_wxyz"], 9)),
        ]
    )
    lines.extend(
        [
            "",
            "Combined Bias:",
            "  gyro_B_minus_rotated_M_rad_s: {}".format(
                _fmt_list(data["combined_bias"]["gyro_B_minus_rotated_M_rad_s"], 9)
            ),
            "  accel_rotated_B_minus_M_m_s2: {}".format(
                _fmt_list(data["combined_bias"]["accel_rotated_B_minus_M_m_s2"], 9)
            ),
            "",
            "Quality:",
            "  gyro_rmse_rad_s: {:.9f}".format(quality["gyro_rmse_rad_s"]),
            "  accel_rmse_m_s2: {:.9f}".format(quality["accel_rmse_m_s2"]),
            "  rotation_information_eigenvalues: {}".format(
                _fmt_list(quality["rotation_information_eigenvalues"], 9)
            ),
            "  translation_information_eigenvalues: {}".format(
                _fmt_list(quality["translation_information_eigenvalues"], 9)
            ),
            "  translation_condition_number: {:.9f}".format(quality["translation_condition_number"]),
            "  split_half_rotation_deg: {}".format(_fmt_optional(quality["split_half_rotation_deg"], 6)),
            "  split_half_translation_cm: {}".format(_fmt_optional(quality["split_half_translation_cm"], 6)),
            "  accepted: {}".format(quality["accepted"]),
        ]
    )
    if quality["notes"]:
        lines.append("  notes:")
        for note in quality["notes"]:
            lines.append("    - {}".format(note))
    else:
        lines.append("  notes: []")
    with open(path, "w", encoding="utf-8") as f:
        f.write("\n".join(lines) + "\n")


def _fmt_list(values: Any, precision: int) -> str:
    return "[" + ", ".join(("{:." + str(precision) + "f}").format(float(v)) for v in values) + "]"


def _fmt_optional(value: Any, precision: int) -> str:
    if value is None:
        return "null"
    return ("{: ." + str(precision) + "f}").format(float(value)).strip()
