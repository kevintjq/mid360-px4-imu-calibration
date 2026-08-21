#!/usr/bin/env python3
"""Interactive online dual-IMU calibration from two ROS1 topics.

The node only subscribes to sensor_msgs/Imu.  It does not publish any topic.
The terminal prompts the operator through static, single-axis, and combined
rotations so that the translation lever arm is observable.
"""
from __future__ import annotations

import sys
import threading
import time
from typing import List, Tuple

import numpy as np
import rospy
import rospkg
from sensor_msgs.msg import Imu

from mid360_px4_calibration.io import sanitize_samples, write_result_txt, write_result_yaml
from mid360_px4_calibration.solver import calibrate
from mid360_px4_calibration.types import CalibrationConfig, ImuSamples


def param(name, default):
    return rospy.get_param("~" + name, default)


def resolve_package_path(path: str) -> str:
    if not path:
        return path
    if path.startswith("/"):
        return path
    return rospkg.RosPack().get_path("mid360_px4_calibration") + "/" + path


class ImuBuffer:
    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._mid: List[Tuple[float, np.ndarray, np.ndarray, str]] = []
        self._body: List[Tuple[float, np.ndarray, np.ndarray, str]] = []

    def add(self, which: str, msg: Imu) -> None:
        stamp = msg.header.stamp.to_sec()
        gyro = np.array([msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z], dtype=float)
        accel = np.array([msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z], dtype=float)
        if not np.isfinite(stamp) or not np.all(np.isfinite(gyro)) or not np.all(np.isfinite(accel)):
            return
        item = (stamp, gyro, accel, msg.header.frame_id or "")
        with self._lock:
            (self._mid if which == "mid" else self._body).append(item)

    def snapshot(self) -> Tuple[ImuSamples, ImuSamples]:
        with self._lock:
            mid = list(self._mid)
            body = list(self._body)
        return _to_samples(mid), _to_samples(body)

    def counts(self) -> Tuple[int, int]:
        with self._lock:
            return len(self._mid), len(self._body)


def _to_samples(items: List[Tuple[float, np.ndarray, np.ndarray, str]]) -> ImuSamples:
    if len(items) < 3:
        raise ValueError("not enough IMU samples")
    stamps = np.array([v[0] for v in items], dtype=float)
    gyro = np.array([v[1] for v in items], dtype=float)
    accel = np.array([v[2] for v in items], dtype=float)
    frame_id = next((v[3] for v in items if v[3]), "")
    return sanitize_samples(ImuSamples(stamps, gyro, accel, frame_id=frame_id))


def config_from_ros_params() -> CalibrationConfig:
    return CalibrationConfig(
        time_search_min_s=float(param("time_search_min_s", -0.5)),
        time_search_max_s=float(param("time_search_max_s", 0.5)),
        time_search_step_s=param("time_search_step_s", None),
        max_samples=int(param("max_samples", 12000)),
        filter_window_s=float(param("filter_window_s", 0.08)),
        filter_polyorder=int(param("filter_polyorder", 3)),
        trim_s=float(param("trim_s", 0.5)),
        gyro_sigma_rad_s=float(param("gyro_sigma_rad_s", 0.03)),
        accel_sigma_m_s2=float(param("accel_sigma_m_s2", 0.2)),
        robust_loss=param("robust_loss", "soft_l1"),
        accepted_gyro_rmse_rad_s=float(param("accepted_gyro_rmse_rad_s", 0.12)),
        accepted_accel_rmse_m_s2=float(param("accepted_accel_rmse_m_s2", 0.8)),
        min_rotation_eigenvalue=float(param("min_rotation_eigenvalue", 1.0e-3)),
        min_translation_eigenvalue=float(param("min_translation_eigenvalue", 1.0e-3)),
        max_translation_condition_number=float(param("max_translation_condition_number", 1.0e6)),
    )


def _wait_for_enter(prompt: str, interactive: bool) -> bool:
    if rospy.is_shutdown():
        return False
    if not interactive:
        rospy.loginfo(prompt)
        return True
    try:
        input(prompt + " 按回车开始 / press Enter to start... ")
        return not rospy.is_shutdown()
    except (EOFError, KeyboardInterrupt):
        return False


def _collect_phase(buffer: ImuBuffer, duration: float, label: str) -> None:
    rospy.loginfo("开始阶段: %s (%.1f s)", label, duration)
    # Use wall time for the operator countdown.  This keeps the workflow
    # usable even if /use_sim_time is enabled but no /clock is available.
    end = time.monotonic() + duration
    last_report = time.monotonic()
    while not rospy.is_shutdown() and time.monotonic() < end:
        time.sleep(0.1)
        now = time.monotonic()
        if now - last_report >= 5.0:
            n_mid, n_body = buffer.counts()
            remaining = max(0.0, end - now)
            rospy.loginfo("  剩余 %.1f s, samples: Mid360=%d PX4=%d", remaining, n_mid, n_body)
            last_report = now


def _motion_report(mid: ImuSamples, body: ImuSamples) -> None:
    gyro_mid = np.linalg.norm(mid.gyro - np.mean(mid.gyro, axis=0), axis=1)
    gyro_body = np.linalg.norm(body.gyro - np.mean(body.gyro, axis=0), axis=1)
    rospy.loginfo(
        "运动激励: Mid360 gyro std=%.3f rad/s, PX4 gyro std=%.3f rad/s; "
        "请确保三轴都有明显转动。",
        float(np.std(gyro_mid)),
        float(np.std(gyro_body)),
    )


def _split_consistency(mid: ImuSamples, body: ImuSamples, config: CalibrationConfig):
    """Return rotation/translation disagreement between two time halves."""
    start = max(mid.start, body.start)
    end = min(mid.end, body.end)
    cut = start + 0.5 * (end - start)
    mid_a = ImuSamples(mid.timestamps[mid.timestamps <= cut], mid.gyro[mid.timestamps <= cut], mid.accel[mid.timestamps <= cut], mid.frame_id)
    mid_b = ImuSamples(mid.timestamps[mid.timestamps >= cut], mid.gyro[mid.timestamps >= cut], mid.accel[mid.timestamps >= cut], mid.frame_id)
    body_a = ImuSamples(body.timestamps[body.timestamps <= cut], body.gyro[body.timestamps <= cut], body.accel[body.timestamps <= cut], body.frame_id)
    body_b = ImuSamples(body.timestamps[body.timestamps >= cut], body.gyro[body.timestamps >= cut], body.accel[body.timestamps >= cut], body.frame_id)
    ra = calibrate(mid_a, body_a, config)
    rb = calibrate(mid_b, body_b, config)
    from scipy.spatial.transform import Rotation
    rot_deg = float(np.degrees(Rotation.from_matrix(rb.R_M_B.T @ ra.R_M_B).magnitude()))
    trans_cm = float(np.linalg.norm(rb.translation_M_B_m - ra.translation_M_B_m) * 100.0)
    return rot_deg, trans_cm


def main() -> int:
    rospy.init_node("mid360_px4_online_calibration")
    mid_topic = param("mid_imu_topic", "/livox/imu")
    body_topic = param("body_imu_topic", "/mavros/imu/data")
    interactive = bool(param("interactive_prompts", True))
    phase_duration = float(param("phase_duration_s", 15.0))
    static_duration = float(param("static_duration_s", 5.0))
    min_samples = int(param("min_samples", 500))
    output_yaml = resolve_package_path(param("output_yaml", "config/online_calibration_result.yaml"))
    output_txt = resolve_package_path(param("output_txt", "config/online_calibration_result.txt"))
    config = config_from_ros_params()
    buffer = ImuBuffer()
    rospy.Subscriber(mid_topic, Imu, lambda msg: buffer.add("mid", msg), queue_size=10000)
    rospy.Subscriber(body_topic, Imu, lambda msg: buffer.add("body", msg), queue_size=10000)
    rospy.loginfo("在线标定已启动，只读取两个 IMU 话题，不发布任何话题。")
    rospy.loginfo("Mid360: %s | PX4/MAVROS: %s", mid_topic, body_topic)

    phases = [
        ("静止，保持设备不动", static_duration),
        ("只绕 X 轴缓慢正反转，幅度约 ±30 度", phase_duration),
        ("只绕 Y 轴缓慢正反转，幅度约 ±30 度", phase_duration),
        ("只绕 Z 轴缓慢正反转，幅度约 ±30 度", phase_duration),
        ("组合绕 X/Y/Z 三轴转动，速度要有变化，不要只做单轴", phase_duration * 2.0),
        ("停止转动，保持静止用于检查", static_duration),
    ]
    for label, duration in phases:
        if not _wait_for_enter(label, interactive):
            rospy.logwarn("在线标定被中止")
            return 2
        _collect_phase(buffer, duration, label)

    try:
        mid, body = buffer.snapshot()
    except ValueError as exc:
        rospy.logerr("读取 IMU 数据失败: %s", exc)
        return 2
    n_mid, n_body = len(mid.timestamps), len(body.timestamps)
    rospy.loginfo("采集完成: Mid360=%d, PX4=%d", n_mid, n_body)
    if n_mid < min_samples or n_body < min_samples:
        rospy.logerr("样本太少（需要至少 %d 个/话题）", min_samples)
        return 2
    if mid.frame_id:
        config.mid_frame = mid.frame_id
    if body.frame_id:
        config.body_frame = body.frame_id
    _motion_report(mid, body)

    rospy.loginfo("开始计算，数据越长、三轴运动越充分，杆臂 XYZ 越稳定。")
    try:
        result = calibrate(mid, body, config)
    except Exception as exc:
        rospy.logerr("标定失败: %s", exc)
        return 3
    if bool(param("validate_split", True)):
        try:
            rot_deg, trans_cm = _split_consistency(mid, body, config)
            result.split_half_rotation_deg = rot_deg
            result.split_half_translation_cm = trans_cm
            rospy.loginfo("前后半段一致性: rotation=%.3f deg, translation=%.2f cm", rot_deg, trans_cm)
            if rot_deg > 2.0:
                result.notes.append("split-half rotation disagreement is above 2 deg")
            if trans_cm > 5.0:
                result.notes.append("split-half translation disagreement is above 5 cm")
            if rot_deg > 2.0 or trans_cm > 5.0:
                result.accepted = False
        except Exception as exc:
            rospy.logwarn("分段一致性检查跳过: %s", exc)
    write_result_yaml(output_yaml, result, config)
    write_result_txt(output_txt, result, config)
    rospy.loginfo("YAML: %s", output_yaml)
    rospy.loginfo("TXT: %s", output_txt)
    rospy.loginfo("Mid360 相对 PX4 body (ENU): xyz=%s m, rpy=%s deg", result.as_yaml_dict(config)["mid360_relative_to_body_enu"]["translation_B_M_m"], result.as_yaml_dict(config)["mid360_relative_to_body_enu"]["rpy_B_M_deg"])
    rospy.loginfo("gyro RMSE=%.4f, accel RMSE=%.4f, accepted=%s", result.gyro_stats.rmse, result.accel_stats.rmse, result.accepted)
    for note in result.notes:
        rospy.logwarn("quality note: %s", note)
    return 0 if result.accepted else 3


if __name__ == "__main__":
    sys.exit(main())
