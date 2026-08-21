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
from dataclasses import replace
from typing import List, Tuple

import numpy as np
import rospy
import roslib.message
import rospkg
from sensor_msgs import point_cloud2
from sensor_msgs.msg import Imu

from mid360_px4_calibration.io import sanitize_samples, write_result_txt, write_result_yaml
from mid360_px4_calibration.lidar_motion import PointCloudScan, estimate_lidar_motion
from mid360_px4_calibration.lidar_solver import calibrate_lidar_body, combine_lidar_sensor_results
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


class PointCloudBuffer:
    def __init__(self, max_scans: int = 600) -> None:
        self._lock = threading.Lock()
        self._scans: List[PointCloudScan] = []
        self._max_scans = max_scans

    def add_anymsg(self, msg) -> None:
        try:
            scan = _decode_point_cloud(msg)
        except Exception as exc:
            rospy.logwarn_throttle(10.0, "point-cloud decode failed: %s", exc)
            return
        if scan is None or len(scan.points) < 80:
            return
        with self._lock:
            self._scans.append(scan)
            if len(self._scans) > self._max_scans:
                self._scans = self._scans[-self._max_scans :]

    def snapshot(self) -> List[PointCloudScan]:
        with self._lock:
            return list(self._scans)


def _decode_point_cloud(msg) -> PointCloudScan:
    connection_header = getattr(msg, "_connection_header", {}) or {}
    type_name = connection_header.get("type", "")
    msg_class = roslib.message.get_message_class(type_name)
    if msg_class is None:
        raise ValueError("unsupported point-cloud message type: {}".format(type_name))
    decoded = msg_class()
    decoded.deserialize(msg._buff)
    header = getattr(decoded, "header", None)
    timestamp = None
    frame_id = ""
    if header is not None:
        timestamp = header.stamp.to_sec()
        frame_id = header.frame_id or ""
    if timestamp is None or timestamp <= 0.0:
        timebase = getattr(decoded, "timebase", None)
        if timebase is not None:
            timestamp = float(timebase) * 1.0e-9
    if timestamp is None or not np.isfinite(timestamp):
        raise ValueError("point cloud has no valid timestamp")
    if type_name == "sensor_msgs/PointCloud2":
        points = np.asarray(
            list(point_cloud2.read_points(decoded, field_names=("x", "y", "z"), skip_nans=True)),
            dtype=float,
        )
    elif hasattr(decoded, "points"):
        points = np.asarray(
            [[float(point.x), float(point.y), float(point.z)] for point in decoded.points],
            dtype=float,
        )
    else:
        raise ValueError("point cloud message has no x/y/z fields")
    points = points[np.all(np.isfinite(points), axis=1)]
    if len(points) > 4000:
        points = points[:: max(1, len(points) // 4000)]
    return PointCloudScan(float(timestamp), points, frame_id)


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
        mid_accel_scale=float(param("mid_accel_scale", 1.0)),
        body_accel_scale=float(param("body_accel_scale", 1.0)),
        mid_gyro_scale=float(param("mid_gyro_scale", 1.0)),
        body_gyro_scale=float(param("body_gyro_scale", 1.0)),
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
        input(prompt + " | press Enter to start... ")
        return not rospy.is_shutdown()
    except (EOFError, KeyboardInterrupt):
        return False


def _collect_phase(buffer: ImuBuffer, duration: float, label: str) -> None:
    rospy.loginfo("Starting phase: %s (%.1f s)", label, duration)
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
            rospy.loginfo("  remaining %.1f s, samples: Mid360=%d PX4=%d", remaining, n_mid, n_body)
            last_report = now


def _motion_report(mid: ImuSamples, body: ImuSamples) -> None:
    gyro_mid = np.std(mid.gyro, axis=0)
    gyro_body = np.std(body.gyro, axis=0)
    rospy.loginfo(
        "Motion excitation, gyro std [x y z]: Mid360=[%.3f %.3f %.3f], PX4=[%.3f %.3f %.3f] rad/s",
        *[float(v) for v in gyro_mid],
        *[float(v) for v in gyro_body],
    )


def _accel_norm_report(mid: ImuSamples, body: ImuSamples, config: CalibrationConfig) -> None:
    """Warn about the common Livox-g versus Pixhawk-m/s^2 unit mismatch."""
    def static_norm(samples: ImuSamples) -> float:
        count = max(10, int(round(5.0 * samples.frequency_hz)))
        count = min(count, len(samples.timestamps) // 2)
        values = np.r_[samples.accel[:count], samples.accel[-count:]]
        return float(np.median(np.linalg.norm(values, axis=1)))

    mid_norm = static_norm(mid)
    body_norm = static_norm(body)
    rospy.loginfo(
        "Raw static acceleration norms: Mid360=%.4f, PX4=%.4f (scales %.6g, %.6g)",
        mid_norm,
        body_norm,
        config.mid_accel_scale,
        config.body_accel_scale,
    )
    if abs(mid_norm - 1.0) < 0.25 and mid_norm < 2.0 and config.mid_accel_scale == 1.0:
        rospy.logwarn("Mid360 acceleration looks like g, not m/s^2. Set mid_accel_scale: 9.80665.")
    if abs(body_norm - 1.0) < 0.25 and body_norm < 2.0 and config.body_accel_scale == 1.0:
        rospy.logwarn("PX4 acceleration looks like g, not m/s^2. Set body_accel_scale: 9.80665.")


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
    body_topic = param("body_imu_topic", "/mavros/imu/data_raw")
    lidar_topic = param("lidar_topic", "/livox/lidar")
    use_pointcloud = bool(param("use_pointcloud", True))
    lidar_required = bool(param("lidar_required", True))
    interactive = bool(param("interactive_prompts", True))
    phase_duration = float(param("phase_duration_s", 4.0))
    static_duration = float(param("static_duration_s", 2.5))
    min_samples = int(param("min_samples", 500))
    output_yaml = resolve_package_path(param("output_yaml", "config/online_calibration_result.yaml"))
    output_txt = resolve_package_path(param("output_txt", "config/online_calibration_result.txt"))
    config = config_from_ros_params()
    buffer = ImuBuffer()
    cloud_buffer = PointCloudBuffer(max_scans=int(param("max_lidar_scans", 300)))
    rospy.Subscriber(mid_topic, Imu, lambda msg: buffer.add("mid", msg), queue_size=10000)
    rospy.Subscriber(body_topic, Imu, lambda msg: buffer.add("body", msg), queue_size=10000)
    if use_pointcloud:
        rospy.Subscriber(lidar_topic, rospy.AnyMsg, cloud_buffer.add_anymsg, queue_size=20)
    rospy.loginfo("Online calibration started. Subscribes only; publishes no topic.")
    rospy.loginfo("Mid360 IMU: %s | PX4/MAVROS IMU: %s", mid_topic, body_topic)
    if use_pointcloud:
        rospy.loginfo("Point cloud: %s", lidar_topic)

    phases = [
        ("static: keep the rigid assembly still", static_duration),
        ("roll: rotate slowly forward/back around X, about +/-30 deg", phase_duration),
        ("pitch: rotate slowly forward/back around Y, about +/-30 deg", phase_duration),
        ("yaw: rotate slowly forward/back around Z, about +/-30 deg", phase_duration),
        ("combined roll/pitch/yaw: change angular speed and excite all axes", phase_duration * 2.0),
        ("static: stop and keep still for validation", static_duration),
    ]
    for label, duration in phases:
        if not _wait_for_enter(label, interactive):
            rospy.logwarn("Online calibration aborted")
            return 2
        _collect_phase(buffer, duration, label)

    try:
        mid, body = buffer.snapshot()
    except ValueError as exc:
        rospy.logerr("Failed to read IMU data: %s", exc)
        return 2
    n_mid, n_body = len(mid.timestamps), len(body.timestamps)
    rospy.loginfo("Collection complete: Mid360=%d, PX4=%d", n_mid, n_body)
    if n_mid < min_samples or n_body < min_samples:
        rospy.logerr("Too few samples (need at least %d per topic)", min_samples)
        return 2
    if mid.frame_id:
        config.mid_frame = mid.frame_id
    if body.frame_id:
        config.body_frame = body.frame_id
    _motion_report(mid, body)
    _accel_norm_report(mid, body, config)

    rospy.loginfo("Starting calibration. Longer data and full 3-axis excitation improve XYZ stability.")
    try:
        if use_pointcloud:
            scans = cloud_buffer.snapshot()
            rospy.loginfo("Point-cloud scans received: %d", len(scans))
            if len(scans) < int(param("min_lidar_scans", 20)):
                raise RuntimeError("too few point-cloud scans for LiDAR motion estimation")
            lidar_motion = estimate_lidar_motion(
                scans,
                voxel_size=float(param("lidar_voxel_size_m", 0.15)),
                max_correspondence_m=float(param("lidar_max_correspondence_m", 2.0)),
                max_iterations=int(param("lidar_icp_iterations", 20)),
                min_correspondences=int(param("lidar_min_correspondences", 80)),
                max_points=int(param("lidar_max_points_per_scan", 2500)),
            )
            rospy.loginfo(
                "LiDAR motion samples: %d, rate %.2f Hz, mean ICP error %.4f m",
                len(lidar_motion.timestamps),
                lidar_motion.frequency_hz,
                float(np.mean(lidar_motion.fitness)),
            )
            mean_icp_error = float(np.mean(lidar_motion.fitness))
            max_icp_error = float(param("max_lidar_icp_error_m", 0.25))
            if not np.isfinite(mean_icp_error) or mean_icp_error > max_icp_error:
                raise RuntimeError(
                    "point-cloud ICP error {:.3f} m exceeds {:.3f} m".format(mean_icp_error, max_icp_error)
                )
            mid_config = replace(
                config,
                body_gyro_scale=config.mid_gyro_scale,
                body_accel_scale=config.mid_accel_scale,
            )
            mid_result = calibrate_lidar_body(lidar_motion, mid, mid_config)
            body_result = calibrate_lidar_body(lidar_motion, body, config)
            result = combine_lidar_sensor_results(mid_result, body_result, config)
            result.pointcloud_motion_samples = len(lidar_motion.timestamps)
            result.pointcloud_mean_icp_error_m = mean_icp_error
        else:
            if lidar_required:
                raise RuntimeError("point-cloud calibration is required but use_pointcloud is false")
            result = calibrate(mid, body, config)
    except Exception as exc:
        rospy.logerr("Calibration failed: %s", exc)
        return 3
    if bool(param("validate_split", True)) and not use_pointcloud:
        try:
            rot_deg, trans_cm = _split_consistency(mid, body, config)
            result.split_half_rotation_deg = rot_deg
            result.split_half_translation_cm = trans_cm
            rospy.loginfo("Split-half consistency: rotation=%.3f deg, translation=%.2f cm", rot_deg, trans_cm)
            if rot_deg > 2.0:
                result.notes.append("split-half rotation disagreement is above 2 deg")
            if trans_cm > 5.0:
                result.notes.append("split-half translation disagreement is above 5 cm")
            if rot_deg > 2.0 or trans_cm > 5.0:
                result.accepted = False
        except Exception as exc:
            rospy.logwarn("Split-half validation skipped: %s", exc)
    elif use_pointcloud:
        result.notes.append("split-half IMU-only validation skipped in point-cloud mode to keep startup fast")
    write_result_yaml(output_yaml, result, config)
    write_result_txt(output_txt, result, config)
    rospy.loginfo("YAML: %s", output_yaml)
    rospy.loginfo("TXT: %s", output_txt)
    rospy.loginfo("Mid360 relative to PX4 body (ENU): xyz=%s m, rpy=%s deg", result.as_yaml_dict(config)["mid360_relative_to_body_enu"]["translation_B_M_m"], result.as_yaml_dict(config)["mid360_relative_to_body_enu"]["rpy_B_M_deg"])
    rospy.loginfo("gyro RMSE=%.4f, accel RMSE=%.4f, accepted=%s", result.gyro_stats.rmse, result.accel_stats.rmse, result.accepted)
    for note in result.notes:
        rospy.logwarn("quality note: %s", note)
    return 0 if result.accepted else 3


if __name__ == "__main__":
    sys.exit(main())
