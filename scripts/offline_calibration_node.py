#!/usr/bin/env python3
from __future__ import annotations

import sys

import rospy
import rospkg

from mid360_px4_calibration.io import write_result_txt, write_result_yaml
from mid360_px4_calibration.rosbag_io import load_imu_topics_from_rosbag
from mid360_px4_calibration.solver import calibrate
from mid360_px4_calibration.types import CalibrationConfig


def param(name, default):
    return rospy.get_param("~" + name, default)


def resolve_package_path(path: str) -> str:
    if not path:
        return path
    if path.startswith("/"):
        return path
    package_path = rospkg.RosPack().get_path("mid360_px4_calibration")
    return package_path + "/" + path


def config_from_ros_params() -> CalibrationConfig:
    return CalibrationConfig(
        time_search_min_s=float(param("time_search_min_s", -0.5)),
        time_search_max_s=float(param("time_search_max_s", 0.5)),
        time_search_step_s=param("time_search_step_s", None),
        max_samples=int(param("max_samples", 6000)),
        filter_window_s=float(param("filter_window_s", 0.08)),
        filter_polyorder=int(param("filter_polyorder", 3)),
        trim_s=float(param("trim_s", 0.5)),
        gyro_sigma_rad_s=float(param("gyro_sigma_rad_s", 0.03)),
        accel_sigma_m_s2=float(param("accel_sigma_m_s2", 0.2)),
        robust_loss=param("robust_loss", "huber"),
        accepted_gyro_rmse_rad_s=float(param("accepted_gyro_rmse_rad_s", 0.12)),
        accepted_accel_rmse_m_s2=float(param("accepted_accel_rmse_m_s2", 0.8)),
        min_rotation_eigenvalue=float(param("min_rotation_eigenvalue", 1.0e-3)),
        min_translation_eigenvalue=float(param("min_translation_eigenvalue", 1.0e-3)),
        max_translation_condition_number=float(param("max_translation_condition_number", 1.0e6)),
    )


def main() -> int:
    rospy.init_node("mid360_px4_offline_calibration")
    bag_path = param("bag_path", "")
    mid_imu_topic = param("mid_imu_topic", "/livox/imu")
    body_imu_topic = param("body_imu_topic", "/mavros/imu/data")
    stamp_source = param("stamp_source", "header")
    output_yaml = param("output_yaml", "calibration_result.yaml")
    output_txt = param("output_txt", "")
    bag_path = resolve_package_path(bag_path)
    output_yaml = resolve_package_path(output_yaml)
    output_txt = resolve_package_path(output_txt)

    config = config_from_ros_params()
    if not bag_path:
        rospy.logerr("~bag_path must be set")
        return 2
    rospy.loginfo("loading IMU topics from bag: %s", bag_path)
    rospy.loginfo("Mid360 IMU topic: %s", mid_imu_topic)
    rospy.loginfo("PX4/MAVROS IMU topic: %s", body_imu_topic)
    samples = load_imu_topics_from_rosbag(bag_path, [mid_imu_topic, body_imu_topic], stamp_source=stamp_source)
    mid = samples[mid_imu_topic]
    body = samples[body_imu_topic]
    rospy.loginfo("Mid360 IMU: %.3f .. %.3f s, %.2f Hz", mid.start, mid.end, mid.frequency_hz)
    rospy.loginfo("PX4/MAVROS IMU: %.3f .. %.3f s, %.2f Hz", body.start, body.end, body.frequency_hz)
    if mid.frame_id:
        config.mid_frame = mid.frame_id
    if body.frame_id:
        config.body_frame = body.frame_id
    rospy.loginfo("result mid_frame: %s", config.mid_frame)
    rospy.loginfo("result body_frame: %s", config.body_frame)

    result = calibrate(mid, body, config)
    if output_yaml:
        write_result_yaml(output_yaml, result, config)
        rospy.loginfo("wrote calibration YAML: %s", output_yaml)
    if output_txt:
        write_result_txt(output_txt, result, config)
        rospy.loginfo("wrote calibration TXT: %s", output_txt)
    rospy.loginfo("dt_B_wrt_M: %.9f s", result.dt_B_wrt_M)
    rospy.loginfo("translation_M_B_m: [%.6f, %.6f, %.6f]", *result.translation_M_B_m)
    rospy.loginfo("gyro RMSE: %.6f rad/s", result.gyro_stats.rmse)
    rospy.loginfo("accel RMSE: %.6f m/s^2", result.accel_stats.rmse)
    rospy.loginfo("accepted: %s", result.accepted)
    for note in result.notes:
        rospy.logwarn("quality note: %s", note)
    return 0 if result.accepted else 3


if __name__ == "__main__":
    sys.exit(main())
