#pragma once

#include <mid360_px4_calibration/calibration_math.h>
#include <deque>

namespace mid360_calibration {
struct StaticLimits {
  double duration = 8.0, max_gap = 0.1;
  double max_gyro = 0.08, max_gyro_std = 0.02;
  double max_accel_std = 0.15, max_accel_step = 0.3;
  double max_rotation_rad = 3.141592653589793 / 180.0;
  size_t min_samples = 100;
};
struct StaticEvaluation {
  StaticStats stats;
  double stable_s = 0, rotation_rad = 0;
  bool ready = false;
  std::string reason;
  double begin = 0, end = 0;
};

// A reset removes the candidate window, not just its displayed progress.
// A complete new window starts at the first clean sample after movement.
class StaticWindow {
 public:
  StaticWindow(const StaticLimits &limits, const Vec3 &bias)
      : limits_(limits), bias_(bias) {}

  void invalidate(const std::string &reason) {
    samples_.clear();
    have_anchor_ = false;
    reason_ = reason;
  }

  void push(const Imu &sample) {
    const bool gap = have_previous_ &&
        (sample.t <= previous_.t || sample.t - previous_.t > limits_.max_gap);
    const bool accel_step = have_previous_ &&
        (sample.a - previous_.a).norm() > limits_.max_accel_step;
    previous_ = sample;
    have_previous_ = true;
    if (gap) {
      invalidate("IMU 缺帧或时间戳异常，重新等待完整静止窗口"); return;
    }
    if ((sample.w-bias_).norm() >= limits_.max_gyro || accel_step ||
        std::abs(sample.a.norm()-9.80665) >= 1.5) {
      invalidate("检测到运动或加速度异常，重新等待完整静止窗口"); return;
    }
    if (have_anchor_ && logSO3(anchor_.transpose()*sample.R_EB).norm() > limits_.max_rotation_rad) {
      invalidate("IMU 姿态持续变化，重新等待完整静止窗口"); return;
    }
    if (!have_anchor_) {
      anchor_ = sample.R_EB;
      stable_begin_ = sample.t;
      have_anchor_ = true;
    }
    samples_.push_back(sample);
    // Retain the boundary sample just before end-duration, so readiness never
    // relies on a shortened window or a floating-point duration tolerance.
    while (samples_.size()>1 && samples_[1].t <= sample.t-limits_.duration)
      samples_.pop_front();
  }

  StaticEvaluation evaluate() const {
    StaticEvaluation r;
    r.reason = reason_;
    if (samples_.empty()) return r;
    const std::vector<Imu> window(samples_.begin(),samples_.end());
    r.begin = window.front().t;
    r.end = window.back().t;
    r.stable_s = r.end-stable_begin_;
    r.stats = staticStats(window,r.begin,r.end,bias_,limits_.max_gap);
    for (const auto &sample:window)
      r.rotation_rad = std::max(r.rotation_rad,logSO3(anchor_.transpose()*sample.R_EB).norm());
    const bool quiet = r.stats.continuous && r.stats.gyro_rms < limits_.max_gyro &&
        r.stats.gyro_std < limits_.max_gyro_std && r.stats.accel_std < limits_.max_accel_std &&
        std::abs(r.stats.accel_mean.norm()-9.80665) < 1.5;
    r.ready = quiet && r.stable_s >= limits_.duration && r.stats.span >= limits_.duration &&
              r.stats.samples >= limits_.min_samples;
    r.reason = quiet ? "状态：稳定，正在采集连续窗口" : "状态：窗口波动过大，继续保持静止";
    return r;
  }

 private:
  StaticLimits limits_;
  Vec3 bias_;
  std::deque<Imu> samples_;
  Imu previous_;
  bool have_previous_ = false, have_anchor_ = false;
  Mat3 anchor_ = Mat3::Identity();
  double stable_begin_ = 0;
  std::string reason_ = "状态：等待新的静止样本";
};
}  // namespace mid360_calibration
