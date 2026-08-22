#include <ros/ros.h>
#include <ros/package.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <algorithm>
#include <array>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <mutex>
#include <sstream>
#include <string>
#include <vector>

namespace {
using Vec3 = Eigen::Vector3d;
using Mat3 = Eigen::Matrix3d;

struct ImuSample { double t; Vec3 w; Vec3 a; };
struct PoseSample { double t; Vec3 p; Mat3 R_W_S; };
struct Result {
  Mat3 R_B_M = Mat3::Identity();
  Vec3 t_B_M = Vec3::Zero();
  Vec3 t_M_B = Vec3::Zero();
  Vec3 t_M_B_enu = Vec3::Zero();
  double reference_stamp = 0.0;
  double gyro_rmse = std::numeric_limits<double>::infinity();
  double accel_rmse = std::numeric_limits<double>::infinity();
  double translation_condition = std::numeric_limits<double>::infinity();
  bool accepted = false;
  std::string note;
};

static constexpr int PB = 32;
static const char *CYAN = "\033[96m";
static const char *YELLOW = "\033[93m";
static const char *GREEN = "\033[92m";
static const char *MAGENTA = "\033[95m";
static const char *RESET = "\033[0m";

double clamp01(double x) { return std::max(0.0, std::min(1.0, x)); }
Mat3 skew(const Vec3 &v) {
  Mat3 s;
  s << 0.0, -v.z(), v.y(), v.z(), 0.0, -v.x(), -v.y(), v.x(), 0.0;
  return s;
}
Vec3 rpyDeg(const Mat3 &R) {
  const double pitch = std::asin(std::max(-1.0, std::min(1.0, -R(2, 0))));
  const double cp = std::cos(pitch);
  const double roll = std::abs(cp) > 1e-8 ? std::atan2(R(2, 1), R(2, 2)) : 0.0;
  const double yaw = std::abs(cp) > 1e-8 ? std::atan2(R(1, 0), R(0, 0)) : std::atan2(-R(1, 2), R(1, 1));
  return Vec3(roll, pitch, yaw) * 180.0 / M_PI;
}
std::string list3(const Vec3 &v) {
  std::ostringstream s;
  s << std::fixed << std::setprecision(9) << "[" << v.x() << ", " << v.y() << ", " << v.z() << "]";
  return s.str();
}
void bar(const std::string &name, double value, const char *color) {
  const int n = static_cast<int>(std::round(clamp01(value) * PB));
  std::cout << color << name << " " << std::setw(3) << static_cast<int>(std::round(clamp01(value) * 100.0))
            << "% [" << std::string(n, '=') << std::string(PB - n, ' ') << "]" << RESET << "\n";
  std::cout.flush();
}

class Calibrator {
 public:
  Calibrator(ros::NodeHandle &nh, ros::NodeHandle &pnh) : nh_(nh), pnh_(pnh) {
    pnh_.param("slam_odom_topic", odom_topic_, std::string("/Odometry"));
    pnh_.param("body_imu_topic", body_imu_topic_, std::string("/mavros/imu/data_raw"));
    pnh_.param("body_pose_topic", body_pose_topic_, std::string("/mavros/imu/data"));
    pnh_.param("output_yaml", output_yaml_, std::string("config/online_calibration_result.yaml"));
    pnh_.param("output_txt", output_txt_, std::string("config/online_calibration_result.txt"));
    pnh_.param("body_accel_scale", body_accel_scale_, 1.0);
    pnh_.param("body_gyro_scale", body_gyro_scale_, 1.0);
    pnh_.param("static_duration_s", static_s_, 8.0);
    pnh_.param("static_validation_duration_s", static_validation_s_, 3.0);
    pnh_.param("motion_timeout_s", motion_timeout_s_, 0.0);
    pnh_.param("motion_update_period_s", update_s_, 0.25);
    pnh_.param("axis_excitation_target_rad", target_rad_, 2.5);
    pnh_.param("min_samples", min_samples_, 1500);
    pnh_.param("min_odom_samples", min_odom_samples_, 30);
    sub_imu_ = nh_.subscribe(body_imu_topic_, 10000, &Calibrator::imuCallback, this);
    sub_pose_ = nh_.subscribe(body_pose_topic_, 1000, &Calibrator::poseCallback, this);
    sub_odom_ = nh_.subscribe(odom_topic_, 2000, &Calibrator::odomCallback, this);
  }

  int run() {
    ROS_INFO("SLAM odom to PX4 IMU extrinsic calibration started.");
    ROS_INFO("SLAM odom: %s | PX4 raw IMU: %s | PX4 ENU attitude: %s",
             odom_topic_.c_str(), body_imu_topic_.c_str(), body_pose_topic_.c_str());
    collectStatic();
    if (!collectMotion() || !ros::ok()) return 2;
    collectStaticValidation();
    if (!checkData()) return 2;
    Result result;
    if (!solve(result)) return 3;
    writeResults(result);
    printResult(result);
    return result.accepted ? 0 : 3;
  }

 private:
  void imuCallback(const sensor_msgs::ImuConstPtr &msg) {
    const double t = msg->header.stamp.toSec();
    if (!std::isfinite(t)) return;
    ImuSample s{t,
      body_gyro_scale_ * Vec3(msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z),
      body_accel_scale_ * Vec3(msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z)};
    if (!s.w.allFinite() || !s.a.allFinite()) return;
    std::lock_guard<std::mutex> lock(mutex_);
    if (body_frame_.empty()) body_frame_ = msg->header.frame_id;
    imu_.push_back(s);
  }

  void poseCallback(const sensor_msgs::ImuConstPtr &msg) {
    const double t = msg->header.stamp.toSec();
    const auto &q = msg->orientation;
    Eigen::Quaterniond quat(q.w, q.x, q.y, q.z);
    if (!std::isfinite(t) || !quat.coeffs().allFinite() || quat.norm() < 1e-6) return;
    std::lock_guard<std::mutex> lock(mutex_);
    pose_.push_back(PoseSample{t, Vec3::Zero(), quat.normalized().toRotationMatrix()});
    if (pose_frame_.empty()) pose_frame_ = msg->header.frame_id;
  }

  void odomCallback(const nav_msgs::OdometryConstPtr &msg) {
    const double t = msg->header.stamp.toSec();
    const auto &p = msg->pose.pose.position;
    const auto &q = msg->pose.pose.orientation;
    Eigen::Quaterniond quat(q.w, q.x, q.y, q.z);
    if (!std::isfinite(t) || !quat.coeffs().allFinite() || quat.norm() < 1e-6) return;
    std::lock_guard<std::mutex> lock(mutex_);
    odom_.push_back(PoseSample{t, Vec3(p.x, p.y, p.z), quat.normalized().toRotationMatrix()});
    if (odom_frame_.empty()) odom_frame_ = msg->header.frame_id;
  }

  void spinFor(double seconds) {
    ros::Rate rate(100);
    const ros::WallTime end = ros::WallTime::now() + ros::WallDuration(seconds);
    while (ros::ok() && ros::WallTime::now() < end) { ros::spinOnce(); rate.sleep(); }
  }

  void collectStatic() {
    ROS_INFO("[Initialization] Keep the assembly still for %.1f seconds.", static_s_);
    const ros::WallTime begin = ros::WallTime::now();
    while (ros::ok()) {
      const double f = (ros::WallTime::now() - begin).toSec() / static_s_;
      bar("[Initialization] Static initialization", f, f >= 1.0 ? GREEN : YELLOW);
      if (f >= 1.0) break;
      spinFor(update_s_);
    }
  }

  Vec3 staticBias() {
    std::lock_guard<std::mutex> lock(mutex_);
    Vec3 b = Vec3::Zero(); size_t n = 0;
    if (imu_.empty()) return b;
    const double end = imu_.front().t + static_s_;
    for (const auto &s : imu_) { if (s.t > end) break; b += s.w; ++n; }
    return n ? b / static_cast<double>(n) : b;
  }

  double latestImuTime() {
    std::lock_guard<std::mutex> lock(mutex_);
    return imu_.empty() ? 0.0 : imu_.back().t;
  }

  double excitation(int axis, double start, const Vec3 &bias) {
    std::lock_guard<std::mutex> lock(mutex_);
    double total = 0.0;
    for (size_t i = 1; i < imu_.size(); ++i) {
      if (imu_[i].t <= start) continue;
      const double dt = imu_[i].t - imu_[i - 1].t;
      if (dt <= 0.0 || dt > 0.1) continue;
      total += std::max(0.0, std::abs(imu_[i].w[axis] - bias[axis]) - 0.015) * dt;
    }
    return total / std::max(target_rad_, 1e-6);
  }

  bool collectMotion() {
    const char *labels[3] = {"ROLL / X", "PITCH / Y", "YAW / Z"};
    const char *help[3] = {"rotate slowly forward/back around X", "rotate slowly forward/back around Y", "rotate slowly forward/back around Z"};
    const Vec3 bias = staticBias();
    for (int axis = 0; axis < 3 && ros::ok(); ++axis) {
      ROS_INFO("[Initialization] Step %d/3: %s", axis + 1, labels[axis]);
      ROS_INFO("[Initialization] %s. This step waits until the bar reaches 100%%.", help[axis]);
      const double start = latestImuTime();
      const ros::WallTime begin = ros::WallTime::now();
      while (ros::ok()) {
        const double f = excitation(axis, start, bias);
        bar("[Initialization] Rotation around X Axis", axis == 0 ? f : axis > 0 ? 1.0 : 0.0, axis == 0 && f < 1.0 ? YELLOW : GREEN);
        bar("[Initialization] Rotation around Y Axis", axis == 1 ? f : axis > 1 ? 1.0 : 0.0, axis == 1 && f < 1.0 ? YELLOW : GREEN);
        bar("[Initialization] Rotation around Z Axis", axis == 2 ? f : 0.0, axis == 2 && f < 1.0 ? YELLOW : GREEN);
        if (f >= 1.0) break;
        if (motion_timeout_s_ > 0.0 && (ros::WallTime::now() - begin).toSec() >= motion_timeout_s_) return false;
        spinFor(update_s_);
      }
    }
    return true;
  }

  void collectStaticValidation() {
    const ros::WallTime begin = ros::WallTime::now();
    while (ros::ok() && (ros::WallTime::now() - begin).toSec() < static_validation_s_) {
      const double f = (ros::WallTime::now() - begin).toSec() / static_validation_s_;
      bar("[Initialization] Static validation", f, f >= 1.0 ? GREEN : YELLOW);
      spinFor(update_s_);
    }
  }

  bool checkData() {
    std::lock_guard<std::mutex> lock(mutex_);
    if (imu_.size() < static_cast<size_t>(min_samples_) || pose_.size() < static_cast<size_t>(min_samples_) ||
        odom_.size() < static_cast<size_t>(min_odom_samples_)) {
      ROS_ERROR("Insufficient data: raw IMU=%zu, ENU pose=%zu, FAST-LIO odom=%zu", imu_.size(), pose_.size(), odom_.size());
      return false;
    }
    if (body_frame_.find("_ned") != std::string::npos || body_frame_.find("_frd") != std::string::npos) {
      ROS_ERROR("PX4 IMU frame '%s' is not ROS base_link/FLU.", body_frame_.c_str());
      return false;
    }
    return true;
  }

  static Vec3 interp(const std::vector<ImuSample> &v, double t, bool gyro) {
    if (v.empty()) return Vec3::Zero();
    auto it = std::lower_bound(v.begin(), v.end(), t, [](const ImuSample &s, double x) { return s.t < x; });
    if (it == v.begin()) return gyro ? it->w : it->a;
    if (it == v.end()) return gyro ? v.back().w : v.back().a;
    const auto &a = *(it - 1); const auto &b = *it;
    const double u = (t - a.t) / std::max(1e-9, b.t - a.t);
    return gyro ? ((1.0 - u) * a.w + u * b.w) : ((1.0 - u) * a.a + u * b.a);
  }

  static bool interpPose(const std::vector<PoseSample> &v, double t, Mat3 &R, Vec3 *p = nullptr) {
    if (v.empty() || t < v.front().t || t > v.back().t) return false;
    auto it = std::lower_bound(v.begin(), v.end(), t,
                               [](const PoseSample &s, double x) { return s.t < x; });
    if (it == v.begin()) { R = it->R_W_S; if (p) *p = it->p; return true; }
    if (it == v.end()) { R = v.back().R_W_S; if (p) *p = v.back().p; return true; }
    const PoseSample &a = *(it - 1), &b = *it;
    const double u = (t - a.t) / std::max(1e-9, b.t - a.t);
    Eigen::Quaterniond qa(a.R_W_S), qb(b.R_W_S);
    R = qa.slerp(u, qb).normalized().toRotationMatrix();
    if (p) *p = (1.0 - u) * a.p + u * b.p;
    return true;
  }

  static Vec3 rotVec(const Mat3 &R) {
    Eigen::AngleAxisd aa(R);
    if (!std::isfinite(aa.angle()) || aa.angle() < 1e-9) return Vec3::Zero();
    return aa.axis() * aa.angle();
  }

  void stage(const std::string &name, double progress) {
    bar("[Calibration] " + name, progress, progress >= 1.0 ? GREEN : MAGENTA);
  }

  bool solve(Result &r) {
    std::vector<ImuSample> imu; std::vector<PoseSample> pose, odom;
    { std::lock_guard<std::mutex> lock(mutex_); imu = imu_; pose = pose_; odom = odom_; }
    std::sort(imu.begin(), imu.end(), [](const ImuSample &a, const ImuSample &b) { return a.t < b.t; });
    std::sort(pose.begin(), pose.end(), [](const PoseSample &a, const PoseSample &b) { return a.t < b.t; });
    std::sort(odom.begin(), odom.end(), [](const PoseSample &a, const PoseSample &b) { return a.t < b.t; });
    if (odom.size() < 3 || pose.size() < 3) return false;
    const Vec3 bias = staticBias();
    for (auto &s : imu) s.w -= bias;
    const Vec3 gravity_enu(0.0, 0.0, -9.80665);
    Vec3 accel_bias = Vec3::Zero();
    size_t accel_bias_samples = 0;
    for (const auto &s : imu) {
      if (s.t > imu.front().t + static_s_) break;
      Mat3 R_EB;
      if (!interpPose(pose, s.t, R_EB)) continue;
      // At rest, f_B + R_EB^T g_ENU should be zero.
      accel_bias += s.a + R_EB.transpose() * gravity_enu;
      ++accel_bias_samples;
    }
    if (accel_bias_samples) accel_bias /= static_cast<double>(accel_bias_samples);

    stage("SLAM odom motion", 0.1);
    std::vector<double> ts; std::vector<Mat3> Rwm; std::vector<Vec3> aw, wm, alpha;
    for (size_t i = 1; i + 1 < odom.size(); ++i) {
      const double dt0 = odom[i].t - odom[i - 1].t, dt1 = odom[i + 1].t - odom[i].t;
      if (dt0 <= 1e-4 || dt1 <= 1e-4 || dt0 > 1.0 || dt1 > 1.0) continue;
      const Vec3 v0 = (odom[i].p - odom[i - 1].p) / dt0;
      const Vec3 v1 = (odom[i + 1].p - odom[i].p) / dt1;
      const Vec3 w = rotVec(odom[i - 1].R_W_S.transpose() * odom[i + 1].R_W_S) * (2.0 / (dt0 + dt1));
      const Vec3 w0 = rotVec(odom[i - 1].R_W_S.transpose() * odom[i].R_W_S) / dt0;
      const Vec3 w1 = rotVec(odom[i].R_W_S.transpose() * odom[i + 1].R_W_S) / dt1;
      ts.push_back(odom[i].t); Rwm.push_back(odom[i].R_W_S); aw.push_back(2.0 * (v1 - v0) / (dt0 + dt1)); wm.push_back(w);
      alpha.push_back(2.0 * (w1 - w0) / (dt0 + dt1));
    }
    if (ts.size() < 10) return false;
    stage("SLAM odom motion", 1.0);

    stage("Rotation alignment", 0.1);
    Mat3 sum = Mat3::Zero(); size_t used = 0;
    for (size_t i = 0; i < ts.size(); ++i) {
      sum += interp(imu, ts[i], true) * wm[i].transpose(); ++used;
    }
    if (used < 10) return false;
    Eigen::JacobiSVD<Mat3> rot_svd(sum, Eigen::ComputeFullU | Eigen::ComputeFullV);
    r.R_B_M = rot_svd.matrixU() * rot_svd.matrixV().transpose();
    if (r.R_B_M.determinant() < 0.0) { Mat3 U = rot_svd.matrixU(); U.col(2) *= -1.0; r.R_B_M = U * rot_svd.matrixV().transpose(); }
    r.gyro_rmse = 0.0;
    for (size_t i = 0; i < ts.size(); ++i) { const Vec3 wb = interp(imu, ts[i], true); r.gyro_rmse += (wb - r.R_B_M * wm[i]).squaredNorm(); }
    r.gyro_rmse = std::sqrt(r.gyro_rmse / ts.size());
    stage("Rotation alignment", 1.0);

    stage("Lever-arm solve", 0.1);
    Eigen::MatrixXd A(3 * ts.size(), 6); Eigen::VectorXd y(3 * ts.size());
    size_t rows = 0;
    for (size_t i = 0; i < ts.size(); ++i) {
      Mat3 R_EB;
      if (!interpPose(pose, ts[i], R_EB)) continue;
      const Vec3 ab = interp(imu, ts[i], false) - accel_bias + R_EB.transpose() * gravity_enu;
      A.block<3, 3>(3 * rows, 0) = r.R_B_M * (skew(alpha[i]) + skew(wm[i]) * skew(wm[i]));
      A.block<3, 3>(3 * rows, 3) = Mat3::Identity();
      y.segment<3>(3 * rows) = ab - r.R_B_M * (Rwm[i].transpose() * aw[i]);
      ++rows;
    }
    if (rows < 10) return false;
    A.conservativeResize(3 * rows, 6);
    y.conservativeResize(3 * rows);
    const Eigen::JacobiSVD<Eigen::MatrixXd> svd(A.leftCols(3), Eigen::ComputeThinU | Eigen::ComputeThinV);
    r.translation_condition = std::numeric_limits<double>::infinity();
    if (svd.singularValues().size() >= 3 && svd.singularValues()(2) > 1e-6) r.translation_condition = svd.singularValues()(0) / svd.singularValues()(2);
    const Eigen::VectorXd x = A.colPivHouseholderQr().solve(y);
    r.t_M_B = x.head<3>(); r.t_B_M = -r.R_B_M * r.t_M_B;
    r.accel_rmse = (A * x - y).norm() / std::sqrt(static_cast<double>(rows));
    stage("Lever-arm solve", 1.0);

    const Mat3 R_W_B_ref = pose.back().R_W_S;
    r.reference_stamp = pose.back().t;
    r.t_M_B_enu = R_W_B_ref * r.t_B_M;
    r.accepted = std::isfinite(r.gyro_rmse) && std::isfinite(r.accel_rmse) &&
                 r.accel_rmse < 1.5 && r.translation_condition < 1e5;
    r.note = r.accepted ? "accepted" : "poor odom/IMU excitation or residual quality";
    return true;
  }

  std::string packagePath(const std::string &p) const { return p.empty() || p[0] == '/' ? p : ros::package::getPath("mid360_px4_calibration") + "/" + p; }
  void writeResults(const Result &r) {
    const std::string y = packagePath(output_yaml_), t = packagePath(output_txt_);
    const Vec3 rpy = rpyDeg(r.R_B_M);
    std::ofstream fy(y), ft(t);
    fy << "slam_odom_to_px4_imu:\n"
       << "  translation_body_FLU_m: " << list3(r.t_B_M) << "\n"
       << "  translation_mid360_relative_to_px4_imu_ENU_m: " << list3(r.t_M_B_enu) << "\n"
       << "  enu_reference_stamp: " << r.reference_stamp << "\n"
       << "  rpy_mid360_to_body_XYZ_deg: " << list3(rpy) << "\n"
       << "quality:\n  accel_rmse_m_s2: " << r.accel_rmse
       << "\n  translation_condition_number: " << r.translation_condition
       << "\n  accepted: " << (r.accepted ? "true" : "false") << "\n  note: \"" << r.note << "\"\n";
    ft << std::fixed << std::setprecision(9)
       << "SLAM odometry Mid360 relative to PX4 IMU\n"
       << "translation_body_FLU_m: " << r.t_B_M.transpose() << "\n"
       << "translation_mid360_relative_to_px4_imu_ENU_m: " << r.t_M_B_enu.transpose() << "\n"
       << "enu_reference_stamp: " << r.reference_stamp << "\n"
       << "rpy_mid360_to_body_XYZ_deg: " << rpy.transpose() << "\n"
       << "accel_rmse_m_s2: " << r.accel_rmse << "\n"
       << "translation_condition_number: " << r.translation_condition << "\n"
       << "accepted: " << (r.accepted ? "true" : "false") << "\n"
       << "note: " << r.note << "\n";
  }
  void printResult(const Result &r) {
    std::cout << "\n" << CYAN << "[Final Result] " << RESET
              << "Mid360 relative to PX4 IMU in ENU (reference attitude) = "
              << r.t_M_B_enu.transpose() << " m\n"
              << CYAN << "[Final Result] " << RESET
              << "Fixed body-frame FLU offset = " << r.t_B_M.transpose() << " m\n"
              << GREEN << "[Final Result] " << RESET
              << "accepted=" << (r.accepted ? "true" : "false")
              << ", accel RMSE=" << r.accel_rmse << "\n";
  }

  ros::NodeHandle nh_, pnh_;
  ros::Subscriber sub_imu_, sub_pose_, sub_odom_;
  std::mutex mutex_;
  std::vector<ImuSample> imu_;
  std::vector<PoseSample> pose_, odom_;
  std::string odom_topic_, body_imu_topic_, body_pose_topic_, output_yaml_, output_txt_;
  std::string body_frame_, pose_frame_, odom_frame_;
  double body_accel_scale_, body_gyro_scale_, static_s_, static_validation_s_, motion_timeout_s_, update_s_, target_rad_;
  int min_samples_, min_odom_samples_;
};
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "mid360_px4_online_calibration");
  ros::NodeHandle nh, pnh("~");
  Calibrator calibrator(nh, pnh);
  return calibrator.run();
}
