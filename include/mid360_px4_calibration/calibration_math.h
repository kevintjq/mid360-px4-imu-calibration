#pragma once

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <algorithm>
#include <cmath>
#include <limits>
#include <string>
#include <vector>

namespace mid360_calibration {
using Vec3 = Eigen::Vector3d;
using Mat3 = Eigen::Matrix3d;
struct Imu { double t; Vec3 w, a; Mat3 R_EB; };
struct Pose { double t; Vec3 p; Mat3 R_WM; };
struct Motion { double t; Vec3 a, w, alpha; };
struct Interval { double begin, end; };
struct Limits {
  double max_imu_gap = 0.1, max_odom_gap = 0.25;
  double derivative_half_window = 0.3;
  double max_gyro_rmse = 0.12, max_accel_rmse = 0.75;
  double max_gyro_relative_rmse = 0.25, min_gyro_correlation = 0.95;
  double min_rotation_ratio = 0.02;
  double max_translation_condition = 1000.0;
  double min_translation_excitation = 0.03, min_alpha_rms = 0.1;
  double max_lever_arm = 2.0;
  size_t min_motion_samples = 150;
};
struct Result {
  Mat3 R_B_M = Mat3::Identity();
  Vec3 t_B_M = Vec3::Zero(), accel_bias = Vec3::Zero();
  double gyro_rmse = 0, accel_rmse = 0, rotation_ratio = 0;
  double gyro_relative_rmse = std::numeric_limits<double>::infinity();
  double gyro_correlation = 0, imu_gyro_rms = 0, odom_gyro_rms = 0;
  double translation_condition = std::numeric_limits<double>::infinity();
  double translation_excitation = 0, alpha_rms = 0;
  size_t samples = 0;
  bool accepted = false;
  std::vector<std::string> reasons;
};
inline Mat3 skew(const Vec3 &v) {
  Mat3 s;
  s << 0, -v.z(), v.y(), v.z(), 0, -v.x(), -v.y(), v.x(), 0;
  return s;
}
inline Vec3 logSO3(const Mat3 &R) {
  Eigen::AngleAxisd aa(R);
  return aa.angle() < 1e-10 ? Vec3::Zero().eval() : (aa.axis() * aa.angle()).eval();
}
// Never extrapolate or interpolate across a dropped-message gap.
inline bool interpolate(const std::vector<Imu> &v, double t, double max_gap, Imu &out) {
  if (v.empty() || !std::isfinite(t) || t < v.front().t || t > v.back().t) return false;
  auto b = std::lower_bound(v.begin(), v.end(), t,
                           [](const Imu &s, double x) { return s.t < x; });
  if (b == v.end()) return false;
  if (b->t == t) { out = *b; return true; }
  if (b == v.begin()) return false;
  const auto &a = *(b - 1);
  const double dt = b->t - a.t;
  if (dt <= 0 || dt > max_gap) return false;
  const double u = (t - a.t) / dt;
  out.t = t;
  out.w = (1-u)*a.w + u*b->w;
  out.a = (1-u)*a.a + u*b->a;
  out.R_EB = Eigen::Quaterniond(a.R_EB).slerp(u, Eigen::Quaterniond(b->R_EB)).toRotationMatrix();
  return true;
}
struct StaticStats {
  size_t samples = 0;
  double span = 0, gyro_rms = 0, gyro_std = 0, accel_std = 0;
  Vec3 gyro_mean = Vec3::Zero(), accel_mean = Vec3::Zero();
  bool continuous = false;
};
inline StaticStats staticStats(const std::vector<Imu> &v, double begin, double end,
                               const Vec3 &bias, double max_gap) {
  StaticStats r;
  double first = 0, last = 0;
  r.continuous = true;
  for (const auto &s : v) {
    if (s.t < begin || s.t > end) continue;
    if (!r.samples) first = s.t;
    else if (s.t - last > max_gap || s.t <= last) r.continuous = false;
    last = s.t;
    ++r.samples;
    r.gyro_mean += s.w;
    r.accel_mean += s.a;
    r.gyro_rms += (s.w - bias).squaredNorm();
  }
  if (!r.samples) { r.continuous = false; return r; }
  r.span = last - first;
  r.gyro_mean /= r.samples;
  r.accel_mean /= r.samples;
  r.gyro_rms = std::sqrt(r.gyro_rms / r.samples);
  for (const auto &s : v) {
    if (s.t < begin || s.t > end) continue;
    r.gyro_std += (s.w - r.gyro_mean).squaredNorm();
    r.accel_std += (s.a - r.accel_mean).squaredNorm();
  }
  r.gyro_std = std::sqrt(r.gyro_std / r.samples);
  r.accel_std = std::sqrt(r.accel_std / r.samples);
  return r;
}
// Local cubic fits smooth noise and support irregular odometry intervals.
// Log(R_i^T R(t)) has first/second derivatives w_i and alpha_i at t_i.
inline std::vector<Motion> differentiate(const std::vector<Pose> &v, const Limits &lim,
                                        const std::vector<Interval> &windows) {
  std::vector<Motion> out;
  for (size_t i = 0; i < v.size(); ++i) {
    double begin = 0, end = 0;
    bool inside = false;
    for (const auto &win : windows) if (v[i].t >= win.begin && v[i].t <= win.end) {
      begin = win.begin; end = win.end; inside = true; break;
    }
    if (!inside) continue;
    size_t l = i, r = i;
    while (l > 0 && v[i].t-v[l-1].t <= lim.derivative_half_window && v[l-1].t >= begin) --l;
    while (r+1 < v.size() && v[r+1].t-v[i].t <= lim.derivative_half_window && v[r+1].t <= end) ++r;
    if (r-l+1 < 5 || v[i].t-v[l].t < 0.5*lim.derivative_half_window ||
        v[r].t-v[i].t < 0.5*lim.derivative_half_window) continue;
    bool continuous = true;
    for (size_t j = l+1; j <= r; ++j)
      if (v[j].t-v[j-1].t <= 0 || v[j].t-v[j-1].t > lim.max_odom_gap) continuous = false;
    if (!continuous) continue;
    Eigen::MatrixXd D(r-l+1, 4), Y(r-l+1, 6);
    for (size_t j = l; j <= r; ++j) {
      const double u = (v[j].t-v[i].t) / lim.derivative_half_window;
      D.row(j-l) << 1, u, u*u, u*u*u;
      Y.block<1,3>(j-l,0) = (v[j].p-v[i].p).transpose();
      Y.block<1,3>(j-l,3) = logSO3(v[i].R_WM.transpose()*v[j].R_WM).transpose();
    }
    const Eigen::MatrixXd coeff = D.colPivHouseholderQr().solve(Y);
    Motion m;
    m.t = v[i].t;
    m.a = v[i].R_WM.transpose()*coeff.block<1,3>(2,0).transpose()*2 /
          (lim.derivative_half_window*lim.derivative_half_window);
    m.w = coeff.block<1,3>(1,3).transpose() / lim.derivative_half_window;
    m.alpha = coeff.block<1,3>(2,3).transpose()*2 /
              (lim.derivative_half_window*lim.derivative_half_window);
    if (m.a.allFinite() && m.w.allFinite() && m.alpha.allFinite() &&
        m.a.norm() < 100 && m.w.norm() < 10 && m.alpha.norm() < 100 &&
        (m.w.norm() > 0.05 || m.alpha.norm() > 0.1)) out.push_back(m);
  }
  return out;
}
struct LeverEquation { Mat3 C, D; Vec3 y; };

// Inputs are paired angular velocities with gyro bias already removed.
inline bool fitRotation(const std::vector<Motion> &motion,const std::vector<Imu> &body,
                        const Limits &lim,Result &r) {
  r.samples=motion.size();
  if(motion.size()!=body.size() || motion.size()<lim.min_motion_samples) {
    r.reasons.push_back("有效运动配对样本不足（检查话题频率、时间同步和里程计连续性）");return false;
  }
  Mat3 cross=Mat3::Zero(),excitation=Mat3::Zero();
  for(size_t i=0;i<motion.size();++i) {
    cross+=body[i].w*motion[i].w.transpose();
    excitation+=motion[i].w*motion[i].w.transpose();
  }
  Eigen::JacobiSVD<Mat3> ex(excitation),svd(cross,Eigen::ComputeFullU|Eigen::ComputeFullV);
  r.rotation_ratio=ex.singularValues()(2)/std::max(1e-12,ex.singularValues()(0));
  Mat3 sign=Mat3::Identity();
  sign(2,2)=(svd.matrixU()*svd.matrixV().transpose()).determinant()<0 ? -1:1;
  r.R_B_M=svd.matrixU()*sign*svd.matrixV().transpose();
  double agreement=0;
  for(size_t i=0;i<motion.size();++i) {
    r.gyro_rmse+=(body[i].w-r.R_B_M*motion[i].w).squaredNorm();
    r.imu_gyro_rms+=body[i].w.squaredNorm();
    r.odom_gyro_rms+=motion[i].w.squaredNorm();
    agreement+=body[i].w.dot(r.R_B_M*motion[i].w);
    r.alpha_rms+=motion[i].alpha.squaredNorm();
  }
  const double denominator=std::sqrt(r.imu_gyro_rms*r.odom_gyro_rms);
  if(denominator>1e-12)r.gyro_correlation=std::max(-1.0,std::min(1.0,agreement/denominator));
  r.gyro_rmse=std::sqrt(r.gyro_rmse/motion.size());
  r.imu_gyro_rms=std::sqrt(r.imu_gyro_rms/motion.size());
  r.odom_gyro_rms=std::sqrt(r.odom_gyro_rms/motion.size());
  r.alpha_rms=std::sqrt(r.alpha_rms/motion.size());
  const double signal=std::sqrt(0.5*(r.imu_gyro_rms*r.imu_gyro_rms+r.odom_gyro_rms*r.odom_gyro_rms));
  if(signal>1e-9)r.gyro_relative_rmse=r.gyro_rmse/signal;
  return true;
}
inline bool fitLever(const std::vector<LeverEquation> &equations,const Vec3 &initial_bias,Result &r) {
  const size_t count=equations.size();
  if(count<10) { r.reasons.push_back("有效杆臂方程不足");return false; }
  Eigen::MatrixXd C(3*count,3),D(3*count,3);
  Eigen::VectorXd y(3*count);
  for(size_t i=0;i<count;++i) {
    C.block<3,3>(3*i,0)=equations[i].C;
    D.block<3,3>(3*i,0)=equations[i].D;
    y.segment<3>(3*i)=equations[i].y;
  }
  // Project out the bias subspace. Unlike simple mean subtraction this also
  // supports the rotating, integrated body-bias basis of matched windows.
  const auto bias_qr=D.colPivHouseholderQr();
  if(bias_qr.rank()<3) { r.reasons.push_back("加速度偏置子系统退化");return false; }
  const Eigen::MatrixXd Cp=C-D*bias_qr.solve(C);
  const Eigen::VectorXd yp=y-D*bias_qr.solve(y);
  Eigen::JacobiSVD<Eigen::MatrixXd> svd(Cp,Eigen::ComputeThinU|Eigen::ComputeThinV);
  const auto singular=svd.singularValues();
  r.translation_excitation=singular(2)/std::sqrt(static_cast<double>(count));
  if(singular(2)>1e-10)r.translation_condition=singular(0)/singular(2);
  const Vec3 inverse_lever=svd.solve(yp);
  const Vec3 residual_bias=bias_qr.solve(y-C*inverse_lever);
  r.t_B_M=-r.R_B_M*inverse_lever;
  r.accel_bias=initial_bias+residual_bias;
  r.accel_rmse=(C*inverse_lever+D*residual_bias-y).norm()/std::sqrt(static_cast<double>(count));
  return true;
}
inline void validateResult(const Limits &lim,Result &r) {
  if (!r.R_B_M.allFinite() || !r.t_B_M.allFinite() || !r.accel_bias.allFinite() ||
      !std::isfinite(r.gyro_rmse) || !std::isfinite(r.accel_rmse)) r.reasons.push_back("求解出现非有限数值");
  if (r.rotation_ratio < lim.min_rotation_ratio) r.reasons.push_back("三轴旋转激励不充分");
  if (r.gyro_rmse > lim.max_gyro_rmse) r.reasons.push_back("角速度匹配误差过大（检查时间同步、单位和刚性安装）");
  if (!std::isfinite(r.gyro_relative_rmse) || r.gyro_relative_rmse > lim.max_gyro_relative_rmse)
    r.reasons.push_back("角速度相对误差过大，不能用低运动幅度掩盖旋转失配");
  if (r.gyro_correlation < lim.min_gyro_correlation)
    r.reasons.push_back("两路角速度运动不一致，请检查时间同步和数据来源");
  if (r.accel_rmse > lim.max_accel_rmse) r.reasons.push_back("加速度残差过大（检查里程计跟踪和振动）");
  if (!std::isfinite(r.translation_condition) || r.translation_condition > lim.max_translation_condition ||
      r.translation_excitation < lim.min_translation_excitation) r.reasons.push_back("杆臂与偏置无法可靠区分，请增加平滑加减速的三轴转动");
  if (r.alpha_rms < lim.min_alpha_rms) r.reasons.push_back("角加速度不足，请避免只做匀速转动");
  if (r.t_B_M.norm() > lim.max_lever_arm) r.reasons.push_back("杆臂超过配置的物理尺寸上限");
  r.accepted = r.reasons.empty();
}

// Instantaneous equations for exact motion inputs. Pose trajectories must use
// solveTrajectory(), which matches both sides of the differentiation filter.
inline Result solve(const std::vector<Motion> &motion,const std::vector<Imu> &imu,
                    const Vec3 &gyro_bias,const Vec3 &initial_accel_bias,const Limits &lim) {
  Result r;std::vector<Motion> ms;std::vector<Imu> bs;
  for(const auto &m:motion) {
    Imu b;
    if(!interpolate(imu,m.t,lim.max_imu_gap,b))continue;
    b.w-=gyro_bias;ms.push_back(m);bs.push_back(b);
  }
  if(!fitRotation(ms,bs,lim,r))return r;
  std::vector<LeverEquation> equations;
  for(size_t i=0;i<ms.size();++i) {
    const auto &m=ms[i];const auto &b=bs[i];
    equations.push_back({r.R_B_M*(skew(m.alpha)+skew(m.w)*skew(m.w)),Mat3::Identity(),
      b.a-initial_accel_bias+b.R_EB.transpose()*Vec3(0,0,-9.80665)-r.R_B_M*m.a});
  }
  if(!fitLever(equations,initial_accel_bias,r))return r;
  validateResult(lim,r);return r;
}
}  // namespace mid360_calibration
