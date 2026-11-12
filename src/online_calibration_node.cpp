#include <ros/ros.h>
#include <ros/package.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <mid360_px4_calibration/calibration_math.h>
#include <mid360_px4_calibration/static_window.h>
#include <mid360_px4_calibration/terminal_ui.h>
#include <mid360_px4_calibration/handling_gate.h>
#include <mid360_px4_calibration/matched_solver.h>

#include <cerrno>
#include <clocale>
#include <cwchar>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <unistd.h>

namespace mc = mid360_calibration;
namespace {
using mc::Vec3;
using mc::Mat3;
std::string number(double x, int precision = 2) {
  std::ostringstream s; s << std::fixed << std::setprecision(precision) << x; return s.str();
}
std::string vectorText(const Vec3 &v) {
  return "[" + number(v.x(),9) + ", " + number(v.y(),9) + ", " + number(v.z(),9) + "]";
}
using Terminal = mc::Terminal;

class Calibrator {
 public:
  Calibrator(ros::NodeHandle &nh, ros::NodeHandle &pnh) : nh_(nh), pnh_(pnh) {
    param("slam_odom_topic", odom_topic_, std::string("/Odometry"));
    param("body_imu_topic", imu_topic_, std::string("/mavros/imu/data"));
    param("output_yaml", output_yaml_, std::string("~/.ros/mid360_px4_calibration/result.yaml"));
    param("output_txt", output_txt_, std::string("~/.ros/mid360_px4_calibration/result.txt"));
    param("body_accel_scale", accel_scale_, 1.0);
    param("body_gyro_scale", gyro_scale_, 1.0);
    param("imu_time_offset_s", imu_offset_, 0.0);
    param("static_duration_s", static_s_, 8.0);
    param("static_validation_duration_s", validation_s_, 5.0);
    param("neutral_duration_s", neutral_s_, 2.0);
    param("static_max_rotation_deg", static_rotation_deg_, 1.0);
    param("static_timeout_s", static_timeout_, 120.0);
    param("pickup_min_duration_s", pickup_min_s_, 5.0);
    param("pickup_min_displacement_m", pickup_distance_, 0.08);
    param("pickup_timeout_s", pickup_timeout_, 120.0);
    param("putdown_buffer_s", putdown_s_, 5.0);
    param("axis_min_duration_s", axis_min_s_, 20.0);
    param("axis_min_active_s", active_s_, 8.0);
    param("axis_direction_target_rad", direction_rad_, 1.5);
    param("motion_rate_threshold_rad_s", motion_rate_, 0.08);
    param("motion_timeout_s", motion_timeout_, 180.0);
    param("motion_update_period_s", update_s_, 0.25);
    param("data_timeout_s", data_timeout_, 2.0);
    param("startup_timeout_s", startup_timeout_, 30.0);
    param("max_recording_duration_s", recording_timeout_, 900.0);
    param("min_static_samples", min_static_, 100);
    param("axis_min_imu_samples", axis_min_imu_, 300);
    param("axis_min_odom_samples", axis_min_odom_, 80);
    param("min_samples", min_samples_, 1500);
    param("min_odom_samples", min_odom_, 300);
    int min_motion; param("min_motion_samples", min_motion, 150);
    limits_.min_motion_samples = std::max(1,min_motion);
    param("max_imu_gap_s", limits_.max_imu_gap, 0.1);
    param("max_odom_gap_s", limits_.max_odom_gap, 0.25);
    param("derivative_half_window_s", limits_.derivative_half_window, 0.3);
    param("max_gyro_rmse_rad_s", limits_.max_gyro_rmse, 0.12);
    param("max_gyro_relative_rmse", limits_.max_gyro_relative_rmse, 0.25);
    param("min_gyro_correlation", limits_.min_gyro_correlation, 0.95);
    param("max_accel_rmse_m_s2", limits_.max_accel_rmse, 0.75);
    param("min_rotation_excitation_ratio", limits_.min_rotation_ratio, 0.02);
    param("max_translation_condition", limits_.max_translation_condition, 1000.0);
    param("min_translation_excitation_rad_s2", limits_.min_translation_excitation, 0.03);
    param("min_angular_accel_rms_rad_s2", limits_.min_alpha_rms, 0.1);
    param("max_lever_arm_m", limits_.max_lever_arm, 2.0);
    if (!std::isfinite(imu_offset_) || !positive(accel_scale_) || !positive(gyro_scale_) ||
        !positive(static_s_) || !positive(validation_s_) || !positive(neutral_s_) ||
        !positive(static_timeout_) || !positive(static_rotation_deg_) || static_rotation_deg_>=180 ||
        !positive(pickup_min_s_) || !positive(pickup_distance_) || !positive(pickup_timeout_) ||
        pickup_timeout_<=pickup_min_s_ || !positive(putdown_s_) ||
        !positive(axis_min_s_) || !positive(active_s_) ||
        !positive(direction_rad_) || !positive(motion_rate_) || !positive(motion_timeout_) ||
        !positive(update_s_) || update_s_ > 1 || !positive(data_timeout_) ||
        !positive(startup_timeout_) || !positive(recording_timeout_) ||
        !positive(limits_.max_imu_gap) || !positive(limits_.max_odom_gap) ||
        !positive(limits_.derivative_half_window) || !positive(limits_.max_gyro_rmse) ||
        !positive(limits_.max_gyro_relative_rmse) || !positive(limits_.min_gyro_correlation) ||
        limits_.min_gyro_correlation>1 ||
        !positive(limits_.max_accel_rmse) || !positive(limits_.min_rotation_ratio) ||
        limits_.min_rotation_ratio > 1 || !positive(limits_.max_translation_condition) ||
        !positive(limits_.min_translation_excitation) || !positive(limits_.min_alpha_rms) ||
        !positive(limits_.max_lever_arm) || min_static_ < 2 || axis_min_imu_ < 2 ||
        axis_min_odom_ < 5 || min_samples_ < 2 || min_odom_ < 5 || min_motion < 10 ||
        active_s_ > axis_min_s_ || axis_min_s_ >= motion_timeout_ ||
        static_timeout_ <= std::max(static_s_,validation_s_))
      throw std::runtime_error("配置无效：时长、阈值须为正数，阶段超时须大于最短时长，样本数量须合理");
    sub_imu_ = nh_.subscribe(imu_topic_, 2000, &Calibrator::imuCallback, this);
    sub_odom_ = nh_.subscribe(odom_topic_, 500, &Calibrator::odomCallback, this);
  }
  int run() {
    ui_.summary("准备开始",{
      "按提示完成操作，达到要求后自动进入下一步。",
      "桌面静止 → 拿起飞机 → 手持三轴往复 → 放下 → 验收",
      "每轴至少 "+number(axis_min_s_,0)+" 秒；拿起、放下过程不参与求解。",
      "IMU  "+imu_topic_,"ODOM "+odom_topic_});
    started_ = ros::WallTime::now().toSec();
    if (!waitForData()) return fail();
    double begin = 0, end = 0;
    if (!collectStill("02 / 08  桌面静止初始化",static_s_,true,min_static_,begin,end)) return fail();
    const auto stats = mc::staticStats(imu_,begin,end,Vec3::Zero(),limits_.max_imu_gap);
    gyro_bias_ = stats.gyro_mean;
    size_t n = 0;
    for (const auto &s : imu_) if (s.t >= begin && s.t <= end) {
      accel_bias_ += s.a+s.R_EB.transpose()*Vec3(0,0,-9.80665); ++n;
    }
    if (!n) { error_="初始化窗口没有有效姿态样本"; return fail(); }
    accel_bias_ /= n;
    ui_.message("✓ 静止初始化完成 · 陀螺仪偏置 "+vectorText(gyro_bias_)+" rad/s");
    if (!waitForPickup()) return fail();
    const char *names[] = {"横滚 ROLL / X", "俯仰 PITCH / Y", "偏航 YAW / Z"};
    const char *help[] = {
      "绕机体前后轴左右倾斜约 ±20–35°，再缓慢返回；往复 3–5 次。",
      "绕机体左右轴前后俯仰约 ±20–35°，再缓慢返回；往复 3–5 次。",
      "尽量保持水平，左右转向约 ±30–45°，再缓慢返回；往复 3–5 次。"};
    for (int axis=0; axis<3; ++axis) {
      if (!collectStill("0"+std::to_string(axis+4)+" / 08  "+names[axis]+" · 手持居中",neutral_s_,false,20,begin,end,true)) return fail();
      if (!collectAxis(axis,names[axis],help[axis])) return fail();
    }
    if (!waitForPutdown()) return fail();
    if (!collectStill("07 / 08  放稳后的静止验证",validation_s_,false,min_static_,begin,end)) return fail();
    if (imu_.size()<static_cast<size_t>(min_samples_) || odom_.size()<static_cast<size_t>(min_odom_)) {
      error_="总样本不足，请检查话题频率；IMU="+std::to_string(imu_.size())+"，odom="+std::to_string(odom_.size()); return fail();
    }
    ui_.panel("08 / 08  求解与质量验收",0,{"采集完成，请稍候。","正在进行同窗口积分配准，计算旋转、杆臂和质量指标……"});
    const mc::Result r = mc::solveTrajectory(odom_,imu_,windows_,gyro_bias_,accel_bias_,limits_);
    report(r);
    if (!r.accepted) {
      ui_.message("未通过质量验收，未覆盖已有结果。请按上面的原因重新采集。",true); return 3;
    }
    if (!writeResults(r)) return fail();
    ui_.summary("✓ 标定完成 · 结果已保存",{
      "FLU 偏移 (m)  "+vectorText(r.t_B_M),
      "YAML  "+resolvedPath(output_yaml_),"TXT   "+resolvedPath(output_txt_)});
    return 0;
  }
 private:
  template<class T> void param(const char *name,T &value,const T &fallback) { pnh_.param(name,value,fallback); }
  static bool positive(double v) { return std::isfinite(v) && v>0; }
  void imuCallback(const sensor_msgs::ImuConstPtr &msg) {
    const auto &q = msg->orientation;
    Eigen::Quaterniond quat(q.w,q.x,q.y,q.z);
    mc::Imu s{msg->header.stamp.toSec()+imu_offset_,
      gyro_scale_*Vec3(msg->angular_velocity.x,msg->angular_velocity.y,msg->angular_velocity.z),
      accel_scale_*Vec3(msg->linear_acceleration.x,msg->linear_acceleration.y,msg->linear_acceleration.z),Mat3::Identity()};
    if (!std::isfinite(s.t) || s.t<=0 || !s.w.allFinite() || !s.a.allFinite() ||
        !quat.coeffs().allFinite() || quat.norm()<1e-6 || msg->orientation_covariance[0]<0 ||
        msg->angular_velocity_covariance[0]<0 || msg->linear_acceleration_covariance[0]<0 ||
        (!imu_.empty() && s.t<=imu_.back().t)) { ++dropped_imu_; return; }
    if (msg->header.frame_id.find("_ned")!=std::string::npos || msg->header.frame_id.find("_frd")!=std::string::npos) {
      error_="IMU 必须使用 ROS FLU 机体系和 ENU 姿态，不能直接使用 NED/FRD 数据"; return;
    }
    if (!imu_.empty() && body_frame_!=msg->header.frame_id) { error_="IMU 坐标系在采集中发生变化"; return; }
    body_frame_ = msg->header.frame_id;
    s.R_EB = quat.normalized().toRotationMatrix();
    imu_.push_back(s); last_imu_wall_ = ros::WallTime::now().toSec();
  }
  void odomCallback(const nav_msgs::OdometryConstPtr &msg) {
    const auto &q = msg->pose.pose.orientation; const auto &p = msg->pose.pose.position;
    Eigen::Quaterniond quat(q.w,q.x,q.y,q.z);
    mc::Pose s{msg->header.stamp.toSec(),Vec3(p.x,p.y,p.z),Mat3::Identity()};
    if (!std::isfinite(s.t) || s.t<=0 || !s.p.allFinite() || !quat.coeffs().allFinite() ||
        quat.norm()<1e-6 || (!odom_.empty() && s.t<=odom_.back().t)) { ++dropped_odom_; return; }
    if (!odom_.empty() && (world_frame_!=msg->header.frame_id || mid_frame_!=msg->child_frame_id)) {
      error_="里程计坐标系在采集中发生变化"; return;
    }
    world_frame_=msg->header.frame_id; mid_frame_=msg->child_frame_id;
    s.R_WM=quat.normalized().toRotationMatrix();
    odom_.push_back(s); last_odom_wall_=ros::WallTime::now().toSec();
  }
  void spinFor(double seconds) {
    ros::WallRate rate(100);
    const double until=ros::WallTime::now().toSec()+seconds;
    while (ros::ok() && ros::WallTime::now().toSec()<until) { ros::spinOnce(); rate.sleep(); }
  }
  bool healthy() {
    if (!ros::ok()) { error_="用户停止了标定"; return false; }
    if (!error_.empty()) return false;
    const double now=ros::WallTime::now().toSec();
    if (now-started_>recording_timeout_) error_="超过总采集时限，请检查数据并重新开始";
    else if (now-last_imu_wall_>data_timeout_ || now-last_odom_wall_>data_timeout_)
      error_="话题停止更新或数据无效，请检查时间戳、姿态有效标志和驱动";
    else if (std::abs(imu_.back().t-odom_.back().t)>data_timeout_)
      error_="IMU 与里程计时间不重叠，请检查时钟和 imu_time_offset_s";
    return error_.empty();
  }
  bool waitForData() {
    const double start=ros::WallTime::now().toSec();
    while (ros::ok() && error_.empty()) {
      spinFor(update_s_);
      const double elapsed=ros::WallTime::now().toSec()-start;
      ui_.panel("01 / 08  连接数据源",elapsed/startup_timeout_,{
        "请先启动 MAVROS 与 FAST-LIO，保持飞机放稳。",
        "等待有效姿态、递增时间戳及两路共同时间范围。"},
        {{"IMU 样本",static_cast<double>(imu_.size()),2,"帧",0},
         {"ODOM 样本",static_cast<double>(odom_.size()),2,"帧",0},
         {"时间差",(!imu_.empty() && !odom_.empty()) ? std::abs(imu_.back().t-odom_.back().t) : std::numeric_limits<double>::infinity(),data_timeout_,"s（上限）",2,false},
         {"消息间隔",(!imu_.empty() && !odom_.empty()) ? ros::WallTime::now().toSec()-std::min(last_imu_wall_,last_odom_wall_) : std::numeric_limits<double>::infinity(),data_timeout_,"s（上限）",2,false},
         {"等待时间",elapsed,startup_timeout_,"s（上限）",1,false}});
      const double now=ros::WallTime::now().toSec();
      if (imu_.size()>=2 && odom_.size()>=2 && now-last_imu_wall_<data_timeout_ &&
          now-last_odom_wall_<data_timeout_ && std::abs(imu_.back().t-odom_.back().t)<data_timeout_) return true;
      if (elapsed>startup_timeout_) { error_="等待话题超时；请先启动 MAVROS 和 FAST-LIO，并确认时间同步"; return false; }
    }
    return false;
  }
  bool waitForPickup() {
    mc::PickupGate gate(odom_.back(),pickup_min_s_,pickup_distance_,limits_.max_odom_gap);
    size_t checked=odom_.size();
    const double start=ros::WallTime::now().toSec();
    ui_.message("请拿起飞机 · 离开原位置约 "+number(pickup_distance_*100+2,0)+" cm 以上，然后手持停稳。无需按键。");
    while (healthy()) {
      spinFor(update_s_);
      if(!healthy())return false;
      for(;checked<odom_.size();++checked)gate.push(odom_[checked]);
      ui_.panel("03 / 08  拿起飞机",gate.progress(),{
        "现在拿起飞机，动作放轻；这一阶段不记录标定运动。",
        "确认离开原位置后，会提示你手持居中并停稳。"},
        {{"准备时间",gate.elapsed(),pickup_min_s_,"s",1},
         {"离开原位",gate.distance()*100,pickup_distance_*100,"cm",1},
         {"位移确认",gate.awayDuration(),0.5,"s",1}});
      if(gate.ready()) { ui_.message("✓ 已检测到离开原位置 · 接下来保持手持，不必放回桌面。");return true; }
      if(ros::WallTime::now().toSec()-start>pickup_timeout_) {
        error_="等待拿起飞机超时；请确认实际拿起且里程计位置正常更新";return false;
      }
    }
    return false;
  }
  bool waitForPutdown() {
    const double start=ros::WallTime::now().toSec();
    ui_.message("✓ 三轴采集完成 · 现在可以把飞机轻放回桌面或地面。");
    while(healthy()) {
      spinFor(update_s_);
      if(!healthy())return false;
      const double elapsed=ros::WallTime::now().toSec()-start;
      ui_.panel("07 / 08  放下飞机",elapsed/putdown_s_,{
        "将飞机轻放在稳定平面上，松手并等待晃动停止。",
        "放下过程不参与求解；随后还需完整静止 "+number(validation_s_,0)+" 秒。"},
        {{"放置缓冲",elapsed,putdown_s_,"s",1}});
      if(elapsed>=putdown_s_)return true;
    }
    return false;
  }
  bool collectStill(const std::string &name,double duration,bool initial,int min_count,double &begin,double &end,bool handheld=false) {
    const double wall_start=ros::WallTime::now().toSec();
    mc::StaticLimits limits;
    limits.duration=duration;
    limits.min_samples=min_count;
    limits.max_gap=limits_.max_imu_gap;
    limits.max_gyro=initial ? 0.08 : 0.05;
    limits.max_rotation_rad=static_rotation_deg_*M_PI/180.0;
    mc::StaticWindow window(limits,initial ? Vec3::Zero().eval() : gyro_bias_);
    size_t checked=imu_.size(), odom_checked=odom_.size();
    Mat3 odom_anchor=odom_.back().R_WM;
    double previous_odom_time=odom_.back().t;
    while (healthy()) {
      spinFor(update_s_);
      if (!healthy()) return false;
      for (;checked<imu_.size();++checked) window.push(imu_[checked]);
      for (;odom_checked<odom_.size();++odom_checked) {
        const auto &pose=odom_[odom_checked];
        const bool gap=pose.t-previous_odom_time>limits_.max_odom_gap;
        const bool moved=mc::logSO3(odom_anchor.transpose()*pose.R_WM).norm()>limits.max_rotation_rad;
        if (gap || moved) {
          window.invalidate(gap ? "里程计缺帧，重新等待完整静止窗口" :
                                  "里程计姿态变化，重新等待完整静止窗口");
          odom_anchor=pose.R_WM;
        }
        previous_odom_time=pose.t;
      }
      const auto state=window.evaluate();
      const auto &stats=state.stats;
      const double unavailable=std::numeric_limits<double>::infinity();
      ui_.panel(name,std::min(1.0,state.stable_s/duration),{
        handheld ? "保持手持，回到舒适的中间姿态，短暂停稳。" : "将飞机放在稳定平面上，松手并保持静止。",
        "检测到运动后完整重新计时。"+state.reason},
        {{"连续静止",state.stable_s,duration,"s",1},
         {"有效样本",static_cast<double>(stats.samples),static_cast<double>(min_count),"帧",0},
         {"姿态变化",stats.samples ? state.rotation_rad*180/M_PI : unavailable,static_rotation_deg_,"deg（上限）",2,false},
         {"角速度 RMS",stats.samples ? stats.gyro_rms : unavailable,limits.max_gyro,"rad/s（上限）",3,false},
         {"角速度波动",stats.samples ? stats.gyro_std : unavailable,limits.max_gyro_std,"rad/s（上限）",3,false},
         {"加速度波动",stats.samples ? stats.accel_std : unavailable,limits.max_accel_std,"m/s²（上限）",3,false}});
      if (state.ready) { begin=state.begin;end=state.end;ui_.clear();return true; }
      if (ros::WallTime::now().toSec()-wall_start>static_timeout_) {
        error_="静止采集超时，检查振动、姿态漂移、话题频率、时间戳和加速度单位"; return false;
      }
    }
    return false;
  }

  bool collectAxis(int axis,const std::string &name,const std::string &help) {
    const double start=imu_.back().t, wall_start=ros::WallTime::now().toSec();
    size_t ii=imu_.size(), oi=odom_.size(), imu_count=0, odom_count=0;
    double pos=0,neg=0,active=0,odom_angle=0;
    while (healthy()) {
      spinFor(update_s_);
      if (!healthy()) return false;
      for (;ii<imu_.size();++ii) {
        const double dt=imu_[ii].t-imu_[ii-1].t;
        if (imu_[ii-1].t<start || dt<=0 || dt>limits_.max_imu_gap) continue;
        ++imu_count;
        const Vec3 w=imu_[ii].w-gyro_bias_;
        if (std::abs(w[axis])<motion_rate_ || std::abs(w[axis])<0.6*w.norm()) continue;
        active+=dt;
        if (w[axis]>0) pos+=w[axis]*dt; else neg-=w[axis]*dt;
      }
      for (;oi<odom_.size();++oi) {
        if (oi==0 || odom_[oi-1].t<start) continue;
        const double dt=odom_[oi].t-odom_[oi-1].t;
        if (dt<=0 || dt>limits_.max_odom_gap) continue;
        ++odom_count;
        const double angle=mc::logSO3(odom_[oi-1].R_WM.transpose()*odom_[oi].R_WM).norm();
        if (angle/dt>motion_rate_*0.5) odom_angle+=angle;
      }
      const double elapsed=std::min(imu_.back().t,odom_.back().t)-start;
      const double progress=std::min({elapsed/axis_min_s_,active/active_s_,pos/direction_rad_,neg/direction_rad_,
        odom_angle/(2*direction_rad_*0.6),static_cast<double>(imu_count)/axis_min_imu_,
        static_cast<double>(odom_count)/axis_min_odom_});
      ui_.panel("0"+std::to_string(axis+4)+" / 08  "+name,progress,{
        help,
        "每次往复约 3–5 秒，自然加减速；继续手持，不要猛甩或连续转圈。"},
        {{"采集时长",elapsed,axis_min_s_,"s",1},
         {"有效运动",active,active_s_,"s",1},
         {"正向转角",pos*180/M_PI,direction_rad_*180/M_PI,"deg",0},
         {"反向转角",neg*180/M_PI,direction_rad_*180/M_PI,"deg",0},
         {"IMU 样本",static_cast<double>(imu_count),static_cast<double>(axis_min_imu_),"帧",0},
         {"ODOM 样本",static_cast<double>(odom_count),static_cast<double>(axis_min_odom_),"帧",0},
         {"里程计转动",odom_angle*180/M_PI,2*direction_rad_*0.6*180/M_PI,"deg",0}});
      if (progress>=1) {
        windows_.push_back({start,std::min(imu_.back().t,odom_.back().t)});
        ui_.message("✓ "+name+" 完成 · "+number(elapsed,1)+" 秒 · 正 / 反 "+number(pos*180/M_PI,0)+"° / "+number(neg*180/M_PI,0)+"°"); return true;
      }
      if (ros::WallTime::now().toSec()-wall_start>motion_timeout_) {
        error_=name+" 采集超时，请检查运动方向、话题频率、安装和里程计跟踪"; return false;
      }
    }
    return false;
  }
  void report(const mc::Result &r) {
    if(r.samples<limits_.min_motion_samples) {
      ui_.summary("✗ 有效运动配对不足",{"未进行完整求解，请检查数据源、时间同步和采样频率。"},
        {{"运动配对 ↑",static_cast<double>(r.samples),static_cast<double>(limits_.min_motion_samples),"帧",0}},false);
      for(const auto &reason:r.reasons)ui_.message("  · "+reason,true);
      return;
    }
    ui_.summary(r.accepted ? "✓ 质量验收通过 · 正在保存" : "✗ 未通过质量验收",{
      r.accepted ? "所有质量门限已通过。" : "请根据失败项检查数据并重新采集。",
      "每项显示：实测值 / 验收门限；箭头说明门限方向。"},
      {{"运动配对 ↑",static_cast<double>(r.samples),static_cast<double>(limits_.min_motion_samples),"帧",0},
       {"角速度误差 ↓",r.gyro_rmse,limits_.max_gyro_rmse,"rad/s",4,false},
       {"相对误差 ↓",r.gyro_relative_rmse*100,limits_.max_gyro_relative_rmse*100,"%",1,false},
       {"运动一致性 ↑",r.gyro_correlation,limits_.min_gyro_correlation,"",4},
       {"加速度误差 ↓",r.accel_rmse,limits_.max_accel_rmse,"m/s²",4,false},
       {"三轴激励比 ↑",r.rotation_ratio,limits_.min_rotation_ratio,"",4},
       {"杆臂条件数 ↓",r.translation_condition,limits_.max_translation_condition,"",2,false},
       {"最弱方向激励 ↑",r.translation_excitation,limits_.min_translation_excitation,"rad/s²",4},
       {"角加速度 RMS ↑",r.alpha_rms,limits_.min_alpha_rms,"rad/s²",4},
       {"杆臂长度 ↓",r.t_B_M.norm(),limits_.max_lever_arm,"m",3,false}},r.accepted);
    for(const auto &reason:r.reasons)ui_.message("  · "+reason,true);
    ui_.message("数据统计 · 丢弃无效/乱序帧：IMU "+std::to_string(dropped_imu_)+"，ODOM "+std::to_string(dropped_odom_));
  }

  static std::string resolvedPath(const std::string &path) {
    if (path.empty()) throw std::runtime_error("输出路径为空");
    if (path.substr(0,2)=="~/") {
      const char *home=std::getenv("HOME");
      if (!home) throw std::runtime_error("HOME 未设置，请指定绝对输出路径");
      return std::string(home)+path.substr(1);
    }
    if (path[0]=='/') return path;
    const auto pkg=ros::package::getPath("mid360_px4_calibration");
    if (pkg.empty()) throw std::runtime_error("找不到 ROS 包，请 source 工作空间或使用绝对输出路径");
    return pkg+"/"+path;
  }
  static void makeParent(const std::string &path) {
    for (size_t slash=1;(slash=path.find('/',slash))!=std::string::npos;++slash) {
      const auto dir=path.substr(0,slash);
      if (mkdir(dir.c_str(),0755)!=0 && errno!=EEXIST) throw std::runtime_error("无法创建输出目录: "+dir);
      struct stat st;
      if (stat(dir.c_str(),&st)!=0 || !S_ISDIR(st.st_mode)) throw std::runtime_error("输出目录无效: "+dir);
    }
  }
  bool writeResults(const mc::Result &r) {
    const std::string yaml=resolvedPath(output_yaml_), txt=resolvedPath(output_txt_);
    const std::string suffix=".tmp."+std::to_string(getpid());
    const std::string yt=yaml+suffix, tt=txt+suffix;
    if (yaml==txt) { error_="YAML 和 TXT 输出路径不能相同"; return false; }
    try {
      makeParent(yaml); makeParent(txt);
      std::ofstream fy(yt), ft(tt);
      if (!fy || !ft) throw std::runtime_error("无法打开输出文件，请检查目录权限");
      const auto q=Eigen::Quaterniond(r.R_B_M);
      const auto &ref=imu_.back();
      fy<<std::fixed<<std::setprecision(9)
        <<"schema_version: 1\nsolver_model: matched_integral_v1\nfastlio_body_to_px4_imu:\n"
        <<"  odom_world_frame: "<<std::quoted(world_frame_)<<"\n"
        <<"  odom_child_frame: "<<std::quoted(mid_frame_)<<"\n"
        <<"  px4_imu_frame: "<<std::quoted(body_frame_)<<"\n"
        <<"  translation_fastlio_body_relative_to_px4_imu_FLU_m: "<<vectorText(r.t_B_M)<<"\n"
        <<"  translation_body_FLU_m: "<<vectorText(r.t_B_M)<<"\n"
        <<"  rotation_quaternion_xyzw: ["<<q.x()<<", "<<q.y()<<", "<<q.z()<<", "<<q.w()<<"]\n"
        <<"  rotation_matrix_B_M:\n";
      for (int i=0;i<3;++i) fy<<"    - "<<vectorText(r.R_B_M.row(i).transpose())<<"\n";
      fy<<"  translation_mid360_relative_to_px4_imu_ENU_m: "<<vectorText(ref.R_EB*r.t_B_M)<<"\n"
        <<"  enu_reference_stamp: "<<ref.t<<"\n"
        <<"bias:\n  gyro_B_rad_s: "<<vectorText(gyro_bias_)<<"\n  accel_B_m_s2: "<<vectorText(r.accel_bias)<<"\n"
        <<"timing:\n  imu_time_offset_s: "<<imu_offset_<<"\n"
        <<"  convention: \"aligned_imu_stamp = raw_imu_stamp + imu_time_offset_s\"\n"
        <<"  automatic_time_offset_estimation: false\n"
        <<"quality:\n  accepted: true\n  gyro_rmse_rad_s: "<<r.gyro_rmse
        <<"\n  gyro_relative_rmse: "<<r.gyro_relative_rmse
        <<"\n  gyro_correlation: "<<r.gyro_correlation
        <<"\n  imu_gyro_rms_rad_s: "<<r.imu_gyro_rms
        <<"\n  odom_gyro_rms_rad_s: "<<r.odom_gyro_rms
        <<"\n  accel_rmse_m_s2: "<<r.accel_rmse
        <<"\n  rotation_excitation_ratio: "<<r.rotation_ratio
        <<"\n  translation_condition_number: "<<r.translation_condition
        <<"\n  translation_excitation_rad_s2: "<<r.translation_excitation
        <<"\n  odom_angular_accel_rms_rad_s2: "<<r.alpha_rms
        <<"\n  odom_motion_samples: "<<r.samples<<"\n"
        <<"limits:\n  max_gyro_rmse_rad_s: "<<limits_.max_gyro_rmse
        <<"\n  max_gyro_relative_rmse: "<<limits_.max_gyro_relative_rmse
        <<"\n  min_gyro_correlation: "<<limits_.min_gyro_correlation
        <<"\n  max_accel_rmse_m_s2: "<<limits_.max_accel_rmse
        <<"\n  min_rotation_excitation_ratio: "<<limits_.min_rotation_ratio
        <<"\n  max_translation_condition: "<<limits_.max_translation_condition
        <<"\n  min_translation_excitation_rad_s2: "<<limits_.min_translation_excitation
        <<"\n  min_angular_accel_rms_rad_s2: "<<limits_.min_alpha_rms
        <<"\n  max_lever_arm_m: "<<limits_.max_lever_arm<<"\n"
        <<"collection:\n  pickup_min_duration_s: "<<pickup_min_s_
        <<"\n  pickup_min_displacement_m: "<<pickup_distance_
        <<"\n  putdown_buffer_s: "<<putdown_s_
        <<"\n  static_max_rotation_deg: "<<static_rotation_deg_
        <<"\n  invalid_or_out_of_order_imu: "<<dropped_imu_
        <<"\n  invalid_or_out_of_order_odom: "<<dropped_odom_<<"\n  axes:\n";
      for (const auto &window:windows_) fy<<"    - {begin: "<<window.begin<<", end: "<<window.end
        <<", duration_s: "<<window.end-window.begin<<"}\n";
      fy.flush();
      if (!fy) throw std::runtime_error("YAML 写入失败（可能磁盘空间不足）");
      fy.close();
      if (!fy) throw std::runtime_error("YAML 关闭失败");
      std::ifstream saved(yt);
      ft<<"Mid360 / PX4 calibration: accepted\nTransform: p_B = R_B_M * p_M + t_B_M\n\n"<<saved.rdbuf();
      ft.flush();
      if (!ft) throw std::runtime_error("TXT 写入失败（可能磁盘空间不足）");
      ft.close();
      if (!ft) throw std::runtime_error("TXT 关闭失败");
      if (std::rename(yt.c_str(),yaml.c_str())!=0) throw std::runtime_error("无法保存 YAML 结果");
      if (std::rename(tt.c_str(),txt.c_str())!=0) throw std::runtime_error("YAML 已保存，但 TXT 保存失败");
      return true;
    } catch (const std::exception &e) {
      std::remove(yt.c_str()); std::remove(tt.c_str()); error_=e.what(); return false;
    }
  }
  int fail() { ui_.message("✗ 标定停止 · "+(error_.empty() ? "采集被中断" : error_),true); return 2; }
  ros::NodeHandle nh_,pnh_;
  ros::Subscriber sub_imu_,sub_odom_;
  Terminal ui_;
  mc::Limits limits_;
  std::vector<mc::Imu> imu_;
  std::vector<mc::Pose> odom_;
  std::vector<mc::Interval> windows_;
  Vec3 gyro_bias_=Vec3::Zero(),accel_bias_=Vec3::Zero();
  std::string imu_topic_,odom_topic_,output_yaml_,output_txt_,body_frame_,world_frame_,mid_frame_,error_;
  double accel_scale_,gyro_scale_,imu_offset_,static_s_,validation_s_,neutral_s_,static_timeout_,static_rotation_deg_;
  double axis_min_s_,active_s_,direction_rad_,motion_rate_,motion_timeout_,update_s_;
  double pickup_min_s_,pickup_distance_,pickup_timeout_,putdown_s_;
  double data_timeout_,startup_timeout_,recording_timeout_,started_=0,last_imu_wall_=0,last_odom_wall_=0;
  int min_static_,axis_min_imu_,axis_min_odom_,min_samples_,min_odom_;
  size_t dropped_imu_=0,dropped_odom_=0;
};
}
int main(int argc,char **argv) {
  std::setlocale(LC_CTYPE,"");
  ros::init(argc,argv,"mid360_px4_online_calibration");
  try {
    ros::NodeHandle nh,pnh("~");
    Calibrator calibrator(nh,pnh); return calibrator.run();
  } catch (const std::exception &e) {
    ROS_ERROR("Calibration failed: %s",e.what()); return 2;
  }
}
