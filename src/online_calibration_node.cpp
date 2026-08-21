#include <ros/ros.h>
#include <ros/package.h>
#include <ros/serialization.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <topic_tools/shape_shifter.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <algorithm>
#include <array>
#include <atomic>
#include <cmath>
#include <cstring>
#include <cstdint>
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

struct ImuSample { double t; Vec3 gyro; Vec3 accel; };
struct Scan { double t; pcl::PointCloud<pcl::PointXYZ>::Ptr cloud; std::string frame; };

struct Result {
  Mat3 R_B_M = Mat3::Identity();
  Vec3 t_B_M = Vec3::Zero();
  double dt = 0.0;
  double gyro_rmse = std::numeric_limits<double>::infinity();
  double accel_rmse = std::numeric_limits<double>::infinity();
  double icp_rmse = std::numeric_limits<double>::infinity();
  bool accepted = false;
  std::string note;
};

static constexpr int PB_WIDTH = 32;
static const char *CYAN = "\033[96m";
static const char *YELLOW = "\033[93m";
static const char *GREEN = "\033[92m";
static const char *MAGENTA = "\033[95m";
static const char *RESET = "\033[0m";

double clamp01(double v) { return std::max(0.0, std::min(1.0, v)); }

void progress(const std::string &label, double value, const char *color, bool newline = true) {
  const int n = static_cast<int>(std::round(clamp01(value) * PB_WIDTH));
  std::cout << color << label << " " << std::setw(3)
            << static_cast<int>(std::round(clamp01(value) * 100.0)) << "% ["
            << std::string(n, '=') << std::string(PB_WIDTH - n, ' ') << "]" << RESET;
  if (newline) std::cout << "\n";
  else std::cout << "\r";
  std::cout.flush();
}

void stage(const std::string &name, double value) {
  progress("[Calibration] " + name, value, value >= 1.0 ? GREEN : MAGENTA);
}

double stamp(const sensor_msgs::ImuConstPtr &msg) { return msg->header.stamp.toSec(); }

class Calibrator {
 public:
  Calibrator(ros::NodeHandle &nh, ros::NodeHandle &pnh) : nh_(nh), pnh_(pnh) {
    pnh_.param("mid_imu_topic", mid_topic_, std::string("/livox/imu"));
    pnh_.param("body_imu_topic", body_topic_, std::string("/mavros/imu/data_raw"));
    pnh_.param("lidar_topic", lidar_topic_, std::string("/livox/lidar"));
    pnh_.param("output_yaml", output_yaml_, std::string("config/online_calibration_result.yaml"));
    pnh_.param("output_txt", output_txt_, std::string("config/online_calibration_result.txt"));
    pnh_.param("mid_accel_scale", mid_accel_scale_, 9.80665);
    pnh_.param("body_accel_scale", body_accel_scale_, 1.0);
    pnh_.param("mid_gyro_scale", mid_gyro_scale_, 1.0);
    pnh_.param("body_gyro_scale", body_gyro_scale_, 1.0);
    pnh_.param("static_duration_s", static_s_, 5.0);
    pnh_.param("motion_max_duration_s", motion_max_s_, 22.0);
    pnh_.param("motion_update_period_s", update_s_, 0.25);
    pnh_.param("axis_excitation_target_rad", target_rad_, 1.2);
    pnh_.param("min_samples", min_samples_, 500);
    pnh_.param("min_lidar_scans", min_scans_, 15);
    pnh_.param("lidar_voxel_size_m", voxel_, 0.20);
    pnh_.param("lidar_max_points_per_scan", max_points_, 1200);
    pnh_.param("lidar_max_correspondence_m", max_corr_, 2.0);
    pnh_.param("lidar_icp_iterations", icp_iterations_, 12);
    pnh_.param("max_lidar_icp_error_m", max_icp_error_, 0.25);
    pnh_.param("time_search_min_s", time_min_, -0.5);
    pnh_.param("time_search_max_s", time_max_, 0.5);
    pnh_.param("time_search_step_s", time_step_, 0.005);
    pnh_.param("use_ansi_progress", ansi_, true);
    mid_sub_ = nh_.subscribe(mid_topic_, 10000, &Calibrator::midCallback, this);
    body_sub_ = nh_.subscribe(body_topic_, 10000, &Calibrator::bodyCallback, this);
    cloud_sub_ = nh_.subscribe(lidar_topic_, 50, &Calibrator::cloudCallback, this);
  }

  int run() {
    ROS_INFO("LI-Init style calibration started (C++ node, subscribes only).");
    ROS_INFO("Mid360 IMU: %s | PX4 IMU: %s | LiDAR: %s", mid_topic_.c_str(), body_topic_.c_str(), lidar_topic_.c_str());
    collectStatic();
    if (!ros::ok()) return 2;
    collectMotion();
    collectStaticValidation();
    if (!checkData()) return 2;
    Result result;
    if (!solve(result)) return 3;
    writeResults(result);
    printResult(result);
    return result.accepted ? 0 : 3;
  }

 private:
  void midCallback(const sensor_msgs::ImuConstPtr &msg) {
    const double t = stamp(msg);
    if (!std::isfinite(t)) return;
    ImuSample s{t, mid_gyro_scale_ * Vec3(msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z),
                mid_accel_scale_ * Vec3(msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z)};
    if (!s.gyro.allFinite() || !s.accel.allFinite()) return;
    std::lock_guard<std::mutex> lock(mutex_); mid_.push_back(s);
  }

  void bodyCallback(const sensor_msgs::ImuConstPtr &msg) {
    const double t = stamp(msg);
    if (!std::isfinite(t)) return;
    ImuSample s{t, body_gyro_scale_ * Vec3(msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z),
                body_accel_scale_ * Vec3(msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z)};
    if (!s.gyro.allFinite() || !s.accel.allFinite()) return;
    std::lock_guard<std::mutex> lock(mutex_); body_.push_back(s);
  }

  static uint32_t u32(const uint8_t *p) { return uint32_t(p[0]) | (uint32_t(p[1]) << 8) | (uint32_t(p[2]) << 16) | (uint32_t(p[3]) << 24); }
  static uint64_t u64(const uint8_t *p) { uint64_t v = 0; for (int i = 7; i >= 0; --i) v = (v << 8) | p[i]; return v; }
  static float f32(const uint8_t *p) { uint32_t u = u32(p); float v; std::memcpy(&v, &u, sizeof(v)); return v; }

  void cloudCallback(const topic_tools::ShapeShifter::ConstPtr &msg) {
    Scan scan; scan.cloud.reset(new pcl::PointCloud<pcl::PointXYZ>());
    if (msg->getDataType() == "sensor_msgs/PointCloud2") {
      sensor_msgs::PointCloud2ConstPtr cloud = msg->instantiate<sensor_msgs::PointCloud2>();
      if (!cloud) return;
      scan.t = cloud->header.stamp.toSec(); scan.frame = cloud->header.frame_id;
      const auto fx = std::find_if(cloud->fields.begin(), cloud->fields.end(), [](const sensor_msgs::PointField &f){return f.name == "x";});
      const auto fy = std::find_if(cloud->fields.begin(), cloud->fields.end(), [](const sensor_msgs::PointField &f){return f.name == "y";});
      const auto fz = std::find_if(cloud->fields.begin(), cloud->fields.end(), [](const sensor_msgs::PointField &f){return f.name == "z";});
      if (fx == cloud->fields.end() || fy == cloud->fields.end() || fz == cloud->fields.end()) return;
      for (uint32_t i = 0; i < cloud->width * cloud->height; ++i) {
        const uint8_t *p = cloud->data.data() + i * cloud->point_step;
        scan.cloud->push_back(pcl::PointXYZ(f32(p + fx->offset), f32(p + fy->offset), f32(p + fz->offset)));
      }
    } else if (msg->getDataType() == "livox_ros_driver2/CustomMsg" || msg->getDataType() == "livox_ros_driver/CustomMsg") {
      std::vector<uint8_t> raw(msg->size()); ros::serialization::OStream stream(raw.data(), static_cast<uint32_t>(raw.size())); msg->write(stream);
      const uint8_t *p = raw.data(); const size_t n = raw.size();
      if (!p || n < 32) return;
      size_t o = 0; if (n < 4 + 8) return; o += 4; const uint32_t sec = u32(p + o); o += 4; const uint32_t nsec = u32(p + o); o += 4;
      if (o + 4 > n) return; const uint32_t flen = u32(p + o); o += 4; if (o + flen > n) return; scan.frame.assign(reinterpret_cast<const char *>(p + o), flen); o += flen;
      if (o + 8 + 4 + 4 + 4 > n) return; const uint64_t timebase = u64(p + o); o += 8; const uint32_t point_num = u32(p + o); o += 4; o += 4; const uint32_t count = std::min(point_num, u32(p + o)); o += 4;
      for (uint32_t i = 0; i < count && o + 19 <= n; ++i) { o += 4; const float x = f32(p + o); const float y = f32(p + o + 4); const float z = f32(p + o + 8); o += 12; o += 3; if (std::isfinite(x) && std::isfinite(y) && std::isfinite(z)) scan.cloud->push_back(pcl::PointXYZ(x, y, z)); }
      scan.t = double(sec) + 1e-9 * double(nsec); if (scan.t <= 0.0) scan.t = 1e-9 * double(timebase);
    } else return;
    if (!std::isfinite(scan.t) || scan.cloud->size() < 80) return;
    if (scan.cloud->size() > static_cast<size_t>(max_points_)) { pcl::PointCloud<pcl::PointXYZ>::Ptr reduced(new pcl::PointCloud<pcl::PointXYZ>()); const size_t step = scan.cloud->size() / max_points_ + 1; for (size_t i = 0; i < scan.cloud->size(); i += step) reduced->push_back((*scan.cloud)[i]); scan.cloud = reduced; }
    std::lock_guard<std::mutex> lock(mutex_); scans_.push_back(scan); if (scans_.size() > 400) scans_.erase(scans_.begin(), scans_.begin() + (scans_.size() - 400));
  }

  void spinFor(double seconds) {
    const ros::WallTime end = ros::WallTime::now() + ros::WallDuration(seconds);
    ros::Rate rate(100);
    while (ros::ok() && ros::WallTime::now() < end) { ros::spinOnce(); rate.sleep(); }
  }

  std::array<double,3> excitation() {
    std::lock_guard<std::mutex> lock(mutex_); std::array<double,3> out{0,0,0}; if (mid_.size() < 5) return out;
    const double static_end = mid_.front().t + static_s_; size_t start = 0; while (start + 1 < mid_.size() && mid_[start].t < static_end) ++start; start = std::min(start, mid_.size() / 2); Vec3 bias = Vec3::Zero(); for (size_t i = 0; i < start; ++i) bias += mid_[i].gyro; if (start) bias /= double(start);
    for (size_t i = start + 1; i < mid_.size(); ++i) { const double dt = std::max(0.0, mid_[i].t - mid_[i-1].t); const Vec3 w = (mid_[i].gyro - bias).cwiseAbs(); for (int k=0;k<3;++k) out[k] += std::max(0.0, w[k] - 0.015) * dt / target_rad_; }
    return out;
  }

  size_t currentScanCount() { std::lock_guard<std::mutex> lock(mutex_); return scans_.size(); }

  void excitationProgress(const std::array<double,3> &e) {
    static bool rendered = false;
    if (rendered) std::cout << "\033[4A";
    progress("[Initialization] Rotation around X Axis", e[0], e[0] >= 1.0 ? GREEN : YELLOW, true);
    progress("[Initialization] Rotation around Y Axis", e[1], e[1] >= 1.0 ? GREEN : YELLOW, true);
    progress("[Initialization] Rotation around Z Axis", e[2], e[2] >= 1.0 ? GREEN : YELLOW, true);
    std::cout << CYAN << "[Initialization] IMU samples=" << currentMidCount()
              << " | LiDAR scans=" << currentScanCount() << RESET << "\n";
    std::cout.flush(); rendered = true;
  }

  void collectStatic() { ROS_INFO("[Initialization] Keep the rigid assembly still."); const ros::WallTime begin = ros::WallTime::now(); while (ros::ok()) { const double f = (ros::WallTime::now()-begin).toSec()/static_s_; progress("[Initialization] Static initialization", f, f>=1?GREEN:YELLOW, true); if (f>=1) break; spinFor(update_s_); } }
  void collectMotion() {
    ROS_INFO("[Initialization] Move only after the static phase; follow the least-complete axis."); const ros::WallTime begin = ros::WallTime::now(); bool enough = false;
    while (ros::ok() && (ros::WallTime::now()-begin).toSec() < motion_max_s_) { const auto e = excitation(); const bool ready = e[0]>=1 && e[1]>=1 && e[2]>=1 && currentMidCount()>=static_cast<size_t>(min_samples_); excitationProgress(e); const char *axis = (e[0] < e[1] && e[0] < e[2]) ? "ROLL/X" : (e[1] < e[2] ? "PITCH/Y" : "YAW/Z"); ROS_INFO_THROTTLE(2.0, "[Initialization] Excitation X=%.0f%% Y=%.0f%% Z=%.0f%%; slowly excite %s", 100*clamp01(e[0]),100*clamp01(e[1]),100*clamp01(e[2]),axis); if (ready) { enough = true; break; } spinFor(update_s_); }
    if (!enough) ROS_WARN("[Initialization] Motion timeout; proceeding with collected data."); else ROS_INFO("[Initialization] Data accumulation finished, calibration begins.");
  }
  void collectStaticValidation() { const ros::WallTime begin = ros::WallTime::now(); while (ros::ok() && (ros::WallTime::now()-begin).toSec() < 1.5) { const double f=(ros::WallTime::now()-begin).toSec()/1.5; progress("[Initialization] Static validation",f,f>=1?GREEN:YELLOW,true); spinFor(update_s_); } }
  size_t currentMidCount() { std::lock_guard<std::mutex> lock(mutex_); return mid_.size(); }

  bool checkData() { std::lock_guard<std::mutex> lock(mutex_); if (mid_.size()<static_cast<size_t>(min_samples_) || body_.size()<static_cast<size_t>(min_samples_)) { ROS_ERROR("Too few IMU samples: Mid360=%zu PX4=%zu (need %d)",mid_.size(),body_.size(),min_samples_); return false; } if (scans_.size()<static_cast<size_t>(min_scans_)) { ROS_ERROR("Too few valid LiDAR scans: %zu (need %d)",scans_.size(),min_scans_); return false; } return true; }

  static Vec3 interp(const std::vector<ImuSample> &v, double t, bool gyro) { if (v.empty()) return Vec3::Zero(); auto it=std::lower_bound(v.begin(),v.end(),t,[](const ImuSample&s,double x){return s.t<x;}); if(it==v.begin()) return gyro?it->gyro:it->accel; if(it==v.end()) return gyro?v.back().gyro:v.back().accel; const ImuSample&a=*(it-1),&b=*it; const double u=(t-a.t)/std::max(1e-9,b.t-a.t); return gyro?((1.0-u)*a.gyro+u*b.gyro):((1.0-u)*a.accel+u*b.accel); }

  bool solve(Result &r) {
    std::vector<ImuSample> mid,body; std::vector<Scan> scans; { std::lock_guard<std::mutex> lock(mutex_); mid=mid_; body=body_; scans=scans_; }
    // Remove the stationary gyro/acceleration means before cross-sensor fitting.
    // This is the same role as LI-Init's initial gravity/bias accumulation and
    // prevents static gravity from being mistaken for a translation signal.
    auto static_mean = [this](const std::vector<ImuSample> &v, bool gyro) {
      Vec3 mean = Vec3::Zero(); size_t n = 0; if (v.empty()) return mean; const double end = v.front().t + static_s_;
      for (const auto &s : v) { if (s.t > end) break; mean += gyro ? s.gyro : s.accel; ++n; }
      return n ? mean / double(n) : mean;
    };
    const Vec3 mid_g0 = static_mean(mid, true), body_g0 = static_mean(body, true);
    const Vec3 mid_a0 = static_mean(mid, false), body_a0 = static_mean(body, false);
    for (auto &s : mid) { s.gyro -= mid_g0; s.accel -= mid_a0; }
    for (auto &s : body) { s.gyro -= body_g0; s.accel -= body_a0; }
    stage("LiDAR motion estimation",0.05); std::sort(scans.begin(),scans.end(),[](const Scan&a,const Scan&b){return a.t<b.t;}); std::vector<double> ts; std::vector<Mat3> rots; std::vector<Vec3> omg; std::vector<Vec3> acc; Mat3 R=Mat3::Identity(); Vec3 pos=Vec3::Zero(); double icp_sum=0; int icp_n=0; pcl::PointCloud<pcl::PointXYZ>::Ptr prev=scans.front().cloud;
    ts.push_back(scans.front().t); rots.push_back(R); omg.push_back(Vec3::Zero()); acc.push_back(Vec3::Zero()); std::vector<Vec3> positions(1,pos);
    for(size_t i=1;i<scans.size();++i){ pcl::IterativeClosestPoint<pcl::PointXYZ,pcl::PointXYZ> icp; icp.setInputSource(scans[i].cloud); icp.setInputTarget(prev); icp.setMaximumIterations(icp_iterations_); icp.setMaxCorrespondenceDistance(max_corr_); pcl::PointCloud<pcl::PointXYZ> aligned; icp.align(aligned); if(!icp.hasConverged()) continue; const double score=icp.getFitnessScore(); if(!std::isfinite(score)||score>max_icp_error_*max_icp_error_) continue; Eigen::Matrix4f T=icp.getFinalTransformation(); Mat3 dR=T.block<3,3>(0,0).cast<double>(); Vec3 dt=T.block<3,1>(0,3).cast<double>(); const Mat3 R_prev=R; const Vec3 p_prev=pos; R=R_prev*dR; // p_prev_frame = dR*p_current_frame + dt; therefore current origin in the previous frame is -dR^T*dt.
      pos=p_prev-R_prev*dR.transpose()*dt; const double dtm=scans[i].t-ts.back(); if(dtm<=1e-4) continue; ts.push_back(scans[i].t); rots.push_back(R); positions.push_back(pos); const Eigen::AngleAxisd aa(dR); omg.push_back(aa.angle()*aa.axis()/dtm); acc.push_back(Vec3::Zero()); icp_sum+=std::sqrt(score); ++icp_n; prev=scans[i].cloud; }
    if(ts.size()<5){ROS_ERROR("LiDAR ICP produced too few valid motions");return false;} for(size_t i=2;i<positions.size();++i){const double dtm=ts[i]-ts[i-1]; if(dtm>1e-4) acc[i]=(positions[i]-2*positions[i-1]+positions[i-2])/(dtm*dtm);} r.icp_rmse=icp_n?icp_sum/icp_n:1e9; stage("LiDAR motion estimation",1.0);
    stage("Temporal alignment",0.1); double best=time_min_, best_cost=1e99; for(double d=time_min_;d<=time_max_+0.5*time_step_;d+=time_step_){double c=0;int n=0; for(size_t i=1;i<ts.size();++i){Vec3 a=interp(body,ts[i]+d,true); Vec3 b=interp(mid,ts[i],true); c+=(a.norm()-b.norm())*(a.norm()-b.norm());++n;} if(n&&c/n<best_cost){best_cost=c/n;best=d;}} r.dt=best; stage("Temporal alignment",1.0);
    stage("Rotation and gyro bias",0.1); Eigen::Matrix3d H=Mat3::Zero(); double gres=0; int gn=0; for(const auto&t:ts){Vec3 wm=interp(mid,t,true);Vec3 wb=interp(body,t+r.dt,true); if(wm.norm()<0.03||wb.norm()<0.03)continue; H+=wb*wm.transpose();++gn;} if(gn<20){ROS_ERROR("Insufficient rotational excitation");return false;} Eigen::JacobiSVD<Mat3> svd(H,Eigen::ComputeFullU|Eigen::ComputeFullV); r.R_B_M=svd.matrixU()*svd.matrixV().transpose(); if(r.R_B_M.determinant()<0){Mat3 U=svd.matrixU();U.col(2)*=-1;r.R_B_M=U*svd.matrixV().transpose();} for(const auto&t:ts){Vec3 e=interp(body,t+r.dt,true)-r.R_B_M*interp(mid,t,true);gres+=e.squaredNorm();} r.gyro_rmse=std::sqrt(gres/ts.size()); stage("Rotation and gyro bias",1.0);
    stage("Translation / gravity refinement",0.1); Eigen::MatrixXd A(3*ts.size(),3); Eigen::VectorXd y(3*ts.size()); int row=0; double ares=0; for(const auto&t:ts){Vec3 wm=interp(mid,t,true), am=interp(mid,t,false), ab=interp(body,t+r.dt,false); Vec3 target=r.R_B_M*am-ab; Mat3 J=wm.squaredNorm()*Mat3::Identity()-wm*wm.transpose(); A.block<3,3>(row,0)=J; y.segment<3>(row)=target; row+=3;} if(row>=9){r.t_B_M=A.topRows(row).colPivHouseholderQr().solve(y.head(row));} for(const auto&t:ts){Vec3 wm=interp(mid,t,true), am=interp(mid,t,false), ab=interp(body,t+r.dt,false); Vec3 e=r.R_B_M*(am+(wm.cross(wm.cross(r.t_B_M))))-ab; ares+=e.squaredNorm();} r.accel_rmse=std::sqrt(ares/ts.size()); stage("Translation / gravity refinement",1.0);
    stage("Validation",0.1); r.accepted=std::isfinite(r.gyro_rmse)&&std::isfinite(r.accel_rmse)&&r.gyro_rmse<0.35&&r.accel_rmse<3.0&&r.icp_rmse<max_icp_error_; r.note=r.accepted?"accepted":"residual or ICP quality is poor; collect stronger 3-axis excitation"; stage("Validation",1.0); return true;
  }

  std::string path(const std::string &p) const { if(!p.empty()&&p[0]=='/') return p; return ros::package::getPath("mid360_px4_calibration")+"/"+p; }
  static std::string list3(const Vec3 &v) { std::ostringstream s; s << std::fixed << std::setprecision(9) << "[" << v.x() << ", " << v.y() << ", " << v.z() << "]"; return s.str(); }
  void writeResults(const Result&r){const std::string y=path(output_yaml_), t=path(output_txt_); Eigen::Vector3d e=r.R_B_M.transpose().eulerAngles(0,1,2)*180.0/M_PI; std::ofstream fy(y),ft(t); fy<<"mid360_relative_to_body_enu:\n  translation_B_M_m: "<<list3(r.t_B_M)<<"\n  rpy_B_M_deg: "<<list3(e)<<"\nquality:\n  dt_B_wrt_M_s: "<<r.dt<<"\n  gyro_rmse_rad_s: "<<r.gyro_rmse<<"\n  accel_rmse_m_s2: "<<r.accel_rmse<<"\n  pointcloud_mean_icp_error_m: "<<r.icp_rmse<<"\n  accepted: "<<(r.accepted?"true":"false")<<"\n  note: \""<<r.note<<"\"\n"; ft<<std::fixed<<std::setprecision(9)<<"Mid360 relative to PX4 body (ENU)\ntranslation_B_M_m: "<<r.t_B_M.transpose()<<"\nrpy_B_M_deg: "<<e.transpose()<<"\ndt_B_wrt_M_s: "<<r.dt<<"\ngyro_rmse_rad_s: "<<r.gyro_rmse<<"\naccel_rmse_m_s2: "<<r.accel_rmse<<"\npointcloud_mean_icp_error_m: "<<r.icp_rmse<<"\naccepted: "<<(r.accepted?"true":"false")<<"\nnote: "<<r.note<<"\n"; ROS_INFO("Result YAML: %s",y.c_str()); ROS_INFO("Result TXT: %s",t.c_str()); }
  void printResult(const Result&r){const Eigen::Vector3d e=r.R_B_M.transpose().eulerAngles(0,1,2)*180.0/M_PI; std::cout<<"\n"<<CYAN<<"[Init Result] "<<RESET<<"Translation Mid360 to PX4 body = "<<r.t_B_M.transpose()<<" m\n"<<CYAN<<"[Init Result] "<<RESET<<"RPY (ENU) = "<<e.transpose()<<" deg\n"<<GREEN<<"[Init Result] "<<RESET<<"accepted="<<(r.accepted?"true":"false")<<", gyro RMSE="<<r.gyro_rmse<<", accel RMSE="<<r.accel_rmse<<"\n"; }

  ros::NodeHandle nh_,pnh_; ros::Subscriber mid_sub_,body_sub_,cloud_sub_; std::mutex mutex_; std::vector<ImuSample> mid_,body_; std::vector<Scan> scans_;
  std::string mid_topic_,body_topic_,lidar_topic_,output_yaml_,output_txt_; double mid_accel_scale_,body_accel_scale_,mid_gyro_scale_,body_gyro_scale_,static_s_,motion_max_s_,update_s_,target_rad_,voxel_,max_corr_,max_icp_error_,time_min_,time_max_,time_step_; int min_samples_,min_scans_,max_points_,icp_iterations_; bool ansi_;
};
}

int main(int argc,char **argv){ros::init(argc,argv,"mid360_px4_online_calibration");ros::NodeHandle nh, pnh("~");Calibrator c(nh,pnh);return c.run();}
