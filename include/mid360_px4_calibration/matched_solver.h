#pragma once
#include <mid360_px4_calibration/calibration_math.h>

namespace mid360_calibration {
namespace matched_detail {
struct Observation {
  Motion motion;
  Imu body;
  Mat3 dR,filtered_R_EB,R_WM;
  Vec3 dp,filtered_accel_E;
};
inline Mat3 expSO3(const Vec3 &v) {
  return v.norm()<1e-12 ? Mat3::Identity().eval() : Eigen::AngleAxisd(v.norm(),v.normalized()).toRotationMatrix().eval();
}
inline bool makeObservation(const std::vector<Pose> &pose,size_t i,const std::vector<Imu> &imu,
                            const std::vector<Interval> &windows,const Vec3 &gyro_bias,
                            const Vec3 &accel_bias,const Limits &lim,Observation &o) {
  double begin=0,end=0;bool inside=false;
  for(const auto &w:windows)if(pose[i].t>=w.begin && pose[i].t<=w.end) {
    begin=w.begin;end=w.end;inside=true;break;
  }
  if(!inside)return false;
  size_t l=i,r=i;
  while(l>0 && pose[i].t-pose[l-1].t<=lim.derivative_half_window && pose[l-1].t>=begin)--l;
  while(r+1<pose.size() && pose[r+1].t-pose[i].t<=lim.derivative_half_window && pose[r+1].t<=end)++r;
  if(r-l+1<5 || pose[i].t-pose[l].t<lim.derivative_half_window*0.5 ||
     pose[r].t-pose[i].t<lim.derivative_half_window*0.5)return false;
  for(size_t j=l+1;j<=r;++j)
    if(pose[j].t-pose[j-1].t<=0 || pose[j].t-pose[j-1].t>lim.max_odom_gap)return false;
  if(!interpolate(imu,pose[i].t,lim.max_imu_gap,o.body) ||
     pose[l].t<imu.front().t || pose[r].t>imu.back().t)return false;
  const size_t n=r-l+1;
  Eigen::MatrixXd design(n,4);
  for(size_t j=l;j<=r;++j) {
    const double u=(pose[j].t-pose[i].t)/lim.derivative_half_window;
    design.row(j-l)<<1,u,u*u,u*u*u;
  }
  // d is the cubic least-squares second-derivative stencil. Its zero constant
  // and linear moments yield the exact Green identity:
  // sum_j d_j f(t_j) = integral k(s) f''(s) ds,
  // k(s) = sum_{t_j <= s} d_j (s-t_j).
  // Apply this SAME operator to position, rotation matrix and IMU acceleration.
  const Eigen::HouseholderQR<Eigen::MatrixXd> qr(design);
  Eigen::Matrix4d upper=qr.matrixQR().topLeftCorner(4,4).template triangularView<Eigen::Upper>();
  Eigen::Vector4d rhs=Eigen::Vector4d::Zero();
  rhs[2]=2/(lim.derivative_half_window*lim.derivative_half_window);
  Eigen::VectorXd projected=Eigen::VectorXd::Zero(n);
  projected.head<4>()=upper.transpose().template triangularView<Eigen::Lower>().solve(rhs);
  const Eigen::VectorXd d=qr.householderQ()*projected;
  o.dR=Mat3::Zero();o.dp=Vec3::Zero();o.motion.alpha=Vec3::Zero();
  for(size_t j=l;j<=r;++j) {
    o.dR+=d[j-l]*(pose[j].R_WM-pose[i].R_WM);
    o.dp+=d[j-l]*(pose[j].p-pose[i].p);
    o.motion.alpha+=d[j-l]*logSO3(pose[i].R_WM.transpose()*pose[j].R_WM);
  }
  const double span=pose[r].t-pose[l].t;
  o.motion.t=pose[i].t;
  o.motion.w=logSO3(pose[l].R_WM.transpose()*pose[r].R_WM)/span;
  o.motion.a=pose[i].R_WM.transpose()*o.dp;
  if(!o.dp.allFinite() || !o.motion.alpha.allFinite() || o.dp.norm()>100 ||
     o.motion.w.norm()>10 || o.motion.alpha.norm()>100 ||
     (o.motion.w.norm()<=0.05 && o.motion.alpha.norm()<=0.1))return false;
  // Split at every kernel knot and raw IMU stamp. Substeps keep orientation
  // integration accurate even for an allowed but relatively long IMU interval.
  std::vector<double> cuts;
  for(size_t j=l;j<=r;++j)cuts.push_back(pose[j].t);
  auto it=std::lower_bound(imu.begin(),imu.end(),pose[l].t,
      [](const Imu &s,double t){return s.t<t;});
  for(;it!=imu.end() && it->t<pose[r].t;++it)if(it->t>pose[l].t)cuts.push_back(it->t);
  std::sort(cuts.begin(),cuts.end());cuts.erase(std::unique(cuts.begin(),cuts.end()),cuts.end());
  o.filtered_R_EB=Mat3::Zero();o.filtered_accel_E=Vec3::Zero();
  Mat3 gyro_delta=Mat3::Identity();
  size_t knot=l;double sum_d=0,sum_dt=0,mass=0;
  const Vec3 gravity(0,0,-9.80665);
  for(size_t k=0;k+1<cuts.size();++k) {
    while(knot<=r && pose[knot].t<=cuts[k]) {
      sum_d+=d[knot-l];sum_dt+=d[knot-l]*(pose[knot].t-pose[i].t);++knot;
    }
    const int steps=std::max(1,static_cast<int>(std::ceil((cuts[k+1]-cuts[k])/0.01)));
    for(int step=0;step<steps;++step) {
      const double a=cuts[k]+(cuts[k+1]-cuts[k])*step/steps;
      const double b=cuts[k]+(cuts[k+1]-cuts[k])*(step+1)/steps;
      const double mid=0.5*(a+b),dt=b-a;
      Imu s0,sm,s1;
      if(!interpolate(imu,a,lim.max_imu_gap,s0) || !interpolate(imu,mid,lim.max_imu_gap,sm) ||
         !interpolate(imu,b,lim.max_imu_gap,s1))return false;
      const double ka=(a-pose[i].t)*sum_d-sum_dt;
      const double km=(mid-pose[i].t)*sum_d-sum_dt;
      const double kb=(b-pose[i].t)*sum_d-sum_dt;
      o.filtered_R_EB+=dt/6*(ka*s0.R_EB+4*km*sm.R_EB+kb*s1.R_EB);
      o.filtered_accel_E+=dt/6*(ka*(s0.R_EB*(s0.a-accel_bias)+gravity)+
          4*km*(sm.R_EB*(sm.a-accel_bias)+gravity)+kb*(s1.R_EB*(s1.a-accel_bias)+gravity));
      mass+=dt*(ka+kb)*0.5;
      const Vec3 w0=s0.w-gyro_bias,w1=s1.w-gyro_bias;
      gyro_delta=gyro_delta*expSO3(0.5*(w0+w1)*dt+w0.cross(w1)*(dt*dt/12));
    }
  }
  if(!std::isfinite(mass) || std::abs(mass-1)>1e-5)return false;
  o.body.w=logSO3(gyro_delta)/span;
  o.R_WM=pose[i].R_WM;
  return o.dR.allFinite() && o.filtered_accel_E.allFinite() && o.filtered_R_EB.allFinite();
}
}  // namespace matched_detail

inline Result solveTrajectory(const std::vector<Pose> &pose,const std::vector<Imu> &imu,
                              const std::vector<Interval> &windows,const Vec3 &gyro_bias,
                              const Vec3 &initial_accel_bias,const Limits &lim) {
  Result r;
  if(pose.size()<5 || imu.size()<2) { r.reasons.push_back("位姿或 IMU 样本不足");return r; }
  std::vector<matched_detail::Observation> observations;
  std::vector<Motion> motion;std::vector<Imu> body;
  for(size_t i=0;i<pose.size();++i) {
    matched_detail::Observation o;
    if(!matched_detail::makeObservation(pose,i,imu,windows,gyro_bias,initial_accel_bias,lim,o))continue;
    observations.push_back(o);motion.push_back(o.motion);body.push_back(o.body);
  }
  if(!fitRotation(motion,body,lim,r))return r;
  std::vector<LeverEquation> equations;
  for(const auto &o:observations) {
    // FAST-LIO world need not be ENU. This local alignment is constant over
    // the integration window; bias remains expressed in the PX4 body frame.
    const Mat3 R_W_E=o.R_WM*r.R_B_M.transpose()*o.body.R_EB.transpose();
    equations.push_back({o.dR,R_W_E*o.filtered_R_EB,R_W_E*o.filtered_accel_E-o.dp});
  }
  if(!fitLever(equations,initial_accel_bias,r))return r;
  validateResult(lim,r);return r;
}
}  // namespace mid360_calibration
