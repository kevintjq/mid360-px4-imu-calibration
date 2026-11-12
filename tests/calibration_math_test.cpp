#include <mid360_px4_calibration/calibration_math.h>
#include <mid360_px4_calibration/static_window.h>
#include <iostream>
#include <stdexcept>
#include <random>
namespace mc = mid360_calibration;
using mc::Vec3; using mc::Mat3;
void require(bool condition,const char *message) {
  if (!condition) throw std::runtime_error(message);
}
struct Fixture {
  std::vector<mc::Motion> motion;
  std::vector<mc::Imu> imu;
  Mat3 R = (Eigen::AngleAxisd(1.57079632679,Vec3::UnitZ())*
            Eigen::AngleAxisd(0.2,Vec3::UnitX())).toRotationMatrix();
  Vec3 lever=Vec3(0.12,-0.07,0.05), gb=Vec3(0.01,-0.02,0.005), ab=Vec3(0.04,-0.03,0.08);
  Fixture() {
    for (int i=0;i<1000;++i) {
      const double t=i*0.02;
      mc::Motion m;
      m.t=t;
      m.w=Vec3(0.5*std::sin(0.8*t),0.6*std::sin(1.1*t),0.45*std::sin(1.4*t));
      m.alpha=Vec3(0.4*std::cos(0.8*t),0.66*std::cos(1.1*t),0.63*std::cos(1.4*t));
      m.a=Vec3(0.3*std::sin(t),0.2*std::cos(t),0.1*std::sin(2*t));
      const Mat3 REB=(Eigen::AngleAxisd(0.3*std::sin(t),Vec3::UnitX())*
                      Eigen::AngleAxisd(0.2*std::cos(t),Vec3::UnitY())).toRotationMatrix();
      const Vec3 inverse=-R.transpose()*lever;
      mc::Imu b{t,R*m.w+gb,R*m.a+R*(mc::skew(m.alpha)+mc::skew(m.w)*mc::skew(m.w))*inverse+
                 ab-REB.transpose()*Vec3(0,0,-9.80665),REB};
      motion.push_back(m); imu.push_back(b);
    }
  }
};
void interpolationTest() {
  std::vector<mc::Imu> v{{1,Vec3::Zero(),Vec3::Zero(),Mat3::Identity()},
                         {1.02,Vec3::Ones(),Vec3::Ones(),Mat3::Identity()},
                         {2,Vec3::Ones(),Vec3::Ones(),Mat3::Identity()}};
  mc::Imu b;
  require(!mc::interpolate(v,0.9,0.1,b),"must reject before first sample");
  require(!mc::interpolate(v,2.1,0.1,b),"must reject after last sample");
  require(!mc::interpolate(v,1.5,0.1,b),"must reject interpolation across gap");
  require(mc::interpolate(v,1.01,0.1,b) && (b.w-Vec3::Constant(0.5)).norm()<1e-10,"linear interpolation incorrect");
}
void staticTest() {
  std::vector<mc::Imu> v;
  for (int i=0;i<=300;++i) v.push_back({i*0.02,Vec3(i%2 ? 0.5:-0.5,0,0),Vec3(0,0,9.80665),Mat3::Identity()});
  auto s=mc::staticStats(v,0,6,Vec3::Zero(),0.1);
  require(s.gyro_rms>0.49 && s.gyro_mean.norm()<0.01,"oscillatory motion must not pass static RMS");
  v.erase(v.begin()+30,v.begin()+60);
  s=mc::staticStats(v,0,6,Vec3::Zero(),0.1);
  require(!s.continuous,"static window must detect missing data");
}
void staticWindowTest() {
  mc::StaticLimits limits;
  mc::StaticWindow still(limits,Vec3::Zero());
  auto sample=[](double t,const Vec3 &w,const Mat3 &R=Mat3::Identity()) {
    return mc::Imu{t,w,Vec3(0,0,9.80665),R};
  };
  for (int i=0;i<=1600;++i)
    still.push(sample(i*0.005,Vec3(0.005*std::sin(i),0,0)));
  require(still.evaluate().ready,"stationary noisy 200 Hz data should pass");
  // One moving sample used to be hidden by the eight-second RMS average.
  still.push(sample(8.005,Vec3(0.1,0,0)));
  require(!still.evaluate().ready && still.evaluate().stats.samples==0,"brief movement did not clear window");
  for (int i=1602;i<=3201;++i) still.push(sample(i*0.005,Vec3::Zero()));
  require(!still.evaluate().ready,"static window passed before eight new seconds");
  still.push(sample(16.015,Vec3::Zero()));
  require(still.evaluate().ready,"complete new static window did not pass");
  // A data gap must reset the actual candidate window as well.
  still.push(sample(16.3,Vec3::Zero()));
  require(!still.evaluate().ready,"static gap did not restart window");

  mc::StaticWindow slow_yaw(limits,Vec3::Zero());
  std::vector<mc::Imu> old_window;
  for (int i=0;i<=3200;++i) {
    const double t=i*0.005;
    const auto b=sample(t,Vec3(0,0,0.04),Eigen::AngleAxisd(0.04*t,Vec3::UnitZ()).toRotationMatrix());
    slow_yaw.push(b);
    if(i<=1600) old_window.push_back(b);
  }
  require(mc::staticStats(old_window,0,8,Vec3::Zero(),0.1).gyro_rms<0.08,"slow-yaw fixture must evade old RMS threshold");
  require(!slow_yaw.evaluate().ready && slow_yaw.evaluate().stable_s<1,"slow yaw was mistaken for gyro bias");

  mc::StaticWindow out_and_back(limits,Vec3::Zero());
  for (int i=0;i<=1000;++i) out_and_back.push(sample(i*0.005,Vec3::Zero()));
  out_and_back.push(sample(5.005,Vec3::Zero(),Eigen::AngleAxisd(0.04,Vec3::UnitX()).toRotationMatrix()));
  for (int i=1002;i<=2600;++i) out_and_back.push(sample(i*0.005,Vec3::Zero()));
  require(!out_and_back.evaluate().ready,"returning to the original attitude reused pre-movement samples");
}
void relativeGyroTest() {
  std::vector<mc::Motion> motion;
  std::vector<mc::Imu> imu;
  for(int i=0;i<10000;++i) {
    const double t=i*0.01;
    mc::Motion m;
    m.t=t;m.a=Vec3::Zero();
    m.w=Vec3(0.05*std::sin(1.4*t),0.05*std::sin(1.8*t),0.05*std::sin(2.2*t));
    m.alpha=Vec3(0.07*std::cos(1.4*t),0.09*std::cos(1.8*t),0.11*std::cos(2.2*t));
    motion.push_back(m);
    imu.push_back({t,Vec3(0.085*std::sin(2.4*t),0.085*std::sin(2.8*t),0.085*std::sin(3.2*t)),Vec3(0,0,9.80665),Mat3::Identity()});
  }
  mc::Limits limits;
  const auto mismatch=mc::solve(motion,imu,Vec3::Zero(),Vec3::Zero(),limits);
  require(mismatch.gyro_rmse<limits.max_gyro_rmse,"fixture must evade old absolute gyro threshold");
  require(!mismatch.accepted && mismatch.gyro_relative_rmse>limits.max_gyro_relative_rmse &&
          mismatch.gyro_correlation<limits.min_gyro_correlation,"unrelated low-amplitude gyro streams passed");
  // A coherent small-amplitude signal must still be accepted.
  Fixture f;
  for(size_t i=0;i<f.motion.size();++i) {
    auto &m=f.motion[i];auto &b=f.imu[i];
    m.w*=0.2;m.alpha*=0.2;
    b.w=f.R*m.w+f.gb+Vec3(0.002*std::sin(13*m.t),0,0);
    b.a=f.R*m.a+f.R*(mc::skew(m.alpha)+mc::skew(m.w)*mc::skew(m.w))*(-f.R.transpose()*f.lever)+
        f.ab-b.R_EB.transpose()*Vec3(0,0,-9.80665);
  }
  auto result=mc::solve(f.motion,f.imu,f.gb,Vec3::Zero(),limits);
  require(result.accepted && result.gyro_relative_rmse<0.05,"coherent low-amplitude motion was rejected");
  // Matching waveform but wrong scale: correlation alone must not suffice.
  for(size_t i=0;i<f.motion.size();++i) f.imu[i].w=1.5*f.R*f.motion[i].w+f.gb;
  result=mc::solve(f.motion,f.imu,f.gb,Vec3::Zero(),limits);
  require(!result.accepted && result.gyro_correlation>0.999 &&
          result.gyro_relative_rmse>limits.max_gyro_relative_rmse,"gyro scale mismatch was accepted");
}
void recoveryTest() {
  Fixture f; mc::Limits lim;
  auto r=mc::solve(f.motion,f.imu,f.gb,Vec3::Zero(),lim);
  require(r.accepted,"well-excited known transform rejected");
  require((r.R_B_M-f.R).norm()<1e-9,"90-degree mount rotation not recovered");
  require((r.t_B_M-f.lever).norm()<1e-9,"lever arm/sign not recovered");
  require((r.accel_bias-f.ab).norm()<1e-9,"constant bias not recovered");
  require(r.gyro_rmse<1e-10 && r.accel_rmse<1e-10,"exact data should have zero residual");
  // Large gyro mismatch must fail even with good acceleration data.
  for (size_t i=0;i<f.imu.size();++i) f.imu[i].w.x()+=0.5*std::sin(13*f.imu[i].t);
  r=mc::solve(f.motion,f.imu,f.gb,Vec3::Zero(),lim);
  require(!r.accepted && r.gyro_rmse>lim.max_gyro_rmse,"bad rotation consistency was accepted");
}
void degeneracyTest() {
  Fixture f; mc::Limits lim;
  // Raw C block looks full rank, but constant C is absorbed by the bias.
  const Vec3 w(0.3,0.4,0.5),alpha(0.1,0.2,0.3);
  const Mat3 C=mc::skew(alpha)+mc::skew(w)*mc::skew(w);
  require(Eigen::FullPivLU<Mat3>(C).rank()==3,"fixture must have full-rank raw lever block");
  for (size_t i=0;i<f.motion.size();++i) {
    f.motion[i].w=w; f.motion[i].alpha=alpha;
    f.imu[i].w=f.R*w+f.gb;
  }
  const auto r=mc::solve(f.motion,f.imu,f.gb,Vec3::Zero(),lim);
  require(!r.accepted && r.translation_excitation<1e-10,"bias-confounded lever arm was accepted");
}
void overlapTest() {
  Fixture f; mc::Limits lim;
  for (auto &b:f.imu) b.t+=100;
  const auto r=mc::solve(f.motion,f.imu,f.gb,Vec3::Zero(),lim);
  require(!r.accepted && r.samples==0,"non-overlapping streams must not be paired");
}
void differentiationTest() {
  std::vector<mc::Pose> poses;
  for (int i=0;i<=1000;++i) {
    const double t=i*0.02+0.002*std::sin(i*0.3);
    poses.push_back({t,Vec3(0.1*t*t,0,0),Eigen::AngleAxisd(0.4*std::sin(t),Vec3::UnitZ()).toRotationMatrix()});
  }
  mc::Limits lim;
  auto motion=mc::differentiate(poses,lim,{{1,19}});
  require(motion.size()>500,"too few smoothed motion samples");
  for (const auto &m:motion) {
    require(m.t>1.14 && m.t<18.86,"derivative fit crossed collection boundary");
    require(std::abs(m.w.z()-0.4*std::cos(m.t))<0.001,"angular velocity smoothing incorrect");
    require(std::abs(m.alpha.z()+0.4*std::sin(m.t))<0.004,"angular acceleration smoothing incorrect");
    require((m.a-Eigen::AngleAxisd(-0.4*std::sin(m.t),Vec3::UnitZ())*Vec3(0.2,0,0)).norm()<1e-8,"linear acceleration frame incorrect");
  }
  // A discontinuity must not be bridged by the polynomial fit.
  poses.erase(poses.begin()+450,poses.begin()+500);
  motion=mc::differentiate(poses,lim,{{1,19}});
  for (const auto &m:motion) require(m.t<8.85 || m.t>10.13,"derivative fit bridged a tracking gap");
}
void trajectoryTest() {
  Fixture truth;
  mc::Limits lim;
  std::mt19937 rng(42);
  std::normal_distribution<double> noise(0,1);
  auto model=[&](double t,mc::Pose &p,mc::Imu &b) {
    const int axis=std::min(2,static_cast<int>(t/20));
    const double u=t-axis*20, k=2*3.141592653589793/5;
    const double sn=std::sin(k*u),cs=std::cos(k*u);
    const double angle=0.4*sn*sn*sn;
    const double rate=1.2*k*sn*sn*cs;
    const double accel=1.2*k*k*(2*sn*cs*cs-sn*sn*sn);
    Vec3 unit=Vec3::Zero();unit[axis]=1;
    const Mat3 RB=Eigen::AngleAxisd(angle,unit).toRotationMatrix();
    const Mat3 RM=RB*truth.R;
    const Vec3 pm(0.1*std::sin(0.3*t),0.1*std::cos(0.4*t),0.1*std::sin(0.2*t));
    const Vec3 am(-0.009*std::sin(0.3*t),-0.016*std::cos(0.4*t),-0.004*std::sin(0.2*t));
    const Vec3 wM=truth.R.transpose()*(unit*rate),aM=truth.R.transpose()*(unit*accel);
    const Vec3 inverse=-truth.R.transpose()*truth.lever;
    const Vec3 ab_world=am+RM*(mc::skew(aM)+mc::skew(wM)*mc::skew(wM))*inverse;
    p={t,pm,RM};
    b={t,unit*rate+truth.gb,RB.transpose()*(ab_world-Vec3(0,0,-9.80665))+truth.ab,RB};
  };
  for (const int hz:{10,20,50,200}) {
    std::vector<mc::Pose> poses;
    std::vector<mc::Imu> imu;
    for (int i=0;i<=60*hz;++i) {
      mc::Pose p;mc::Imu b;
      model(i/static_cast<double>(hz),p,b);
      p.p+=Vec3(noise(rng),noise(rng),noise(rng))*0.0005;
      p.R_WM=p.R_WM*Eigen::AngleAxisd(0.0002*noise(rng),Vec3::UnitX()).toRotationMatrix();
      poses.push_back(p);
    }
    for (int i=0;i<=6000;++i) {
      mc::Pose p;mc::Imu b;model(i*0.01,p,b);
      b.w+=Vec3(noise(rng),noise(rng),noise(rng))*0.003;
      b.a+=Vec3(noise(rng),noise(rng),noise(rng))*0.02;
      imu.push_back(b);
    }
    auto motion=mc::differentiate(poses,lim,{{0,20},{20,40},{40,60}});
    const auto result=mc::solve(motion,imu,truth.gb,Vec3::Zero(),lim);
    require(result.accepted,"noisy three-axis trajectory rejected");
    require(mc::logSO3(result.R_B_M*truth.R.transpose()).norm()<0.01,"trajectory rotation error too large");
    require((result.t_B_M-truth.lever).norm()<0.02,"trajectory lever error exceeds 2 cm");
    std::cout<<"trajectory "<<hz<<" Hz: lever error "<<(result.t_B_M-truth.lever).norm()
             <<" m, gyro RMSE "<<result.gyro_rmse<<", accel RMSE "<<result.accel_rmse<<"\n";
  }
}
int main() {
  try {
    interpolationTest(); staticTest(); staticWindowTest(); relativeGyroTest(); recoveryTest(); degeneracyTest(); overlapTest(); differentiationTest(); trajectoryTest();
    std::cout<<"PASS: static restart/attitude gates, relative gyro consistency, interpolation, extrinsics, bias degeneracy, overlap, derivatives, 10/20/50/200 Hz\n";
    return 0;
  } catch (const std::exception &e) { std::cerr<<"FAIL: "<<e.what()<<"\n"; return 1; }
}
