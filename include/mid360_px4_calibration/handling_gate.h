#pragma once
#include <mid360_px4_calibration/calibration_math.h>
namespace mid360_calibration {
// Observe displacement before opening motion collection; this does not infer
// altitude or physical contact with a table.
class PickupGate {
 public:
  PickupGate(const Pose &reference,double min_duration,double min_distance,double max_gap)
      : reference_(reference),min_duration_(min_duration),min_distance_(min_distance),max_gap_(max_gap),last_t_(reference.t) {}
  void push(const Pose &pose) {
    if(pose.t<=last_t_)return;
    if(pose.t-last_t_>max_gap_)away_since_=-1;
    last_t_=pose.t;
    distance_=(pose.p-reference_.p).norm();
    if(distance_<min_distance_)away_since_=-1;
    else if(away_since_<0)away_since_=pose.t;
  }
  double elapsed() const { return last_t_-reference_.t; }
  double distance() const { return distance_; }
  double awayDuration() const { return away_since_<0 ? 0 : last_t_-away_since_; }
  bool ready() const { return elapsed()>=min_duration_ && distance_>=min_distance_ && awayDuration()>=0.5; }
  double progress() const { return std::min({1.0,elapsed()/min_duration_,distance_/min_distance_,awayDuration()/0.5}); }
 private:
  Pose reference_;
  double min_duration_,min_distance_,max_gap_,last_t_,distance_=0,away_since_=-1;
};
}  // namespace mid360_calibration
