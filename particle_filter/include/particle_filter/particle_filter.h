#pragma once

#include "laser_scan.h"
#include "pose.h"

#include <random>

namespace localization {

using Time = float;

class MotionModel {
 private:
  std::random_device rd_;
  std::mt19937 gen_;

 public:
  MotionModel();
  util::Pose ForwardPredict(const util::Pose& pose_global_frame,
                            const float translation_robot_frame,
                            const float rotation_robot_frame,
                            const Time delta_t);
};

class ParticleFilter {
 private:
  util::Pose start_pose_;

 public:
  ParticleFilter() = delete;
  explicit ParticleFilter(const util::Pose& start_pose);

  void UpdateOdom(const util::Pose& odom_delta);
  void UpdateObservation(const util::LaserScan& laser_scan);
};
}  // namespace localization