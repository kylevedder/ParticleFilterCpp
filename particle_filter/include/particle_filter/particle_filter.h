#pragma once

#include "laser_scan.h"
#include "map.h"
#include "pose.h"

#include <array>
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
                            const float rotation_robot_frame);
};

class SensorModel {
 private:
  std::random_device rd_;
  std::mt19937 gen_;
  util::Map map_;

 public:
  SensorModel(const util::Map& map);
  float GetProbability(const util::Pose& pose_global_frame,
                       const util::LaserScan& laser_scan) const;
};

struct Particle {
  util::Pose pose;
  float weight;
  Particle() : pose(), weight(0.0f) {}
  Particle(const util::Pose& pose, const float weight)
      : pose(pose), weight(weight) {}
};

class ParticleFilter {
 private:
  bool initialized_;
  std::random_device rd_;
  std::mt19937 gen_;
  static constexpr int kNumParticles = 50;
  std::array<Particle, kNumParticles> particles_;
  MotionModel motion_model_;
  SensorModel sensor_model_;

 public:
  ParticleFilter() = delete;
  explicit ParticleFilter(const util::Map& map);
  ParticleFilter(const util::Map& map, const util::Pose& start_pose);

  bool IsInitialized() const;

  void InitalizePose(const util::Pose& start_pose);

  void UpdateOdom(const float& translation, const float& rotation);
  void UpdateObservation(const util::LaserScan& laser_scan);

  void DrawParticles(ros::Publisher* particle_pub) const;
};
}  // namespace localization