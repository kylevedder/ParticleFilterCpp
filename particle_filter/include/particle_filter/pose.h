#pragma once

#include "eigen3/Eigen/Core"

namespace util {
struct Pose {
  Eigen::Vector2f tra;
  float rot;
  Pose() : tra(), rot(){};
  Pose(const Eigen::Vector2f& tra, const float& rot) : tra(tra), rot(rot){};
};
}  // namespace util