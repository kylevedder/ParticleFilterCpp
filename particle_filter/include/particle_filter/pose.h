#pragma once

#include "eigen3/Eigen/Core"
#include "eigen3/Eigen/Geometry"

#include "geometry_msgs/Twist.h"
#include "math_util.h"

namespace util {
struct Pose {
  Eigen::Vector2f tra;
  float rot;
  Pose() : tra(), rot(){};
  explicit Pose(const geometry_msgs::Twist& twist)
      : tra(twist.linear.x, twist.linear.y), rot(twist.angular.z) {}
  Pose(const Eigen::Vector2f& tra, const float& rot) : tra(tra), rot(rot){};

  util::Pose operator-(const util::Pose& o) const {
    return {tra - o.tra, math_util::AngleMod(rot - o.rot)};
  }

  util::Pose operator+(const util::Pose& o) const {
    return {tra + o.tra, math_util::AngleMod(rot + o.rot)};
  }

  geometry_msgs::Twist ToTwist() const {
    geometry_msgs::Twist twist;
    twist.linear.x = tra.x();
    twist.linear.y = tra.y();
    twist.angular.z = rot;
    return twist;
  }

  Eigen::Affine2f ToAffine() const {
    Eigen::Affine2f transform;
    transform.rotate(rot);
    transform.translate(tra);
    return transform;
  }
};
}  // namespace util