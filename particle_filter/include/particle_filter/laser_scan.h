#pragma once

#include "sensor_msgs/LaserScan.h"

#include "eigen3/Eigen/Core"
#include "eigen3/Eigen/Geometry"

#include "pose.h"

#include <vector>

namespace util {
class LaserScan {
 public:
  sensor_msgs::LaserScan ros_laser_scan_;
  std::vector<Eigen::Vector2f> robot_frame_points_;

  LaserScan() = delete;
  explicit LaserScan(const sensor_msgs::LaserScan& ros_laser_scan)
      : ros_laser_scan_(ros_laser_scan),
        robot_frame_points_(ros_laser_scan_.ranges.size()) {
    for (size_t i = 0; i < ros_laser_scan_.ranges.size(); ++i) {
      const float theta =
          ros_laser_scan_.angle_min + i * ros_laser_scan_.angle_increment;
      const float& depth = ros_laser_scan_.ranges[i];
      robot_frame_points_[i] = {sin(theta) * depth, cos(theta) * depth};
    }
  }

  std::vector<Eigen::Vector2f> TransformPointsFrame(
      const Eigen::Affine2f& transform) const {
    auto transformed_points = robot_frame_points_;
    for (auto& point : transformed_points) {
      point = transform * point;
    }
    return transformed_points;
  }

  Eigen::Vector2f GetRayEndpoint(const size_t ray_index,
                                 const util::Pose& obs_pose) const {
    NP_CHECK(ray_index < ros_laser_scan_.ranges.size());
    const float ray_angle_obs_frame = math_util::AngleMod(
        ros_laser_scan_.angle_min +
        ros_laser_scan_.angle_increment * ray_index + obs_pose.rot);
    const float& range_max = ros_laser_scan_.range_max;

    return {math_util::Sin(ray_angle_obs_frame) * range_max,
            math_util::Cos(ray_angle_obs_frame) * range_max};
  }
};
}  // namespace util