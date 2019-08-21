#include "ros/ros.h"

#include "particle_filter/particle_filter.h"

#include "particle_filter/geometry.h"
#include "particle_filter/math_util.h"

#include "eigen3/Eigen/Geometry"

namespace localization {

MotionModel::MotionModel() : rd_(), gen_(rd_()) {}

util::Pose FollowTrajectory(const util::Pose& pose_global_frame,
                            const float& distance_along_arc,
                            const float& rotation) {
  const Eigen::Rotation2Df robot_to_global_frame(pose_global_frame.rot);
  const Eigen::Vector2f robot_forward_global_frame =
      robot_to_global_frame * Eigen::Vector2f(1, 0);

  if (rotation == 0) {
    ROS_INFO("Rotation is zero!");
    util::Pose updated_pose = pose_global_frame;
    updated_pose.tra += robot_forward_global_frame * distance_along_arc;
    return updated_pose;
  }

  const float circle_radius = distance_along_arc / rotation;

  const float move_x_dist = sin(rotation) * circle_radius;
  const float move_y_dist =
      -(cos(fabs(rotation)) * circle_radius - circle_radius);

  ROS_INFO("Translation: %f Rotation: %f Circle Radius %f Move X %f Move Y %f!",
           distance_along_arc, rotation, circle_radius, move_x_dist,
           move_y_dist);

  const Eigen::Vector2f movement_arc_robot_frame(move_x_dist, move_y_dist);
  const Eigen::Vector2f movement_arc_global_frame =
      robot_to_global_frame * movement_arc_robot_frame;

  return {movement_arc_global_frame + pose_global_frame.tra,
          math_util::AngleMod(rotation + pose_global_frame.rot)};
}

util::Pose MotionModel::ForwardPredict(const util::Pose& pose_global_frame,
                                       const float translation_robot_frame,
                                       const float rotation_robot_frame,
                                       const Time delta_t) {
  static constexpr float kDistanceAlongArcStdDev = 0.05f;
  static constexpr float kRotStdDev = 0.4f;
  std::normal_distribution<> distance_along_arc_dist(translation_robot_frame,
                                                     kDistanceAlongArcStdDev);
  std::normal_distribution<> rotation_dist(rotation_robot_frame, kRotStdDev);

  const float distance_along_arc = distance_along_arc_dist(gen_);
  const float rotation = rotation_dist(gen_);

  return FollowTrajectory(pose_global_frame, distance_along_arc, rotation);
}

ParticleFilter::ParticleFilter(const util::Pose& start_pose)
    : start_pose_(start_pose) {}

void ParticleFilter::UpdateOdom(const util::Pose& odom_delta) {}
}  // namespace localization