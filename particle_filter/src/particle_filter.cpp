#include "ros/ros.h"

#include "particle_filter/constants.h"
#include "particle_filter/particle_filter.h"

#include "particle_filter/array_util.h"
#include "particle_filter/geometry.h"
#include "particle_filter/math_util.h"

#include "visualization_msgs/MarkerArray.h"

#include "eigen3/Eigen/Geometry"

namespace localization {

MotionModel::MotionModel() : rd_(), gen_(rd_()) {}

util::Pose FollowTrajectory(const util::Pose& pose_global_frame,
                            const float& distance_along_arc,
                            const float& rotation) {
  NP_CHECK(std::isfinite(distance_along_arc));
  NP_CHECK(std::isfinite(rotation));
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

  // ROS_INFO("Translation: %f Rotation: %f Circle Radius %f Move X %f Move Y
  // %f!",
  //          distance_along_arc, rotation, circle_radius, move_x_dist,
  //          move_y_dist);

  const Eigen::Vector2f movement_arc_robot_frame(move_x_dist, move_y_dist);
  const Eigen::Vector2f movement_arc_global_frame =
      robot_to_global_frame * movement_arc_robot_frame;

  return {movement_arc_global_frame + pose_global_frame.tra,
          math_util::AngleMod(rotation + pose_global_frame.rot)};
}

util::Pose MotionModel::ForwardPredict(const util::Pose& pose_global_frame,
                                       const float translation_robot_frame,
                                       const float rotation_robot_frame) {
  NP_CHECK(std::isfinite(translation_robot_frame));
  NP_CHECK(std::isfinite(rotation_robot_frame));
  static constexpr float kDistanceAlongArcStdDev = 0.01f;
  static constexpr float kRotStdDev = 0.05f;
  std::normal_distribution<> distance_along_arc_dist(translation_robot_frame,
                                                     kDistanceAlongArcStdDev);
  std::normal_distribution<> rotation_dist(rotation_robot_frame, kRotStdDev);

  const float distance_along_arc = distance_along_arc_dist(gen_);
  const float rotation = rotation_dist(gen_);

  return FollowTrajectory(pose_global_frame, distance_along_arc, rotation);
}

SensorModel::SensorModel(const util::Map& map)
    : rd_(), gen_(rd_()), map_(map) {}

float GetDepthProbability(const float& sensor_reading, const float& map_reading,
                          const float& ray_min, const float& ray_max) {
  return math_util::ProbabilityDensityExp(sensor_reading, 0.01f) +
         math_util::ProbabilityDensityGuassian(sensor_reading, map_reading,
                                               0.01f) *
             4 +
         math_util::ProbabilityDensityUniform(sensor_reading, ray_min,
                                              ray_max) *
             0.1 +
         math_util::ProbabilityDensityUniform(sensor_reading, ray_max - 0.01f,
                                              ray_max) *
             0.02;
}

float SensorModel::GetProbability(const util::Pose& pose_global_frame,
                                  const util::LaserScan& laser_scan) const {
  float probability_sum = 0;

  const float& range_min = laser_scan.ros_laser_scan_.range_min;
  const float& range_max = laser_scan.ros_laser_scan_.range_max;

  for (size_t i = 0; i < laser_scan.ros_laser_scan_.ranges.size(); ++i) {
    const float& range = laser_scan.ros_laser_scan_.ranges[i];
    if (!std::isfinite(range) || range < kMinReading || range > kMaxReading) {
      continue;
    }
    const Eigen::Vector2f endpoint =
        laser_scan.GetRayEndpoint(i, pose_global_frame);
    const Eigen::Vector2f& startpoint = pose_global_frame.tra;
    const float distance_to_map_wall =
        std::max(map_.MinDistanceAlongRay(startpoint, endpoint), range_min);

    probability_sum +=
        GetDepthProbability(range, distance_to_map_wall, range_min, range_max);
  }
  return probability_sum / laser_scan.ros_laser_scan_.ranges.size();
}

ParticleFilter::ParticleFilter(const util::Map& map)
    : initialized_(false), rd_(), gen_(rd_()), sensor_model_(map) {}

ParticleFilter::ParticleFilter(const util::Map& map,
                               const util::Pose& start_pose)
    : initialized_(true), rd_(), gen_(rd_()), sensor_model_(map) {
  InitalizePose(start_pose);
}

bool ParticleFilter::IsInitialized() const { return initialized_; }

void ParticleFilter::InitalizePose(const util::Pose& start_pose) {
  for (Particle& p : particles_) {
    p = Particle(start_pose, 1.0f);
  }
  initialized_ = true;
}

void ParticleFilter::UpdateOdom(const float& translation,
                                const float& rotation) {
  if (!initialized_) {
    ROS_WARN("Particle filter not initialized yet!");
    return;
  }
  for (Particle& p : particles_) {
    p.pose = motion_model_.ForwardPredict(p.pose, translation, rotation);
  }
}

void ParticleFilter::UpdateObservation(const util::LaserScan& laser_scan) {
  if (!initialized_) {
    ROS_WARN("Particle filter not initialized yet!");
    return;
  }

  for (Particle& p : particles_) {
    p.weight = sensor_model_.GetProbability(p.pose, laser_scan);
  }

  const float total_weights = [this]() -> float {
    float sum = 0;
    for (const auto& p : particles_) {
      sum += p.weight;
    }
    return sum;
  }();

  std::uniform_real_distribution<> distribution(0.0f, total_weights);

  auto resampled_particles = particles_;
  for (Particle& new_p : resampled_particles) {
    float weight_sample = distribution(gen_);
    NP_CHECK(weight_sample < total_weights);
    for (const Particle& old_p : particles_) {
      if (weight_sample <= old_p.weight) {
        new_p = old_p;
        break;
      }
      weight_sample -= old_p.weight;
    }
  }

  particles_ = resampled_particles;
}

void ParticleFilter::DrawParticles(ros::Publisher* particle_pub) const {
  static visualization_msgs::MarkerArray particle_markers;
  for (visualization_msgs::Marker& marker : particle_markers.markers) {
    marker.action = marker.DELETE;
  }
  particle_pub->publish(particle_markers);
  particle_markers.markers.clear();
  float max_weight = 0;
  for (const Particle& p : particles_) {
    max_weight = std::max(p.weight, max_weight);
  }
  for (const Particle& p : particles_) {
    {
      visualization_msgs::Marker marker;
      marker.header.frame_id = "map";
      marker.header.stamp = ros::Time();
      marker.ns = "particles";
      marker.id = particle_markers.markers.size();
      marker.type = visualization_msgs::Marker::SPHERE;
      marker.action = visualization_msgs::Marker::ADD;
      marker.pose.position.x = p.pose.tra.x();
      marker.pose.position.y = p.pose.tra.y();
      marker.pose.position.z = 0;
      marker.scale.x = 0.02;
      marker.scale.y = 0.02;
      marker.scale.z = 0.02;
      marker.color.a = p.weight / max_weight;
      marker.color.r = 1.0;
      marker.color.g = 0.0;
      marker.color.b = 0.0;
      particle_markers.markers.push_back(marker);
    }
    {
      visualization_msgs::Marker marker;
      marker.header.frame_id = "map";
      marker.header.stamp = ros::Time();
      marker.ns = "particles";
      marker.id = particle_markers.markers.size();
      marker.type = visualization_msgs::Marker::ARROW;
      marker.action = visualization_msgs::Marker::ADD;
      geometry_msgs::Point start;
      start.x = p.pose.tra.x();
      start.y = p.pose.tra.y();
      geometry_msgs::Point end;
      const Eigen::Vector2f delta(math_util::Cos(p.pose.rot) * 0.1f,
                                  math_util::Sin(p.pose.rot) * 0.1f);
      end.x = p.pose.tra.x() + delta.x();
      end.y = p.pose.tra.y() + delta.y();
      marker.points.push_back(start);
      marker.points.push_back(end);
      marker.scale.x = 0.01;
      marker.scale.y = 0.01;
      marker.scale.z = 0.01;
      marker.color.a = p.weight / max_weight;
      marker.color.r = 0.0;
      marker.color.g = 1.0;
      marker.color.b = 0.0;
      particle_markers.markers.push_back(marker);
    }
  }

  particle_pub->publish(particle_markers);
}

}  // namespace localization