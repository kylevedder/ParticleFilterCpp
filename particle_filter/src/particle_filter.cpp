#include "ros/ros.h"

#include "particle_filter/constants.h"
#include "particle_filter/particle_filter.h"

#include "particle_filter/array_util.h"
#include "particle_filter/geometry.h"
#include "particle_filter/math_util.h"
#include "particle_filter/visualization.h"

#include "visualization_msgs/MarkerArray.h"

#include <iomanip>
#include <limits>

#include "eigen3/Eigen/Geometry"

namespace localization {

MotionModel::MotionModel() : rd_(), gen_(0) {}

util::Pose MotionModel::ForwardPredict(const util::Pose& pose_global_frame,
                                       const float translation_robot_frame,
                                       const float rotation_robot_frame) {
  NP_CHECK(std::isfinite(translation_robot_frame));
  NP_CHECK(std::isfinite(rotation_robot_frame));
  std::normal_distribution<> distance_along_arc_dist(
      translation_robot_frame, kPFMoveAlongArcNoiseStddev);
  std::normal_distribution<> rotation_dist(rotation_robot_frame,
                                           kPFMoveRotateNoiseStddev);

  const float distance_along_arc = distance_along_arc_dist(gen_);
  const float rotation = rotation_dist(gen_);

  return geometry::FollowTrajectory(pose_global_frame, distance_along_arc,
                                    rotation);
}

SensorModel::SensorModel(const util::Map& map) : map_(map) {}

float GetDepthProbability(const float& sensor_reading, const float& map_reading,
                          const float& ray_min, const float& ray_max) {
  const float people_noise =
      math_util::ProbabilityDensityExp(sensor_reading, 0.01f) * 0.0f;
  const float sensor_reading_noise =
      math_util::ProbabilityDensityGuassian(sensor_reading, map_reading,
                                            kPFLaserReadingNoiseStddev) *
      1.0f;
  const float sensor_random_reading =
      math_util::ProbabilityDensityUniform(sensor_reading, ray_min, ray_max) *
      0.0f;
  const float sensor_random_ray_max =
      math_util::ProbabilityDensityUniform(sensor_reading, ray_max - 0.01f,
                                           ray_max) *
      0.0f;
  return people_noise + sensor_reading_noise + sensor_random_reading +
         sensor_random_ray_max;
}

float SensorModel::GetProbability(const util::Pose& pose_global_frame,
                                  const util::LaserScan& laser_scan,
                                  util::LaserScan* filtered_laser_scan) const {
  NP_NOT_NULL(filtered_laser_scan);
  if (laser_scan.ros_laser_scan_.ranges.empty()) {
    return 0;
  }

  float probability_sum = 0;

  const float& range_min = laser_scan.ros_laser_scan_.range_min;
  const float& range_max = laser_scan.ros_laser_scan_.range_max;

  for (size_t i = 0; i < laser_scan.ros_laser_scan_.ranges.size(); ++i) {
    float range = laser_scan.ros_laser_scan_.ranges[i];
    if (!std::isfinite(range)) {
      range = range_max;
    }
    range = math_util::Clamp(range_min, range, range_max);

    const Eigen::Vector2f endpoint =
        laser_scan.GetRayEndpoint(i, pose_global_frame);
    const Eigen::Vector2f& startpoint = pose_global_frame.tra;

    NP_CHECK((endpoint - startpoint).norm() >= range_min - kEpsilon);
    NP_CHECK((endpoint - startpoint).norm() <= range_max + kEpsilon);

    const float distance_to_map_wall =
        std::max(map_.MinDistanceAlongRay(startpoint, endpoint), range_min);
    NP_FINITE(distance_to_map_wall);
    NP_CHECK(range_min - kEpsilon <= distance_to_map_wall);
    NP_CHECK(distance_to_map_wall <= range_max + kEpsilon);

    const float depth_probability =
        GetDepthProbability(range, distance_to_map_wall, range_min, range_max);
    if (depth_probability > 0.0f || range > range_max / 2.0f) {
      (*filtered_laser_scan).ros_laser_scan_.ranges[i] =
          std::numeric_limits<float>::quiet_NaN();
    }
    probability_sum += depth_probability;
  }
  return probability_sum / laser_scan.ros_laser_scan_.ranges.size();
}

ParticleFilter::ParticleFilter(const util::Map& map)
    : initialized_(false), rd_(), gen_(0), sensor_model_(map) {}

ParticleFilter::ParticleFilter(const util::Map& map,
                               const util::Pose& start_pose)
    : initialized_(true), rd_(), gen_(0), sensor_model_(map) {
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

float ScanSimilarity(const util::LaserScan& scan1, const util::Pose& pose1,
                     const util::LaserScan& scan2, const util::Pose& pose2) {
  const auto global_points_1 =
      scan1.TransformPointsFrameSparse(pose1.ToAffine());
  const auto global_points_2 =
      scan2.TransformPointsFrameSparse(pose2.ToAffine());

  if (global_points_1.empty() || global_points_2.empty()) {
    return 0.0f;
  }

  float total_error = 0;
  for (const Eigen::Vector2f& p1 : global_points_1) {
    float min_sq_distance = std::numeric_limits<float>::max();
    for (const Eigen::Vector2f& p2 : global_points_2) {
      const float candidate_sq = (p1 - p2).squaredNorm();
      NP_FINITE(p1.x());
      NP_FINITE(p1.y());
      NP_FINITE(p2.x());
      NP_FINITE(p2.y());
      if (!std::isfinite(candidate_sq)) {
        std::cout << std::setprecision(20) << std::fixed << p1.x() << p1.y()
                  << p2.x() << p2.y() << candidate_sq << std::endl;
      }
      NP_FINITE(candidate_sq);
      min_sq_distance = std::min(candidate_sq, min_sq_distance);
    }
    NP_FINITE(min_sq_distance);
    if (min_sq_distance < std::numeric_limits<float>::max()) {
      total_error += min_sq_distance;
    }
    NP_FINITE(total_error);
  }
  NP_FINITE(total_error);
  const float average_error =
      total_error / static_cast<float>(global_points_1.size());
  // std::cout << "Average error: " << average_error << std::endl;
  if (average_error < 0.01f) {
    return 100.0f;
  }
  return 1.0f / average_error;
}

util::LaserScan ParticleFilter::ReweightParticles(
    const util::LaserScan& laser_scan) {
  util::LaserScan filtered_laser_scan = laser_scan;
  for (Particle& p : particles_) {
    filtered_laser_scan = laser_scan;
    p.weight =
        sensor_model_.GetProbability(p.pose, laser_scan, &filtered_laser_scan);
    const float similarity =
        0.0f *
        ScanSimilarity(laser_scan, p.pose, p.prev_filtered_laser, p.prev_pose);
    p.weight += similarity;
    // std::cout << "Weight: " << p.weight - similarity << " Sim: " <<
    // similarity << " Weight + Sim: " << p.weight << std::endl;

    p.prev_filtered_laser = filtered_laser_scan;
    p.prev_pose = p.pose;

    NP_FINITE(p.pose.tra.x());
    NP_FINITE(p.pose.tra.y());
    NP_FINITE(p.pose.rot);
  }
  return filtered_laser_scan;
}

void ParticleFilter::ResampleParticles() {
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
    NP_CHECK(weight_sample <= total_weights);
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

void ParticleFilter::UpdateObservation(const util::LaserScan& laser_scan,
                                       ros::Publisher* sampled_scan_pub) {
  if (!initialized_) {
    ROS_WARN("Particle filter not initialized yet!");
    return;
  }

  const auto filtered_laser_scan = ReweightParticles(laser_scan);

  sampled_scan_pub->publish(filtered_laser_scan.ros_laser_scan_);

  ResampleParticles();
}

util::Pose ParticleFilter::MaxWeight() const {
  Particle const* max_particle = &particles_[0];
  for (Particle const& p : particles_) {
    if (max_particle->weight < p.weight) {
      max_particle = &p;
    }
  }
  return max_particle->pose;
}

util::Pose ParticleFilter::WeightedCentroid() const {
  float total_weight = 0;
  for (const Particle& p : particles_) {
    total_weight += p.weight;
  }

  util::Pose weighted_centroid({0, 0}, 0);
  for (const Particle& p : particles_) {
    const float scale = p.weight / total_weight;
    weighted_centroid.tra += p.pose.tra * scale;
    weighted_centroid.rot =
        math_util::AngleMod(weighted_centroid.rot + p.pose.rot * scale);
  }
  return weighted_centroid;
}

void ParticleFilter::DrawParticles(ros::Publisher* particle_pub) const {
  if (!initialized_) {
    ROS_WARN("Particle filter not initialized yet!");
    return;
  }
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
    const float alpha =
        ((max_weight > 0.0f) ? (p.weight / max_weight) : 0.0f) / 2;
    visualization::DrawPose(p.pose, "map", "particles", 1, 0, 0, alpha,
                            &particle_markers);
  }

  const util::Pose estimate = WeightedCentroid();

  visualization::DrawPose(estimate, "map", "estimate", 0, 0, 1, 1,
                          &particle_markers);

  particle_pub->publish(particle_markers);
}
}  // namespace localization