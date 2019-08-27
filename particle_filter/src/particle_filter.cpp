#include "ros/ros.h"

#include "particle_filter/constants.h"
#include "particle_filter/particle_filter.h"

#include "particle_filter/array_util.h"
#include "particle_filter/geometry.h"
#include "particle_filter/math_util.h"

#include "visualization_msgs/MarkerArray.h"

#include <iomanip>
#include <limits>

#include "eigen3/Eigen/Geometry"

namespace localization {

MotionModel::MotionModel() : rd_(), gen_(rd_()) {}

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

SensorModel::SensorModel(const util::Map& map)
    : rd_(), gen_(rd_()), map_(map) {}

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
  if (laser_scan.ros_laser_scan_.ranges.empty()) {
    return 0;
  }
  float probability_sum = 0;

  const float& range_min = laser_scan.ros_laser_scan_.range_min;
  const float& range_max = laser_scan.ros_laser_scan_.range_max;

  std::vector<float> probabilities;

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
    probabilities.push_back(depth_probability);
    probability_sum += depth_probability;
  }

  *filtered_laser_scan = laser_scan;
  for (size_t i = 0; i < filtered_laser_scan->ros_laser_scan_.ranges.size();
       ++i) {
    const float& p = probabilities[i];
    if (p > 0.0f) {
      (*filtered_laser_scan).ros_laser_scan_.ranges[i] =
          std::numeric_limits<float>::quiet_NaN();
    }
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
  NP_FINITE(start_pose.tra.x());
  NP_FINITE(start_pose.tra.y());
  NP_FINITE(start_pose.rot);
  for (Particle& p : particles_) {
    p = Particle(start_pose, 1.0f);
    NP_FINITE(p.pose.tra.x());
    NP_FINITE(p.pose.tra.y());
    NP_FINITE(p.pose.rot);
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
    NP_FINITE(p.pose.tra.x());
    NP_FINITE(p.pose.tra.y());
    NP_FINITE(p.pose.rot);
    NP_FINITE(p.weight);
    p.pose = motion_model_.ForwardPredict(p.pose, translation, rotation);
    NP_FINITE(p.pose.tra.x());
    NP_FINITE(p.pose.tra.y());
    NP_FINITE(p.pose.rot);
    NP_FINITE(p.weight);
  }
}

void ParticleFilter::UpdateObservation(const util::LaserScan& laser_scan,
                                       ros::Publisher* sampled_scan_pub) {
  if (!initialized_) {
    ROS_WARN("Particle filter not initialized yet!");
    return;
  }

  util::LaserScan filtered_laser_scan = laser_scan;
  for (Particle& p : particles_) {
    NP_FINITE(p.pose.tra.x());
    NP_FINITE(p.pose.tra.y());
    NP_FINITE(p.pose.rot);
    NP_FINITE(p.weight);
    p.weight =
        sensor_model_.GetProbability(p.pose, laser_scan, &filtered_laser_scan);
    NP_FINITE(p.pose.tra.x());
    NP_FINITE(p.pose.tra.y());
    NP_FINITE(p.pose.rot);
    NP_FINITE(p.weight);
  }

  sampled_scan_pub->publish(filtered_laser_scan.ros_laser_scan_);

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
      NP_FINITE(p.pose.tra.x());
      NP_FINITE(p.pose.tra.y());
      marker.pose.position.x = p.pose.tra.x();
      marker.pose.position.y = p.pose.tra.y();
      marker.pose.position.z = 0;
      marker.scale.x = 0.05;
      marker.scale.y = 0.05;
      marker.scale.z = 0.05;
      marker.color.a = ((max_weight > 9.0f) ? (p.weight / max_weight) : 1.0f);
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
      NP_FINITE(p.pose.tra.x());
      NP_FINITE(p.pose.tra.y());
      start.x = p.pose.tra.x();
      start.y = p.pose.tra.y();
      NP_FINITE(start.x);
      NP_FINITE(start.y);
      geometry_msgs::Point end;
      const Eigen::Vector2f delta(math_util::Cos(p.pose.rot) * 0.4f,
                                  math_util::Sin(p.pose.rot) * 0.4f);
      NP_FINITE(delta.x());
      NP_FINITE(delta.y());
      end.x = p.pose.tra.x() + delta.x();
      end.y = p.pose.tra.y() + delta.y();
      NP_FINITE(end.x);
      NP_FINITE(end.y);
      marker.points.push_back(start);
      marker.points.push_back(end);
      marker.scale.x = 0.02;
      marker.scale.y = 0.1;
      marker.scale.z = 0.1;
      marker.color.a = ((max_weight > 9.0f) ? (p.weight / max_weight) : 1.0f);
      marker.color.r = 0.0;
      marker.color.g = 1.0;
      marker.color.b = 0.0;
      particle_markers.markers.push_back(marker);
    }
  }
  particle_pub->publish(particle_markers);
}
}  // namespace localization