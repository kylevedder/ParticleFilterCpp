#include <signal.h>
#include <cmath>
#include "particle_filter/crash_handling.h"
#include "particle_filter/geometry.h"
#include "particle_filter/math_util.h"
#include "particle_filter/particle_filter.h"
#include "particle_filter/util.h"
#include "particle_filter/visualization.h"

#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <visualization_msgs/MarkerArray.h>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include <fstream>

void DrawGroundTruth(const util::Pose& ground_truth,
                     ros::Publisher* ground_truth_pub) {
  visualization_msgs::MarkerArray arr;
  visualization::DrawPose(ground_truth, "map", "ground_truth", 0, 1, 0, 1,
                          &arr);
  ground_truth_pub->publish(arr);
}

struct ParticleFilterWrapper {
  util::Map map;
  localization::ParticleFilter particle_filter;

  ros::Publisher particle_pub;
  ros::Publisher sampled_laser_pub;
  ros::Publisher grid_belief_pub;
  ros::Publisher reference_pub;

  ParticleFilterWrapper() = delete;
  ParticleFilterWrapper(const util::Map& map, ros::NodeHandle* n)
      : map(map), particle_filter(map) {
    particle_pub =
        n->advertise<visualization_msgs::MarkerArray>("particles", 10);
    sampled_laser_pub =
        n->advertise<sensor_msgs::LaserScan>("sampled_laser", 100);
    grid_belief_pub =
        n->advertise<visualization_msgs::MarkerArray>("grid_belief", 10);
    reference_pub =
        n->advertise<visualization_msgs::MarkerArray>("reference", 10);
  }

  void GridSearchBelief(const util::LaserScan& laser) {
    visualization_msgs::MarkerArray arr;
    float max_score = 0;
    static constexpr float kXMin = -0.75;
    static constexpr float kXMax = 0.75;
    static constexpr float kXDel = 0.1;
    static constexpr float kYMin = -0.75;
    static constexpr float kYMax = 0.75;
    static constexpr float kYDel = 0.1;
    static constexpr float kThetaMin = -kPi / 2;
    static constexpr float kThetaMax = kPi / 2 + kEpsilon;
    static constexpr float kThetaDel = kPi / 8;

    const auto& wc = particle_filter.WeightedCentroid();

    auto make_position = [wc](const float x, const float y,
                              const float theta) -> util::Pose {
      const Eigen::Vector2f offset_vector(x, y);
      return util::Pose(wc.tra + Eigen::Rotation2Df(wc.rot) * offset_vector,
                        theta + wc.rot);
    };

    for (float x = kXMin; x <= kXMax; x += kXDel) {
      for (float y = kYMin; y <= kYMax; y += kYDel) {
        for (float theta = kThetaMin; theta <= kThetaMax; theta += kThetaDel) {
          const util::Pose current_pose = make_position(x, y, theta);
          const float score =
              particle_filter.ScoreObservation(current_pose, laser);
          max_score = std::max(max_score, score);
        }
      }
    }

    for (float x = kXMin; x <= kXMax; x += kXDel) {
      for (float y = kYMin; y <= kYMax; y += kYDel) {
        for (float theta = kThetaMin; theta <= kThetaMax; theta += kThetaDel) {
          const Eigen::Vector2f offset_vector(x, y);
          const util::Pose current_pose = make_position(x, y, theta);
          const float score =
              particle_filter.ScoreObservation(current_pose, laser);
          const float red = score / max_score;
          visualization::DrawPose(current_pose, "map", "grid_search", red, 0, 0,
                                  1, &arr, theta);
        }
      }
    }

    ROS_INFO("Published %zu grid elements", arr.markers.size());
    grid_belief_pub.publish(arr);

    visualization_msgs::MarkerArray ref_arr;
    const Eigen::Vector2f offset_tra(1, 0.5);
    const float rotation = 0;
    const util::Pose base_link_reference(offset_tra, rotation);
    visualization::DrawPose(base_link_reference, "base_link", "reference", 0, 0,
                            0, 1, &ref_arr);

    const util::Pose global_reference(
        wc.tra + Eigen::Rotation2Df(wc.rot) * offset_tra, rotation + wc.rot);
    visualization::DrawPose(global_reference, "map", "reference", 1, 1, 1, 1,
                            &ref_arr, 0.1);
    ref_arr.markers.push_back(
        visualization::ToLineList(laser, wc, map, "map", "obs", 1, 0, 0, 1));
    reference_pub.publish(ref_arr);
  }

  void LaserCallback(const sensor_msgs::LaserScan::ConstPtr& msg) {
    util::LaserScan laser(*msg);
    particle_filter.UpdateObservation(laser, &sampled_laser_pub);
    particle_filter.DrawParticles(&particle_pub);

    GridSearchBelief(laser);
  }

  void OdomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    util::Pose odom(msg->pose.pose);
    particle_filter.UpdateOdom(odom.tra.x(), odom.rot);
    particle_filter.DrawParticles(&particle_pub);
  }
};

int main(int argc, char** argv) {
  util::PrintCurrentWorkingDirectory();
  config_reader::ConfigReader reader(
      {"src/ParticleFilterCpp/particle_filter/config/pf_config.lua"});
  ros::init(argc, argv, "particle_filter", ros::init_options::NoSigintHandler);

  if (signal(SIGINT, util::crash::FatalSignalHandler) == SIG_ERR) {
    std::cerr << "Cannot trap SIGINT" << std::endl;
    exit(-1);
  }
  if (signal(SIGSEGV, util::crash::FatalSignalHandler) == SIG_ERR) {
    std::cerr << "Cannot trap SIGSEGV" << std::endl;
    exit(-1);
  }
  if (signal(SIGABRT, util::crash::FatalSignalHandler) == SIG_ERR) {
    std::cerr << "Cannot trap SIGABRT" << std::endl;
    exit(-1);
  }

  ros::NodeHandle n;

  ParticleFilterWrapper wrapper(
      util::Map("/home/k/code/catkin_ws/src/particle_filter/maps/loop.map"),
      &n);

  ros::Subscriber laser_sub = n.subscribe(
      "/scan", 1000, &ParticleFilterWrapper::LaserCallback, &wrapper);
  ros::Subscriber odom_sub = n.subscribe(
      "/odom", 1000, &ParticleFilterWrapper::OdomCallback, &wrapper);

  ros::spin();

  return 0;
}
