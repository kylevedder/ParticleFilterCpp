#include "geometry_msgs/Twist.h"
#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"

#include "eigen3/Eigen/Core"
#include "eigen3/Eigen/Geometry"

#include "particle_filter/geometry.h"
#include "particle_filter/math_util.h"
#include "particle_filter/particle_filter.h"

#include <cmath>

#include <visualization_msgs/MarkerArray.h>

util::Map MakeMap() {
  util::Map m;
  m.walls.push_back({{-10, 2}, {10, 2}});
  m.walls.push_back({{-10, -2}, {10, -2}});
  m.walls.push_back({{-10, -2}, {-10, 2}});
  m.walls.push_back({{10, -2}, {10, 2}});
  return m;
}

struct ParticleFilterWrapper {
  localization::ParticleFilter particle_filter;
  ros::Publisher marker_pub;

  ParticleFilterWrapper() = delete;
  ParticleFilterWrapper(const util::Map& map, ros::NodeHandle* n)
      : particle_filter(map) {
    marker_pub = n->advertise<visualization_msgs::MarkerArray>("particles", 10);
  }

  void StartCallback(const geometry_msgs::Twist::ConstPtr& msg) {
    if (particle_filter.IsInitialized()) {
      return;
    }
    ROS_INFO("Got Start");
    util::Pose start(*msg);
    particle_filter.InitalizePose(start);
  }

  void LaserCallback(const sensor_msgs::LaserScan::ConstPtr& msg) {
    util::LaserScan laser(*msg);
    particle_filter.UpdateObservation(laser);
    particle_filter.DrawParticles(&marker_pub);
  }

  void OdomCallback(const geometry_msgs::Twist::ConstPtr& msg) {
    util::Pose odom(*msg);
    particle_filter.UpdateOdom(odom.tra.x(), odom.rot);
    particle_filter.DrawParticles(&marker_pub);
  }
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "particle_filter");

  ros::NodeHandle n;

  ParticleFilterWrapper wrapper(MakeMap(), &n);

  ros::Subscriber initial_pose_sub = n.subscribe(
      "initial_pose", 1000, &ParticleFilterWrapper::StartCallback, &wrapper);
  ros::Subscriber laser_sub = n.subscribe(
      "laser", 1000, &ParticleFilterWrapper::LaserCallback, &wrapper);
  ros::Subscriber odom_sub =
      n.subscribe("odom", 1000, &ParticleFilterWrapper::OdomCallback, &wrapper);

  ros::spin();

  return 0;
}
