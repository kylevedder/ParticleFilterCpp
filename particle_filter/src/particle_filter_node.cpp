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

void DrawGroundTruth(const util::Pose& ground_truth,
                     ros::Publisher* ground_truth_pub) {
  {
    visualization_msgs::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time();
    marker.ns = "ground_truth_body";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = ground_truth.tra.x();
    marker.pose.position.y = ground_truth.tra.y();
    marker.pose.position.z = 0;
    marker.scale.x = 0.1;
    marker.scale.y = 0.1;
    marker.scale.z = 0.1;
    marker.color.a = 1.0;
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    ground_truth_pub->publish(marker);
  }
  {
    visualization_msgs::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time();
    marker.ns = "ground_truth_arrow";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::ARROW;
    marker.action = visualization_msgs::Marker::ADD;
    geometry_msgs::Point start;
    start.x = ground_truth.tra.x();
    start.y = ground_truth.tra.y();
    geometry_msgs::Point end;
    const Eigen::Vector2f delta(math_util::Cos(ground_truth.rot) * 0.4f,
                                math_util::Sin(ground_truth.rot) * 0.4f);
    end.x = ground_truth.tra.x() + delta.x();
    end.y = ground_truth.tra.y() + delta.y();
    marker.points.push_back(start);
    marker.points.push_back(end);
    marker.scale.x = 0.02;
    marker.scale.y = 0.1;
    marker.scale.z = 0.1;
    marker.color.a = 1.0;
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    ground_truth_pub->publish(marker);
  }
}

struct ParticleFilterWrapper {
  localization::ParticleFilter particle_filter;
  ros::Publisher particle_pub;
  ros::Publisher ground_truth_pub;

  ParticleFilterWrapper() = delete;
  ParticleFilterWrapper(const util::Map& map, ros::NodeHandle* n)
      : particle_filter(map) {
    particle_pub =
        n->advertise<visualization_msgs::MarkerArray>("particles", 10);
    ground_truth_pub =
        n->advertise<visualization_msgs::Marker>("ground_truth", 10);
  }

  void StartCallback(const geometry_msgs::Twist::ConstPtr& msg) {
    util::Pose start_pose(*msg);
    DrawGroundTruth(start_pose, &ground_truth_pub);
    if (particle_filter.IsInitialized()) {
      return;
    }
    particle_filter.InitalizePose(start_pose);
    ROS_INFO("Initialized Particle Filter");
  }

  void LaserCallback(const sensor_msgs::LaserScan::ConstPtr& msg) {
    util::LaserScan laser(*msg);
    particle_filter.UpdateObservation(laser);
    particle_filter.DrawParticles(&particle_pub);
  }

  void OdomCallback(const geometry_msgs::Twist::ConstPtr& msg) {
    util::Pose odom(*msg);
    particle_filter.UpdateOdom(odom.tra.x(), odom.rot);
    particle_filter.DrawParticles(&particle_pub);
  }
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "particle_filter");

  ros::NodeHandle n;

  ParticleFilterWrapper wrapper(MakeMap(), &n);

  ros::Subscriber initial_pose_sub = n.subscribe(
      "true_pose", 1000, &ParticleFilterWrapper::StartCallback, &wrapper);
  ros::Subscriber laser_sub = n.subscribe(
      "laser", 1000, &ParticleFilterWrapper::LaserCallback, &wrapper);
  ros::Subscriber odom_sub =
      n.subscribe("odom", 1000, &ParticleFilterWrapper::OdomCallback, &wrapper);

  ros::spin();

  return 0;
}
