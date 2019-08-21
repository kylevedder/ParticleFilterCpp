#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"

#include "eigen3/Eigen/Core"
#include "eigen3/Eigen/Geometry"

#include "particle_filter/geometry.h"
#include "particle_filter/math_util.h"
#include "particle_filter/particle_filter.h"

#include <cmath>

#include <visualization_msgs/Marker.h>

localization::MotionModel m;

ros::Publisher marker_pub;

void LaserCallback(const sensor_msgs::LaserScan::ConstPtr& msg) {
  ROS_INFO("Got laser");

  const auto pred_pose = m.ForwardPredict({{0, 0}, 0}, 2.0f, 0.5, 0.1f);

  visualization_msgs::Marker points;
  points.header.frame_id = "/map";
  points.header.stamp = ros::Time::now();
  points.ns = "points_and_lines";
  points.action = visualization_msgs::Marker::ADD;
  points.pose.orientation.w = 1.0;

  static int id_ctr = 0;
  points.id = id_ctr++;

  points.type = visualization_msgs::Marker::POINTS;
  points.scale.x = 0.05;
  points.scale.y = 0.05;

  points.color.r = 1;
  points.color.g = 0.882f;
  points.color.b = 0.208f;
  points.color.a = 1.0;

  geometry_msgs::Point p;
  p.x = pred_pose.tra.x();
  p.y = pred_pose.tra.y();
  p.z = 0;

  points.points.push_back(p);
  marker_pub.publish(points);
  ROS_INFO("Published to %f, %f", pred_pose.tra.x(), pred_pose.tra.y());
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "particle_filter");

  ros::NodeHandle n;
  marker_pub = n.advertise<visualization_msgs::Marker>("motion_model", 10);

  ros::Subscriber sub = n.subscribe("laser", 1000, LaserCallback);

  ros::spin();

  return 0;
}
