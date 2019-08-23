#include "geometry_msgs/Twist.h"
#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"

#include "eigen3/Eigen/Core"
#include "eigen3/Eigen/Geometry"

#include "particle_filter/constants.h"
#include "particle_filter/geometry.h"
#include "particle_filter/map.h"
#include "particle_filter/math_util.h"
#include "particle_filter/pose.h"

#include <cmath>

std_msgs::Header MakeHeader() {
  static uint32_t seq = 0;
  std_msgs::Header header;
  header.seq = (++seq);
  header.frame_id = "map";
  header.stamp = ros::Time::now();
  return header;
}

Eigen::Vector2f GetRayReturn(const util::Pose& ray, const util::Map& map) {
  Eigen::Vector2f delta =
      Eigen::Rotation2Df(ray.rot) * Eigen::Vector2f(kMaxReading - kEpsilon, 0);
  for (const util::Wall& w : map.walls) {
    const Eigen::Vector2f& ray_start = ray.tra;
    const Eigen::Vector2f& ray_end = ray.tra + delta;
    const auto res =
        geometry::CheckLineLineIntersection(w.p1, w.p2, ray_start, ray_end);
    if (!res.first) {
      continue;
    }
    const Eigen::Vector2f& collision_end = res.second;
    const Eigen::Vector2f& collision_delta = (collision_end - ray.tra);
    if (delta.squaredNorm() > collision_delta.squaredNorm()) {
      delta = collision_delta;
    }
  }
  if (delta.squaredNorm() < math_util::Sq(kMinReading)) {
    return delta.normalized() * kMinReading;
  }
  return delta;
}

sensor_msgs::LaserScan MakeScan(const util::Pose& robot_pose,
                                const util::Map& map) {
  sensor_msgs::LaserScan scan;
  scan.header = MakeHeader();
  scan.angle_min = kMinAngle;
  scan.angle_max = kMaxAngle;
  scan.angle_increment = kAngleDelta;
  scan.range_min = kMinReading;
  scan.range_max = kMaxReading;
  scan.scan_time = 0;
  scan.time_increment = 0;

  for (int ray_idx = 0; ray_idx < kNumReadings; ++ray_idx) {
    const float angle =
        kMinAngle + static_cast<float>(kAngleDelta * ray_idx) + robot_pose.rot;
    const util::Pose ray(robot_pose.tra, angle);
    const Eigen::Vector2f ray_return = GetRayReturn(ray, map);
    const float norm = ray_return.norm();
    scan.ranges.push_back(norm);
  }

  return scan;
}

util::Map MakeMap() {
  util::Map m;
  m.walls.push_back({{-10, 2}, {10, 2}});
  m.walls.push_back({{-10, -2}, {10, -2}});
  m.walls.push_back({{-10, -2}, {-10, 2}});
  m.walls.push_back({{10, -2}, {10, 2}});
  return m;
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "simulator");

  ros::NodeHandle n;

  ros::Publisher initial_pose_pub =
      n.advertise<geometry_msgs::Twist>("initial_pose", 100);
  ros::Publisher scan_pub = n.advertise<sensor_msgs::LaserScan>("laser", 10);
  ros::Publisher odom_pub = n.advertise<geometry_msgs::Twist>("odom", 10);

  ros::Rate loop_rate(10);

  const util::Map map = MakeMap();

  util::Pose current_pose({0, 0}, 0);

  for (size_t i = 0; i < 20; ++i) {
    initial_pose_pub.publish(current_pose.ToTwist());
    ros::spinOnce();
    ROS_INFO("Published start!");
  }

  int iteration = 0;
  while (ros::ok()) {
    if (iteration >= 200) {
      iteration = 0;
    }

    const util::Pose move = ((iteration < 100) ? util::Pose({-0.05, 0}, 0)
                                               : util::Pose({0.05, 0}, 0));
    // ROS_INFO("Translate: %f Rotate %f", move.tra.x(), move.rot);
    current_pose = current_pose + move;

    const sensor_msgs::LaserScan scan = MakeScan(current_pose, map);
    scan_pub.publish(scan);
    odom_pub.publish(move.ToTwist());
    ros::spinOnce();
    loop_rate.sleep();
    ++iteration;
  }

  return 0;
}