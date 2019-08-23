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

#include <tf/transform_broadcaster.h>

#include <cmath>
#include <random>
#include <string>

std::random_device rd;
std::mt19937 gen(rd());

std_msgs::Header MakeHeader(const std::string& frame_id) {
  static uint32_t seq = 0;
  std_msgs::Header header;
  header.seq = (++seq);
  header.frame_id = frame_id;
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
                                const util::Map& map,
                                const float noise_stddev) {
  std::normal_distribution<> noise_dist(0.0f, noise_stddev);

  sensor_msgs::LaserScan scan;
  scan.header = MakeHeader("base_link");
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
    const float norm = ray_return.norm() + noise_dist(gen);
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
      n.advertise<geometry_msgs::Twist>("true_pose", 1);
  ros::Publisher scan_pub = n.advertise<sensor_msgs::LaserScan>("laser", 10);
  ros::Publisher odom_pub = n.advertise<geometry_msgs::Twist>("odom", 10);

  ros::Rate loop_rate(6);

  const util::Map map = MakeMap();
  util::Pose current_pose({8, 0}, 0);

  int iteration = 0;
  while (ros::ok()) {
    initial_pose_pub.publish(current_pose.ToTwist());
    if (iteration >= 200) {
      iteration = 0;
    }

    util::Pose move({0.05f, 0}, -0.1f);
    current_pose =
        geometry::FollowTrajectory(current_pose, move.tra.x(), move.rot);
    ROS_INFO("Current Pose: (%f, %f) %f", current_pose.tra.x(),
             current_pose.tra.y(), current_pose.rot);

    static tf::TransformBroadcaster br;
    tf::Transform transform;
    transform.setOrigin(
        tf::Vector3(current_pose.tra.x(), current_pose.tra.y(), 0.0));
    tf::Quaternion q;
    q.setRPY(0, 0, current_pose.rot);
    transform.setRotation(q);
    br.sendTransform(
        tf::StampedTransform(transform, ros::Time::now(), "map", "base_link"));

    const sensor_msgs::LaserScan scan =
        MakeScan(current_pose, map, kLaserReadingNoiseStddev);
    scan_pub.publish(scan);
    std::normal_distribution<> along_arc_dist(0.0f, kMoveAlongArcNoiseStddev);
    std::normal_distribution<> rotation_dist(0.0f, kMoveRotateNoiseStddev);
    move.tra.x() += along_arc_dist(gen);
    move.rot += rotation_dist(gen);

    odom_pub.publish(move.ToTwist());
    ros::spinOnce();
    loop_rate.sleep();
    ++iteration;
  }

  return 0;
}