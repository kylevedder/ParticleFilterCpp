#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"

#include "eigen3/Eigen/Core"
#include "eigen3/Eigen/Geometry"

#include "particle_filter/geometry.h"
#include "particle_filter/math_util.h"
#include "particle_filter/pose.h"

#include <cmath>

static constexpr float kMinAngle = -kPi / 2;
static constexpr float kMaxAngle = kPi / 2;
static constexpr int kNumReadings = 100;
static constexpr float kAngleDelta =
    std::abs(kMaxAngle - kMinAngle) / static_cast<float>(kNumReadings - 1);
static constexpr float kMinReading = 0.1f;
static constexpr float kMaxReading = 5.0f;

struct Wall {
  Eigen::Vector2f p1;
  Eigen::Vector2f p2;
  Wall() : p1(), p2(){};
  Wall(const Eigen::Vector2f& p1, const Eigen::Vector2f& p2) : p1(p1), p2(p2){};
};

struct Map {
  std::vector<Wall> walls;
};

std_msgs::Header MakeHeader() {
  static uint32_t seq = 0;
  std_msgs::Header header;
  header.seq = (++seq);
  header.frame_id = "map";
  header.stamp = ros::Time::now();
  return header;
}

Eigen::Vector2f GetRayReturn(const util::Pose& ray, const Map& map) {
  Eigen::Vector2f delta =
      Eigen::Rotation2Df(ray.rot) * Eigen::Vector2f(kMaxReading - kEpsilon, 0);
  for (const Wall& w : map.walls) {
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

sensor_msgs::LaserScan MakeScan(const util::Pose& robot_pose, const Map& map) {
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

Map MakeMap() {
  Map m;
  m.walls = {{{2, 2}, {2, -1}}, {{-1, 2}, {2, 2}}, {{-1, -1}, {2, -1}}};
  return m;
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "simulator");

  ros::NodeHandle n;

  ros::Publisher scan_pub = n.advertise<sensor_msgs::LaserScan>("laser", 1000);

  ros::Rate loop_rate(10);

  const Map map = MakeMap();

  int iteration = 0;
  while (ros::ok()) {
    ++iteration;
    const float angle = math_util::AngleMod(iteration / 10.0f);
    const Eigen::Vector2f offset =
        Eigen::Rotation2Df(angle) * Eigen::Vector2f(1, 0);
    const sensor_msgs::LaserScan scan =
        MakeScan({offset, math_util::AngleMod(angle * 2)}, map);
    scan_pub.publish(scan);
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}