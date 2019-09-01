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

// std::random_device rd;
std::mt19937 gen(0);

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

void PublishTransforms(const util::Pose& current_pose) {
  static tf::TransformBroadcaster br;
  tf::Transform transform;
  transform.setOrigin(
      tf::Vector3(current_pose.tra.x(), current_pose.tra.y(), 0.0));
  tf::Quaternion q;
  q.setRPY(0, 0, current_pose.rot);
  transform.setRotation(q);
  br.sendTransform(
      tf::StampedTransform(transform, ros::Time::now(), "map", "base_link"));
}

util::Pose AddExecutionOdomNoise(util::Pose move) {
  std::normal_distribution<> along_arc_dist(
      0.0f, kMoveAlongArcExecutionNoiseStddev * move.tra.norm());
  std::normal_distribution<> rotation_dist(
      0.0f, kMoveRotateExecutionNoiseStddev * move.rot);
  move.tra.x() += along_arc_dist(gen);
  move.rot += rotation_dist(gen);
  return move;
}

util::Pose AddReadingOdomNoise(util::Pose move) {
  std::normal_distribution<> along_arc_dist(
      0.0f, kMoveAlongArcReadingNoiseStddev * move.tra.norm());
  std::normal_distribution<> rotation_dist(
      0.0f, kMoveRotateReadingNoiseStddev * move.rot);
  move.tra.x() += along_arc_dist(gen);
  move.rot += rotation_dist(gen);
  return move;
}

util::Pose CommandSpinCircle(const util::Pose& current_pose) {
  return {{0.05f, 0}, -0.1f};
}

util::Pose DriveToPose(const util::Pose& current_pose,
                       const util::Pose& goal_pose) {
  const util::Pose delta = goal_pose - current_pose;
  static float kMinRotationalErrorToHalt = kPi / 4;
  static float kRotP = 0.5f;
  static float kMaxRot = 0.1f;
  static float kTraP = 0.5f;
  static float kMaxTra = 0.1f;
  static float kTraHaltThreshold = 0.05f;
  static float kTraRotThreshold = 0.05f;

  // At goal translationally and rotationally.
  if (delta.tra.squaredNorm() < Sq(kTraHaltThreshold) &&
      fabs(delta.rot) < kTraRotThreshold) {
    ROS_INFO("Drive complete!");
    return {{0, 0}, 0};
  }

  // At goal translationally but not rotationally.
  if (delta.tra.squaredNorm() < Sq(kTraHaltThreshold)) {
    ROS_INFO("Rotate to final!");
    const float rot_cmd =
        math_util::Sign(delta.rot) * std::min(kMaxRot, fabs(delta.rot) * kRotP);
    return {{0, 0}, rot_cmd};
  }

  const Eigen::Vector2f current_to_goal_norm = delta.tra.normalized();
  const float desired_angle = math_util::AngleMod(
      atan2(current_to_goal_norm.y(), current_to_goal_norm.x()));
  const float angle_to_goal =
      math_util::AngleMod(desired_angle - current_pose.rot);
  ROS_INFO("Desired angle %f  Current Angle %f, Angle to goal: %f",
           desired_angle, current_pose.rot, angle_to_goal);

  const float rot_cmd = math_util::Sign(angle_to_goal) *
                        std::min(kMaxRot, fabs(angle_to_goal) * kRotP);

  // If angle is too far away to correct while moving.
  if (fabs(angle_to_goal) > kMinRotationalErrorToHalt) {
    ROS_INFO("Correct angle to drive!");
    return {{0, 0}, rot_cmd};
  }

  const Eigen::Vector2f tra_cmd = [&delta]() -> Eigen::Vector2f {
    const Eigen::Vector2f uncapped_tra_cmd =
        Eigen::Rotation2Df(-delta.rot) * (delta.tra * kTraP);
    if (uncapped_tra_cmd.squaredNorm() > Sq(kMaxTra)) {
      return {kMaxTra, 0};
    }
    return {fabs(uncapped_tra_cmd.x()), 0};
  }();

  ROS_INFO("Drive to goal!");
  return {tra_cmd, rot_cmd};
}

util::Pose CommandEndToEnd(const util::Pose& current_pose) {
  enum State { DRIVE_POINT1, DRIVE_POINT2 };
  static State state = DRIVE_POINT1;
  static const util::Pose point1({8, 0}, 0);
  static const util::Pose point2({0, 0}, 0);

  while (true) {
    switch (state) {
      case DRIVE_POINT1: {
        ROS_INFO("DRIVE_POINT1");
        const util::Pose cmd = DriveToPose(current_pose, point1);
        if (cmd != util::Pose({0, 0}, 0)) {
          return cmd;
        }
        state = State::DRIVE_POINT2;
        break;
      }
      case DRIVE_POINT2: {
        ROS_INFO("DRIVE_POINT2");
        const util::Pose cmd = DriveToPose(current_pose, point2);
        if (cmd != util::Pose({0, 0}, 0)) {
          return cmd;
        }
        state = State::DRIVE_POINT1;
        break;
      }
    }
  };
  return {{0, 0}, 0};
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "simulator");

  ros::NodeHandle n;

  ros::Publisher initial_pose_pub =
      n.advertise<geometry_msgs::Twist>("true_pose", 1);
  ros::Publisher scan_pub = n.advertise<sensor_msgs::LaserScan>("laser", 10);
  ros::Publisher odom_pub = n.advertise<geometry_msgs::Twist>("odom", 10);

  ros::Rate loop_rate(5);

  const util::Map map("src/particle_filter/maps/rectangle_small_bump.map");
  util::Pose current_pose({8, 0}, 0);

  while (ros::ok()) {
    const util::Pose desired_move = CommandEndToEnd(current_pose);
    const util::Pose executed_move = AddExecutionOdomNoise(desired_move);
    const util::Pose reported_move = AddReadingOdomNoise(executed_move);
    current_pose = geometry::FollowTrajectory(
        current_pose, executed_move.tra.x(), executed_move.rot);

    PublishTransforms(current_pose);
    scan_pub.publish(MakeScan(current_pose, map, kLaserReadingNoiseStddev));
    odom_pub.publish(reported_move.ToTwist());
    initial_pose_pub.publish(current_pose.ToTwist());
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}