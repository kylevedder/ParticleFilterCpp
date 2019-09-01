#include "geometry_msgs/Twist.h"
#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"

#include "eigen3/Eigen/Core"
#include "eigen3/Eigen/Geometry"

#include <signal.h>
#include <cmath>
#include "particle_filter/crash_handling.h"
#include "particle_filter/geometry.h"
#include "particle_filter/math_util.h"
#include "particle_filter/particle_filter.h"
#include "particle_filter/visualization.h"

#include <visualization_msgs/MarkerArray.h>
#include <fstream>

void DrawGroundTruth(const util::Pose& ground_truth,
                     ros::Publisher* ground_truth_pub) {
  visualization_msgs::MarkerArray arr;
  visualization::DrawPose(ground_truth, "map", "ground_truth", 0, 1, 0, 1,
                          &arr);
  ground_truth_pub->publish(arr);
}

struct ParticleFilterWrapper {
  localization::ParticleFilter particle_filter;
  util::Pose ground_truth;
  ros::Publisher particle_pub;
  ros::Publisher ground_truth_pub;
  ros::Publisher sampled_laser_pub;

  static constexpr auto kErrorFile = "error.csv";

  ParticleFilterWrapper() = delete;
  ParticleFilterWrapper(const util::Map& map, ros::NodeHandle* n)
      : particle_filter(map) {
    particle_pub =
        n->advertise<visualization_msgs::MarkerArray>("particles", 10);
    ground_truth_pub =
        n->advertise<visualization_msgs::MarkerArray>("ground_truth", 10);
    sampled_laser_pub =
        n->advertise<sensor_msgs::LaserScan>("sampled_laser", 100);
    std::ofstream out(kErrorFile, std::fstream::out);
    out << "max_error_x,max_error_y,max_error_norm,max_error_theta,cent_error_"
           "x,cent_error_y,cent_error_norm,cent_error_theta\n";
    out.close();
  }

  void StartCallback(const geometry_msgs::Twist::ConstPtr& msg) {
    ground_truth = util::Pose(*msg);
    DrawGroundTruth(ground_truth, &ground_truth_pub);
    if (particle_filter.IsInitialized()) {
      return;
    }
    particle_filter.InitalizePose(ground_truth);
  }

  void WriteError(const util::Pose& max_estimate_error,
                  const util::Pose& weighted_centroid_error) const {
    std::ofstream out(kErrorFile, std::fstream::out | std::fstream::app);
    out << max_estimate_error.tra.x() << ", ";
    out << max_estimate_error.tra.y() << ", ";
    out << max_estimate_error.tra.norm() << ", ";
    out << max_estimate_error.rot << ", ";
    out << weighted_centroid_error.tra.x() << ", ";
    out << weighted_centroid_error.tra.y() << ", ";
    out << weighted_centroid_error.tra.norm() << ", ";
    out << weighted_centroid_error.rot << "\n";
    out.close();
  }

  void LaserCallback(const sensor_msgs::LaserScan::ConstPtr& msg) {
    util::LaserScan laser(*msg);
    particle_filter.UpdateObservation(laser, &sampled_laser_pub);
    particle_filter.DrawParticles(&particle_pub);
    const util::Pose max_estimate = particle_filter.MaxWeight();
    const util::Pose weighted_centroid = particle_filter.WeightedCentroid();
    const util::Pose max_estimate_error = (max_estimate - ground_truth);
    const util::Pose weighted_centroid_error =
        (weighted_centroid - ground_truth);
    WriteError(max_estimate_error, weighted_centroid_error);
    static int iteration_count = 0;
    ++iteration_count;
    if (iteration_count >= 200) {
      exit(0);
    }
  }

  void OdomCallback(const geometry_msgs::Twist::ConstPtr& msg) {
    util::Pose odom(*msg);
    particle_filter.UpdateOdom(odom.tra.x(), odom.rot);
    particle_filter.DrawParticles(&particle_pub);
  }
};

int main(int argc, char** argv) {
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
      util::Map(
          "/home/k/code/catkin_ws/src/particle_filter/maps/rectangle.map"),
      &n);

  ros::Subscriber initial_pose_sub = n.subscribe(
      "true_pose", 1000, &ParticleFilterWrapper::StartCallback, &wrapper);
  ros::Subscriber laser_sub = n.subscribe(
      "laser", 1000, &ParticleFilterWrapper::LaserCallback, &wrapper);
  ros::Subscriber odom_sub =
      n.subscribe("odom", 1000, &ParticleFilterWrapper::OdomCallback, &wrapper);

  ros::spin();

  return 0;
}
