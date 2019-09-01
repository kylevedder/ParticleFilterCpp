#include <rosbag/bag.h>
#include <rosbag/view.h>

#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>

#include "particle_filter/constants.h"
#include "particle_filter/particle_filter.h"
#include "particle_filter/pose.h"

struct ParticleFilterWrapper {
  localization::ParticleFilter particle_filter;
  util::Pose ground_truth;

  static constexpr auto kErrorFile = "error.csv";

  ParticleFilterWrapper() = delete;
  explicit ParticleFilterWrapper(const util::Map& map) : particle_filter(map) {
    std::ofstream out(kErrorFile, std::fstream::out);
    out << "max_error_x,max_error_y,max_error_norm,max_error_theta,cent_error_"
           "x,cent_error_y,cent_error_norm,cent_error_theta\n";
    out.close();
  }

  void StartCallback(const geometry_msgs::Twist::ConstPtr& msg) {
    ground_truth = util::Pose(*msg);
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
    particle_filter.UpdateObservation(laser, nullptr);
    const util::Pose max_estimate = particle_filter.MaxWeight();
    const util::Pose weighted_centroid = particle_filter.WeightedCentroid();
    const util::Pose max_estimate_error = (max_estimate - ground_truth);
    const util::Pose weighted_centroid_error =
        (weighted_centroid - ground_truth);
    WriteError(max_estimate_error, weighted_centroid_error);
  }

  void OdomCallback(const geometry_msgs::Twist::ConstPtr& msg) {
    util::Pose odom(*msg);
    particle_filter.UpdateOdom(odom.tra.x(), odom.rot);
  }
};

std::string GetBagName(int argc, char** argv) {
  if (argc != 2) {
    std::cerr << "Usage: " << argv[0] << " [rosbag.bag]" << std::endl;
    exit(-1);
  }
  return {argv[1]};
}

int main(int argc, char** argv) {
  ParticleFilterWrapper wrapper(util::Map(
      "/home/k/code/catkin_ws/src/particle_filter/maps/rectangle.map"));

  const std::string bag_name = GetBagName(argc, argv);
  rosbag::Bag bag(bag_name, rosbag::bagmode::Read);

  for (rosbag::MessageInstance const& m : rosbag::View(bag)) {
    if (m.getTopic() == "true_pose") {
      geometry_msgs::Twist::ConstPtr msg =
          m.instantiate<geometry_msgs::Twist>();
      NP_CHECK(msg != nullptr);
      wrapper.StartCallback(msg);
    } else if (m.getTopic() == "odom") {
      geometry_msgs::Twist::ConstPtr msg =
          m.instantiate<geometry_msgs::Twist>();
      NP_CHECK(msg != nullptr);
      wrapper.OdomCallback(msg);
    } else if (m.getTopic() == "laser") {
      sensor_msgs::LaserScan::ConstPtr msg =
          m.instantiate<sensor_msgs::LaserScan>();
      NP_CHECK(msg != nullptr);
      wrapper.LaserCallback(msg);
    }
  }

  return 0;
}
