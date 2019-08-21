#pragma once

#include "sensor_msgs/LaserScan.h"

namespace util {
class LaserScan {
 private:
  sensor_msgs::LaserScan ros_laser_scan_;

 public:
  LaserScan() = delete;
  explicit LaserScan(const sensor_msgs::LaserScan& ros_laser_scan)
      : ros_laser_scan_(ros_laser_scan) {}
};
}  // namespace util