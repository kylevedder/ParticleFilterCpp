#include "particle_filter/map.h"

#include <fstream>
#include <sstream>

namespace util {

Map::Map(const std::string& filepath) {
  std::fstream map_file(filepath);
  if (!map_file.is_open()) {
    std::cerr << "Failed to open " << filepath << std::endl;
  }
  CHECK(map_file.is_open());

  std::string line;
  while (std::getline(map_file, line)) {
    float x1 = 0;
    float y1 = 0;
    float x2 = 0;
    float y2 = 0;
    std::istringstream iss(line);
    if (!(iss >> x1 >> y1 >> x2 >> y2)) {
      break;
    }
    walls.push_back({{x1, y1}, {x2, y2}});
  }
}

float Map::MinDistanceAlongRay(const Eigen::Vector2f& ray_start,
                               const Eigen::Vector2f& ray_end) const {
  NP_FINITE(ray_start.x());
  NP_FINITE(ray_start.y());
  NP_FINITE(ray_end.x());
  NP_FINITE(ray_end.y());
  NP_CHECK(ray_start != ray_end);
  float sq_distance = (ray_end - ray_start).squaredNorm();
  NP_FINITE(sq_distance);
  for (const Wall& w : walls) {
    const auto res =
        geometry::CheckLineLineIntersection(w.p1, w.p2, ray_start, ray_end);
    if (!res.first) {
      continue;
    }
    const float candidate_sq_distance = (res.second - ray_start).squaredNorm();
    if (candidate_sq_distance < sq_distance) {
      sq_distance = candidate_sq_distance;
    }

    NP_FINITE(sq_distance);
  }

  return sqrt(sq_distance);
}

visualization_msgs::Marker Map::ToMarker() const {
  visualization_msgs::Marker marker;
  marker.header.frame_id = "map";
  marker.header.stamp = ros::Time();
  marker.ns = "map";
  marker.id = 0;
  marker.type = visualization_msgs::Marker::LINE_LIST;
  marker.action = visualization_msgs::Marker::ADD;

  for (const Wall& w : walls) {
    geometry_msgs::Point p1;
    p1.x = w.p1.x();
    p1.y = w.p1.y();
    geometry_msgs::Point p2;
    p2.x = w.p2.x();
    p2.y = w.p2.y();
    marker.points.push_back(p1);
    marker.points.push_back(p2);
  }

  marker.scale.x = 0.02;
  marker.scale.y = 0.1;
  marker.scale.z = 0.1;
  marker.color.a = 1;
  marker.color.r = 1;
  marker.color.g = 0;
  marker.color.b = 0;
  return marker;
}

}  // namespace util
