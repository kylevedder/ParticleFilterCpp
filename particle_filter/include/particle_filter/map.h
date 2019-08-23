#pragma once

#include "eigen3/Eigen/Core"
#include "eigen3/Eigen/Geometry"

#include "geometry.h"

#include <vector>

namespace util {

struct Wall {
  Eigen::Vector2f p1;
  Eigen::Vector2f p2;
  Wall() : p1(), p2(){};
  Wall(const Eigen::Vector2f& p1, const Eigen::Vector2f& p2) : p1(p1), p2(p2){};
};

struct Map {
  std::vector<Wall> walls;

  float MinDistanceAlongRay(const Eigen::Vector2f& ray_start,
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
      const float candidate_sq_distance =
          (res.second - ray_start).squaredNorm();
      if (candidate_sq_distance < sq_distance) {
        sq_distance = candidate_sq_distance;
      }

      NP_FINITE(sq_distance);
    }

    return sqrt(sq_distance);
  }
};

}  // namespace util