#pragma once

#include "motion_solver/core/types.hpp"

namespace motion_solver::simulator {

class Scene {
 public:
  Scene() = default;
  ~Scene() = default;

  void addPoint(const Vec3& point) { points_.push_back(point); }
  void addPoints(const std::vector<Vec3>& points) {
    points_.insert(points_.end(), points.begin(), points.end());
  }

 protected:
  std::vector<Vec3> points_;
};

}  // namespace motion_solver::simulator