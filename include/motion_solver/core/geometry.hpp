#pragma once

#include <algorithm>
#include <cmath>
#include "motion_solver/core/types.hpp"

namespace motion_solver {

inline auto rad2deg(Scalar rad) -> Scalar { return rad * 180.0 / M_PI; }

inline auto deg2rad(Scalar deg) -> Scalar { return deg * M_PI / 180.0; }

inline auto vectorAngle(const Vec3& vec1, const Vec3& vec2) -> Scalar {
  return rad2deg(std::acos(std::clamp(vec1.normalized().dot(vec2.normalized()), -1.0, 1.0)));
}

inline auto vectorIncludedAngle(const Vec3& vec1, const Vec3& vec2) -> Scalar {
  auto angle = rad2deg(std::acos(std::clamp(vec1.normalized().dot(vec2.normalized()), -1.0, 1.0)));
  return std::min(angle, 180.0 - angle);
}

inline auto pointLineDistance(const Vec3& point, const Vec3& line_point,
                              const Vec3& line_dir) -> Scalar {
  return (point - line_point).cross(line_dir).norm() / line_dir.norm();
}

}  // namespace motion_solver