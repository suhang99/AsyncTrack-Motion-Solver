#pragma once

#include <ceres/ceres.h>

#include <cstdio>
#include <optional>

#include "motion_solver/camera_model/pinhole_camera.hpp"
#include "motion_solver/core/types.hpp"

namespace motion_solver::solver {

struct ReprojectionError {
  ReprojectionError(const std::array<double, 4>& intrinsics, double pixel_x, double pixel_y,
                    double pixel_t)
      : intrinsics(intrinsics), x(pixel_x), y(pixel_y), t(pixel_t) {}

  static auto create(const std::array<double, 4>& intrinsics, double pixel_x, double pixel_y,
                     double pixel_t) -> ceres::CostFunction* {
    return new ceres::AutoDiffCostFunction<ReprojectionError, 2, 3, 3>(
        new ReprojectionError(intrinsics, pixel_x, pixel_y, pixel_t));
  }

  template <typename T>
  auto operator()(const T* const velocity, const T* const point, T* residuals) const -> bool {
    // Get camera position at time t
    std::array<T, 3> world_T_cam;
    world_T_cam[0] = velocity[0] * t;
    world_T_cam[1] = velocity[1] * t;
    world_T_cam[2] = velocity[2] * t;

    // Transform point to camera frame
    std::array<T, 3> cam_T_point;
    cam_T_point[0] = point[0] - world_T_cam[0];
    cam_T_point[1] = point[1] - world_T_cam[1];
    cam_T_point[2] = point[2] - world_T_cam[2];

    // Project point to image plane
    std::array<T, 2> pixel;
    pixel[0] = cam_T_point[0] / cam_T_point[2] * intrinsics[0] + intrinsics[2];
    pixel[1] = cam_T_point[1] / cam_T_point[2] * intrinsics[1] + intrinsics[3];

    residuals[0] = T(x) - pixel[0];
    residuals[1] = T(y) - pixel[1];

    return true;
  }

  std::array<double, 4> intrinsics;
  double x, y, t;
};

auto runMotionOptimizer(const std::shared_ptr<motion_solver::camera_model::PinholeCamera>& camera,
                        const std::vector<std::vector<motion_solver::StampedPixel>>& all_pixels,
                        Eigen::Vector3d initial_velocity = Eigen::Vector3d::Random(),
                        const std::vector<Eigen::Vector3d>& initial_landmarks = {})
    -> std::optional<Eigen::Vector3d>;

}  // namespace motion_solver::solver