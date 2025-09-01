#pragma once

#include <Eigen/Core>
#include <string>

namespace motion_solver::camera_model {

struct CameraConfig {
  size_t width;
  size_t height;
  std::string camera_name;
  std::array<double, 4> intrinsics;
  std::vector<double> distortion;
  std::string distortion_model;
};

class CameraBase {
 public:
  CameraBase() = default;

  CameraBase(size_t width, size_t height, std::string camera_name = "")
      : width_(width), height_(height), camera_name_(std::move(camera_name)) {}

  virtual ~CameraBase() = default;

  [[nodiscard]] virtual auto getWidth() const -> size_t { return width_; }

  [[nodiscard]] virtual auto getHeight() const -> size_t { return height_; }

  [[nodiscard]] virtual auto getName() const -> std::string { return camera_name_; }

  [[nodiscard]] virtual auto project(const Eigen::Vector3d &point) const -> Eigen::Vector2d = 0;

  [[nodiscard]] virtual auto unproject(const Eigen::Vector2d &point) const -> Eigen::Vector3d = 0;

  [[nodiscard]] virtual auto unprojectUnitSphere(const Eigen::Vector2d &point) const
      -> Eigen::Vector3d {
    return unproject(point).normalized();
  }

  [[nodiscard]] virtual auto projectJacobian(const Eigen::Vector3d &point) const
      -> Eigen::Matrix<double, 2, 3> = 0;

  [[nodiscard]] virtual auto unprojectJacobian(const Eigen::Vector3d &point) const
      -> Eigen::Matrix<double, 3, 2> = 0;

  [[nodiscard]] virtual auto unprojectUnitSphereJacobian(const Eigen::Vector3d &point) const
      -> Eigen::Matrix<double, 3, 2> = 0;

  [[nodiscard]] virtual auto undistort(const Eigen::Vector2d &point) const -> Eigen::Vector2d {
    return point;
  }

  [[nodiscard]] virtual auto distort(const Eigen::Vector2d &point) const -> Eigen::Vector2d {
    return point;
  }

  [[nodiscard]] virtual auto undistortJacobian(const Eigen::Vector2d &point) const
      -> Eigen::Matrix<double, 2, 2> {
    (void)point;
    return Eigen::Matrix<double, 2, 2>::Identity();
  }

  [[nodiscard]] virtual auto distortJacobian(const Eigen::Vector2d &point) const
      -> Eigen::Matrix<double, 2, 2> {
    (void)point;
    return Eigen::Matrix<double, 2, 2>::Identity();
  }

  [[nodiscard]] virtual auto isInFrame(const Eigen::Vector2d &point) const -> bool {
    return point.x() >= 0 && point.x() < static_cast<double>(width_) && point.y() >= 0 &&
           point.y() < static_cast<double>(height_);
  }

 protected:
  size_t width_;
  size_t height_;
  std::string camera_name_;
};

}  // namespace motion_solver::camera_model