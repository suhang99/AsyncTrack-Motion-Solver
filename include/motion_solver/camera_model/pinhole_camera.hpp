#pragma once

#include <Eigen/Core>

#include <motion_solver/camera_model/camera_base.hpp>

namespace motion_solver::camera_model {

class PinholeCamera : public CameraBase {
 public:
  PinholeCamera() = default;

  ~PinholeCamera() override = default;

  explicit PinholeCamera(const CameraConfig &config)
      : CameraBase(config.width, config.height, config.camera_name),
        intrinsics_(config.intrinsics),
        distortion_(config.distortion) {
    K_ = getIntrinsicMatrix();
    K_inv_ = K_.inverse();
  }

  [[nodiscard]] auto getIntrinsics() const -> std::array<double, 4> { return intrinsics_; }

  [[nodiscard]] auto getDistortion() const -> std::vector<double> { return distortion_; }

  [[nodiscard]] auto project(const Eigen::Vector3d &point) const -> Eigen::Vector2d override;

  [[nodiscard]] auto unproject(const Eigen::Vector2d &point) const -> Eigen::Vector3d override;

  [[nodiscard]] auto unprojectUnitSphere(const Eigen::Vector2d &point) const
      -> Eigen::Vector3d override;

  [[nodiscard]] auto projectJacobian(const Eigen::Vector3d &point) const
      -> Eigen::Matrix<double, 2, 3> override;

  [[nodiscard]] auto unprojectJacobian(const Eigen::Vector3d &point) const
      -> Eigen::Matrix<double, 3, 2> override;

  [[nodiscard]] auto unprojectUnitSphereJacobian(const Eigen::Vector3d &point) const
      -> Eigen::Matrix<double, 3, 2> override;

  // TODO(Ericsii): Add distortion model

 private:
  [[nodiscard]] inline auto getIntrinsicMatrix() const -> Eigen::Matrix3d {
    Eigen::Matrix3d intrinsic_matrix = Eigen::Matrix3d::Identity();
    intrinsic_matrix(0, 0) = intrinsics_[0];
    intrinsic_matrix(1, 1) = intrinsics_[1];
    intrinsic_matrix(0, 2) = intrinsics_[2];
    intrinsic_matrix(1, 2) = intrinsics_[3];
    return intrinsic_matrix;
  }

  std::array<double, 4> intrinsics_{0.0, 0.0, 0.0, 0.0};  // fx fy cx cy
  std::vector<double> distortion_;
  Eigen::Matrix3d K_{Eigen::Matrix3d::Identity()}, K_inv_{Eigen::Matrix3d::Identity()};
  // TODO(Ericsii): Add distortion model
};  // class PinholeCamera

}  // namespace motion_solver::camera_model