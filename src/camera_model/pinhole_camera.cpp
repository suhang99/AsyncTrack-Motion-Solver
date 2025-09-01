#include <motion_solver/camera_model/pinhole_camera.hpp>

#include <Eigen/Geometry>

namespace motion_solver::camera_model {

auto PinholeCamera::project(const Eigen::Vector3d &point) const -> Eigen::Vector2d {
  return (K_ * point).hnormalized();
}

auto PinholeCamera::unproject(const Eigen::Vector2d &point) const -> Eigen::Vector3d {
  return (K_inv_ * point.homogeneous());
}

auto PinholeCamera::unprojectUnitSphere(const Eigen::Vector2d &point) const -> Eigen::Vector3d {
  return (K_inv_ * point.homogeneous()).normalized();
}

auto PinholeCamera::projectJacobian(const Eigen::Vector3d &point) const
    -> Eigen::Matrix<double, 2, 3> {
  (void)point;
  Eigen::Matrix<double, 2, 3> jacobian;
  jacobian << intrinsics_[0], 0, intrinsics_[2], 0, intrinsics_[1], intrinsics_[3];
  return jacobian;
}

auto PinholeCamera::unprojectJacobian(const Eigen::Vector3d &point) const
    -> Eigen::Matrix<double, 3, 2> {
  (void)point;
  Eigen::Matrix<double, 3, 2> jacobian;
  jacobian << 1.0 / intrinsics_[0], 0, 0, 1.0 / intrinsics_[1], 0, 0;
  return jacobian;
}

auto PinholeCamera::unprojectUnitSphereJacobian(const Eigen::Vector3d &point) const
    -> Eigen::Matrix<double, 3, 2> {
  (void)point;
  Eigen::Matrix<double, 3, 2> jacobian;
  jacobian << 1.0 / intrinsics_[0], 0, 0, 1.0 / intrinsics_[1], 0, 0;
  return jacobian;
}

}  // namespace motion_solver::camera_model