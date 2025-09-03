// Copyright 2025 Hang Su, Yunlong Feng, Mobile Perception Lab
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#pragma once

#include <ceres/autodiff_cost_function.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <tuple>
#include <vector>

namespace motion_solver::solver {

using BearingVector = Eigen::Vector3d;
using StampedBearingVector = std::tuple<double, BearingVector>;

template <typename T>
auto skewSymmetric(const Eigen::Matrix<T, 3, 1> &vec) -> Eigen::Matrix<T, 3, 3> {
  Eigen::Matrix<T, 3, 3> skew;
  // clang-format off
  skew << T(0), -vec(2), vec(1), \
          vec(2), T(0), -vec(0), \
          -vec(1), vec(0), T(0);
  // clang-format on
  return skew;
}

struct AngularVelCost {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  explicit AngularVelCost(const std::vector<StampedBearingVector> &bearing_vectors)  // NOLINT
      : stamped_bearing_vectors_(bearing_vectors) {}

  template <typename T>
  auto operator()(const T *const angle_vel_T, const T *const axis_T,  // NOLINT
                  T *residual) const -> bool {
    using Vector3 = Eigen::Matrix<T, 3, 1>;
    using VectorX = Eigen::Matrix<T, Eigen::Dynamic, 1>;
    using Matrix3 = Eigen::Matrix<T, 3, 3>;
    using MatrixX = Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic>;

    Eigen::Map<Vector3 const> const axis(axis_T);
    Eigen::AngleAxis<T> angular_vel(angle_vel_T[0], axis);

    size_t cols = 6;
    size_t rows = 3 * stamped_bearing_vectors_.size();

    MatrixX data = MatrixX::Zero(rows, cols);

    for (std::size_t i = 0; i < stamped_bearing_vectors_.size(); ++i) {
      const auto &[t, bearing_vec] = stamped_bearing_vectors_[i];

      Matrix3 rot =
          Eigen::AngleAxis<T>(angular_vel.angle() * t, angular_vel.axis()).toRotationMatrix();

      Vector3 rotated_bearing_vec = rot * bearing_vec.cast<T>();

      auto skew = skewSymmetric(rotated_bearing_vec);
      data.template block<3, 3>(i * 3, 0) = skew;
      data.template block<3, 3>(i * 3, 3) = -t * skew;
    }

    MatrixX Mat = data.transpose() * data;

    // compute nuclear norm
    Eigen::JacobiSVD<MatrixX> svd(Mat);
    VectorX singular_values = svd.singularValues();

    residual[0] = singular_values.sum() * singular_values(5);
    // residual[0] = singular_values(5);

    return true;
  }

  static auto create(const std::vector<StampedBearingVector> &bearing_vectors)
      -> ceres::CostFunction * {
    return new ceres::AutoDiffCostFunction<AngularVelCost, 1, 1, 3>(
        new AngularVelCost(bearing_vectors));
  }

  const std::vector<StampedBearingVector> stamped_bearing_vectors_;
};

auto solveAngularVel(const std::vector<std::vector<StampedBearingVector>> &input_data,
                     Eigen::AngleAxisd &angular_vel, bool is_planar_motion = false,
                     double angle_lower_bound = -2 * M_PI, double angle_upper_bound = 2 * M_PI)
    -> bool;

auto solveLinearVelTwoStages(const std::vector<std::vector<StampedBearingVector>> &input_data)
    -> Eigen::Vector3d;

auto solveLinearMotion(const std::vector<std::vector<StampedBearingVector>> &input_data,
                       size_t motion_order = 1) -> std::vector<Eigen::Vector3d>;

auto solvePointGivenMotion(const std::vector<StampedBearingVector> &input_data,
                           const std::vector<Eigen::Vector3d> &motion) -> Eigen::Vector3d;

}  // namespace motion_solver::solver
