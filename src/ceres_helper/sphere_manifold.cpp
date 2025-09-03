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

#include <motion_solver/ceres_helper/sphere_manifold.hpp>

#if CERES_VERSION_MAJOR < 2 || CERES_VERSION_MINOR < 2

namespace motion_solver::manifold {

auto SphereManifold3d::Plus(const double *x_0, const double *delta,  // NOLINT
                            double *x_plus_delta) const -> bool {
  Eigen::Map<const Eigen::Vector3d> x_vec(x_0);
  Eigen::Map<const Eigen::Vector3d> delta_vec(delta);
  Eigen::Map<Eigen::Vector3d> x_plus_delta_vec(x_plus_delta);

  Eigen::Vector3d temp = x_vec + delta_vec;
  x_plus_delta_vec = temp.normalized();
  return true;
}

auto SphereManifold3d::ComputeJacobian(const double *x_0, double *jacobian) const -> bool {
  Eigen::Map<const Eigen::Vector3d> x_vec(x_0);
  Eigen::Map<Eigen::Matrix<double, 3, 3, Eigen::RowMajor>> jacobian_mat(jacobian);
  jacobian_mat = Eigen::Matrix3d::Identity() - x_vec * x_vec.transpose() / x_vec.norm();
  return true;
}

}  // namespace motion_solver::manifold

#endif