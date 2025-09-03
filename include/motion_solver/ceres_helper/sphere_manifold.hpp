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

#include <ceres/ceres.h>

namespace motion_solver::manifold {

#if CERES_VERSION_MAJOR >= 2 && CERES_VERSION_MINOR >= 2
// For ceres version >= 2.2
#include <ceres/sphere_manifold.h>
using SphereManifold3d = ceres::SphereManifold<3>;
#else
#include <ceres/local_parameterization.h>
class SphereManifold3d : public ceres::LocalParameterization {
 public:
  SphereManifold3d() = default;

  ~SphereManifold3d() override = default;

  auto Plus(const double *x_0, const double *delta, double *x_plus_delta) const -> bool override;

  auto ComputeJacobian(const double *x_0, double *jacobian) const -> bool override;

  [[nodiscard]] auto GlobalSize() const -> int override { return 3; }

  [[nodiscard]] auto LocalSize() const -> int override { return 3; }
};  // class SphereManifold3d
#endif

}  // namespace motion_solver::manifold
