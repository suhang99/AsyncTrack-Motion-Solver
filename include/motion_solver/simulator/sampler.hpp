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

#include <random>
#include "motion_solver/core/types.hpp"

namespace motion_solver::simulator {

class Sampler {
 public:
  Sampler() = default;
  ~Sampler() = default;

  using Distribution = std::uniform_real_distribution<Scalar>;

  void setRandomSeed(int seed) { gen_.seed(seed); }

  auto sampleSignedUnitScalar() -> Scalar {
    Distribution dist(-1, 1);
    return dist(gen_);
  }

  auto sampleUnitScalar() -> Scalar {
    Distribution dist(0, 1);
    return dist(gen_);
  }

  auto sampleScalar(Scalar lower_bound, Scalar upper_bound) -> Scalar {
    Distribution dist(lower_bound, upper_bound);
    return dist(gen_);
  }

  template <int Dim>
  inline auto sampleVec(Scalar lower_bound, Scalar upper_bound) -> Eigen::Matrix<Scalar, Dim, 1> {
    auto dist = Distribution(lower_bound, upper_bound);
    return Eigen::Matrix<Scalar, Dim, 1>::NullaryExpr([this, &dist]() { return dist(gen_); });
  }

  template <int Dim>
  inline auto sampleUnitVec() -> Eigen::Matrix<Scalar, Dim, 1> {
    // TODO(Su Hang): Sample uniformly from the unit sphere
    return sampleUnitCube<Dim>().normalized();
  }

  template <int Dim>
  inline auto sampleUnitCube() -> Eigen::Matrix<Scalar, Dim, 1> {
    Eigen::Matrix<Scalar, Dim, 1> vec;
    Distribution dist(-1, 1);
    for (int i = 0; i < Dim; ++i) {
      vec(i) = dist(gen_);
    }
    return vec;
  }

 private:
  std::random_device rd_;
  std::mt19937 gen_{rd_()};
};

}  // namespace motion_solver::simulator
