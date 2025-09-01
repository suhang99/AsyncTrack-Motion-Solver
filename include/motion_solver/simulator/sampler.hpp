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
