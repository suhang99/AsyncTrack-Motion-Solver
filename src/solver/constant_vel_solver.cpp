#include <omp.h>
#include <Eigen/Eigen>
#include <motion_solver/ceres_helper/sphere_manifold.hpp>
#include <motion_solver/solver/constant_vel_solver.hpp>
#include "motion_solver/core/geometry.hpp"

namespace motion_solver::solver {

auto solveAngularVel(const std::vector<std::vector<StampedBearingVector>> &input_data,
                     Eigen::AngleAxisd &angular_vel, bool is_planar_motion,
                     double angle_lower_bound, double angle_upper_bound) -> bool {
  ceres::Problem problem;
  ceres::Solver::Options options;

  problem.AddParameterBlock(&angular_vel.angle(), 1);
  problem.AddParameterBlock(angular_vel.axis().data(), 3, new manifold::SphereManifold3d());
  problem.SetParameterLowerBound(&angular_vel.angle(), 0, angle_lower_bound);
  problem.SetParameterUpperBound(&angular_vel.angle(), 0, angle_upper_bound);

  if (is_planar_motion) {
    problem.SetParameterBlockConstant(angular_vel.axis().data());
  }

  for (const auto &track : input_data) {
    ceres::CostFunction *cost_func = AngularVelCost::create(track);
    problem.AddResidualBlock(cost_func, nullptr, &angular_vel.angle(), angular_vel.axis().data());
  }

  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);

  if (angular_vel.angle() < 0) {
    angular_vel.angle() = -angular_vel.angle();
    angular_vel.axis() = -angular_vel.axis();
  }

  return summary.IsSolutionUsable();
}

auto solveLinearMotion(const std::vector<std::vector<StampedBearingVector>> &input_data,
                       size_t motion_order) -> std::vector<Eigen::Vector3d> {
  const auto motion_degrees = motion_order * 3;
  // Build data matrix with velocity and acceleration
  auto cols = static_cast<Eigen::Index>(motion_degrees + input_data.size() * 3);
  Eigen::Index rows = 0;
  for (const auto &track : input_data) {
    rows += static_cast<Eigen::Index>(track.size() * 3);
  }

  Eigen::MatrixXd data_matrix(rows, cols);
  data_matrix.setZero();
  size_t measurement_count = 0;
  std::vector<double> motion_coefficients(motion_order + 1, 1);
  for (size_t i = 1; i <= motion_order; ++i) {
    motion_coefficients[i] = 1.0 / (motion_coefficients[i - 1] * static_cast<double>(i));
  }

  for (size_t i = 0; i < input_data.size(); ++i) {
    for (const auto &[t, bearing_vec] : input_data[i]) {
      auto skew_matrix = skewSymmetric(bearing_vec);
      data_matrix.block<3, 3>(static_cast<Eigen::Index>(measurement_count * 3),
                              static_cast<Eigen::Index>(i * 3)) = skew_matrix;
      for (size_t order = 1; order <= motion_order; ++order) {
        data_matrix.block<3, 3>(static_cast<Eigen::Index>(measurement_count * 3),
                                static_cast<Eigen::Index>(cols - (motion_order - order + 1) * 3)) =
            -motion_coefficients[order] * std::pow(t, order) * skew_matrix;
      }
      measurement_count++;
    }
  }

  // Apply Schur complement
  Eigen::SparseMatrix<double> data_sparse = data_matrix.sparseView();
  Eigen::SparseMatrix<double> M_sparse = data_sparse.transpose() * data_sparse;
  Eigen::SparseMatrix<double> A_sparse =
      M_sparse.topLeftCorner(M_sparse.rows() - motion_degrees, M_sparse.cols() - motion_degrees);
  Eigen::SparseMatrix<double> B_sparse =
      M_sparse.topRightCorner(M_sparse.rows() - motion_degrees, motion_degrees);
  Eigen::SparseMatrix<double> D_sparse = M_sparse.bottomRightCorner(motion_degrees, motion_degrees);

  // Matrix inversion for block diagonal matrix
  Eigen::MatrixXd A_inv(A_sparse.rows(), A_sparse.cols());
  A_inv.setZero();
#pragma omp parallel for
  for (int block_idx = 0; block_idx < A_sparse.rows() / 3; ++block_idx) {
    Eigen::Index start_row = static_cast<Eigen::Index>(block_idx) * 3;
    A_inv.block<3, 3>(start_row, start_row) =
        A_sparse.block(start_row, start_row, 3, 3).toDense().inverse();
  }

  Eigen::SparseMatrix<double> S_sparse = D_sparse - B_sparse.transpose() * A_inv * B_sparse;

  Eigen::JacobiSVD<Eigen::MatrixXd> svd(S_sparse.toDense(), Eigen::ComputeFullV);
  Eigen::VectorXd motion_params = svd.matrixV().col(static_cast<Eigen::Index>(motion_degrees - 1));

  // Solve the first point to determine whether the motion is pointing in the right direction
  std::vector<Eigen::Vector3d> motions;
  motions.reserve(motion_order);
  for (size_t i = 0; i < motion_order; ++i) {
    motions.emplace_back(motion_params.segment(static_cast<Eigen::Index>(i * 3), 3));
  }
  Eigen::Vector3d vec_P1 = solvePointGivenMotion(input_data.front(), motions);
  if (vec_P1.z() < 0) {
    for (auto &motion : motions) {
      motion = -motion;
    }
  }
  return motions;
}

auto solvePointGivenMotion(const std::vector<StampedBearingVector> &input_data,
                           const std::vector<Eigen::Vector3d> &motion) -> Eigen::Vector3d {
  Eigen::MatrixXd mat_F =
      Eigen::MatrixXd::Zero(static_cast<Eigen::Index>(3 * input_data.size()), 3);
  Eigen::MatrixXd mat_G = Eigen::MatrixXd::Zero(static_cast<Eigen::Index>(3 * input_data.size()),
                                                static_cast<Eigen::Index>(3 * motion.size()));
  std::vector<double> motion_coefficients(motion.size() + 1, 1);
  for (size_t i = 1; i <= motion.size(); ++i) {
    motion_coefficients[i] = 1.0 / (motion_coefficients[i - 1] * static_cast<double>(i));
  }
  for (size_t i = 0; i < input_data.size(); ++i) {
    const auto &[t, bearing_vec] = input_data[i];
    Eigen::Matrix3d f_skew = skewSymmetric(bearing_vec);
    mat_F.block<3, 3>(static_cast<Eigen::Index>(3 * i), 0) = f_skew;
    for (size_t j = 0; j < motion.size(); ++j) {
      mat_G.block<3, 3>(static_cast<Eigen::Index>(3 * i), static_cast<Eigen::Index>(3 * j)) =
          -motion_coefficients[j + 1] * std::pow(t, j + 1) * f_skew;
    }
  }
  Eigen::VectorXd motion_vector(motion.size() * 3);
  for (size_t i = 0; i < motion.size(); ++i) {
    motion_vector.segment(static_cast<Eigen::Index>(i * 3), 3) = motion[i];
  }
  return -(mat_F.transpose() * mat_F).inverse() * mat_F.transpose() * mat_G * motion_vector;
}

}  // namespace motion_solver::solver