#include "motion_solver/solver/optimizer.hpp"

#include <ceres/cost_function.h>
#include <ceres/iteration_callback.h>
#include <ceres/loss_function.h>
#include <ceres/problem.h>
#include <ceres/types.h>

#include "motion_solver/camera_model/pinhole_camera.hpp"
#include "motion_solver/ceres_helper/sphere_manifold.hpp"

namespace motion_solver::solver {

class Callback : public ceres::IterationCallback {
 public:
  explicit Callback(const std::array<double, 3>& velocity) : velocity_(velocity) {}

  auto operator()(const ceres::IterationSummary& summary) -> ceres::CallbackReturnType override {
    std::cout << "Iteration " << summary.iteration << " cost " << summary.cost << std::endl;
    Eigen::Vector3d vel(velocity_[0], velocity_[1], velocity_[2]);
    std::cout << "vel = " << vel.transpose() << std::endl;
    std::cout << "speed = " << vel.norm() << std::endl << std::endl;

    std::vector<double> residuals;

    return ceres::SOLVER_CONTINUE;
  }

 private:
  const std::array<double, 3>& velocity_;
};

auto runMotionOptimizer(const std::shared_ptr<motion_solver::camera_model::PinholeCamera>& camera,
                        const std::vector<std::vector<StampedPixel>>& all_pixels,
                        Eigen::Vector3d initial_velocity,
                        const std::vector<Eigen::Vector3d>& initial_landmarks)
    -> std::optional<Eigen::Vector3d> {
  ceres::Solver::Options options;
  // options.minimizer_progress_to_stdout = true;
  options.linear_solver_type = ceres::DENSE_QR;
  options.max_num_iterations = 100;
  ceres::Solver::Summary summary;
  ceres::Problem problem;

  auto num_landmarks = all_pixels.size();

  // Initialize parameters
  std::vector<Eigen::Vector3d> landmarks(num_landmarks, Eigen::Vector3d::Random());
  if (not initial_landmarks.empty()) {
    landmarks = initial_landmarks;
  }

  initial_velocity.normalize();
  std::array<double, 3> velocity{initial_velocity.x(), initial_velocity.y(), initial_velocity.z()};

  const auto& intrinsics = camera->getIntrinsics();
  auto* loss_function = new ceres::TrivialLoss();

  size_t landmark_index = 0;
  for (const auto& pixels : all_pixels) {
    for (const auto& pixel : pixels) {
      auto [t, point] = pixel;
      ceres::CostFunction* cost_function =
          ReprojectionError::create(intrinsics, point.x(), point.y(), t);
      problem.AddResidualBlock(cost_function, loss_function, velocity.data(),
                               landmarks[landmark_index].data());
    }
    ++landmark_index;
  }

  // Apply velocity unit norm constraint
#if CERES_VERSION_MAJOR >= 2 && CERES_VERSION_MINOR >= 2
  // For ceres version >= 2.2
  ceres::Manifold* unit_norm_manifold = new motion_solver::manifold::SphereManifold3d();
  problem.SetManifold(velocity.data(), unit_norm_manifold);
#else
  ceres::LocalParameterization* unit_norm_manifold =
      new motion_solver::manifold::SphereManifold3d();
  problem.SetParameterization(velocity.data(), unit_norm_manifold);
#endif

  // Callback iteration_callback(velocity);
  // options.callbacks.push_back(&iteration_callback);

  ceres::Solve(options, &problem, &summary);

  Eigen::Vector3d optimized_velocity(velocity[0], velocity[1], velocity[2]);
  if (landmarks.front().z() < 0) {
    optimized_velocity = -optimized_velocity;
  }
  optimized_velocity.normalize();
  if (summary.IsSolutionUsable()) {
    return optimized_velocity;
  }

  return std::nullopt;
}

}  // namespace motion_solver::solver