#include "motion_solver/simulator/motion.hpp"

namespace motion_solver::simulator {

auto LinearMotion::getPose(double time) -> Pose {
  // Compute the body pose in world frame at time t
  Mat3 Rotation_body_0_body_t =
      Eigen::AngleAxisd(angular_vel_.angle() * time, angular_vel_.axis()).toRotationMatrix();
  Vec3 Translation_body_0_body_t = linear_vel_ * time + 0.5 * acceleration_ * time * time;

  Pose T_body_0_body_t = Pose::Identity();
  T_body_0_body_t.prerotate(Rotation_body_0_body_t);
  T_body_0_body_t.pretranslate(Translation_body_0_body_t);

  auto T_world_body_t = T_world_body_0_ * T_body_0_body_t;

  return T_world_body_t;
}

}  // namespace motion_solver::simulator