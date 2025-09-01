#pragma once

#include <Eigen/Dense>
#include <utility>
#include "motion_solver/core/types.hpp"

namespace motion_solver::simulator {

class MotionBase {
 public:
  MotionBase() = default;
  virtual ~MotionBase() = default;
  virtual auto getPose(double timestamp) -> Pose = 0;
};

class LinearMotion : public MotionBase {
 public:
  LinearMotion() = default;
  ~LinearMotion() override = default;

  void setInitialPose(Pose pose) { T_world_body_0_ = std::move(pose); }
  void setLinearVelocity(Vec3 vel) { linear_vel_ = std::move(vel); }
  void setAcceleration(Vec3 accel) { acceleration_ = std::move(accel);}
  void setAngularVelocity(Eigen::AngleAxisd angular_vel) { angular_vel_ = std::move(angular_vel); }
  auto getPose(double time) -> Pose override;

 protected:
  Pose T_world_body_0_ = Pose::Identity();  // Initial body pose in world frame
  Vec3 linear_vel_ = Vec3::Zero();          // Velocity in body frame
  Vec3 acceleration_ = Vec3::Zero();        // Acceleration in body frame
  Eigen::AngleAxisd angular_vel_ = Eigen::AngleAxisd::Identity();  // Angular velocity in body frame
};

}  // namespace motion_solver::simulator