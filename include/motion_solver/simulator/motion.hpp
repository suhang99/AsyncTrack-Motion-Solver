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