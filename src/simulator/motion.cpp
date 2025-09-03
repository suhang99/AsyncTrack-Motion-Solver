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