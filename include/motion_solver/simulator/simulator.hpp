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

#include <memory>
#include <utility>
#include "motion_solver/camera_model/pinhole_camera.hpp"
#include "motion_solver/simulator/motion.hpp"
#include "motion_solver/simulator/sampler.hpp"
#include "motion_solver/simulator/scene.hpp"

namespace motion_solver::simulator {

class Simulator {
 public:
  Simulator() = default;
  ~Simulator() = default;

  void setMotion(std::shared_ptr<motion_solver::simulator::MotionBase> motion) {
    motion_ = std::move(motion);
  }

  void setScene(std::shared_ptr<motion_solver::simulator::Scene> scene) {
    scene_ = std::move(scene);
  }

 private:
  std::shared_ptr<motion_solver::simulator::MotionBase> motion_;
  std::shared_ptr<motion_solver::simulator::Scene> scene_;
};

}  // namespace motion_solver::simulator