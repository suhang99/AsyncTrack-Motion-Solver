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