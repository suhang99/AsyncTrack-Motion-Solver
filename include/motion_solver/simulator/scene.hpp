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

#include "motion_solver/core/types.hpp"

namespace motion_solver::simulator {

class Scene {
 public:
  Scene() = default;
  ~Scene() = default;

  void addPoint(const Vec3& point) { points_.push_back(point); }
  void addPoints(const std::vector<Vec3>& points) {
    points_.insert(points_.end(), points.begin(), points.end());
  }

 protected:
  std::vector<Vec3> points_;
};

}  // namespace motion_solver::simulator