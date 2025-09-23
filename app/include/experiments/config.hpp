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

#include <yaml-cpp/yaml.h>
#include <array>
#include "motion_solver/core/types.hpp"

namespace motion_solver {

struct Config {
  int rounds;
  std::string output_path;
  int random_seed;
  bool verbose;
  struct NoiseConfig {
    bool enable_pixel;
    bool enable_timestamp;
    bool enable_rotation;
    Scalar pixel_level;
    Scalar timestamp_level;
    bool rounding;
  } noise;
  struct SceneConfig {
    int num_landmarks;
    int num_measurement;
    std::array<Scalar, 3> landmark_center;
    Scalar landmark_bbox;
    Scalar timestamp_lower, timestamp_upper;
  } scene;
  struct CameraConfig {
    int width, height;
    std::array<Scalar, 4> intrinsics;
    std::vector<double> distortion;
  } camera;
  struct MotionConfig {
    Scalar angular_lower, angular_upper;
    Scalar linear_lower, linear_upper;
    Scalar accel_lower, accel_upper;
    size_t order;
  } motion;
  struct SolverConfig {
    bool solve_angular, solve_linear, solve_two_stage;
    bool linear_preconditioner;
  } solver;
};

inline auto loadConfig(const std::string& path) -> Config {
  YAML::Node param = YAML::LoadFile(path);
  // clang-format off
  Config config{
    .rounds = param["rounds"].as<int>(100),
    .output_path = param["output_path"].as<std::string>("output.txt"),
    .random_seed = param["random_seed"].as<int>(0),
    .verbose = param["verbose"].as<bool>(false),
    .noise = {
      .enable_pixel = param["noise"]["pixel"]["enable"].as<bool>(false),
      .enable_timestamp = param["noise"]["timestamp"]["enable"].as<bool>(false),
      .enable_rotation = param["noise"]["rotation"]["enable"].as<bool>(false),
      .pixel_level = param["noise"]["pixel"]["level"].as<Scalar>(),
      .timestamp_level = param["noise"]["timestamp"]["level"].as<Scalar>(),
      .rounding = param["noise"]["rounding"].as<bool>(false),
    },
    .scene = {
      .num_landmarks = param["scene"]["num_landmarks"].as<int>(10),
      .num_measurement = param["scene"]["num_measurement"].as<int>(10),
      .landmark_center = param["scene"]["landmark_center"].as<std::array<Scalar, 3>>(),
      .landmark_bbox = param["scene"]["landmark_bbox"].as<Scalar>(1),
      .timestamp_lower = param["scene"]["timestamp_range"][0].as<Scalar>(-1.0),
      .timestamp_upper = param["scene"]["timestamp_range"][1].as<Scalar>(1.0),
    },
    .camera = {
      .width = param["camera"]["width"].as<int>(640),
      .height = param["camera"]["height"].as<int>(480),
      .intrinsics = param["camera"]["intrinsics"].as<std::array<Scalar, 4>>(),
      .distortion = param["camera"]["distortion"].as<std::vector<double>>(),
    },
    .motion = {
      .angular_lower = param["motion"]["angular_vel_range"][0].as<Scalar>(-1.0),
      .angular_upper = param["motion"]["angular_vel_range"][1].as<Scalar>(1.0),
      .linear_lower = param["motion"]["linear_vel_range"][0].as<Scalar>(-1.0),
      .linear_upper = param["motion"]["linear_vel_range"][1].as<Scalar>(1.0),
      .accel_lower = param["motion"]["acceleration_range"][0].as<Scalar>(0.0),
      .accel_upper = param["motion"]["acceleration_range"][1].as<Scalar>(0.0),
      .order = param["motion"]["order"].as<size_t>(1),
    },
    .solver = {
      .solve_angular = param["solver"]["solve_angular"].as<bool>(false),
      .solve_linear = param["solver"]["solve_linear"].as<bool>(false),
      .solve_two_stage = param["solver"]["solve_two_stage"].as<bool>(false),
      .linear_preconditioner = param["solver"]["linear_preconditioner"].as<bool>(false),
    }
  };
  // clang-format on
  return config;
}

}  // namespace motion_solver