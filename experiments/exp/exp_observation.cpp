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

#include <filesystem>
#include <fstream>
#include <iostream>
#include "experiments/config.hpp"
#include "experiments/utils.hpp"

auto main(int argc, char** argv) -> int {
  if (argc != 2) {
    std::cerr << "Usage: " << argv[0] << " [param.yaml]" << std::endl;
  }

  std::filesystem::path param_file(argv[1]);
  if (!std::filesystem::exists(param_file)) {
    std::cerr << "File not found: " << param_file << std::endl;
    return 1;
  }

  if (std::string{param_file.extension()}.find("yaml") == std::string::npos) {
    std::cerr << "Invalid file extension: " << std::string{param_file.extension()} << std::endl;
    return 1;
  }

  auto config = motion_solver::loadConfig(param_file.string());
  std::vector<int> num_landmarks{3, 4, 5, 10, 20, 30, 50, 100, 200, 1000};
  std::vector<int> num_measurement{3, 4, 5, 10, 15, 20, 25, 30, 40, 50};

  std::ofstream output_file(config.output_path);
  if (not output_file.is_open()) {
    std::cerr << "Error open file: " << config.output_path << std::endl;
  }

  output_file << "#landmark, #measurement, mean error" << std::endl;

  config.scene.num_measurement = 5;
  for (auto n_landmark : num_landmarks) {
    config.scene.num_landmarks = n_landmark;
    auto [mag_errors, axis_errors, linear_errors, accel_errors] = motion_solver::runTest(config);
    for (const auto& error : linear_errors) {
      output_file << config.scene.num_landmarks << ", " << config.scene.num_measurement << ", "
                  << error << std::endl;
    }
  }

  config.scene.num_landmarks = 5;
  for (auto n_measurement : num_measurement) {
    config.scene.num_measurement = n_measurement;
    auto [mag_errors, axis_errors, linear_errors, accel_errors] = motion_solver::runTest(config);
    for (const auto& error : linear_errors) {
      output_file << config.scene.num_landmarks << ", " << config.scene.num_measurement << ", "
                  << error << std::endl;
    }
  }

  output_file.close();
  std::cout << "Finished!\n";

  return 0;
}