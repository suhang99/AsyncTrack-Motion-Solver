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
  std::vector<int> num_landmarks{5, 10, 20, 30, 50, 100};
  std::vector<int> num_measurement{5, 10, 20, 30, 50};
  std::vector<motion_solver::Scalar> time_noise_selection(11);
  for (auto i = 0; i < 11; ++i) {
    time_noise_selection[i] = i * 0.005;
  }

  std::ofstream output_file(config.output_path);
  if (not output_file.is_open()) {
    std::cerr << "Error open file: " << config.output_path << std::endl;
  }

  output_file << "#landmark, #measurement, time level, mean error" << std::endl;

  for (auto n_landmark : num_landmarks) {
    config.scene.num_landmarks = n_landmark;
    for (auto n_measurement : num_measurement) {
      config.scene.num_measurement = n_measurement;
      for (auto time_noise : time_noise_selection) {
        config.noise.timestamp_level = time_noise;
        auto [mag_errors, axis_errors, linear_errors, accel_errors] = runTest(config);
        for (const auto& error : linear_errors) {
          output_file << config.scene.num_landmarks << ", " << config.scene.num_measurement << ", "
                      << time_noise << ", " << error << std::endl;
        }
      }
    }
  }

  output_file.close();
  std::cout << "Finished!\n";

  return 0;
}