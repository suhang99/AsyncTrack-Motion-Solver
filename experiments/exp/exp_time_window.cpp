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
  std::vector<double> time_windows;
  for (auto i = 1; i < 11; ++i) {
    time_windows.emplace_back(i * 0.1);
  }

  std::ofstream output_file(config.output_path);
  if (not output_file.is_open()) {
    std::cerr << "Error open file: " << config.output_path << std::endl;
  }

  output_file << "#landmark, #measurement, time_window, mean error" << std::endl;
  for (auto time_window : time_windows) {
    config.scene.timestamp_upper = time_window;
    auto [mag_errors, axis_errors, linear_errors, accel_errors] = motion_solver::runTest(config);
    for (const auto& error : linear_errors) {
      output_file << std::format("{}, {}, {}, {}\n", config.scene.num_landmarks,
                                 config.scene.num_measurement, time_window, error);
    }
  }

  output_file.close();
  std::cout << "Finished!\n";

  return 0;
}