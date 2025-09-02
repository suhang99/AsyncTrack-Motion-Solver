#include <filesystem>
#include <fstream>
#include <iostream>
#include "experiments/config.hpp"
#include "experiments/utils.hpp"

using std::cout;
using std::endl;

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
  printConfig(config);

  auto [mag_errors, axis_errors, linear_errors, accel_errors] = runTest(config);

  motion_solver::Scalar mean_mag_error;
  motion_solver::Scalar mean_axis_error;
  motion_solver::Scalar mean_linear_error;
  motion_solver::Scalar mean_accel_error;
  mean_mag_error = std::accumulate(mag_errors.begin(), mag_errors.end(), 0.0) /
                   static_cast<motion_solver::Scalar>(mag_errors.size());
  mean_axis_error = std::accumulate(axis_errors.begin(), axis_errors.end(), 0.0) /
                    static_cast<motion_solver::Scalar>(axis_errors.size());
  mean_linear_error = std::accumulate(linear_errors.begin(), linear_errors.end(), 0.0) /
                      static_cast<motion_solver::Scalar>(linear_errors.size());
  mean_accel_error = std::accumulate(accel_errors.begin(), accel_errors.end(), 0.0) /
                     static_cast<motion_solver::Scalar>(accel_errors.size());

  if (config.solver.solve_linear) {
    cout << "Mean linear error: " << mean_linear_error << endl;
    cout << "Mean acceleration error: " << mean_accel_error << endl;
  }
  if (config.solver.solve_angular) {
    cout << "Mean mag error: " << mean_mag_error << endl;
    cout << "Mean axis error: " << mean_axis_error << endl;
  }

  std::ofstream output_file(config.output_path);
  if (not output_file.is_open()) {
    std::cerr << "Error open file: " << config.output_path << std::endl;
  }

  output_file
      << "Linear Velocity Error, Accleration Error, Angular Magnitude Error, Angular Axis Error, "
      << std::endl;

  for (auto i = 0; i < config.rounds; ++i) {
    output_file << linear_errors[i] << "," << accel_errors[i] << "," << mag_errors[i] << ", "
                << axis_errors[i] << std::endl;
  }
  output_file.close();

  return 0;
}