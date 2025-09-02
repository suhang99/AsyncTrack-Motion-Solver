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

  std::ofstream output_file(config.output_path);
  if (not output_file.is_open()) {
    std::cerr << "Error open file: " << config.output_path << std::endl;
  }

  auto motion = std::make_shared<motion_solver::simulator::LinearMotion>();
  auto sampler = std::make_shared<motion_solver::simulator::Sampler>();
  motion_solver::camera_model::CameraConfig camera_config;
  camera_config.width = config.camera.width;
  camera_config.height = config.camera.height;
  camera_config.camera_name = "camera";
  camera_config.intrinsics = config.camera.intrinsics;
  camera_config.distortion = std::vector<double>{};
  camera_config.distortion_model = "none";
  auto camera = std::make_shared<motion_solver::Camera>(camera_config);

  if (config.random_seed != 0) {
    sampler->setRandomSeed(config.random_seed);
  }

  std::vector<double> running_time(config.rounds, -1);

  for (int round = 0; round < config.rounds; ++round) {
    // Sample velocity
    auto [gt_linear_vel, gt_accel, gt_angular_vel] = sampleMotion(config.motion, sampler);

    motion->setLinearVelocity(gt_linear_vel);
    motion->setAngularVelocity(gt_angular_vel);
    if (config.motion.order >= 2) {
      motion->setAcceleration(gt_accel);
    }

    auto landmarks = sampleLandmarks(config.scene, sampler);

    auto [all_stamped_pixels, original_bearings] =
        generateMeasurements(landmarks, camera, motion, config.scene, sampler);

    auto [all_noisy_pixels, all_stamped_bearings] =
        addMeasurementNoise(all_stamped_pixels, camera, config.noise, sampler);

    auto start = omp_get_wtime();
    auto est_motion =
        motion_solver::solver::solveLinearMotion(all_stamped_bearings, config.motion.order);
    auto end = omp_get_wtime();
    running_time[round] = end - start;
  }

  double total_time = 0;
  for (auto time : running_time) {
    total_time += time;
  }
  std::cout << "mean running time = " << total_time / config.rounds << "\n";

  output_file.close();
  std::cout << "Finished!\n";

  return 0;
}