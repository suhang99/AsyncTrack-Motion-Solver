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

#include <cmath>
#include <iomanip>
#include <iostream>
#include <limits>
#include <opencv2/opencv.hpp>
#include "experiments/config.hpp"
#include "motion_solver/core/types.hpp"
#include "motion_solver/motion_solver.hpp"

namespace motion_solver {

using Camera = motion_solver::camera_model::PinholeCamera;

const auto kNaN = std::numeric_limits<Scalar>::quiet_NaN();

inline void validateIncidence(
    const std::vector<std::vector<motion_solver::solver::StampedBearingVector>>& input_data,
    const std::vector<Vec3>& landmarks, const Vec3& gt_vel) {
  std::cout << "Incidence validation" << std::endl;

  for (auto i = 0UL; i < input_data.size(); ++i) {
    const auto& landmark = landmarks[i];
    auto data_size = input_data[i].size();
    Eigen::VectorXd unknowns(6);
    unknowns.head(3) = landmark;
    unknowns.tail(3) = gt_vel;
    Eigen::MatrixXd data_matrix(3 * data_size, 6);
    for (auto j = 0UL; j < data_size; ++j) {
      const auto& [t, bearing] = input_data[i][j];
      data_matrix.block<3, 3>(static_cast<Eigen::Index>(3 * j), 0) =
          motion_solver::solver::skewSymmetric(bearing);
      data_matrix.block<3, 3>(static_cast<Eigen::Index>(3 * j), 3) =
          -t * motion_solver::solver::skewSymmetric(bearing);
    }
    std::cout << (data_matrix * unknowns).norm() << " ";
  }
  std::cout << std::endl;
}

inline void printConfig(const Config& config) {
  std::cout << "--------------------" << std::endl;
  std::cout << "#rounds = " << config.rounds << std::endl;
  if (config.noise.enable_pixel) {
    std::cout << "Pixel noise level: " << config.noise.pixel_level << std::endl;
  }
  if (config.noise.enable_timestamp) {
    std::cout << "Timestamp noise level: " << config.noise.timestamp_level << std::endl;
  }
  if (config.noise.rounding) {
    std::cout << "Pixel rounding error" << std::endl;
  }
  std::cout << "#Landmarks = " << config.scene.num_landmarks
            << "\t #Measurements = " << config.scene.num_measurement << std::endl;
  std::cout << "Angular vel range: [" << config.motion.angular_lower << ","
            << config.motion.angular_upper << "]" << std::endl;
  std::cout << "Linear vel range: [" << config.motion.linear_lower << ","
            << config.motion.linear_upper << "]" << std::endl;
  std::cout << "Acceleration range: [" << config.motion.accel_lower << ","
            << config.motion.accel_upper << "]" << std::endl;
  std::cout << "--------------------" << std::endl;
}

inline auto sampleMotion(Config::MotionConfig config,
                         const std::shared_ptr<simulator::Sampler>& sampler) -> Twist {
  // Sample velocity
  auto linear_lower = config.linear_lower;
  auto linear_upper = config.linear_upper;
  auto angular_lower = config.angular_lower;
  auto angular_upper = config.angular_upper;
  auto accel_lower = config.accel_lower;
  auto accel_upper = config.accel_upper;

  auto gt_linear_vel = sampler->sampleVec<3>(linear_lower, linear_upper);
  auto gt_angular_vel = Eigen::AngleAxisd(sampler->sampleScalar(angular_lower, angular_upper),
                                          sampler->sampleUnitVec<3>());
  auto gt_accel = sampler->sampleVec<3>(accel_lower, accel_upper);
  return std::make_tuple(gt_linear_vel, gt_accel, gt_angular_vel);
}

inline auto sampleLandmarks(Config::SceneConfig config,
                            const std::shared_ptr<simulator::Sampler>& sampler)
    -> std::vector<Vec3> {
  // Sample landmarks
  auto center = config.landmark_center;
  auto bbox_size = config.landmark_bbox;
  auto num_landmarks = config.num_landmarks;
  Vec3 landmark_center(center[0], center[1], center[2]);
  std::vector<Vec3> landmarks;
  landmarks.reserve(num_landmarks);
  for (auto i = 0; i < num_landmarks; i++) {
    Vec3 shift = sampler->sampleUnitCube<3>() * bbox_size;
    landmarks.emplace_back(landmark_center + shift);
  }
  return landmarks;
}

inline auto generateMeasurements(const std::vector<Vec3>& landmarks,
                                 const std::shared_ptr<Camera>& camera,
                                 const std::shared_ptr<simulator::LinearMotion>& motion,
                                 Config::SceneConfig config,
                                 const std::shared_ptr<simulator::Sampler>& sampler)
    -> std::tuple<std::vector<std::vector<StampedPixel>>,
                  std::vector<std::vector<motion_solver::solver::StampedBearingVector>>> {
  std::vector<std::vector<StampedPixel>> all_stamped_pixels;
  std::vector<std::vector<motion_solver::solver::StampedBearingVector>> original_bearings;
  all_stamped_pixels.reserve(landmarks.size());
  original_bearings.reserve(landmarks.size());

  const auto t_upper = config.timestamp_upper;
  const auto t_lower = config.timestamp_lower;

  for (const auto& P_world_landmark : landmarks) {
    std::vector<StampedPixel> stamped_pixels;
    std::vector<motion_solver::solver::StampedBearingVector> stamped_bearings;
    for (int j = 0; j < config.num_measurement; ++j) {
      Scalar timestamp = sampler->sampleScalar(t_lower, t_upper);
      auto T_world_cam = motion->getPose(timestamp);
      auto T_cam_world = T_world_cam.inverse();
      Vec3 bearing = (T_cam_world.matrix() * P_world_landmark.homogeneous()).hnormalized();
      auto pixel = camera->project(bearing);
      stamped_pixels.emplace_back(timestamp, pixel);
      stamped_bearings.emplace_back(timestamp, bearing);
    }
    all_stamped_pixels.push_back(stamped_pixels);
    original_bearings.push_back(stamped_bearings);
  }

  return std::make_tuple(all_stamped_pixels, original_bearings);
}

inline auto addMeasurementNoise(std::vector<std::vector<StampedPixel>> all_stamped_pixels,
                                const std::shared_ptr<Camera>& camera, Config::NoiseConfig config,
                                const std::shared_ptr<simulator::Sampler>& sampler)
    -> std::tuple<std::vector<std::vector<StampedPixel>>,
                  std::vector<std::vector<motion_solver::solver::StampedBearingVector>>> {
  std::vector<std::vector<StampedPixel>> all_noisy_pixels = std::move(all_stamped_pixels);
  std::vector<std::vector<motion_solver::solver::StampedBearingVector>> all_stamped_bearings;
  all_stamped_bearings.reserve(all_noisy_pixels.size());

  // Add noise to measurements
  if (config.enable_pixel) {
    for (auto& stamped_pixels : all_noisy_pixels) {
      for (auto& [t, pixel] : stamped_pixels) {
        pixel += sampler->sampleUnitVec<2>() * config.pixel_level;
      }
    }
  }
  if (config.enable_timestamp) {
    for (auto& stamped_pixels : all_noisy_pixels) {
      for (auto& [t, pixel] : stamped_pixels) {
        t += sampler->sampleSignedUnitScalar() * config.timestamp_level;
      }
    }
  }

  if (config.rounding) {
    for (auto& stamped_pixels : all_noisy_pixels) {
      for (auto& [t, pixel] : stamped_pixels) {
        pixel = pixel.array().round();
      }
    }
  }

  // Convert measurements to bearing vectors by unprojecting
  for (const auto& stamped_pixels : all_noisy_pixels) {
    std::vector<motion_solver::solver::StampedBearingVector> stamped_bearings;
    for (const auto& [t, pixel] : stamped_pixels) {
      auto bearing = camera->unproject(pixel).normalized();
      stamped_bearings.emplace_back(t, bearing);
    }
    all_stamped_bearings.push_back(stamped_bearings);
  }

  return {all_noisy_pixels, all_stamped_bearings};
}

inline auto runTest(const Config& config) -> std::tuple<std::vector<Scalar>, std::vector<Scalar>,
                                                        std::vector<Scalar>, std::vector<Scalar>> {
  std::vector<Scalar> angular_vel_magitude_errors(config.rounds, kNaN);
  std::vector<Scalar> angular_vel_axis_errors(config.rounds, kNaN);
  std::vector<Scalar> linear_vel_errors(config.rounds, kNaN);
  std::vector<Scalar> acceleration_errors(config.rounds, kNaN);

  auto motion = std::make_shared<simulator::LinearMotion>();
  auto sampler = std::make_shared<simulator::Sampler>();
  motion_solver::camera_model::CameraConfig camera_config;
  camera_config.width = config.camera.width;
  camera_config.height = config.camera.height;
  camera_config.camera_name = "camera";
  camera_config.intrinsics = config.camera.intrinsics;
  camera_config.distortion = std::vector<double>{};
  camera_config.distortion_model = "none";
  auto camera = std::make_shared<Camera>(camera_config);

  if (config.random_seed != 0) {
    sampler->setRandomSeed(config.random_seed);
  }

  if (config.verbose) {
    std::cout << "Round\tLinear\tAccel\tAng(mag,axis)\tGT Vel\t\t\tGT Angular" << std::endl;
  }

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

    // Run solver
    Eigen::AngleAxisd est_angular_vel{0.0, Eigen::Vector3d::UnitX()};
    double magnitude_error = std::numeric_limits<double>::quiet_NaN();
    double axis_angle_error = std::numeric_limits<double>::quiet_NaN();
    if (config.solver.solve_angular) {
      auto success = motion_solver::solver::solveAngularVel(all_stamped_bearings, est_angular_vel);
      if (success) {
        magnitude_error = std::abs(est_angular_vel.angle() - gt_angular_vel.angle()) * 180.0 / M_PI;
        axis_angle_error = vectorAngle(gt_angular_vel.axis(), est_angular_vel.axis());
        angular_vel_magitude_errors[round] = magnitude_error;
        angular_vel_axis_errors[round] = axis_angle_error;
      }
    }

    double vel_error = std::numeric_limits<double>::quiet_NaN();
    double accel_error = std::numeric_limits<double>::quiet_NaN();
    if (config.solver.solve_linear) {
      Eigen::AngleAxisd angular_vel;
      if (config.solver.solve_two_stage and config.solver.solve_angular) {
        angular_vel = est_angular_vel;
      } else {
        angular_vel = gt_angular_vel;
      }

      // Rotation compensation for all bearing vectors
      if (!config.noise.enable_rotation) {
        for (auto& stamped_bearings : all_stamped_bearings) {
          for (auto& [t, bearing] : stamped_bearings) {
            auto Rotation =
                Eigen::AngleAxisd(angular_vel.angle() * t, angular_vel.axis()).toRotationMatrix();
            bearing = Rotation * bearing;
          }
        }
      }

      auto est_motion =
          motion_solver::solver::solveLinearMotion(all_stamped_bearings, config.motion.order);
      Eigen::Vector3d est_linear_vel = Eigen::Vector3d::Zero();
      Eigen::Vector3d est_acceleration = Eigen::Vector3d::Zero();
      est_linear_vel = est_motion.at(0);
      if (config.motion.order >= 2) {
        est_acceleration = est_motion.at(1);
      }

      // auto opt_result = solver::runMotionOptimizer(camera, all_noisy_pixels);
      // if (opt_result.has_value()) {
      //   double error = vectorAngle(gt_linear_vel, *opt_result);
      //   std::cout << "opt error = " << error << std::endl;
      //   std::cout << "solver error = " << vectorAngle(gt_linear_vel, est_linear_vel) <<
      //   std::endl;
      // } else {
      //   std::cout << "Fail to optimize" << std::endl;
      // }

      vel_error = vectorAngle(gt_linear_vel, est_linear_vel);
      accel_error = vectorAngle(gt_accel, est_acceleration);
      linear_vel_errors[round] = vel_error;
      acceleration_errors[round] = accel_error;
    }
    if (config.verbose) {
      std::cout << std::fixed << std::setprecision(2);
      std::cout << round << "\t";
      std::cout << vel_error << "\t";
      std::cout << accel_error << "\t";
      std::cout << magnitude_error << "\t";
      std::cout << axis_angle_error << "\t";
      std::cout << std::endl;
    }
  }

  return std::make_tuple(angular_vel_magitude_errors, angular_vel_axis_errors, linear_vel_errors,
                         acceleration_errors);
}

}  // namespace motion_solver