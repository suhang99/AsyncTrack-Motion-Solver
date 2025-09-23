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

#include "experiments/generate_points.hpp"

using motion_solver::solver::BearingVector;
using motion_solver::solver::StampedBearingVector;

auto generatePointsFromMotion(
    const Eigen::AngleAxisd &angular_vel, const Eigen::Vector3d &linear_vel,
    const std::vector<std::tuple<Eigen::Vector3d, std::vector<double>>> &points_3d,
    const Eigen::Isometry3d &initial_pose) -> std::vector<std::vector<StampedBearingVector>> {
  assert(points_3d.size() > 1);

  std::vector<std::vector<StampedBearingVector>> stamped_bv;
  for (const auto &point_3d : points_3d) {
    std::vector<StampedBearingVector> stamped_bv_i;
    stamped_bv_i.reserve(std::get<1>(point_3d).size());
    for (const auto &timestamp : std::get<1>(point_3d)) {
      Eigen::Matrix3d delta_rotation =
          Eigen::AngleAxisd(angular_vel.angle() * timestamp, angular_vel.axis()).toRotationMatrix();
      Eigen::Vector3d delta_trans = linear_vel * timestamp;

      Eigen::Isometry3d delta_pose = Eigen::Isometry3d::Identity();
      delta_pose.linear() = delta_rotation;
      delta_pose.translation() = delta_trans;

      Eigen::Isometry3d cur_pose = initial_pose * delta_pose;
      cur_pose = cur_pose.inverse();

      stamped_bv_i.emplace_back(
          timestamp, cur_pose.rotation() * std::get<0>(point_3d) + cur_pose.translation());
    }
    stamped_bv.emplace_back(stamped_bv_i);
  }

  return stamped_bv;
}

auto generatePoints(std::tuple<double, double> angular_vel_range,
                    std::tuple<double, double> linear_vel_range,
                    std::tuple<double, double> timestamp_range, double point_radius,
                    size_t track_count, size_t track_length)
    -> std::tuple<std::vector<std::vector<motion_solver::solver::StampedBearingVector>>,
                  Eigen::AngleAxisd, Eigen::Vector3d> {
  using std::uniform_real_distribution;
  std::random_device random_device;
  std::mt19937 gen(random_device());
  uniform_real_distribution<double> dis_unit(-1.0, 1.0);
  uniform_real_distribution<double> dis_angular_vel(std::get<0>(angular_vel_range),
                                                    std::get<1>(angular_vel_range));
  uniform_real_distribution<double> dis_linear_vel(std::get<0>(linear_vel_range),
                                                   std::get<1>(linear_vel_range));
  uniform_real_distribution<double> dis_timestamp(std::get<0>(timestamp_range),
                                                  std::get<1>(timestamp_range));

  Eigen::AngleAxisd angular_vel(
      dis_angular_vel(gen),
      Eigen::Vector3d(dis_unit(gen), dis_unit(gen), dis_unit(gen)).normalized());
  Eigen::Vector3d linear_vel(dis_linear_vel(gen), dis_linear_vel(gen), dis_linear_vel(gen));

  std::vector<std::tuple<Eigen::Vector3d, std::vector<double>>> points_3d_timestamps;
  points_3d_timestamps.reserve(track_count);
  for (size_t i = 0; i < track_count; ++i) {
    Eigen::Vector3d point(dis_unit(gen) * point_radius, dis_unit(gen) * point_radius,
                          dis_unit(gen) * point_radius);
    std::vector<double> timestamps;
    timestamps.reserve(track_length);
    for (size_t j = 0; j < track_length; ++j) {
      timestamps.push_back(dis_timestamp(gen));
    }
    std::sort(timestamps.begin(), timestamps.end());
    points_3d_timestamps.emplace_back(point, timestamps);
  }

  return {generatePointsFromMotion(angular_vel, linear_vel, points_3d_timestamps), angular_vel,
          linear_vel};
}