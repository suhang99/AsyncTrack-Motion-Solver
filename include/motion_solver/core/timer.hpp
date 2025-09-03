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

#include <chrono>
#include <iostream>
#include <string>
#include <unordered_map>
#include <vector>

namespace motion_solver {

class ScopedTimer {
 public:
  using Clock = std::chrono::high_resolution_clock;
  using TimeUnit = std::chrono::milliseconds;

  explicit ScopedTimer(std::string label = "", bool verbose = false)
      : start_(Clock::now()), label_(std::move(label)), verbose_(verbose) {};
  ~ScopedTimer() {
    auto end = Clock::now();
    auto duration = std::chrono::duration_cast<TimeUnit>(end - start_);
    if (verbose_) {
      std::cout << label_ << " took " << duration.count() << " ms" << std::endl;
    }
  }

 private:
  std::chrono::time_point<Clock> start_;
  std::string label_;
  bool verbose_;
};

class Timer {
 public:
  using Clock = std::chrono::high_resolution_clock;
  using TimeUnit = std::chrono::milliseconds;

  Timer() = default;
  explicit Timer(std::string label) : label_(std::move(label)) {}
  ~Timer() = default;

  void start() { start_ = Clock::now(); }
  void stop() {
    auto end = Clock::now();
    auto duration = std::chrono::duration_cast<TimeUnit>(end - start_);
    records_.emplace_back(duration.count());
  }
  auto getRecords() -> const std::vector<double>& { return records_; }

 private:
  std::string label_;
  std::chrono::time_point<Clock> start_;
  std::vector<double> records_;
};

class Profiler {
 public:
  using Clock = std::chrono::high_resolution_clock;

  Profiler() = default;
  ~Profiler() = default;

  void start(const std::string& label) {
    if (timers_.find(label) == timers_.end()) {
      timers_[label] = Timer(label);
    }
    timers_[label].start();
  }

  void stop(const std::string& label) { timers_[label].stop(); }

  void reset() { timers_.clear(); }

  void report() {
    for (auto& [label, timer] : timers_) {
      double mean_time = 0;
      for (const auto& record : timer.getRecords()) {
        mean_time += record;
      }
      mean_time /= static_cast<double>(timer.getRecords().size());
      std::cout << label << " took " << mean_time << " ms" << std::endl;
    }
  }

 private:
  std::unordered_map<std::string, Timer> timers_;
};

}  // namespace motion_solver