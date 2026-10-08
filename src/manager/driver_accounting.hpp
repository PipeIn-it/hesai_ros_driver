// Copyright 2026 PipeIn Development
// SPDX-License-Identifier: MIT
//
// What the driver tells about its packets and frames (9B): whether the packet-loss topic is
// worth advertising, and the frames the SDK completed but never handed on. ROS-free, so the
// rules are testable without a lidar.
#pragma once

#include <atomic>
#include <cstdint>
#include <string>
#include <utility>
#include <vector>

#include "driver_param.h"  // NULL_TOPIC

namespace hesai_ros_driver {

/// Whether the packet-loss topic is advertised and fed (H2).
struct LossTopicPolicy {
  static bool ShouldAdvertise(bool /*loss_tool_enabled*/, const std::string& topic) {
    return topic != NULL_TOPIC;
  }
};

/// The frames the driver published, fed with each published frame's SDK frame_index (H2).
/// Thread-safe: the SDK's thread counts, the status thread reads.
class FrameAccounting {
 public:
  void OnPublished(int /*frame_index*/) { published_.fetch_add(1, std::memory_order_relaxed); }
  uint64_t published() const { return published_.load(std::memory_order_relaxed); }
  uint64_t skipped() const { return skipped_.load(std::memory_order_relaxed); }

 private:
  std::atomic<uint64_t> published_{0};
  std::atomic<uint64_t> skipped_{0};
};

/// The frame counts for the driver's /diagnostics entry. Nothing yet.
inline void AppendFrameAccounting(const FrameAccounting& /*accounting*/,
                                  std::vector<std::pair<std::string, std::string>>* /*values*/) {}

}  // namespace hesai_ros_driver
