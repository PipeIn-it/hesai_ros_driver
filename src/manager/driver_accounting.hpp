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

/// Whether the packet-loss topic is advertised and fed (H2): only with the SDK's packet-loss
/// tool on. Without it the SDK never counts, and the topic carried a packet count frozen at 0.
struct LossTopicPolicy {
  static bool ShouldAdvertise(bool loss_tool_enabled, const std::string& topic) {
    return loss_tool_enabled && topic != NULL_TOPIC;
  }
};

/// The frames the driver published and skipped, and the SDK's packet totals (H2), fed with each
/// published frame's SDK frame_index. The SDK counts every frame it completes in frame_index,
/// also the ones it drops (at most kMinPointsOfOneFrame points, or failed to start), so the gaps
/// between the published indices are the frames skipped (J.33).
/// Thread-safe: the SDK's frame thread feeds it, the status thread reads.
class FrameAccounting {
 public:
  /// Counts a published frame and returns the frames skipped right before it. The first frame
  /// only sets the baseline (its index may be above 0, J.33); an index that goes back (the SDK
  /// counts again) sets a new one. Called from one thread (the SDK's frame thread).
  uint64_t OnPublished(int frame_index) {
    published_.fetch_add(1, std::memory_order_relaxed);
    uint64_t gap = 0;
    if (have_last_ && frame_index > last_index_) {
      gap = static_cast<uint64_t>(static_cast<int64_t>(frame_index) - last_index_ - 1);
    }
    have_last_ = true;
    last_index_ = frame_index;
    if (gap > 0) skipped_.fetch_add(gap, std::memory_order_relaxed);
    return gap;
  }
  uint64_t published() const { return published_.load(std::memory_order_relaxed); }
  uint64_t skipped() const { return skipped_.load(std::memory_order_relaxed); }

  /// The SDK's cumulative packet totals, as its loss callback reports them with each published
  /// frame (only with the packet-loss tool on).
  void OnPacketTotals(uint32_t total, uint32_t lost) {
    packets_total_.store(total, std::memory_order_relaxed);
    packets_lost_.store(lost, std::memory_order_relaxed);
    have_packet_totals_.store(true, std::memory_order_release);
  }
  bool have_packet_totals() const { return have_packet_totals_.load(std::memory_order_acquire); }
  uint32_t packets_total() const { return packets_total_.load(std::memory_order_relaxed); }
  uint32_t packets_lost() const { return packets_lost_.load(std::memory_order_relaxed); }

 private:
  std::atomic<uint64_t> published_{0};
  std::atomic<uint64_t> skipped_{0};
  bool have_last_ = false;   // frame thread only
  int64_t last_index_ = 0;   // frame thread only
  std::atomic<uint32_t> packets_total_{0};
  std::atomic<uint32_t> packets_lost_{0};
  std::atomic<bool> have_packet_totals_{false};
};

/// The frame and packet counts for the driver's 1 Hz /diagnostics entry. The packet totals are
/// "n/a" until the SDK reports them (never without the packet-loss tool).
inline void AppendFrameAccounting(const FrameAccounting& accounting,
                                  std::vector<std::pair<std::string, std::string>>* values) {
  values->emplace_back("frames_published", std::to_string(accounting.published()));
  values->emplace_back("frames_skipped", std::to_string(accounting.skipped()));
  const bool known = accounting.have_packet_totals();
  values->emplace_back("packets_total", known ? std::to_string(accounting.packets_total()) : "n/a");
  values->emplace_back("packets_lost", known ? std::to_string(accounting.packets_lost()) : "n/a");
}

}  // namespace hesai_ros_driver
