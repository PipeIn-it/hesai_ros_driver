// Copyright 2026 PipeIn Development
// SPDX-License-Identifier: MIT
//
// The driver's packet and frame accounting (9B): when the packet-loss topic is advertised, and
// the frames the SDK completed but never handed on.
#include <gtest/gtest.h>

#include <string>
#include <utility>
#include <vector>

#include "driver_accounting.hpp"

using hesai_ros_driver::FrameAccounting;
using hesai_ros_driver::LossTopicPolicy;

TEST(LossTopicPolicy, NoTopicIsNeverAdvertised) {
  EXPECT_FALSE(LossTopicPolicy::ShouldAdvertise(true, NULL_TOPIC));
  EXPECT_FALSE(LossTopicPolicy::ShouldAdvertise(false, NULL_TOPIC));
}

TEST(LossTopicPolicy, TopicWithTheToolIsAdvertised) {
  EXPECT_TRUE(LossTopicPolicy::ShouldAdvertise(true, "/lidar_packets_loss"));
}

TEST(FrameAccounting, CountsThePublishedFrames) {
  FrameAccounting accounting;
  for (int index = 1; index <= 3; ++index) accounting.OnPublished(index);
  EXPECT_EQ(accounting.published(), 3u);
}

// H2 (9B.3): the SDK computes the loss counts only with its packet-loss tool on. Without it the
// topic carried a packet count frozen at 0 and a loss count nothing had set (H3): it must not
// be advertised.
TEST(LossTopicPolicy, NotAdvertisedWhenToolDisabled) {
  EXPECT_FALSE(LossTopicPolicy::ShouldAdvertise(false, "/lidar_packets_loss"));
}

// H2 (9B.3): the SDK drops a frame of at most 1,000 points (kMinPointsOfOneFrame) and a frame it
// failed to start, and counts each in frame_index: the gaps between the published frames are the
// frames skipped (small or failed, J.33).
TEST(FrameAccounting, SkippedSmallFramesAreCountedFromFrameIndexGaps) {
  FrameAccounting accounting;
  accounting.OnPublished(10);
  accounting.OnPublished(11);
  accounting.OnPublished(15);
  EXPECT_EQ(accounting.published(), 3u);
  EXPECT_EQ(accounting.skipped(), 3u);
}

// J.33: the first frame after start-up only sets the baseline (its index may be above 0).
TEST(FrameAccounting, FirstFrameEstablishesBaseline) {
  FrameAccounting accounting;
  accounting.OnPublished(7);
  EXPECT_EQ(accounting.skipped(), 0u);
  accounting.OnPublished(8);
  EXPECT_EQ(accounting.skipped(), 0u);
}

// A frame index that goes back (the SDK started counting again) sets a new baseline: never a
// negative or a four-billion gap.
TEST(FrameAccounting, IndexGoingBackSetsANewBaseline) {
  FrameAccounting accounting;
  for (int index : {100, 101, 5, 6}) accounting.OnPublished(index);
  EXPECT_EQ(accounting.skipped(), 0u);
}

// 9B.3: the driver's 1 Hz /diagnostics entry carries the frames published and skipped and the
// SDK's packet totals.
TEST(FrameAccounting, ReportsOnDiagnostics) {
  FrameAccounting accounting;
  accounting.OnPublished(1);
  accounting.OnPublished(3);
  accounting.OnPacketTotals(1000, 7);
  std::vector<std::pair<std::string, std::string>> values;
  hesai_ros_driver::AppendFrameAccounting(accounting, &values);
  auto value = [&values](const std::string& key) {
    for (const auto& kv : values) {
      if (kv.first == key) return kv.second;
    }
    return std::string("<missing>");
  };
  EXPECT_EQ(value("frames_published"), "2");
  EXPECT_EQ(value("frames_skipped"), "1");
  EXPECT_EQ(value("packets_total"), "1000");
  EXPECT_EQ(value("packets_lost"), "7");
}
