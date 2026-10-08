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
