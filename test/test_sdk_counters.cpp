// Copyright 2026 PipeIn Development
// SPDX-License-Identifier: MIT
//
// The SDK's packet-loss counters (9B.2, H3): the XT32's parser (Udp6_1Parser, through
// GeneralParser) must start them at zero. The driver publishes total_loss_count_ on the loss
// topic; uninitialised, it reads whatever the memory held.
#include <gtest/gtest.h>

#include <cstdlib>
#include <cstring>
#include <new>

#include "udp6_1_parser.h"

using hesai::lidar::LidarPointXYZIRT;
using hesai::lidar::Udp6_1Parser;

TEST(SdkCounters, Udp6_1ParserStartsWithZeroedLossCounters) {
  using Parser = Udp6_1Parser<LidarPointXYZIRT>;
  void* memory = std::malloc(sizeof(Parser));
  ASSERT_NE(memory, nullptr);
  std::memset(memory, 0xFF, sizeof(Parser));  // what a reused heap block may hold
  auto* parser = new (memory) Parser();
  EXPECT_EQ(parser->total_loss_count_, 0u);
  EXPECT_EQ(parser->total_start_seqnum_, 0u);
  EXPECT_EQ(parser->current_seqnum_, 0u);
  EXPECT_EQ(parser->total_packet_count_, 0u);  // set at HEAD
  parser->~Parser();
  std::free(memory);
}

// H2 (9B.3): a packet sequence number that goes back (the lidar restarted, a packet came late)
// must not count about four billion lost packets: the unsigned difference underflowed. Only the
// real hole (53 after 51) counts.
TEST(SdkLossArithmetic, BackwardsSequenceDoesNotUnderflow) {
  hesai::lidar::HsLidarXTV1Tail tail{};
  uint32_t start = 0, last = 0, loss = 0, start_time = 0, total_loss = 0, total_start = 0;
  for (uint32_t seq : {100u, 101u, 102u, 50u, 51u, 53u}) {
    tail.m_u32SeqNum = seq;
    tail.CalPktLoss(start, last, loss, start_time, total_loss, total_start);
  }
  EXPECT_EQ(total_loss, 1u);
}
