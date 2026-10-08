// Copyright 2026 PipeIn Development
// SPDX-License-Identifier: MIT
//
// The CUDA node's frame loop (HesaiLidarSdkGpu::Run in the SDK fork), its step for a packet
// inside a frame, on the CPU: the loop itself needs a GPU and a lidar, so the step lives in the
// host helper AddPacketToGpuFrame (driver/gpu_frame_step.h). Review of 9B, finding 3.
#include <gtest/gtest.h>

#include <memory>

#include "gpu_frame_step.h"

using hesai::lidar::AddPacketToGpuFrame;
using hesai::lidar::kMaxPacketNumPerFrame;
using hesai::lidar::kMaxPointsNumPerFrame;
using Frame = hesai::lidar::LidarDecodedFrame<hesai::lidar::LidarPointXYZIRT>;
using Packet = hesai::lidar::LidarDecodedPacket<hesai::lidar::LidarPointXYZIRT>;

namespace {

constexpr uint32_t kXtPointsPerPacket = 8 * 32;  // XT32: 8 blocks of 32 lasers

/// A decoded XT32 packet inside a frame (no frame boundary).
std::unique_ptr<Packet> XtPacket() {
  auto packet = std::make_unique<Packet>();
  packet->block_num = 8;
  packet->laser_num = 32;
  packet->points_num = kXtPointsPerPacket;
  packet->distance_unit = 0.004;
  packet->sensor_timestamp = 1790000000000000ULL;
  packet->scan_complete = false;
  return packet;
}

/// The GPU loop's state across packets: its frame, the Lidar's own frame, the packets kept.
struct GpuLoop {
  std::unique_ptr<Frame> frame = std::make_unique<Frame>();
  std::unique_ptr<Frame> lidar_frame = std::make_unique<Frame>();
  hesai::lidar::UdpFrame_t udp_packet_frame;
  int packet_index = 0;
  std::unique_ptr<Packet> decoded = XtPacket();
  hesai::lidar::UdpPacket packet;

  void Add(int packets) {
    for (int i = 0; i < packets; ++i) {
      AddPacketToGpuFrame(*decoded, packet, *frame, packet_index, udp_packet_frame, *lidar_frame);
    }
  }
};

}  // namespace

// Finding 3 (review of 9B): after kMaxPacketNumPerFrame packets without a frame boundary the
// loop drops the frame ("fail to start a new frame"). The frame it publishes must start again
// empty, and the dropped frame must count in its frame_index, so that the driver's
// frames_skipped sees the gap (J.33) as it does with the CPU SDK. Today the loop resets the
// Lidar's own frame instead: its frame keeps the failed frame's points (stale points in the next
// cloud) and its frame_index stays.
TEST(GpuFrameLoop, AFrameThatFailedToStartIsDroppedAndCounted) {
  GpuLoop loop;
  loop.frame->frame_index = 41;
  loop.Add(kMaxPacketNumPerFrame);

  EXPECT_EQ(loop.packet_index, 0);
  EXPECT_TRUE(loop.udp_packet_frame.empty());
  EXPECT_EQ(loop.frame->points_num, 0u) << "the failed frame's points stay in the published frame";
  EXPECT_EQ(loop.frame->frame_index, 42) << "the dropped frame is not counted in frame_index";

  loop.Add(3);  // the next frame carries only its own packets
  EXPECT_EQ(loop.frame->points_num, 3 * kXtPointsPerPacket);
}

// Finding 3: two frames in a row that fail to start never take the published frame past its
// buffer: today points_num reaches kMaxPointsNumPerFrame after the second failure and grows past
// it with the next packets, which ToRosMsg then reads (source_driver_ros1.hpp, the point loop).
TEST(GpuFrameLoop, TwoFailedFramesNeverOverrunTheFrame) {
  GpuLoop loop;
  loop.Add(2 * kMaxPacketNumPerFrame + 10);

  EXPECT_LE(loop.frame->points_num, kMaxPointsNumPerFrame);
  EXPECT_EQ(loop.frame->points_num, 10 * kXtPointsPerPacket);
  EXPECT_EQ(loop.frame->frame_index, 2) << "both dropped frames count in frame_index";
}
