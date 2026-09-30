// Firing-block timing of the PandarXT-32 packet (SDK xt_block_timing.h), and
// that the Udp6_1 parser applies it to every point it decodes.
#include <gtest/gtest.h>

#include <cstdint>
#include <cstring>
#include <sstream>
#include <string>

#include "udp6_1_parser.h"
#include "xt_block_timing.h"

using hesai::lidar::HS_LIDAR_PRE_HEADER;
using hesai::lidar::HsLidarXTV1BodyAzimuth;
using hesai::lidar::HsLidarXTV1BodyChannelData;
using hesai::lidar::HsLidarXTV1Header;
using hesai::lidar::HsLidarXTV1Tail;
using hesai::lidar::LidarDecodedFrame;
using hesai::lidar::LidarDecodedPacket;
using hesai::lidar::LidarPointXYZIRT;
using hesai::lidar::Udp6_1Parser;
using hesai::lidar::UdpPacket;
using hesai::lidar::XtBlockStartOffsetUs;
using hesai::lidar::XtIsDualReturn;

namespace {

constexpr uint8_t kSingleStrongest = 0x37;
constexpr uint8_t kDualLastStrongest = 0x39;

// Manual B.3, per 1-based block m: single return -50 (8 - m) us; dual return
// blocks 8&7 / 6&5 / 4&3 / 2&1 at 0 / -50 / -100 / -150 us.
constexpr double kSingleUs[8] = {-350, -300, -250, -200, -150, -100, -50, 0};
constexpr double kDualUs[8] = {-150, -150, -100, -100, -50, -50, 0, 0};

// The shipped XT32 firetime file: channel n fires 6 + 1.512 (n - 1) us after the
// start of block 8, i.e. manual B.4's 5.632 + dt(n).
std::string Xt32Firetimes() {
  std::ostringstream s;
  s << "Channel,fire time(us)\n";
  for (int n = 1; n <= 32; ++n) s << n << "," << 6.0 + 1.512 * (n - 1) << "\n";
  return s.str();
}

// One XT32 UDP payload (manual 3.1): pre-header, header, 8 blocks x 32 channels
// at 10 m, tail with the given return mode and packet time.
UdpPacket Xt32Packet(uint8_t return_mode, uint32_t us) {
  UdpPacket p{};  // zeroed
  uint8_t* b = p.buffer;
  b[0] = 0xEE;
  b[1] = 0xFF;
  b[2] = 6;  // PandarXT, UDP protocol 6.1
  b[3] = 1;
  auto* hdr = reinterpret_cast<HsLidarXTV1Header*>(b + sizeof(HS_LIDAR_PRE_HEADER));
  hdr->m_u8LaserNum = 32;
  hdr->m_u8BlockNum = 8;
  hdr->m_u8DistUnit = 4;
  uint8_t* body = reinterpret_cast<uint8_t*>(hdr) + sizeof(HsLidarXTV1Header);
  const size_t block_size = sizeof(HsLidarXTV1BodyAzimuth) + 32 * sizeof(HsLidarXTV1BodyChannelData);
  for (int blk = 0; blk < 8; ++blk) {
    auto* az = reinterpret_cast<HsLidarXTV1BodyAzimuth*>(body + blk * block_size);
    az->m_u16Azimuth = static_cast<uint16_t>(1000 + 18 * blk);
    auto* ch = reinterpret_cast<HsLidarXTV1BodyChannelData*>(body + blk * block_size +
                                                             sizeof(HsLidarXTV1BodyAzimuth));
    for (int c = 0; c < 32; ++c) ch[c].m_u16Distance = 2500;
  }
  auto* tail = reinterpret_cast<HsLidarXTV1Tail*>(body + 8 * block_size);
  tail->m_u8ReturnMode = return_mode;
  tail->m_u16MotorSpeed = 600;
  const uint8_t utc[6] = {126, 9, 30, 12, 0, 0};  // 2026-09-30 12:00:00
  std::memcpy(tail->m_u8UTC, utc, sizeof(utc));
  tail->m_u32Timestamp = us;
  p.packet_len = static_cast<int16_t>(reinterpret_cast<uint8_t*>(tail) + sizeof(HsLidarXTV1Tail) - b);
  return p;
}

}  // namespace

TEST(XtBlockTiming, SingleReturnBlocksFireFiftyMicrosecondsApart) {
  for (int b = 0; b < 8; ++b) EXPECT_DOUBLE_EQ(XtBlockStartOffsetUs(b, 8, 32, false), kSingleUs[b]) << b;
}

TEST(XtBlockTiming, DualReturnBlocksShareAFiring) {
  for (int b = 0; b < 8; ++b) EXPECT_DOUBLE_EQ(XtBlockStartOffsetUs(b, 8, 32, true), kDualUs[b]) << b;
}

TEST(XtBlockTiming, OnlyTheXt32LayoutIsCorrected) {
  EXPECT_DOUBLE_EQ(XtBlockStartOffsetUs(0, 6, 32, false), 0.0);  // 6-block packets
  EXPECT_DOUBLE_EQ(XtBlockStartOffsetUs(0, 8, 16, false), 0.0);  // XT16
  EXPECT_DOUBLE_EQ(XtBlockStartOffsetUs(8, 8, 32, false), 0.0);  // out of range
  EXPECT_DOUBLE_EQ(XtBlockStartOffsetUs(-1, 8, 32, true), 0.0);
}

TEST(XtBlockTiming, DualReturnModes) {
  for (uint16_t m : {0x39, 0x3B, 0x3C}) EXPECT_TRUE(XtIsDualReturn(m)) << std::hex << m;
  for (uint16_t m : {0x33, 0x37, 0x38, 0x00}) EXPECT_FALSE(XtIsDualReturn(m)) << std::hex << m;
}

TEST(XtBlockTiming, TheParserStampsEachPointAtItsFiringTime) {
  for (uint8_t mode : {kSingleStrongest, kDualLastStrongest}) {
    Udp6_1Parser<LidarPointXYZIRT> parser;
    std::string firetimes = Xt32Firetimes();
    ASSERT_EQ(parser.LoadFiretimesString(&firetimes[0]), 0);
    LidarDecodedPacket<LidarPointXYZIRT> packet;
    packet.use_timestamp_type = 0;  // lidar time, as the driver runs
    ASSERT_EQ(parser.DecodePacket(packet, Xt32Packet(mode, 250000)), 0);
    EXPECT_EQ(packet.return_mode, mode);
    packet.packet_index = 0;
    LidarDecodedFrame<LidarPointXYZIRT> frame;
    ASSERT_EQ(parser.ComputeXYZI(frame, packet), 0);
    EXPECT_EQ(frame.return_mode, mode);

    const double t0 = packet.sensor_timestamp * 1e-6;  // the packet time
    const double* block_us = (mode == kDualLastStrongest) ? kDualUs : kSingleUs;
    for (int blk = 0; blk < 8; ++blk) {
      for (int n = 1; n <= 32; ++n) {
        // Manual B.4: t(m, n) = T(m) + dt(n), with T(8) = t0 + 5.632 us and
        // dt(n) = 1.512 (n - 1) + 0.368 us.
        // A double at a 2026 epoch resolves 2^-22 s (0.24 us): allow two steps,
        // still far finer than the 1.512 us channel and 50 us block spacing.
        const double want = t0 + (block_us[blk] + 5.632 + 1.512 * (n - 1) + 0.368) * 1e-6;
        EXPECT_NEAR(frame.points[blk * 32 + (n - 1)].timestamp, want, 5e-7)
            << "mode 0x" << std::hex << int(mode) << std::dec << " block " << blk + 1 << " channel " << n;
      }
    }
  }
}
