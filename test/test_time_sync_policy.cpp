// Unit tests for the frame stamping policy (src/manager/time_sync_policy.hpp).
#include <gtest/gtest.h>

#include <cstdint>
#include <vector>

#include "time_sync_policy.hpp"

using hesai_ros_driver::DecodePtpClockStatus;
using hesai_ros_driver::DecodePtpStatusType1;
using hesai_ros_driver::DiagLevel;
using hesai_ros_driver::FrameTimes;
using hesai_ros_driver::HostReason;
using hesai_ros_driver::PtpStatus;
using hesai_ros_driver::StampMode;
using hesai_ros_driver::StampResult;
using hesai_ros_driver::TimeSyncConfig;
using hesai_ros_driver::TimeSyncPolicy;
using hesai_ros_driver::TimestampSource;

namespace {

constexpr double kHost0 = 1790770000.0;  // 2026-09-30, UTC
constexpr double kFramePeriod = 0.1;     // 600 rpm
constexpr double kSpan = 0.089;          // measured XT32 frame span
constexpr double kLatency = 0.001;       // measured arrival latency

PtpStatus Locked(int64_t offset_ns = 500) {
  PtpStatus s;
  s.reachable = true;
  s.port_state = 9;
  s.master_offset_ns = offset_ns;
  s.clock_status = 2;
  return s;
}

// Drives frames the way the lidar does: sensor time = UTC + tai, host sees
// each frame kLatency after its last point.
class Driver {
 public:
  explicit Driver(TimeSyncPolicy* p, int tai = 37) : p_(p), tai_(tai) {}

  StampResult Next(double span = kSpan, uint16_t rpm = 600) {
    const double start_utc = kHost0 + n_ * kFramePeriod;
    FrameTimes f;
    f.sensor_start_s = start_utc + tai_;
    f.sensor_end_s = f.sensor_start_s + span;
    f.spin_rpm = rpm;
    last_start_utc_ = start_utc;
    const double host_now = start_utc + span + kLatency;
    ++n_;
    return p_->StampFrame(f, host_now, Mono());
  }

  double Mono() const { return 100.0 + n_ * kFramePeriod; }
  double last_start_utc() const { return last_start_utc_; }
  void set_tai(int tai) { tai_ = tai; }

 private:
  TimeSyncPolicy* p_;
  int tai_;
  int n_ = 0;
  double last_start_utc_ = 0.0;
};

}  // namespace

TEST(TimeSyncPolicy, HostModeBeforeAnyPtpStatus) {
  TimeSyncPolicy p{TimeSyncConfig()};
  Driver d(&p);
  const StampResult r = d.Next();
  EXPECT_EQ(r.mode, StampMode::kHost);
  EXPECT_EQ(r.reason, HostReason::kNoPtpStatus);
  EXPECT_TRUE(r.point_times_valid);
  EXPECT_TRUE(r.mode_changed);  // the first frame always reports its mode
  // The frame start on the host clock: late by the arrival latency only, not
  // by the frame span, since the point offsets count from the start.
  EXPECT_NEAR(r.stamp_s, d.last_start_utc() + kLatency, 1e-6);
}

TEST(TimeSyncPolicy, SwitchingModesMovesTheStampByTheLatencyOnly) {
  TimeSyncPolicy p{TimeSyncConfig()};
  Driver d(&p);
  p.UpdatePtpStatus(Locked(), d.Mono());
  StampResult r;
  for (int i = 0; i < 5; ++i) r = d.Next();
  ASSERT_EQ(r.mode, StampMode::kSensor);
  EXPECT_NEAR(r.stamp_s, d.last_start_utc(), 1e-6);
  p.UpdatePtpStatus(Locked(200000), d.Mono());  // offset too large: host time
  r = d.Next();
  ASSERT_EQ(r.mode, StampMode::kHost);
  EXPECT_NEAR(r.stamp_s, d.last_start_utc() + kLatency, 1e-6);
  p.UpdatePtpStatus(Locked(), d.Mono());
  for (int i = 0; i < 5; ++i) r = d.Next();
  ASSERT_EQ(r.mode, StampMode::kSensor);
  EXPECT_NEAR(r.stamp_s, d.last_start_utc(), 1e-6);
  const auto s = p.Snapshot(d.Mono());
  EXPECT_EQ(s.transitions, 3u);
  EXPECT_EQ(s.non_monotonic, 0u);
}

TEST(TimeSyncPolicy, SwitchesToSensorAfterFiveConsistentFrames) {
  TimeSyncPolicy p{TimeSyncConfig()};
  Driver d(&p);
  p.UpdatePtpStatus(Locked(), d.Mono());
  for (int i = 0; i < 4; ++i) {
    const StampResult r = d.Next();
    EXPECT_EQ(r.mode, StampMode::kHost) << i;
    EXPECT_EQ(r.reason, HostReason::kConfirming) << i;
  }
  const StampResult r = d.Next();
  EXPECT_EQ(r.mode, StampMode::kSensor);
  EXPECT_EQ(r.reason, HostReason::kNone);
  EXPECT_TRUE(r.mode_changed);
  // Stamped with the frame's own start time, in UTC.
  EXPECT_NEAR(r.stamp_s, d.last_start_utc(), 1e-6);
  const auto s = p.Snapshot(d.Mono());
  EXPECT_EQ(s.tai_offset_s, 37);
  EXPECT_NEAR(s.latency_s, kLatency, 1e-6);
  EXPECT_EQ(TimeSyncPolicy::Level(s, p.config()), DiagLevel::kOk);
}

TEST(TimeSyncPolicy, WholeSecondOffsetIsDetected) {
  for (int tai : {0, 37, 38}) {
    TimeSyncPolicy p{TimeSyncConfig()};
    Driver d(&p, tai);
    p.UpdatePtpStatus(Locked(), d.Mono());
    StampResult r;
    for (int i = 0; i < 5; ++i) r = d.Next();
    EXPECT_EQ(r.mode, StampMode::kSensor) << "tai " << tai;
    EXPECT_NEAR(r.stamp_s, d.last_start_utc(), 1e-6) << "tai " << tai;
    EXPECT_EQ(p.Snapshot(d.Mono()).tai_offset_s, tai);
  }
}

TEST(TimeSyncPolicy, FallsBackAtOnceOnPtpProblems) {
  struct Case {
    PtpStatus status;
    HostReason reason;
  };
  PtpStatus not_slave = Locked();
  not_slave.port_state = 8;
  PtpStatus free_run = Locked();
  free_run.clock_status = 0;
  PtpStatus frozen = Locked();
  frozen.clock_status = 3;
  PtpStatus tracking_far = Locked(200000);
  tracking_far.clock_status = 1;
  const std::vector<Case> cases = {
      {not_slave, HostReason::kPtpNotSlave},
      {free_run, HostReason::kPtpNotLocked},
      {frozen, HostReason::kPtpNotLocked},
      {tracking_far, HostReason::kPtpOffsetTooLarge},
      {Locked(200000), HostReason::kPtpOffsetTooLarge},
      {Locked(-200000), HostReason::kPtpOffsetTooLarge},
  };
  for (const Case& c : cases) {
    TimeSyncPolicy p{TimeSyncConfig()};
    Driver d(&p);
    p.UpdatePtpStatus(Locked(), d.Mono());
    for (int i = 0; i < 6; ++i) d.Next();
    p.UpdatePtpStatus(c.status, d.Mono());
    const StampResult r = d.Next();
    EXPECT_EQ(r.mode, StampMode::kHost);
    EXPECT_EQ(r.reason, c.reason);
    EXPECT_TRUE(r.mode_changed);
    // Re-entry needs a fresh confirmation.
    p.UpdatePtpStatus(Locked(), d.Mono());
    for (int i = 0; i < 4; ++i) EXPECT_EQ(d.Next().mode, StampMode::kHost);
    EXPECT_EQ(d.Next().mode, StampMode::kSensor);
  }
}

TEST(TimeSyncPolicy, TrackingWithinTheOffsetLimitKeepsPtpTime) {
  // The XT32 reads Tracking whenever it is more than its 1 us lock threshold
  // off -- every grandmaster wobble. The master offset is what decides.
  TimeSyncPolicy p{TimeSyncConfig()};
  Driver d(&p);
  p.UpdatePtpStatus(Locked(), d.Mono());
  for (int i = 0; i < 5; ++i) d.Next();
  PtpStatus tracking = Locked(60000);  // 60 us, as measured at 12:20Z
  tracking.clock_status = 1;
  p.UpdatePtpStatus(tracking, d.Mono());
  const StampResult r = d.Next();
  EXPECT_EQ(r.mode, StampMode::kSensor);
  EXPECT_FALSE(r.mode_changed);
  EXPECT_NEAR(r.stamp_s, d.last_start_utc(), 1e-6);
}

TEST(TimeSyncPolicy, UnknownClockStatusDoesNotBlock) {
  TimeSyncPolicy p{TimeSyncConfig()};
  Driver d(&p);
  PtpStatus s = Locked();
  s.clock_status = -1;  // 0x09 not answered; port state and offset still gate
  p.UpdatePtpStatus(s, d.Mono());
  StampResult r;
  for (int i = 0; i < 5; ++i) r = d.Next();
  EXPECT_EQ(r.mode, StampMode::kSensor);
}

TEST(TimeSyncPolicy, UnreachableAndStaleStatus) {
  TimeSyncPolicy p{TimeSyncConfig()};
  Driver d(&p);
  p.UpdatePtpStatus(Locked(), d.Mono());
  for (int i = 0; i < 5; ++i) d.Next();
  // One or two lost replies keep the last good status.
  p.MarkPtcUnreachable();
  EXPECT_EQ(d.Next().mode, StampMode::kSensor);
  p.MarkPtcUnreachable();
  EXPECT_EQ(d.Next().mode, StampMode::kSensor);
  p.MarkPtcUnreachable();  // the third in a row
  EXPECT_EQ(d.Next().reason, HostReason::kPtcUnreachable);
  // A good reply resets the count: two more failures are tolerated again.
  p.UpdatePtpStatus(Locked(), d.Mono());
  for (int i = 0; i < 5; ++i) d.Next();
  p.MarkPtcUnreachable();
  p.MarkPtcUnreachable();
  EXPECT_EQ(d.Next().mode, StampMode::kSensor);

  // Never a good reply: the first failure already means no PTP status.
  TimeSyncPolicy n{TimeSyncConfig()};
  Driver dn(&n);
  n.MarkPtcUnreachable();
  EXPECT_EQ(dn.Next().reason, HostReason::kPtcUnreachable);

  TimeSyncPolicy q{TimeSyncConfig()};
  Driver e(&q);
  q.UpdatePtpStatus(Locked(), e.Mono());
  StampResult r;
  for (int i = 0; i < 400; ++i) r = e.Next();  // 40 s without a new status
  EXPECT_EQ(r.reason, HostReason::kPtpStatusStale);
}

TEST(TimeSyncPolicy, ImplausibleFrameIsHostWithoutPointTimes) {
  TimeSyncPolicy p{TimeSyncConfig()};
  Driver d(&p);
  p.UpdatePtpStatus(Locked(), d.Mono());
  for (int i = 0; i < 5; ++i) d.Next();
  const StampResult stepped = d.Next(4.3);  // PTP servo step inside the frame
  EXPECT_EQ(stepped.mode, StampMode::kHost);
  EXPECT_EQ(stepped.reason, HostReason::kImplausibleFrame);
  EXPECT_FALSE(stepped.point_times_valid);
  // No start to recover: the arrival time itself.
  EXPECT_NEAR(stepped.stamp_s, d.last_start_utc() + 4.3 + kLatency, 1e-6);
  // 1200 rpm allows 75 ms, so the 600 rpm span is implausible there.
  EXPECT_FALSE(d.Next(kSpan, 1200).point_times_valid);
  // Unknown spin rate falls back to frame_span_max_s (0.15 s).
  EXPECT_TRUE(d.Next(0.14, 0).point_times_valid);
  EXPECT_FALSE(d.Next(0.16, 0).point_times_valid);
  EXPECT_EQ(p.Snapshot(d.Mono()).implausible_frames, 3u);
}

TEST(TimeSyncPolicy, NonWholeSecondOffsetStaysHost) {
  TimeSyncPolicy p{TimeSyncConfig()};
  p.UpdatePtpStatus(Locked(), 100.0);
  FrameTimes f;
  f.sensor_start_s = kHost0 + 37.4;
  f.sensor_end_s = f.sensor_start_s + kSpan;
  f.spin_rpm = 600;
  for (int i = 0; i < 6; ++i) {
    // A constant 37.4 s between the sensor clock and the host clock.
    const StampResult r = p.StampFrame(f, f.sensor_end_s - 37.4, 100.0);
    EXPECT_EQ(r.mode, StampMode::kHost);
    EXPECT_EQ(r.reason, HostReason::kOffsetFraction);
    f.sensor_start_s += kFramePeriod;
    f.sensor_end_s += kFramePeriod;
  }
}

TEST(TimeSyncPolicy, OffsetChangeIsReconfirmed) {
  TimeSyncPolicy p{TimeSyncConfig()};
  Driver d(&p, 37);
  p.UpdatePtpStatus(Locked(), d.Mono());
  for (int i = 0; i < 5; ++i) d.Next();
  d.set_tai(38);  // a leap second
  const StampResult changed = d.Next();
  EXPECT_EQ(changed.mode, StampMode::kHost);
  EXPECT_EQ(changed.reason, HostReason::kTaiOffsetChanged);
  for (int i = 0; i < 3; ++i) EXPECT_EQ(d.Next().reason, HostReason::kConfirming);
  const StampResult r = d.Next();
  EXPECT_EQ(r.mode, StampMode::kSensor);
  EXPECT_NEAR(r.stamp_s, d.last_start_utc(), 1e-6);
}

TEST(TimeSyncPolicy, ForcedHostIgnoresPtp) {
  TimeSyncConfig cfg;
  cfg.source = TimestampSource::kHost;
  TimeSyncPolicy p{cfg};
  Driver d(&p);
  p.UpdatePtpStatus(Locked(), d.Mono());
  StampResult r;
  for (int i = 0; i < 10; ++i) r = d.Next();
  EXPECT_EQ(r.mode, StampMode::kHost);
  EXPECT_EQ(r.reason, HostReason::kForcedHost);
  EXPECT_EQ(TimeSyncPolicy::Level(p.Snapshot(d.Mono()), cfg), DiagLevel::kOk);
}

TEST(TimeSyncPolicy, ForcedSensorWithFixedOffsetForReplay) {
  TimeSyncConfig cfg;
  cfg.source = TimestampSource::kSensor;
  cfg.tai_offset_auto = false;
  cfg.tai_offset_s = 37;
  TimeSyncPolicy p{cfg};
  // Recorded a week ago, replayed now, no PTC.
  FrameTimes f;
  f.sensor_start_s = kHost0 - 7 * 86400.0 + 37.0;
  f.sensor_end_s = f.sensor_start_s + kSpan;
  f.spin_rpm = 600;
  const StampResult r = p.StampFrame(f, kHost0, 100.0);
  EXPECT_EQ(r.mode, StampMode::kSensor);
  EXPECT_NEAR(r.stamp_s, kHost0 - 7 * 86400.0, 1e-6);
  EXPECT_EQ(TimeSyncPolicy::Level(p.Snapshot(100.0), cfg), DiagLevel::kOk);
}

TEST(TimeSyncPolicy, ForcedSensorWithAutoOffsetSkipsPtp) {
  TimeSyncConfig cfg;
  cfg.source = TimestampSource::kSensor;
  TimeSyncPolicy p{cfg};
  Driver d(&p);
  StampResult r;
  for (int i = 0; i < 5; ++i) r = d.Next();
  EXPECT_EQ(r.mode, StampMode::kSensor);
  EXPECT_NEAR(r.stamp_s, d.last_start_utc(), 1e-6);
}

TEST(TimeSyncPolicy, FixedOffsetMustMatchHostInAutoMode) {
  TimeSyncConfig cfg;
  cfg.tai_offset_auto = false;
  cfg.tai_offset_s = 36;  // wrong for this lidar
  TimeSyncPolicy p{cfg};
  Driver d(&p);
  p.UpdatePtpStatus(Locked(), d.Mono());
  const StampResult r = d.Next();
  EXPECT_EQ(r.mode, StampMode::kHost);
  EXPECT_EQ(r.reason, HostReason::kOffsetFraction);
}

TEST(TimeSyncPolicy, FreeRunningClockUsesHostTimeButKeepsPointOffsets) {
  TimeSyncPolicy p{TimeSyncConfig()};
  p.UpdatePtpStatus(Locked(), 100.0);
  FrameTimes f;
  f.sensor_start_s = 12.0;  // seconds since power-up
  f.sensor_end_s = 12.0 + kSpan;
  f.spin_rpm = 600;
  const StampResult r = p.StampFrame(f, kHost0, 100.0);
  EXPECT_EQ(r.mode, StampMode::kHost);
  EXPECT_EQ(r.reason, HostReason::kSensorTimeInvalid);
  // Arrival minus the frame's own span: the frame start, like every host stamp.
  EXPECT_NEAR(r.stamp_s, kHost0 - kSpan, 1e-6);
  EXPECT_TRUE(r.point_times_valid);
}

TEST(TimeSyncPolicy, HostOffsetReseedsOnClockStep) {
  TimeSyncPolicy p{TimeSyncConfig()};
  Driver d(&p);
  for (int i = 0; i < 20; ++i) d.Next();
  d.set_tai(41);  // sensor clock jumps 4 s, no PTP status
  const StampResult r = d.Next();
  EXPECT_EQ(r.mode, StampMode::kHost);
  // Re-seeded: still the frame start on the host clock, not 4 s in the future.
  EXPECT_NEAR(r.stamp_s, d.last_start_utc() + kLatency, 1e-6);
}

TEST(TimeSyncPolicy, NonMonotonicStampsAreCounted) {
  TimeSyncPolicy p{TimeSyncConfig()};
  FrameTimes f;
  f.sensor_start_s = kHost0 + 37.0;
  f.sensor_end_s = f.sensor_start_s + kSpan;
  f.spin_rpm = 600;
  EXPECT_FALSE(p.StampFrame(f, kHost0 + 1.0, 100.0).non_monotonic);
  EXPECT_TRUE(p.StampFrame(f, kHost0 + 0.5, 100.1).non_monotonic);
  EXPECT_EQ(p.Snapshot(100.2).non_monotonic, 1u);
}

TEST(TimeSyncPolicy, PacketStampFollowsFrameMode) {
  TimeSyncPolicy p{TimeSyncConfig()};
  Driver d(&p);
  d.Next();
  const double sensor = kHost0 + 37.0 + 0.05;
  // HOST: aligned by the arrival offset of the first frame.
  EXPECT_NEAR(p.StampPacket(sensor, kHost0), sensor - 37.0 + kLatency, 1e-6);
  p.UpdatePtpStatus(Locked(), d.Mono());
  for (int i = 0; i < 5; ++i) d.Next();
  EXPECT_NEAR(p.StampPacket(sensor, kHost0), sensor - 37.0, 1e-9);
}

TEST(TimeSyncPolicy, LatencyAboveLimitWarns) {
  TimeSyncPolicy p{TimeSyncConfig()};
  p.UpdatePtpStatus(Locked(), 100.0);
  FrameTimes f;
  f.sensor_start_s = kHost0 + 37.0;
  f.sensor_end_s = f.sensor_start_s + kSpan;
  f.spin_rpm = 600;
  StampResult r;
  for (int i = 0; i < 5; ++i) {
    // PTP time 50 ms behind the host clock.
    r = p.StampFrame(f, f.sensor_end_s - 37.0 + 0.050, 100.0 + i * 0.1);
    f.sensor_start_s += kFramePeriod;
    f.sensor_end_s += kFramePeriod;
  }
  EXPECT_EQ(r.mode, StampMode::kSensor);
  const auto s = p.Snapshot(100.5);
  EXPECT_NEAR(s.latency_s, 0.050, 1e-6);
  EXPECT_EQ(TimeSyncPolicy::Level(s, p.config()), DiagLevel::kWarn);
}

TEST(TimeSyncPolicy, AutoHostModeWarns) {
  TimeSyncPolicy p{TimeSyncConfig()};
  Driver d(&p);
  EXPECT_EQ(TimeSyncPolicy::Level(p.Snapshot(d.Mono()), p.config()), DiagLevel::kWarn);  // no frames
  d.Next();
  EXPECT_EQ(TimeSyncPolicy::Level(p.Snapshot(d.Mono()), p.config()), DiagLevel::kWarn);
}

TEST(TimeSyncPolicy, DecodesLivePtcReplies) {
  // PTC 0x06 type 1 as read from the orca-01 XT32 on 2026-09-30.
  const std::vector<uint8_t> type1 = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfd, 0xdc,
                                      0x00, 0x00, 0x00, 0x09, 0x00, 0x00, 0x00, 0x8d};
  PtpStatus s;
  ASSERT_TRUE(DecodePtpStatusType1(type1.data(), type1.size(), &s));
  EXPECT_EQ(s.master_offset_ns, -548);
  EXPECT_EQ(s.port_state, 9);
  EXPECT_FALSE(DecodePtpStatusType1(type1.data(), 15, &s));

  std::vector<uint8_t> status(54, 0);
  status[48] = 2;
  EXPECT_EQ(DecodePtpClockStatus(status.data(), status.size()), 2);
  EXPECT_EQ(DecodePtpClockStatus(status.data(), 48), -1);
}
