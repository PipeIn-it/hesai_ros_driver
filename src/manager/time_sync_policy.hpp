/*
 * File: time_sync_policy.hpp
 * Description: Chooses the header stamp of each decoded lidar frame.
 *
 * When the lidar is PTP-locked its packet time is PTP time, which is TAI on a
 * linuxptp grandmaster (37 s ahead of UTC today). A frame is then stamped with
 * its own start time minus that whole-second offset: SENSOR mode. On any doubt
 * the frame keeps the arrival-based stamp, sensor time aligned to host time by
 * a smoothed offset: HOST mode, the behaviour this driver always had.
 *
 * ROS-free and header-only, so the ROS1 and ROS2 wrappers share it and it can
 * be unit-tested (test/test_time_sync_policy.cpp). Thread-safe: the SDK's
 * publishing thread stamps frames while the status thread updates PTP state.
 */
#pragma once

#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <mutex>
#include <string>

namespace hesai_ros_driver {

// Clock for PTP status age: immune to wall-clock steps and to sim time.
inline double MonotonicNowSec() {
  return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
}

enum class TimestampSource { kAuto, kSensor, kHost };

enum class StampMode { kHost, kSensor };

// Why a frame was not stamped with sensor time. kNone means it was.
enum class HostReason {
  kNone,
  kForcedHost,         // timestamp_source: host
  kNoPtpStatus,        // no PTP status read from the lidar yet
  kPtcUnreachable,     // the last PTC query failed
  kPtpStatusStale,     // no successful PTC query for ptp_status_max_age_s
  kPtpNotSlave,        // PTP port state is not SLAVE
  kPtpNotLocked,       // lidar clock status is not Locked
  kPtpOffsetTooLarge,  // |master offset| above ptp_max_offset_ns
  kSensorTimeInvalid,  // sensor clock outside 2000..2100 (free-running lidar)
  kImplausibleFrame,   // frame spans more time than one revolution allows
  kOffsetFraction,     // sensor - host is not a whole number of seconds
  kTaiOffsetChanged,   // the whole-second offset changed; re-confirming
  kConfirming,         // offset seen fewer than confirm_frames times in a row
};

inline const char* HostReasonName(HostReason r) {
  switch (r) {
    case HostReason::kNone: return "none";
    case HostReason::kForcedHost: return "timestamp_source is host";
    case HostReason::kNoPtpStatus: return "no PTP status from the lidar yet";
    case HostReason::kPtcUnreachable: return "PTC unreachable";
    case HostReason::kPtpStatusStale: return "PTP status stale";
    case HostReason::kPtpNotSlave: return "PTP port not SLAVE";
    case HostReason::kPtpNotLocked: return "lidar PTP clock not Locked";
    case HostReason::kPtpOffsetTooLarge: return "PTP master offset too large";
    case HostReason::kSensorTimeInvalid: return "sensor clock not set";
    case HostReason::kImplausibleFrame: return "frame time span implausible";
    case HostReason::kOffsetFraction: return "sensor-host offset not whole seconds";
    case HostReason::kTaiOffsetChanged: return "TAI-UTC offset changed";
    case HostReason::kConfirming: return "confirming TAI-UTC offset";
  }
  return "unknown";
}

inline const char* StampModeName(StampMode m) {
  return m == StampMode::kSensor ? "sensor (PTP)" : "host (arrival)";
}

// PTC 0x06 query type 1 reports the linuxptp port state; 9 is SLAVE.
constexpr int32_t kPtpPortStateSlave = 9;
// PTC 0x09 PTP clock status: 0 Free Run, 1 Tracking, 2 Locked, 3 Frozen.
constexpr int kPtpClockLocked = 2;

inline const char* PtpClockStatusName(int s) {
  switch (s) {
    case 0: return "free run";
    case 1: return "tracking";
    case 2: return "locked";
    case 3: return "frozen";
  }
  return "unknown";
}

struct TimeSyncConfig {
  TimestampSource source = TimestampSource::kAuto;
  bool tai_offset_auto = true;       // tai_utc_offset_s: auto
  int tai_offset_s = 0;              // used when tai_offset_auto is false
  int64_t ptp_max_offset_ns = 100000;
  double ptp_status_max_age_s = 30.0;
  double frame_span_max_s = 0.15;    // used when the spin rate is unknown
  bool publish_time_field = true;
  int confirm_frames = 5;
  double max_fraction_s = 0.25;      // allowed non-integer part of sensor - host
  double latency_warn_s = 0.020;     // diagnostics WARN threshold
  double host_alpha = 0.05;          // arrival-offset EMA gain
  double host_snap_s = 0.5;          // re-seed the EMA when the offset jumps
};

// Latest PTP state read from the lidar over PTC.
struct PtpStatus {
  bool reachable = false;
  int32_t port_state = -1;           // PTC 0x06 type 1
  int64_t master_offset_ns = 0;      // PTC 0x06 type 1
  int clock_status = -1;             // PTC 0x09; -1 when not read
};

struct FrameTimes {
  double sensor_start_s = 0.0;       // earliest point time, sensor clock
  double sensor_end_s = 0.0;         // latest point time, sensor clock
  uint16_t spin_rpm = 0;
};

struct StampResult {
  double stamp_s = 0.0;
  StampMode mode = StampMode::kHost;
  HostReason reason = HostReason::kNoPtpStatus;
  bool point_times_valid = false;    // per-point sensor-clock offsets are usable
  bool mode_changed = false;
  bool non_monotonic = false;
};

struct TimeSyncSnapshot {
  TimestampSource source = TimestampSource::kAuto;
  StampMode mode = StampMode::kHost;
  HostReason reason = HostReason::kNoPtpStatus;
  bool have_frames = false;
  bool have_offset = false;          // tai_offset_s holds a value
  int tai_offset_s = 0;
  bool have_status = false;
  PtpStatus ptp;
  double status_age_s = -1.0;
  double latency_s = 0.0;            // host_now - (sensor_end - tai_offset)
  double frame_span_s = 0.0;
  uint64_t frames = 0;
  uint64_t sensor_frames = 0;
  uint64_t host_frames = 0;
  uint64_t transitions = 0;
  uint64_t implausible_frames = 0;
  uint64_t non_monotonic = 0;
};

enum class DiagLevel { kOk, kWarn };

// Decodes the 16-byte PTC 0x06 type-1 reply: int64 master offset (ns),
// int32 port state, int32 elapsed time, all big-endian.
inline bool DecodePtpStatusType1(const uint8_t* data, size_t len, PtpStatus* out) {
  if (data == nullptr || len < 16 || out == nullptr) return false;
  uint64_t offset = 0;
  for (int i = 0; i < 8; ++i) offset = (offset << 8) | data[i];
  uint32_t state = 0;
  for (int i = 8; i < 12; ++i) state = (state << 8) | data[i];
  out->master_offset_ns = static_cast<int64_t>(offset);
  out->port_state = static_cast<int32_t>(state);
  return true;
}

// PTP clock status byte of the PTC 0x09 (lidar status) reply: after uptime
// (4), motor speed (2), 8 temperatures (32), GPS PPS and NMEA (2), start-up
// count (4) and operating time (4). Returns -1 for a short reply.
inline int DecodePtpClockStatus(const uint8_t* data, size_t len) {
  constexpr size_t kOffset = 48;
  if (data == nullptr || len <= kOffset) return -1;
  return data[kOffset];
}

class TimeSyncPolicy {
 public:
  explicit TimeSyncPolicy(const TimeSyncConfig& cfg) : cfg_(cfg) {}

  const TimeSyncConfig& config() const { return cfg_; }

  void UpdatePtpStatus(const PtpStatus& status, double mono_now_s) {
    std::lock_guard<std::mutex> lock(mutex_);
    status_ = status;
    status_.reachable = true;
    have_status_ = true;
    status_mono_s_ = mono_now_s;
  }

  // A failed PTC query. The previous values stay visible in diagnostics.
  void MarkPtcUnreachable() {
    std::lock_guard<std::mutex> lock(mutex_);
    status_.reachable = false;
    have_status_ = true;
  }

  StampResult StampFrame(const FrameTimes& f, double host_now_s, double mono_now_s) {
    std::lock_guard<std::mutex> lock(mutex_);
    StampResult r;
    ++frames_;
    have_frames_ = true;

    const bool time_valid = ValidSensorTime(f.sensor_start_s) && ValidSensorTime(f.sensor_end_s);
    const double span = f.sensor_end_s - f.sensor_start_s;
    last_span_s_ = span;
    // Offsets inside a frame are usable even on a free-running clock; a span
    // longer than a revolution means the clock stepped inside the frame.
    const bool span_ok = span > 0.0 && span <= MaxSpan(f.spin_rpm);
    const bool plausible = time_valid && span_ok;
    r.point_times_valid = span_ok;
    if (!span_ok) ++implausible_frames_;

    // Kept up to date in every mode, so a fall-back to HOST never jumps.
    if (time_valid) UpdateHostOffset(f.sensor_start_s - host_now_s);

    int k = 0;
    const HostReason reason = SensorBlocker(f, host_now_s, mono_now_s, time_valid, plausible, &k);
    if (reason == HostReason::kNone) {
      r.mode = StampMode::kSensor;
      r.stamp_s = f.sensor_start_s - k;
      active_offset_s_ = k;
      ++sensor_frames_;
    } else {
      r.mode = StampMode::kHost;
      r.stamp_s = (time_valid && host_offset_init_) ? f.sensor_start_s - host_offset_s_ : host_now_s;
      ++host_frames_;
    }
    r.reason = reason;
    if (time_valid && HaveOffset()) latency_s_ = host_now_s - (f.sensor_end_s - OffsetForDiagnostics());

    if (have_last_stamp_ && r.stamp_s <= last_stamp_s_) {
      r.non_monotonic = true;
      ++non_monotonic_;
    }
    last_stamp_s_ = r.stamp_s;
    have_last_stamp_ = true;

    r.mode_changed = !have_last_mode_ || r.mode != last_mode_;
    if (have_last_mode_ && r.mode != last_mode_) ++transitions_;
    have_last_mode_ = true;
    last_mode_ = r.mode;
    last_reason_ = reason;
    return r;
  }

  // Stamp for the raw-packet topic: the same clock choice as the last frame.
  double StampPacket(double sensor_first_s, double host_now_s) const {
    std::lock_guard<std::mutex> lock(mutex_);
    if (have_last_mode_ && last_mode_ == StampMode::kSensor) return sensor_first_s - active_offset_s_;
    if (ValidSensorTime(sensor_first_s) && host_offset_init_) return sensor_first_s - host_offset_s_;
    return host_now_s;
  }

  TimeSyncSnapshot Snapshot(double mono_now_s) const {
    std::lock_guard<std::mutex> lock(mutex_);
    TimeSyncSnapshot s;
    s.source = cfg_.source;
    s.mode = have_last_mode_ ? last_mode_ : StampMode::kHost;
    s.reason = last_reason_;
    s.have_frames = have_frames_;
    s.have_offset = HaveOffset();
    s.tai_offset_s = OffsetForDiagnostics();
    s.have_status = have_status_;
    s.ptp = status_;
    s.status_age_s = (have_status_ && status_mono_s_ >= 0.0) ? mono_now_s - status_mono_s_ : -1.0;
    s.latency_s = latency_s_;
    s.frame_span_s = last_span_s_;
    s.frames = frames_;
    s.sensor_frames = sensor_frames_;
    s.host_frames = host_frames_;
    s.transitions = transitions_;
    s.implausible_frames = implausible_frames_;
    s.non_monotonic = non_monotonic_;
    return s;
  }

  static DiagLevel Level(const TimeSyncSnapshot& s, const TimeSyncConfig& cfg) {
    if (!s.have_frames) return DiagLevel::kWarn;
    if (s.mode == StampMode::kSensor) return LatencyTooHigh(s, cfg) ? DiagLevel::kWarn : DiagLevel::kOk;
    return s.source == TimestampSource::kHost ? DiagLevel::kOk : DiagLevel::kWarn;
  }

  static std::string Summary(const TimeSyncSnapshot& s, const TimeSyncConfig& cfg) {
    if (!s.have_frames) return "no frames yet";
    if (s.mode == StampMode::kSensor) {
      std::string msg = "PTP time (sensor clock - " + std::to_string(s.tai_offset_s) + " s)";
      if (LatencyTooHigh(s, cfg)) msg += "; PTP time and host clock disagree";
      return msg;
    }
    return std::string("arrival time: ") + HostReasonName(s.reason);
  }

 private:
  // Only meaningful live: a replay forced to sensor time compares a recorded
  // clock with the replay clock.
  static bool LatencyTooHigh(const TimeSyncSnapshot& s, const TimeSyncConfig& cfg) {
    return s.source == TimestampSource::kAuto && std::fabs(s.latency_s) > cfg.latency_warn_s;
  }

  static bool ValidSensorTime(double t) {
    constexpr double kMin = 946684800.0;   // 2000-01-01
    constexpr double kMax = 4102444800.0;  // 2100-01-01
    return t >= kMin && t <= kMax;
  }

  // One revolution plus half, so a frame that holds a clock step is caught.
  double MaxSpan(uint16_t spin_rpm) const {
    if (spin_rpm >= 60 && spin_rpm <= 3000) return 1.5 * 60.0 / spin_rpm;
    return cfg_.frame_span_max_s;
  }

  void UpdateHostOffset(double raw) {
    if (!host_offset_init_ || std::fabs(raw - host_offset_s_) > cfg_.host_snap_s) {
      host_offset_s_ = raw;
      host_offset_init_ = true;
      return;
    }
    host_offset_s_ = (1.0 - cfg_.host_alpha) * host_offset_s_ + cfg_.host_alpha * raw;
  }

  void ResetConfirmation() {
    confirmed_ = false;
    candidate_count_ = 0;
  }

  bool HaveOffset() const { return !cfg_.tai_offset_auto || confirmed_ || candidate_count_ > 0; }

  int OffsetForDiagnostics() const {
    if (!cfg_.tai_offset_auto) return cfg_.tai_offset_s;
    return confirmed_ ? confirmed_offset_s_ : candidate_offset_s_;
  }

  HostReason PtpBlocker(double mono_now_s) const {
    if (!have_status_) return HostReason::kNoPtpStatus;
    if (!status_.reachable) return HostReason::kPtcUnreachable;
    if (mono_now_s - status_mono_s_ > cfg_.ptp_status_max_age_s) return HostReason::kPtpStatusStale;
    if (status_.port_state != kPtpPortStateSlave) return HostReason::kPtpNotSlave;
    if (status_.clock_status >= 0 && status_.clock_status != kPtpClockLocked) return HostReason::kPtpNotLocked;
    if (std::llabs(status_.master_offset_ns) > cfg_.ptp_max_offset_ns) return HostReason::kPtpOffsetTooLarge;
    return HostReason::kNone;
  }

  HostReason SensorBlocker(const FrameTimes& f, double host_now_s, double mono_now_s,
                           bool time_valid, bool plausible, int* k) {
    if (cfg_.source == TimestampSource::kHost) return HostReason::kForcedHost;
    if (!time_valid) {
      ResetConfirmation();
      return HostReason::kSensorTimeInvalid;
    }
    if (cfg_.source == TimestampSource::kAuto) {
      const HostReason ptp = PtpBlocker(mono_now_s);
      if (ptp != HostReason::kNone) {
        ResetConfirmation();
        return ptp;
      }
    }
    if (!plausible) {
      ResetConfirmation();
      return HostReason::kImplausibleFrame;
    }

    // The end of the frame arrived moments ago, so sensor_end - host_now is
    // the offset plus only the arrival latency, whatever the spin rate.
    const double x = f.sensor_end_s - host_now_s;
    if (!cfg_.tai_offset_auto) {
      // A fixed offset forced to sensor time (bag or pcap replay) is trusted
      // as configured; in auto mode it must still match the host clock.
      if (cfg_.source == TimestampSource::kAuto && std::fabs(x - cfg_.tai_offset_s) > cfg_.max_fraction_s) {
        return HostReason::kOffsetFraction;
      }
      *k = cfg_.tai_offset_s;
      return HostReason::kNone;
    }

    const double rounded = std::round(x);
    if (std::fabs(x - rounded) > cfg_.max_fraction_s || std::fabs(rounded) > 1e9) {
      ResetConfirmation();
      return HostReason::kOffsetFraction;
    }
    const int kc = static_cast<int>(rounded);
    if (confirmed_) {
      if (kc == confirmed_offset_s_) {
        *k = kc;
        return HostReason::kNone;
      }
      confirmed_ = false;
      candidate_offset_s_ = kc;
      candidate_count_ = 1;
      return HostReason::kTaiOffsetChanged;
    }
    if (candidate_count_ > 0 && kc == candidate_offset_s_) {
      ++candidate_count_;
    } else {
      candidate_offset_s_ = kc;
      candidate_count_ = 1;
    }
    if (candidate_count_ >= cfg_.confirm_frames) {
      confirmed_ = true;
      confirmed_offset_s_ = kc;
      *k = kc;
      return HostReason::kNone;
    }
    return HostReason::kConfirming;
  }

  const TimeSyncConfig cfg_;
  mutable std::mutex mutex_;

  PtpStatus status_;
  bool have_status_ = false;
  double status_mono_s_ = -1.0;

  double host_offset_s_ = 0.0;
  bool host_offset_init_ = false;

  bool confirmed_ = false;
  int confirmed_offset_s_ = 0;
  int candidate_offset_s_ = 0;
  int candidate_count_ = 0;
  int active_offset_s_ = 0;

  bool have_frames_ = false;
  bool have_last_mode_ = false;
  StampMode last_mode_ = StampMode::kHost;
  HostReason last_reason_ = HostReason::kNoPtpStatus;
  bool have_last_stamp_ = false;
  double last_stamp_s_ = 0.0;
  double latency_s_ = 0.0;
  double last_span_s_ = 0.0;

  uint64_t frames_ = 0;
  uint64_t sensor_frames_ = 0;
  uint64_t host_frames_ = 0;
  uint64_t transitions_ = 0;
  uint64_t implausible_frames_ = 0;
  uint64_t non_monotonic_ = 0;
};

}  // namespace hesai_ros_driver
